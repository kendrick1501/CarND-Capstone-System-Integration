#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 5

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.has_image = False
        self.ego_pose = None
        self.base_waypoints = []
        self.final_waypoints = []
        self.camera_image = None
        self.traffic_lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        #sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        
        sub4 = rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        self.camera_image = None
        self.tl_classifier = TLClassifier()
        self.stop_line_positions = self.config['stop_line_positions']
        
        self.loop()

    def loop(self):    
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
          stop_wp, state = self.process_traffic_lights()
          if self.has_image:
            self.has_image = False
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        rate.sleep()

    def pose_cb(self, msg):
        self.ego_pose = msg

    def waypoints_cb(self, msg):
		if not len(self.base_waypoints):
			self.base_waypoints = msg.waypoints
        
    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg

    def traffic_cb(self, msg):
        self.traffic_lights = msg

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        self.camera_image.encoding = 'rgb8'
        
    def distance(self, p1, p2):
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
		return dl(p1, p2)       

    def get_closest_waypoint(self, pose, waypoints):
		"""Identifies the closest path waypoint to the given position
				https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
		Args:
				pose (Pose): position to match a waypoint to

		Returns:
				int: index of the closest waypoint in self.waypoints

		"""
		min_dist = 50.
		closest_wp = -1
		if pose is not None:
			for indx in range(len(waypoints)):
				dist = self.distance(pose.pose.position, waypoints[indx].pose.pose.position)
				if dist < min_dist:
					min_dist = dist
					closest_wp = indx 
		return closest_wp

    def get_light_state(self):
        """Determines the current color of the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.camera_image is None:
          return TrafficLight.RED
        else:
          cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, 'rgb8')
          state = self.tl_classifier.get_classification(cv_image)

        #Get classification
        return state
        
    def get_tl_stop_waypoint(self): #Check the location of the closest traffic light stop with respect to the ego vehicle pose
		min_dist = 80.
		closest_tl_stop = -1
		stop_wp = -1
		tmp_wp = PoseStamped()
		
		if self.last_state == TrafficLight.RED or self.last_state == TrafficLight.YELLOW:
		  q_x = self.ego_pose.pose.orientation.x
		  q_y = self.ego_pose.pose.orientation.y
		  q_z = self.ego_pose.pose.orientation.z
		  q_w = self.ego_pose.pose.orientation.w
		  euler_angles = tf.transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])
		  abs_yaw = math.fabs(euler_angles[-1])
		
		  for indx in range(len(self.stop_line_positions)):
			  delta_x = self.stop_line_positions[indx][0] - self.ego_pose.pose.position.x
			  delta_y = self.stop_line_positions[indx][1] - self.ego_pose.pose.position.y
			  dist = math.sqrt( delta_x**2 + delta_y**2)
			  if dist < min_dist:
			    if abs_yaw>=math.pi/2:
				      delta_x *=-1; delta_y *=-1
			
			    theta = math.fabs(math.atan2(delta_y, delta_x))				
			    if theta < math.pi/2:
				    min_dist = dist
				    closest_tl_stop = indx
				    break
		  
		  if closest_tl_stop>=0:
		    tmp_wp.pose.position.x = self.stop_line_positions[indx][0]
		    tmp_wp.pose.position.y = self.stop_line_positions[indx][1]
		    tmp_wp.pose.position.z = 0. 
		    stop_wp = self.get_closest_waypoint(tmp_wp, self.base_waypoints)
		return stop_wp

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
        location and color

        Returns:
        int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        stop_wp = -1
        state = TrafficLight.UNKNOWN

        if self.camera_image is not None:
          state = self.get_light_state()
          '''
          Publish upcoming red lights at 10 Hz.
          Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
          of times till we start using it. Otherwise the previous stable state is
          used.
          '''
          if self.state != state:
              self.state_count = 0
              self.state = state
          elif self.state_count >= STATE_COUNT_THRESHOLD:
              self.last_state = self.state
              if self.state == TrafficLight.RED or self.state == TrafficLight.YELLOW:
                stop_wp = self.get_tl_stop_waypoint()
          else:
            self.state_count += 1
            
          self.last_wp = stop_wp
          
        #rospy.logwarn("Stop waypoint: %d. Light state: %d", stop_wp, state)          
        return stop_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
