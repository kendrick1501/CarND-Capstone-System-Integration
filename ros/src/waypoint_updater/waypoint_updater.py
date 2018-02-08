#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from std_msgs.msg import Int32, Bool, Float32
from copy import deepcopy

import math
import tf
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 150 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
        sub3 = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size = 1)
        
        #sub4 = rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        sub5 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb, queue_size = 1)
        sub6 = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.breaking_flag_pub = rospy.Publisher('/breaking_flag', Bool, queue_size=1)
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        self.breaking_acc_pub = rospy.Publisher('/breaking_acc', Float32, queue_size=1)

        self.current_pose = PoseStamped()
        self.current_velocity = 0
        
        self.base_waypoints = Lane()
        self.base_waypoints_length = 0
        self.final_waypoints = Lane()
        
        self.traffic_lights = TrafficLightArray()
        self.traffic_waypoint = -1
        
        self.obstacle_waypoint = -1
        
        self.breaking = False
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        
        self.loop()
    
    def loop(self):    
        rate = rospy.Rate(20) # 20Hz
        while not rospy.is_shutdown():
          self.breaking_flag_pub.publish(self.breaking)
          lane = Lane()
          lane.header.stamp = rospy.Time().now()
          lane.header.frame_id = '/world'
          if len(self.base_waypoints.waypoints):
            self.final_waypoints.waypoints = self.get_final_waypoints()
            lane.waypoints = self.final_waypoints.waypoints
            self.final_waypoints_pub.publish(lane)	
        rate.sleep()

    def pose_cb(self, msg):
        self.current_pose = msg
        
    def current_velocity_cb(self, msg):
    	self.current_velocity = msg.twist.linear.x

    def waypoints_cb(self, msg):
        if not self.base_waypoints_length:
        	self.base_waypoints = msg
        	self.base_waypoints_length = len(self.base_waypoints.waypoints)

    def traffic_cb(self, msg):
        if self.traffic_waypoint!=msg.data:
            rospy.logwarn("Traffic waypoint: %d", msg.data)
        self.traffic_waypoint = msg.data
        
        
    def traffic_lights_cb(self, msg):
        self.traffic_lights = msg

    def obstacle_cb(self, msg):
        #self.obstacle_waypoint = msg.data
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.linear.x = velocity

    def distance_base_waypoints(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    
    def distance(self, p1, p2):
    	dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    	return dl(p1, p2)
    	
    def get_closest_waypoint(self, waypoints, waypoint):
    	min_dist = 50.
    	closest_wp = -1
    	if waypoint is not None:
		  	for indx in range(len(waypoints)):
		  		dist = self.distance(waypoint.pose.position, waypoints[indx].pose.pose.position)
		  		if dist < min_dist:
		  			min_dist = dist
		  			closest_wp = indx
    	return closest_wp
    	   	
    def get_next_waypoint(self, waypoint, waypoints):
    	closest_wp = self.get_closest_waypoint(waypoints, waypoint)
    	wp = waypoints[closest_wp].pose
    	delta_x = wp.pose.position.x - waypoint.pose.position.x
    	delta_y = wp.pose.position.y - waypoint.pose.position.y
    	theta = math.atan2(delta_y, delta_x)
    	
    	q_x = self.current_pose.pose.orientation.x
    	q_y = self.current_pose.pose.orientation.y
    	q_z = self.current_pose.pose.orientation.z
    	q_w = self.current_pose.pose.orientation.w
    	euler_angles = tf.transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])
    	yaw = math.fabs(euler_angles[-1])
 
    	delta_rad = math.fabs(theta-yaw) 
    	if delta_rad > math.pi/4: #Choose the waypoint ahead of next to correct high heading differentials
    		closest_wp+=1
    	return closest_wp
    	
    def get_final_waypoints(self):
        final_waypoints = []
        next_wp = self.get_next_waypoint(self.current_pose, self.base_waypoints.waypoints) + 1
        stop_wp = self.traffic_waypoint-2
        if stop_wp<0: self.breaking = False

        if stop_wp>=0 and stop_wp>next_wp-10 and self.breaking is not True:
          self.breaking = True
          
          #rospy.logwarn("Stop waypoint: %.2f - %.2f", self.base_waypoints.waypoints[stop_wp].pose.pose.position.x, self.base_waypoints.waypoints[stop_wp].pose.pose.position.y)
          dist_to_stop = self.distance_base_waypoints(self.base_waypoints.waypoints, next_wp, stop_wp)
          
          acc = 0
          if dist_to_stop > 0.5: acc = - self.current_velocity**2 / (2*dist_to_stop)
          if acc < self.decel_limit: acc = self.decel_limit
          self.breaking_acc_pub.publish(acc)
          rospy.logwarn("Acceleration cmd: %.2f. Stopping distance: %.2f", acc, dist_to_stop)
          
          tmp_wp = Waypoint()
          for i in range(next_wp, stop_wp): #Append new waypoints with decreasing velocity reference
            indx = i % self.base_waypoints_length
            tmp_wp = deepcopy(self.base_waypoints.waypoints[indx])
            dist_to_prev_wp = self.distance_base_waypoints(self.base_waypoints.waypoints, indx-1, indx)
            #rospy.logwarn("Distance to next waypoint: %.2f", dist_to_next_wp)
            if i>next_wp:
              tmp_velocity = final_waypoints[-1].twist.twist.linear.x**2 + 2*acc*dist_to_prev_wp
            else:	
              tmp_velocity = self.current_velocity**2 + 2*acc*dist_to_prev_wp

            if tmp_velocity<0.:
              tmp_wp.twist.twist.linear.x = 0
            else:
              tmp_wp.twist.twist.linear.x = math.sqrt(tmp_velocity)
              
            final_waypoints.append(tmp_wp)
            rospy.logwarn("Waypoint %d new velocity: %.2f", indx, tmp_wp.twist.twist.linear.x)
            
          for i in range(stop_wp, next_wp+LOOKAHEAD_WPS): #Append the remaining waypoints to complete LOOKAHEAD_WPS waypoints
            indx = i % self.base_waypoints_length
            tmp_wp = deepcopy(self.base_waypoints.waypoints[indx])
            tmp_wp.twist.twist.linear.x = 0
            final_waypoints.append(tmp_wp)
            #rospy.logwarn("new waypoint velocity: %.2f - %.2f", tmp_wp.twist.twist.linear.x, tmp_wp.twist.twist.angular.z)
        
        elif stop_wp>=0 and stop_wp>next_wp-10 and self.breaking is True:
          next_wp_breaking = self.get_next_waypoint(self.current_pose, self.final_waypoints.waypoints)
          stop_wp_breaking = self.get_closest_waypoint(self.final_waypoints.waypoints, self.base_waypoints.waypoints[stop_wp].pose)
          
          for i in range(next_wp_breaking, stop_wp_breaking): #Keep the final waypoints ahead of the ego vehicle until stop_wp
            indx = i % len(self.final_waypoints.waypoints)
            tmp_wp = deepcopy(self.final_waypoints.waypoints[indx])
            final_waypoints.append(tmp_wp)
            #rospy.logwarn("wp velocity: %.2f. ego vel: %.2f", tmp_wp.twist.twist.linear.x, self.current_velocity)
          
          for i in range(stop_wp, next_wp+LOOKAHEAD_WPS): #Append the remaining waypoints to complete LOOKAHEAD_WPS waypoints
            indx = i % self.base_waypoints_length
            tmp_wp = deepcopy(self.base_waypoints.waypoints[indx])
            tmp_wp.twist.twist.linear.x = 0
            final_waypoints.append(tmp_wp)
          
        else:
          for i in range(next_wp, next_wp + LOOKAHEAD_WPS):
            indx = i % self.base_waypoints_length
            final_waypoints.append(self.base_waypoints.waypoints[indx])
                       
        return final_waypoints
    	
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
