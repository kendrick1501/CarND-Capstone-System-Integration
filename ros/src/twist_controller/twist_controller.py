import rospy
import math
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
from std_msgs.msg import Int32, Float32

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, kp, ki, kd):   
        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        self.min_speed = 0.
        
        self.velocity_control = PID(kp, ki, kd, mn=-1., mx=1.)
        self.steering_control = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        self.brake_filter = LowPassFilter(0.25, 0.02)
        self.braking_flag = True
        self.min_brake = 0.
        
        self.brake = 0
        self.throttle = 0
        
        self.last_time = None
        
        sub1 = rospy.Subscriber('/traffic_waypoint', Int32, self.stop_point_cb, queue_size = 1)
        sub1 = rospy.Subscriber('/breaking_acc', Float32, self.breaking_acc_cb, queue_size = 1)
        
    def stop_point_cb(self,msg):
        if msg.data<0:
          self.braking_flag=False
        else:
          self.braking_flag=True
          
    def breaking_acc_cb(self, msg):
        self.min_brake = math.fabs(msg.data) * self.vehicle_mass * self.wheel_radius
        #rospy.logwarn("Acceleration command: %.2f", msg.data)

    def scale_brake(self, cmd, break_deadband): #Converts the output of the velocity controller to torque commands
        
        max_brake = min(2*self.min_brake, math.fabs(self.decel_limit) * self.vehicle_mass * self.wheel_radius)
        min_brake = 50
        max_cmd = break_deadband
        min_cmd = -1
        m = (min_brake - max_brake) / (max_cmd - min_cmd)
        brake = min(max_brake, m * (cmd - max_cmd) + min_brake)
        return brake

    def control(self, cmd_linear_velocity, cmd_angular_velocity, current_velocity, dbw_enable):
        # Return throttle, brake, steer
        
        if self.last_time is None:
            self.last_time = rospy.get_time()
            delta_t = 0.02
        else:
          time = rospy.get_time()
          delta_t = time - self.last_time
          self.last_time = time
        
        velocity_deadband = 0.0 
        brake_deadband = -0.1
        if self.braking_flag: #Defines a velocity deadband to avoid the non fully stop behavior of the ego car at traffic lights
          velocity_deadband = 0.15
          brake_deadband = -0.001          
        
        self.brake = 0
        self.throttle = 0
        steering = 0.
        
        if not dbw_enable:
        	self.velocity_control.reset()
        else:
        	velocity_error = cmd_linear_velocity - current_velocity
        	control = self.velocity_control.step(velocity_error, delta_t)
        	if control>velocity_deadband and not self.braking_flag:
        	  self.throttle = control
        	elif control<brake_deadband:
        	  brake = self.scale_brake(control, brake_deadband)
        	  self.brake = self.brake_filter.filt(brake)     	  
        	elif self.braking_flag:
        	  self.brake = self.brake_filter.filt(1)
        	  #self.velocity_control.reset()
        	
        	steering = self.steering_control.get_steering(cmd_linear_velocity, cmd_angular_velocity, current_velocity)
        
        return self.throttle, self.brake, steering
