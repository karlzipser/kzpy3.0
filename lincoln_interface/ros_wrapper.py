'''
Created on Apr 26, 2017

@author: Sascha Hornauer
'''
import roslib

roslib.load_manifest('dbw_mkz_msgs')

import rospy
import std_msgs.msg
import numpy as np

from dbw_mkz_msgs.msg import SteeringCmd


class Lilliput_steering():

    steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=10)

    previous_steering_command = None

    def __init__(self):
        self.lilliput_steering()

    def steering_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
        '''
        The dbw_mkz SteeringCmd message looks like this:
        # Steering Wheel
        float32 steering_wheel_angle_cmd        # rad, range -8.2 to 8.2
        float32 steering_wheel_angle_velocity   # rad/s, range 0 to 8.7, 0 = maximum
        
        # Enable
        bool enable
        
        # Ignore driver overrides
        bool ignore
        
        # Disable the driver override audible warning
        bool quiet
        
        # Watchdog counter (optional)
        uint8 count
        '''
        
        steeringCmd_msg = SteeringCmd()
        
        # The original steering values are in between 0 and 100 where 100 is max left and 0 is max right.
        # This is converted to the range -8.2 to 8.2
        
        steering_lilliput = data.data
        steering_norm = steering_lilliput / 100.0
        
        steer_max_left = -8.2 # rad
        steer_max_right = 8.2 # rad
        
        steer_range = np.abs(steer_max_left-steer_max_right)
        
        steer_output_rad = (steering_norm * steer_range) - (steer_range / 2.0)
        
        # Smooth the steering commands heavily
        if(self.previous_steering_command == None):
            self.previous_steering_command = steer_output_rad
            
        # First we calculate the absolute difference of two consecutive steering commands and divide it by the possible
        # range to reach normed values
        diff_values_normed = np.abs(steer_output_rad-self.previous_steering_command)/steer_range
        
        # If the difference in between values is high ( sudden movement ), the new value factor goes towards 0
        # and the old value factor, its opposite, goes towards 1
        new_value_factor = 1-diff_values_normed
        old_value_factor = diff_values_normed
        
        # To avoid having no movement at all in emergency situations the factors are limited
        if new_value_factor < 0.1:
            new_value_factor = 0.1
            old_value_factor = 0.9
        
        steer_output_rad = (steer_output_rad*new_value_factor + self.previous_steering_command*old_value_factor) 
        
        
        
        steeringCmd_msg.steering_wheel_angle_cmd = steer_output_rad
        
        self.steer_pub.publish(steeringCmd_msg)
    
    def lilliput_steering(self):
        #rospy.init_node('lilliput_steering', anonymous=True)
        rospy.Subscriber("/bair_car/cmd/steer", std_msgs.msg.Int32, self.steering_callback)
        
        
        #rospy.spin()
        
        



#test = Lilliput_steering()