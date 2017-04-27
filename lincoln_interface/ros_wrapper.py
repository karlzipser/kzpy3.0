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
        
        steeringCmd_msg.steering_wheel_angle_cmd = steer_output_rad
        
        self.steer_pub.publish(steeringCmd_msg)
    
    def lilliput_steering(self):
        #rospy.init_node('lilliput_steering', anonymous=True)
        rospy.Subscriber("/bair_car/cmd/steer", std_msgs.msg.Int32, self.steering_callback)
        
        
        #rospy.spin()
        
        



#test = Lilliput_steering()