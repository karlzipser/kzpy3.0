

import std_msgs.msg
import rospy 


steer_cmd_pub = rospy.Publisher('/steer', std_msgs.msg.Int32, queue_size=5) 

rospy.init_node('a') 

#while True:
for i in range(100):
	steer_cmd_pub.publish(std_msgs.msg.Int32(i))
	print i
	time.sleep(0.001)









from kzpy3.vis import *
#import roslib
import std_msgs.msg
import rospy

steer_list = [0]

def steer_callback(msg):
	global steer_list
	steer_list.append(msg.data)

rospy.init_node('listener',anonymous=True)

rospy.Subscriber('/steer', std_msgs.msg.Int32, callback=steer_callback)

steer_list = [-1]
while steer_list[-1] != 99: #not rospy.is_shutdown():
	print steer_list[-1]
	time.sleep(0.1)




