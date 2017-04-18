
from kzpy3.utils import *
import std_msgs.msg
import rospy

ard_str = '(0,0,0)'

def ard_str_callback(msg):
	global ard_str
	ard_str = msg.data

rospy.init_node('motor_servo_listener',anonymous=True)

rospy.Subscriber('/bair_car/ard_str', std_msgs.msg.String, callback=ard_str_callback)


while not rospy.is_shutdown():
	print ard_str
	time.sleep(0.01)




