# export ROS_MASTER_URI=http://192.168.43.113:11311

from kzpy3.teg2.global_run_params import *
from kzpy3.teg2.gps_fence.geometry import *
from kzpy3.vis import *
import roslib
import std_msgs.msg
import rospy
"""
GPS2_lat = -999.9
GPS2_long = 0
GPS2_speed = 0
GPS2_angle = 0

def GPS2_lat_callback(msg):
	global GPS2_lat
	GPS2_lat = msg.data


rospy.init_node('listener',anonymous=True)

rospy.Subscriber('GPS2_lat', std_msgs.msg.Float32, callback=GPS2_lat_callback)


while not rospy.is_shutdown():
	try:
		print GPS2_lat
		time.sleep(0.5)
	except Exception as e:
		print e.message, e.args
"""
#os.environ["ROS_MASTER_URI"] = "http://192.168.43.113:11311"

GPS2_lat = -999.99
GPS2_long = -999.99
GPS2_speed = -999.99
GPS2_angle = -999.99

def GPS2_lat_callback(msg):
	global GPS2_lat
	GPS2_lat = msg.data
	
def GPS2_long_callback(msg):
	global GPS2_long
	GPS2_long = msg.data
	
def GPS2_speed_callback(msg):
	global GPS2_speed
	GPS2_speed = msg.data
	
def GPS2_angle_callback(msg):
	global GPS2_angle
	GPS2_angle = msg.data

rospy.init_node('listener',anonymous=True)
	
rospy.Subscriber('/bair_car/GPS2_lat', std_msgs.msg.Float32, callback=GPS2_lat_callback)
rospy.Subscriber('/bair_car/GPS2_long', std_msgs.msg.Float32, callback=GPS2_long_callback)
rospy.Subscriber('/bair_car/GPS2_speed', std_msgs.msg.Float32, callback=GPS2_speed_callback)
rospy.Subscriber('/bair_car/GPS2_angle', std_msgs.msg.Float32, callback=GPS2_angle_callback)
plt.ion()


miles_per_deg_lat = 68.94
miles_per_deg_lon_at_37p88 = 54.41
meters_per_mile = 1609.34

GPS2_lat_orig,GPS2_long_orig = 37.8814506531,-122.27844238
rng = 50
plt.figure(1)

#plt.xlim(GPS2_long_orig-rng,GPS2_long_orig+rng)
#plt.ylim(GPS2_lat_orig-rng,GPS2_lat_orig+rng)
plt.xlim(-rng,rng)
plt.ylim(-rng,rng)
plt_square()
ctr = 0
while not rospy.is_shutdown():

	try:
		print GPS2_lat,GPS2_long,GPS2_speed,GPS2_angle
		if GPS2_lat > -999:
			dx = (GPS2_lat-GPS2_lat_orig)*miles_per_deg_lat*meters_per_mile
			dy = (GPS2_long-GPS2_long_orig)*miles_per_deg_lon_at_37p88*meters_per_mile
			rp = rotatePoint([dx,dy],[dx+1,dy],GPS2_angle)
			if np.mod(ctr,2) == 0:
				clr = 'r'
			else:
				clr = 'k'
			#plt.plot(dy,dx,clr+'o')
			plt.plot([dy,rp[1]],[dx,rp[0]],clr)
		plt.pause(0.5);#time.sleep(0.5)
	except Exception as e:
		print e.message, e.args
	ctr += 1



raw_input()


gps_data = ['lat','long','speed','angle']

for g in gps_data:
	print "GPS2_"+g+" = -999.99"

for g in gps_data:
	print """def GPS2_"""+g+"""_callback(msg):
	global GPS2_"""+g+"""
	GPS2_"""+g+""" = msg.data
	"""

for g in gps_data:
	print """rospy.Subscriber('/bair_car/GPS2_"""+g+"""', std_msgs.msg.Float32, callback=GPS2_"""+g+"""_callback)"""

