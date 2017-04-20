########################################################
#          CAFFE SETUP SECTION
import caffe
caffe.set_device(0)
caffe.set_mode_gpu()
from kzpy3.utils import *
import cv2
os.chdir(home_path) # this is for the sake of the train_val.prototxt
import kzpy3.teg2.car_run_params
#from kzpy3.teg2.car_run_params import *
print 'CCCCCCCCCCCCCCC'
def setup_solver():
	solver = caffe.SGDSolver(solver_file_path)
	for l in [(k, v.data.shape) for k, v in solver.net.blobs.items()]:
		print(l)
	for l in [(k, v[0].data.shape) for k, v in solver.net.params.items()]:
		print(l)
	return solver


#
########################################################


########################################################
#          ROSPY SETUP SECTION
import roslib
import std_msgs.msg
import geometry_msgs.msg
import cv2
from cv_bridge import CvBridge,CvBridgeError
import rospy
from sensor_msgs.msg import Image
bridge = CvBridge()
print 'QQQQQQQQQQQQQQQQQQQQ'
rospy.init_node('listener',anonymous=True)
print 'SSSSSSSSSSSSSSSSSSS'
left_list = []
right_list = []


def right_callback(data):
	globalleft_list, right_list, solver
	cimg = bridge.imgmsg_to_cv2(data,"bgr8")
	if len(right_list) > 5:
		right_list = right_list[-5:]
	right_list.append(cimg)
def left_callback(data):
	global left_list, right_list
	cimg = bridge.imgmsg_to_cv2(data,"bgr8")
	if len(left_list) > 5:
		left_list = left_list[-5:]
	left_list.append(cimg)
##
########################################################

rospy.Subscriber("/bair_car/zed/right/image_rect_color",Image,right_callback,queue_size = 1)
rospy.Subscriber("/bair_car/zed/left/image_rect_color",Image,left_callback,queue_size = 1)

ctr = 0

t0 = time.time()
time_step = Timer(1)
caffe_enter_timer = Timer(2)
folder_display_timer = Timer(30)
git_pull_timer = Timer(60)
reload_timer = Timer(10)


def run_loop(Arduinos,M):
	print 'AAAAAAAAAAAAAa'
	M['caffe_steer'] = 49
	M['caffe_motor'] = 49
	solver = setup_solver()
	if weights_file_path != None:
		print "loading " + weights_file_path
		solver.net.copy_from(weights_file_path)

	while M['Stop_Arduinos'] == False:
		print 'BBBBBBBBB'
		if M['current_state'] in [M['state_three'],M['state_five'],M['state_six'],M['state_seven']]:
			
			if use_caffe:
				if M['current_state'].state_transition_timer.time() < 5:

					print "waiting before entering caffe mode..."
					#steer_cmd_pub.publish(std_msgs.msg.Int32(49))
					#motor_cmd_pub.publish(std_msgs.msg.Int32(49))
					time.sleep(0.5)
					continue
				else:
					if len(left_list) > 4:
						l0 = left_list[-2]
						l1 = left_list[-1]
						r0 = right_list[-2]
						r1 = right_list[-1]
						
						cv2.imshow('l0',cv2.cvtColor(l0,cv2.COLOR_RGB2BGR))
						if cv2.waitKey(30) & 0xFF == ord('q'):
							pass
						solver.net.blobs['ZED_data'].data[0,0,:,:] = l0[:,:,0]
						solver.net.blobs['ZED_data'].data[0,1,:,:] = l1[:,:,0]
						solver.net.blobs['ZED_data'].data[0,2,:,:] = r0[:,:,0]
						solver.net.blobs['ZED_data'].data[0,3,:,:] = r1[:,:,0]
						solver.net.blobs['ZED_data'].data[0,4,:,:] = l0[:,:,1]
						solver.net.blobs['ZED_data'].data[0,5,:,:] = l1[:,:,1]
						solver.net.blobs['ZED_data'].data[0,6,:,:] = r0[:,:,1]
						solver.net.blobs['ZED_data'].data[0,7,:,:] = r1[:,:,1]
						solver.net.blobs['ZED_data'].data[0,8,:,:] = l0[:,:,2]
						solver.net.blobs['ZED_data'].data[0,9,:,:] = l1[:,:,2]
						solver.net.blobs['ZED_data'].data[0,10,:,:] = r0[:,:,2]
						solver.net.blobs['ZED_data'].data[0,11,:,:] = r1[:,:,2]
							

						solver.net.blobs['metadata'].data[0,0,:,:] = Racing#target_data[0]/99. #current steer
						solver.net.blobs['metadata'].data[0,1,:,:] = 0#target_data[len(target_data)/2]/99. #current motor
						solver.net.blobs['metadata'].data[0,2,:,:] = Follow
						solver.net.blobs['metadata'].data[0,3,:,:] = Direct
						solver.net.blobs['metadata'].data[0,4,:,:] = Play
						solver.net.blobs['metadata'].data[0,5,:,:] = Furtive
						

						solver.net.forward(start='ZED_data',end='ip2')

						caf_steer = 100*solver.net.blobs['ip2'].data[0,9]
						caf_motor = 100*solver.net.blobs['ip2'].data[0,19]

						caf_motor = int((caf_motor-49.) * motor_gain + 49)
						caf_steer = int((caf_steer-49.) * steer_gain + 49)

						if caf_motor > 99:
							caf_motor = 99
						if caf_motor < 0:
							caf_motor = 0
						if caf_steer > 99:
							caf_steer = 99
						if caf_steer < 0:
							caf_steer = 0

						print(('caffe',caf_steer,caf_motor))

		else:
			pass
		"""
		if reload_timer.check():
			reload(kzpy3.teg2.car_run_params)
			from kzpy3.teg2.car_run_params import *
			reload_timer.reset()
		"""



