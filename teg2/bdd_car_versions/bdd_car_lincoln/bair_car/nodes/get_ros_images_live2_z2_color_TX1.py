#!/usr/bin/env python


import kzpy3.teg2.bdd_car_versions.bdd_car_rewrite.runtime_params as rp
#from kzpy3.teg2.bdd_car_versions.bdd_car_rewrite.runtime_params import *
#import aruco_code
from kzpy3.utils import *

import roslib
import std_msgs.msg
import geometry_msgs.msg
import cv2
from cv_bridge import CvBridge,CvBridgeError
from ros_wrapper import Lilliput_steering
import rospy
from sensor_msgs.msg import Image
os.environ['GLOG_minloglevel'] = '2'
import caffe
caffe.set_device(0)
caffe.set_mode_gpu()
import cv2
os.chdir(home_path)

os.environ['STOP'] = 'False'

try:
	if os.environ['STOP'] == 'True':
		assert(False)
 # this is for the sake of the train_val.prototxt

	def setup_solver(solver_file_path):
		solver = caffe.SGDSolver(solver_file_path)
		for l in [(k, v.data.shape) for k, v in solver.net.blobs.items()]:
			print(l)
		for l in [(k, v[0].data.shape) for k, v in solver.net.params.items()]:
			print(l)
		return solver

	solver = None


	bridge = CvBridge()
	rospy.init_node('run_caffe',anonymous=True)

	left_list = []
	right_list = []

	state = 0
	previous_state = 0
	
	# For reloading the parameter file
	reload_timer = Timer(30)



	def state_callback(data):
		global state, previous_state
		if state != data.data:
			if state in [3,5,6,7] and previous_state in [3,5,6,7]:
				pass
			else:
				previous_state = state
		state = data.data
	def right_callback(data):
		global left_list, right_list, solver
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


	rospy.Subscriber("/bair_car/zed/right/image_rect_color",Image,right_callback,queue_size = 1)
	rospy.Subscriber("/bair_car/zed/left/image_rect_color",Image,left_callback,queue_size = 1)
	rospy.Subscriber('/bair_car/state', std_msgs.msg.Int32,state_callback)
	#rospy.Subscriber('/bair_car/steer', std_msgs.msg.Int32,steer_callback)
	#rospy.Subscriber('/bair_car/motor', std_msgs.msg.Int32,motor_callback)

	steer_cmd_pub = rospy.Publisher('cmd/steer', std_msgs.msg.Int32, queue_size=100)
	motor_cmd_pub = rospy.Publisher('cmd/motor', std_msgs.msg.Int32, queue_size=100)
	### ARUCO ROS CODE
	aruco_cmd_pub = rospy.Publisher('cmd/evasion_active', std_msgs.msg.Int32, queue_size=100)
	### ARUCO ROS CODE
	#freeze_cmd_pub = rospy.Publisher('cmd/freeze', std_msgs.msg.Int32, queue_size=100)
	#model_name_pub = rospy.Publisher('/bair_car/model_name', std_msgs.msg.String, queue_size=10)
	lilliput_steering = Lilliput_steering()


	caffe_enter_timer = Timer(2)
	caf_steer_previous = 49
	caf_motor_previous = 49
	
	while not rospy.is_shutdown():
		# Check timer for parameter file update
		if reload_timer.check():
			reload(rp)
			reload_timer.reset()
		
		
		if state in [3,5,6,7,10]:
			if rp.use_caffe:
				if solver == None:
					solver = setup_solver(rp.solver_file_path)
					if rp.weights_file_path != None:
						print "loading " + rp.weights_file_path
						solver.net.copy_from(rp.weights_file_path)
				if (previous_state not in [3,5,6,7]):
					previous_state = state
					caffe_enter_timer.reset()
				if not caffe_enter_timer.check():
					print "waiting before entering caffe mode..."
					steer_cmd_pub.publish(std_msgs.msg.Int32(49))
					motor_cmd_pub.publish(std_msgs.msg.Int32(49))
					time.sleep(0.1)
					continue
				else:
					if len(left_list) > 4:
						l0 = left_list[-2]
						l1 = left_list[-1]
						r0 = right_list[-2]
						r1 = right_list[-1]


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
							
						solver.net.blobs['metadata'].data[0,0,:,:] = rp.Racing#target_data[0]/99. #current steer
						solver.net.blobs['metadata'].data[0,1,:,:] = 0#target_data[len(target_data)/2]/99. #current motor
						solver.net.blobs['metadata'].data[0,2,:,:] = rp.Follow
						solver.net.blobs['metadata'].data[0,3,:,:] = rp.Direct
						solver.net.blobs['metadata'].data[0,4,:,:] = rp.Play
						solver.net.blobs['metadata'].data[0,5,:,:] = rp.Furtive
						
						solver.net.forward(start='ZED_data',end='ip2')

						caf_steer = 100*solver.net.blobs['ip2'].data[0,9]
						caf_motor = 100*solver.net.blobs['ip2'].data[0,19]

						lilliput_steering
					
						caf_steer = int((caf_steer+caf_steer_previous)/2.0)
						caf_steer_previous = caf_steer
						caf_motor = int((caf_motor+caf_motor_previous)/2.0)
						caf_motor_previous = caf_motor


						if caf_motor > 99:
							caf_motor = 99
						if caf_motor < 0:
							caf_motor = 0
						if caf_steer > 99:
							caf_steer = 99
						if caf_steer < 0:
							caf_steer = 0

						# turn of motor
						caf_motor = 49

						
						if state in [3,6,10]:			
							steer_cmd_pub.publish(std_msgs.msg.Int32(caf_steer))
						if state in [6,7,10]:
							motor_cmd_pub.publish(std_msgs.msg.Int32(caf_motor))





		else:
			caffe_enter_timer.reset()

	


except Exception as e:
	print("********** Exception ***********************",'red')
	print(e.message, e.args)
	os.environ['STOP'] = 'True'
	rospy.signal_shutdown(d2s(e.message,e.args))

	
