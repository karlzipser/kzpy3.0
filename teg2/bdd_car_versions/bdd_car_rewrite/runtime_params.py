# This is used to specifiy caffe mode and data file name information


from kzpy3.utils import time_str
from kzpy3.utils import opjh
import os
import numpy as np
print "***************** bdd_car_rewrite_run_params.py"

computer_name = "MR_Unknown"
try:  
   computer_name = os.environ["COMPUTER_NAME"]
except KeyError: 
   print """********** Please set the environment variable computer_name ***********
   e.g.,
   export COMPUTER_NAME="Mr_Orange"
   """


####################### general car settings ################
#
for i in range(5):
	print('*************' + computer_name + '***********')
Direct = 1.
Follow = 0.
Play = 0.
Furtive = 0.
Caf = 0.0
Racing = 0.0
Location =  'Fern' # 'local' #'Smyth_tape'

solver_file_path = opjh("kzpy3/caf5/z2_color/solver_live.prototxt")
#weights_file_path = opjh("kzpy3/caf5/z2_color/z2_color.caffemodel")
#weights_file_path = opjh("kzpy3/caf7/z2_color/z2_color_iter_16400000.caffemodel")
#weights_file_path = opjh("kzpy3/caf7/z2_color/z2_color_aruco2_iter_1200000.caffemodel")
weights_file_path = opjh("kzpy3/caf7/z2_color/z2_color_aruco3_iter_11900000.caffemodel")


verbose = False
use_caffe = True
steer_gain = 1.0
motor_gain = 1.0
acc2rd_threshold = 150

PID_min_max = [1.5,2.5]

gyro_freeze_threshold = 150
acc_freeze_threshold_x = 5
acc_freeze_threshold_y_max = 12
acc_freeze_threshold_y_min = 5
acc_freeze_threshold_z = 5
motor_freeze_threshold = 55
n_avg_IMU = 10
#
###################################################################

####################### specific car settings ################
#
if computer_name == 'Mr_Orange':
	#PID_min_max = [2.,3.]
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Silver':
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Blue':
	#PID_min_max = [1.5,2.5]
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Yellow':
	#motor_gain = 0.9
	pass
if computer_name == 'Mr_Black':
	PID_min_max = [2.,3.]
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_White':
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Teal':
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Audi':
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Purple':
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_LightBlue':
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Blue_Original':
	motor_gain = 0.5
	pass


#
###################################################################
# motor_gain = 1.0 # override individual settings

if Direct == 1:
	task = 'direct'
elif Play == 1:
	task = 'play'
elif Follow == 1:
	task = 'follow'
elif Furtive == 1:
	task = 'furtive'
elif Racing == 1:
	task = 'racing'
else:
	assert(False)


foldername = ''
if Follow == 1:
	foldername = 'follow_'

model_name = solver_file_path.split('/')[-2]

if Caf == 1:
	foldername = foldername + 'caffe2_' + model_name +'_'

foldername = foldername + task + '_'

foldername = foldername + Location + '_'

foldername = foldername + time_str() + '_'

foldername = foldername + computer_name

#
###################################################################
# Aruco code parameters

ar_params={
'ar_motor_command' : 49, # This is the resting command for stop
'ar_max_left_steering_angle' : np.deg2rad(-130),
'ar_max_right_steering_angle' : np.deg2rad(130),
'ar_max_left_command' : 100,
'ar_max_right_command' : 0,
'ar_left_range' : 50,
'ar_right_range' : 50,
'ar_min_perceived_distance' : 9999,
'ar_critical_distance' : 0.75,
'ar_stop_distance' : 0.5,
'ar_max_motor' : 70,
'ar_min_motor' : 59,
'ar_override_motor':49,
'ar_override_steer':49 } # Full stop. Backwards is not considered
        


