# This is used to specifiy caffe mode and data file name information


from kzpy3.utils import time_str
from kzpy3.utils import opjh
import os

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
Location =  'rewrite_test' # 'local' #'Smyth_tape'

solver_file_path = opjh("kzpy3/caf5/z2_color/solver_live.prototxt")
#weights_file_path = opjh("kzpy3/caf6/z2_color_more/z2_color_more.caffemodel")
#weights_file_path = opjh("kzpy3/caf6/z2_color_more/z2_color_more_2.caffemodel")
#weights_file_path = opjh("kzpy3/caf7/z2_color/z2_color_state_1_5_6_7_iter_4000000.caffemodel")
#weights_file_path = opjh("kzpy3/caf7/z2_color/z2_color_state_1_5_6_7_iter_6000000.caffemodel")
#weights_file_path = opjh("kzpy3/caf7/z2_color/z2_color_state_1_5_6_7_iter_10900000.caffemodel")
#weights_file_path = opjh("kzpy3/caf7/z2_color/z2_color_state_1_5_6_7_iter_8000000.caffemodel")
#weights_file_path = opjh("kzpy3/caf6/z2_color_more/z2_color_more_3.caffemodel")
#weights_file_path = opjh("kzpy3/caf7/z2_color/solver_state_1_5_6_7_plus_extra_Smyth_racing_iter_400000.caffemodel")
weights_file_path = opjh("kzpy3/caf5/z2_color/z2_color.caffemodel")
verbose = False
use_caffe = True
steer_gain = 1.0
motor_gain = 0.25
acc2rd_threshold = 100
if False:
	gyro_freeze_threshold = 500
	acc_freeze_threshold_x = 12
	acc_freeze_threshold_y = 12
	acc_freeze_threshold_z = 12
	acc_freeze_threshold_z_neg = -7
	motor_freeze_threshold = 60
#
###################################################################

####################### specific car settings ################
#
if computer_name == 'Mr_Orange':
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Silver':
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Blue':
	#motor_gain = 1.0
	pass
if computer_name == 'Mr_Yellow':
	#motor_gain = 0.9
	pass
if computer_name == 'Mr_Black':
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





