#!/usr/bin/env python

from kzpy3.utils import *
import ard_MSE
import ard_ser_in
import threading
import std_msgs.msg
import geometry_msgs.msg
import rospy

import kzpy3.teg2.bdd_car_versions.bdd_car_rewrite.runtime_params as rp
os.environ['STOP_ARDUINOS'] = 'False'
baudrate = 115200
timeout = 0.1
Arduinos = ard_ser_in.assign_serial_connections(ard_ser_in.get_arduino_serial_connections(baudrate,timeout))
time_step = Timer(1)
folder_display_timer = Timer(10)
git_pull_timer = Timer(60)
reload_timer = Timer(30)


M = {}
M['acc2rd_threshold'] = rp.acc2rd_threshold
M['steer_gain'] = rp.steer_gain
M['motor_gain'] = rp.motor_gain
M['Stop_Arduinos'] = False
M['PID_min_max'] = rp.PID_min_max
M['aruco_evasion_active'] = 0

def caffe_steer_callback(msg):
    global M
    M['caffe_steer'] = msg.data

def caffe_motor_callback(msg):
    global M
    M['caffe_motor'] = msg.data

print "here!"
"""
rospy.init_node('run_arduino',anonymous=True)
rospy.Subscriber('cmd/steer', std_msgs.msg.Int32, callback=caffe_steer_callback)
rospy.Subscriber('cmd/motor', std_msgs.msg.Int32, callback=caffe_motor_callback)

M['state_pub'] = rospy.Publisher('state', std_msgs.msg.Int32, queue_size=5) 
M['steer_pub'] = rospy.Publisher('steer', std_msgs.msg.Int32, queue_size=5) 
M['motor_pub'] = rospy.Publisher('motor', std_msgs.msg.Int32, queue_size=5) 
M['encoder_pub'] = rospy.Publisher('encoder', std_msgs.msg.Float32, queue_size=5)
M['gyro_pub'] = rospy.Publisher('gyro', geometry_msgs.msg.Vector3, queue_size=100)
M['gyro_heading_pub'] = rospy.Publisher('gyro_heading', geometry_msgs.msg.Vector3, queue_size=100)
M['acc_pub'] = rospy.Publisher('acc', geometry_msgs.msg.Vector3, queue_size=100)
"""
def arduino_mse_thread():
    ard_MSE.run_loop(Arduinos,M)

def arduino_motor_control_thread():
    ard_motor_control.run_loop(M)

def arduino_imu_thread():
    ard_IMU.run_loop(Arduinos,M)

def arduino_sig_thread():
    ard_SIG.run_loop(Arduinos,M)

def arduino_master_thread():
    #while M['Stop_Arduinos'] == False or not rospy.is_shutdown():
    while not rospy.is_shutdown():
        if os.environ['STOP_ARDUINOS'] == 'True':
            break
        if time_step.check():
            time_step.reset()
            reload_timer.reset()
            M['steer_gain'] = rp.steer_gain
            M['motor_gain'] = rp.motor_gain
            M['acc2rd_threshold'] = rp.acc2rd_threshold
            M['PID_min_max'] = rp.PID_min_max
        print query_states()
        #if 'steer_percent' in M and 'motor_percent' in M and 'current_state' in M:
        #    if M['current_state'] != None:
        #       print(M['steer_percent'],M['motor_percent'],M['current_state'].name)
        time.sleep(0.5)

def query_states():
    if 'steer_percent' in M and 'motor_percent' in M and 'current_state' in M:
            if M['current_state'] != None:
                return(M['steer_percent'],M['motor_percent'],M['current_state'].name)
    return None


ard_MSE.setup(M,Arduinos)
threading.Thread(target=arduino_mse_thread).start()
#threading.Thread(target=arduino_master_thread).start()

"""
q = raw_input('')
while q not in ['q','Q']:
    q = raw_input('')
M['Stop_Arduinos'] = True
os.environ['STOP_ARDUINOS'] = 'True'


print("exiting 'arduino_node.py'")

"""