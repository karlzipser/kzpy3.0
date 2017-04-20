#!/usr/bin/env python

from kzpy3.utils import *
import ard_MSE
import ard_IMU
import ard_SIG
import ard_ser_in
import threading
import std_msgs.msg
import geometry_msgs.msg
import rospy

M = {}



def caffe_steer_callback(msg):
    global M
    M['caffe_steer'] = msg.data

def caffe_motor_callback(msg):
    global M
    M['caffe_motor'] = msg.data

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

def arduino_mse_thread():
    ard_MSE.run_loop(Arduinos,M)

def arduino_motor_control_thread():
    ard_motor_control.run_loop(M)

def arduino_imu_thread():
    ard_IMU.run_loop(Arduinos,M)

def arduino_sig_thread():
    ard_SIG.run_loop(Arduinos,M)

def arduino_master_thread():
    while M['Stop_Arduinos'] == False:
        """
        try:
            print M['steer_pwm_lst'][-1],M['steer_pwm_smooth_lst'][-1]
            figure(1)
            clf()
            plot(M['steer_pwm_lst'][-100:],'b')
            plot(M['steer_pwm_smooth_lst'][-100:],'r')
            plot(M['motor_pwm_lst'][-100:],'b')
            plot(M['motor_pwm_smooth_lst'][-100:],'r')
            ylim(500,3000)
            pause(0.0001)
        except:
            pass
        """
        try:
            M['state_pub'].publish(std_msgs.msg.Int32(M['current_state'].number))
            print(M['caffe_steer_pwm'],M['current_state'].name,M['steer_pwm_lst'][-1],M['steer_percent'],M['motor_percent'],M['acc'])
        except:
            pass
        #else:
        #    M['state'] = np.random.choice([1,2,3,4])
        time.sleep(0.5)
    


M['Stop_Arduinos'] = False

baudrate = 115200
timeout = 0.1

Arduinos = ard_ser_in.assign_serial_connections(ard_ser_in.get_arduino_serial_connections(baudrate,timeout))
ard_MSE.setup(M,Arduinos)
threading.Thread(target=arduino_mse_thread).start()
threading.Thread(target=arduino_imu_thread).start()
threading.Thread(target=arduino_sig_thread).start()
threading.Thread(target=arduino_master_thread).start()



q = raw_input('')
while q not in ['q','Q']:
    q = raw_input('')
M['Stop_Arduinos'] = True
rospy.signal_shutdown("""M['Stop_Arduinos'] = True""")



