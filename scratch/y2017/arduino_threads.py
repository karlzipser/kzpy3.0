#!/usr/bin/env python

from kzpy3.utils import *
import arduino_MSE
import arduino_IMU
import arduino_SIG
import arduino_serial_in
import threading



def arduino_mse_thread():
    arduino_MSE.run_loop(Arduinos,M)

def arduino_motor_control_thread():
    arduino_motor_control.run_loop(M)

def arduino_imu_thread():
    arduino_IMU.run_loop(Arduinos,M)

def arduino_sig_thread():
    arduino_SIG.run_loop(Arduinos,M)

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
        #if 'acc' in M:
        #    print M['acc']
        #else:
        #    M['state'] = np.random.choice([1,2,3,4])
        time.sleep(0.1)

M = {}
M['Stop_Arduinos'] = False

baudrate = 115200
timeout = 0.1

Arduinos = arduino_serial_in.assign_serial_connections(arduino_serial_in.get_arduino_serial_connections(baudrate,timeout))
arduino_MSE.setup(M)
threading.Thread(target=arduino_mse_thread).start()
threading.Thread(target=arduino_imu_thread).start()
threading.Thread(target=arduino_sig_thread).start()
threading.Thread(target=arduino_master_thread).start()


q = raw_input('')
while q not in ['q','Q']:
    
    print M['LED_signal']
 
    q = raw_input('')
M['Stop_Arduinos'] = True

