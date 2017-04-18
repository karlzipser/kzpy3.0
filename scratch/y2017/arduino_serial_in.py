from kzpy3.vis import *
import arduino_MSE
import arduino_IMU
import arduino_SIG
import arduino_motor_control
import threading

baudrate=115200
timeout=0.1 

"""
sudo chmod 666 /dev/ttyACM*

"""

messages_dic = {}
messages_dic['Stop_Arduinos'] = False

def get_arduino_serial_connections(baudrate, timeout):
    sers = []
    ACM_ports = [os.path.join('/dev', p) for p in os.listdir('/dev') if 'ttyACM' in p]
    for ACM_port in ACM_ports:
        try:
            sers.append(serial.Serial(ACM_port, baudrate=baudrate, timeout=timeout))
            print('Opened {0}'.format(ACM_port))
        except:
            pass
    return sers

def assign_serial_connections(sers):
    Arduinos = {}
    for ser in sers:
        for _ in xrange(100):
            try:
                ser_str = ser.readline()
                
                exec('ser_tuple = list({0})'.format(ser_str))
                if ser_tuple[0] in ['mse']:
                    print(d2s('Port',ser.port,'is the MSE:',ser_str))
                    Arduinos['MSE'] = ser
                    break
                elif ser_tuple[0] in ['acc','gyro','head']:
                    print(d2s('Port',ser.port,'is the IMU:',ser_str))
                    Arduinos['IMU'] = ser
                    break
                elif ser_tuple[0] in ['GPS2']:
                    print(d2s('Port',ser.port,'is the SIG:',ser_str))
                    
                    Arduinos['SIG'] = ser
                    break
            except:
                pass
        else:
            print('Unable to identify port {0}'.format(ser.port))
            messages_dic['Stop_Arduinos'] = True
    return Arduinos




Arduinos = assign_serial_connections(get_arduino_serial_connections(baudrate,timeout))


"""
while not rospy.is_shutdown():
    for ser in serial_connections:
        try:
            ser_str = ser.readline()
            #print ser_str
            exec('ser_tuple = list({0})'.format(ser_str))
            if ser_tuple[0] == 'mse':
                button_in_pub.publish(ser_tuple[1])
                steer_in_pub.publish(ser_tuple[2])
                motor_in_pub.publish(ser_tuple[3])
        except:
            pass
"""

M = messages_dic

def arduino_mse_thread():
    arduino_MSE.run_loop(Arduinos,messages_dic)

def arduino_motor_control_thread():
    arduino_motor_control.run_loop(messages_dic)

def arduino_imu_thread():
    arduino_IMU.run_loop(Arduinos,messages_dic)

def arduino_sig_thread():
    arduino_SIG.run_loop(Arduinos,messages_dic)

def arduino_master_thread():
    while messages_dic['Stop_Arduinos'] == False:
        try:
            print messages_dic['steer_pwm_lst'][-1],M['steer_pwm_smooth_lst'][-1]
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
        #if 'acc' in messages_dic:
        #    print messages_dic['acc']
        #else:
        #    messages_dic['state'] = np.random.choice([1,2,3,4])
        time.sleep(0.1)

arduino_MSE.setup(messages_dic)
#arduino_motor_control.setup(messages_dic)
threading.Thread(target=arduino_mse_thread).start()
#threading.Thread(target=arduino_motor_control_thread).start()
threading.Thread(target=arduino_imu_thread).start()
threading.Thread(target=arduino_sig_thread).start()
threading.Thread(target=arduino_master_thread).start()

q = raw_input('')
while q not in ['q','Q']:
    
    print messages_dic['LED_signal']
 
    q = raw_input('')
messages_dic['Stop_Arduinos'] = True

