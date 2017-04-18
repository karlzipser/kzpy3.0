import threading

from kzpy3.utils import *






def run_loop(Arduinos,messages_dic):
    lock = threading.Lock()
    M = messages_dic
    while M['Stop_Arduinos'] == False:




        if M['state'] == 4:


            

            
            if M['state'] == 1:
                M['write_str'] = d2n( '(', int(M['steer_pwm_lst'][-1]), ',', int(M['motor_pwm_lst'][-1]+10000), ')')

            elif M['state'] == 2:
                lock.acquire()
                if 'external_write_str_A' in M:
                    M['write_str'] = M['external_write_str_A']
                lock.release()

            elif M['state'] == 3:

                if 'external_write_str_B' in M:
                    M['write_str'] = M['external_write_str_B']


            if 'write_str' in M:
                Arduinos['MSE'].write(M['write_str'])

        if M['state'] == 2:
            try:        



                M['external_write_str_A'] = d2n( '(', int(M['steer_pwm_smooth_lst'][-1]), ',', int(M['motor_pwm_smooth_lst'][-1]+10000), ')')
                #print M['external_write_str_A']
                lock.release()
            except Exception as e:
                print e

        elif M['state'] == 3:
            try:
                if len(M['steer_pwm_smooth_lst']) >= n_avg_steer:
                    M['steer_pwm_smooth_lst'].append((array(M['steer_pwm_lst'][-n_avg_steer:]).mean()))
                    M['motor_pwm_smooth_lst'].append((array(M['motor_pwm_smooth_lst'][-n_avg_motor:]).mean()))
                    lock.acquire()

                    if M['encoder_lst'][-1] > 5:
                        M['motor_pwm_smooth_lst'][-1] -= 10*np.abs(M['encoder_lst'][-1]-4.5)/1000.0
                    elif M['encoder_lst'][-1] < 4:
                        M['motor_pwm_smooth_lst'][-1] += 10*np.abs(M['encoder_lst'][-1]-4.5)/1000.0
                    print M['motor_pwm_smooth_lst'][-1],M['encoder_lst'][-1]
                    lock.release()
                    M['button_pwm_smooth_lst'].append(array(M['button_pwm_lst'][-n_avg_button:]).mean())
                else:
                    M['steer_pwm_smooth_lst'].append(M['steer_pwm_lst'][-1])
                    M['motor_pwm_smooth_lst'].append(M['motor_pwm_lst'][-1])
                    M['button_pwm_smooth_lst'].append(M['button_pwm_lst'][-1])
                lock.acquire()
                M['external_write_str_B'] = d2n( '(', int(M['steer_pwm_smooth_lst'][-1]), ',', int(M['motor_pwm_smooth_lst'][-1]+10000), ')')
                #print M['external_write_str_B']
                lock.release()
            except Exception as e:
                print e       



"""

Smoothing of steering
smoothing of motor_pwm_lst
smoothing of button
want to calibrate on smoothed values, because there can be spikes in raw signals

"""


