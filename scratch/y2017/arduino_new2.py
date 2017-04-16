import os, serial, threading, Queue
import threading
from kzpy3.vis import *

"""
sudo chmod 666 /dev/ttyACM*

"""

buttons = [0,1900,1700,1424,870]
BUTTON_DELTA = 50






ACM_port='/dev/ttyACM2'
baudrate=115200
timeout=0.25
Arduinos = {}
Arduinos['motor'] = serial.Serial(ACM_port,baudrate=baudrate,timeout=timeout)


caffe_mode = 1



GRAPHICS = True

if GRAPHICS:
    n_lst_steps = 1000
else:
    n_lst_steps = 100

button_pwm_lst = []
button_pwm_out_lst = []
steer_pwm_lst = []
steer_pwm_out_lst = []
motor_pwm_lst = []
motor_pwm_out_lst = []

n_avg_steer_pwm_out = 20
n_avg_motor_pwm_out = 20
n_avg_button_pwm_out = 4



hist_timer = Timer(2000000)
steer_motor_timer = Timer(0.2)
state_timer = Timer(0.05)




t0 = time.time()
deltas = []

state = 2
previous_state = 1
state_transition_timer = Timer(0)


steer_max = buttons[4]
motor_max = buttons[4]
steer_min = buttons[1]
motor_min = buttons[1]

if GRAPHICS:
    steer_max_lst = []
    motor_max_lst = []
    steer_min_lst = []
    motor_min_lst = []
    steer_null_lst = []
    motor_null_lst = []

while True:

    try:        
        read_str = Arduinos['motor'].readline()
        
        if True:
            t1 = time.time()
            deltas.append(t1-t0)
            t0 = t1
            #print read_str
        
        exec('motor_data = list({0})'.format(read_str))

        button_pwm_lst.append(motor_data[0])
        steer_pwm_lst.append(motor_data[1])
        motor_pwm_lst.append(motor_data[2])

        if GRAPHICS:
            steer_max_lst.append(steer_max)
            motor_max_lst.append(motor_max)
            steer_min_lst.append(steer_min)
            motor_min_lst.append(motor_min)
            steer_null_lst.append(steer_null)
            motor_null_lst.append(motor_null)

        if len(steer_pwm_out_lst) >= n_avg_steer_pwm_out:
            steer_pwm_out_lst.append(array(steer_pwm_lst[-n_avg_steer_pwm_out:]).mean())
            motor_pwm_out_lst.append(array(motor_pwm_lst[-n_avg_motor_pwm_out:]).mean())
            button_pwm_out_lst.append(array(button_pwm_lst[-n_avg_button_pwm_out:]).mean())
        else:
            steer_pwm_out_lst.append(steer_pwm_lst[-1])


        for s in range(1,5):
            if np.abs(button_pwm_out_lst[-1]-buttons[s]) < BUTTON_DELTA:
                if state != s:
                    previous_state = state
                    state = s
                    state_transition_timer.reset()
                continue



        if GRAPHICS:
            if state_timer.check():
                #print (state,previous_state,dp(state_transition_timer.time(),2))
                state_timer.reset()



        if len(button_pwm_lst) > n_lst_steps*1.2:
            button_pwm_lst = button_pwm_lst[-n_lst_steps:]
        if len(steer_pwm_lst) > n_lst_steps*1.2:
            steer_pwm_lst = steer_pwm_lst[-n_lst_steps:]
        if len(motor_pwm_lst) > n_lst_steps*1.2:
            motor_pwm_lst = motor_pwm_lst[-n_lst_steps:]
        if len(steer_pwm_out_lst) > n_lst_steps*1.2:
            steer_pwm_out_lst = steer_pwm_out_lst[-n_lst_steps:]
        if len(motor_pwm_out_lst) > n_lst_steps*1.2:
            motor_pwm_out_lst = motor_pwm_out_lst[-n_lst_steps:]






        if GRAPHICS:
            if hist_timer.check():
                figure('hist');clf();plt.xlim(700,2000)
                plt.hist(button_pwm_lst,bins=200)
                pause(0.0001)
                hist_timer.reset()
            if steer_motor_timer.check():
                figure('steer_motor');clf();plt.xlim(0,1000);plt.ylim(700,2500)

                plot(steer_min_lst[-1000:],'r:'); plot(motor_min_lst[-1000:],'b:');
                plot(steer_null_lst[-1000:],'r:'); plot(motor_null_lst[-1000:],'b:');
                plot(steer_max_lst[-1000:],'r:'); plot(motor_max_lst[-1000:],'b:');
                plot(steer_pwm_out_lst[-1000:],'k'); plot(motor_pwm_out_lst[-1000:],'k'); plot(button_pwm_out_lst[-1000:],'k')
                plot(steer_pwm_lst[-1000:],'r'); plot(motor_pwm_lst[-1000:],'b'); plot(button_pwm_lst[-1000:],'g')
                
                pause(0.0001)            
                steer_motor_timer.reset()




        if state == 4:
            if state_transition_timer.time() < 0.5:
                set_null = False
            else:
                if set_null == False:
                    steer_null = array(steer_pwm_lst[-20:]).mean()
                    motor_null = array(motor_pwm_lst[-20:]).mean()
                    set_null = True
                    steer_max = steer_null
                    motor_max = motor_null
                    steer_min = steer_null
                    motor_min = motor_null
                else:
                    if steer_pwm_out_lst[-1] > steer_max:
                        steer_max = steer_pwm_out_lst[-1]
                    if motor_pwm_out_lst[-1] > motor_max:
                        motor_max = motor_pwm_out_lst[-1]
                    if steer_pwm_out_lst[-1] < steer_min:
                        steer_min = steer_pwm_out_lst[-1]
                    if motor_pwm_out_lst[-1] < motor_min:
                        motor_min = motor_pwm_out_lst[-1]




        write_str = d2n( '(', int(steer_pwm_out_lst[-1]), ',', int(motor_pwm_out_lst[-1]+10000), ')')
        print write_str
        Arduinos['motor'].write(write_str)




    except Exception as e:
        print e






