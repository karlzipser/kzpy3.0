import threading
from kzpy3.utils import *



##############################################################3
#
def setup(M,buttons=[0,1900,1700,1424,870]):

    M['state_transition_timer'] = Timer(0)
    M['n_avg_steer'] = 20
    M['n_avg_motor'] = 20
    M['n_avg_button'] = 15
    M['n_avg_encoder'] = 15
    M['steer_null'] = 1400
    M['motor_null'] = 1500
    M['steer_max'] = M['steer_null']+1
    M['motor_max'] = M['motor_null']+1
    M['steer_min'] = M['motor_null']-1
    M['motor_min'] = M['motor_null']-1
    M['buttons'] = buttons

    for q in ['button_pwm_lst',
        'steer_pwm_lst',
        'motor_pwm_lst',
        'steer_pwm_write_lst',
        'motor_pwm_write_lst',
        'encoder_lst']:
        M[q] = []

    M['state'] = 2
    M['previous_state'] = 1
    M['calibrated'] = False



#        
##############################################################3
#

def run_loop(Arduinos,M,BUTTON_DELTA=50,n_lst_steps=30):
    lock = threading.Lock()
    if 'MSE' not in Arduinos:
        print('MSE arduino not found')
        M['Stop_Arduinos'] = True
        return

    calibration_signal_timer = Timer(0.01)

    while M['Stop_Arduinos'] == False:

        if not serial_data_to_messages(Arduinos,M):
            continue

        buttons_to_state(Arduinos,M,BUTTON_DELTA)

        manage_list_lengths(M,n_lst_steps)

        if len(M['steer_pwm_lst']) >= M['n_avg_steer']:
            M['smooth_steer'] = array(M['steer_pwm_lst'][-M['n_avg_steer']:]).mean()
        else:
            M['smooth_steer'] = M['steer_null']

        if len(M['motor_pwm_lst']) >= M['n_avg_motor']:  
            M['smooth_motor'] = array(M['motor_pwm_lst'][-M['n_avg_motor']:]).mean()
        else:
            M['smooth_motor'] = M['motor_null']


        M['PID'] = [1,2]

        if M['PID'][0] < 0:
            M['pid_motor_pwm'] = M['smooth_motor']
        else:
            if M['state_transition_timer'].time()>0.5:
                if not 'pid_motor_pwm' in M:
                    M['pid_motor_pwm'] = M['smooth_motor']
                pid_low = M['PID'][0]
                pid_high = M['PID'][1]
                pid_mid = (pid_low+pid_high)/2.0
                if M['encoder_lst'][-1] > pid_high:
                    M['pid_motor_pwm'] -= 10*np.abs(M['encoder_lst'][-1]-pid_mid)/100.0
                elif M['encoder_lst'][-1] < pid_low:
                    M['pid_motor_pwm'] += 10*np.abs(M['encoder_lst'][-1]-pid_mid)/100.0
            else:
                M['pid_motor_pwm'] = M['smooth_motor']


        if M['state'] == 4:
            process_state_4(M,n_lst_steps)
            continue
        else:
            if not M['calibrated']:
                if calibration_signal_timer.check():
                    s = np.random.choice([1,2,3])
                    LED_signal = d2n('(',s*100+s*10+1001,')')
                    Arduinos['SIG'].write(LED_signal)
                    calibration_signal_timer.reset()
                continue



        M['raw_write_str'] = d2n( '(', int(M['steer_pwm_lst'][-1]), ',', int(M['motor_pwm_lst'][-1]+10000), ')')
        M['smooth_write_str'] = d2n( '(', int(M['smooth_steer']), ',', int(M['smooth_motor']+10000), ')')
        M['pid_write_str'] = d2n( '(', int(M['smooth_steer']), ',', int(M['pid_motor_pwm']+10000), ')')




        if M['state'] == 1:
            Arduinos['MSE'].write(M['raw_write_str'])
        if M['state'] == 2:
            Arduinos['MSE'].write(M['smooth_write_str'])
        if M['state'] == 3:
            Arduinos['MSE'].write(M['pid_write_str'])
        
        M['steer_percent'] = pwm_to_percent(M,M['steer_null'],M['steer_pwm_lst'][-1],M['steer_max'],M['steer_min'])
        M['motor_percent'] = pwm_to_percent(M,M['motor_null'],M['motor_pwm_lst'][-1],M['motor_max'],M['motor_min'])

#        
##############################################################3
#





def serial_data_to_messages(Arduinos,M):
    read_str = Arduinos['MSE'].readline()
    try:
        exec('mse_input = list({0})'.format(read_str))
    except:
        return False
    if len(mse_input) == 7 and mse_input[0] == 'mse':
        M['button_pwm_lst'].append(mse_input[1])
        M['steer_pwm_lst'].append(mse_input[2])
        M['motor_pwm_lst'].append(mse_input[3])
        M['steer_pwm_write_lst'].append(mse_input[4])
        M['motor_pwm_write_lst'].append(mse_input[5])
        M['encoder_lst'].append(mse_input[6])
        return True
    else:
        return False

"""
def enter_state_4():
    pass
def leave_state_4():
    pass    
def enter_state_4():
    pass
def enter_state_4():
    pass
"""

def buttons_to_state(Arduinos,M,BUTTON_DELTA):
    for s in range(1,5):
        if np.abs(M['button_pwm_lst'][-1]-M['buttons'][s]) < BUTTON_DELTA:
            if M['state'] != s:
                if M['state'] == 4:
                    M['steer_pwm_lst'] = [M['steer_null']]
                    M['motor_pwm_lst'] = [M['motor_null']]
                M['previous_state'] = M['state']
                M['state'] = s
                M['state_transition_timer'].reset()
                M['LED_signal'] = d2n('(',M['state']*100+M['state']*10+1001,')')
                Arduinos['SIG'].write(M['LED_signal'])
            return


def manage_list_lengths(M,n_lst_steps):
    for k in M:
        if type(M[k]) == list:        
            if len(M[k]) > 1.2 * n_lst_steps:
                M[k] = M[k][-n_lst_steps:]   







def process_state_4(M,n_lst_steps):
    M['calibrated'] = False
    if M['state_transition_timer'].time() < 0.5:
        M['set_null'] = False
    else:
        if M['set_null'] == False:
            M['steer_null'] = array(M['steer_pwm_lst'][-n_lst_steps:]).mean()
            M['motor_null'] = array(M['motor_pwm_lst'][-n_lst_steps:]).mean()
            M['set_null'] = True
            M['steer_max'] = M['steer_null']
            M['motor_max'] = M['motor_null']
            M['steer_min'] = M['steer_null']
            M['motor_min'] = M['motor_null']
        else:
            if M['smooth_steer'] > M['steer_max']:
                M['steer_max'] = M['smooth_steer']
            if M['smooth_motor'] > M['motor_max']:
                M['motor_max'] = M['smooth_motor']
            if M['smooth_steer'] < M['steer_min']:
                M['steer_min'] = M['smooth_steer']
            if M['smooth_motor'] < M['motor_min']:
                M['motor_min'] = M['smooth_motor']
        
    if np.abs(M['steer_max']-M['steer_min']) > 100 and np.abs(M['motor_max']-M['motor_min']) > 100:
        M['calibrated'] = True
     

def pwm_to_percent(M,null_pwm,current_pwm,max_pwm,min_pwm):
    current_pwm -= null_pwm
    max_pwm -= null_pwm
    min_pwm -= null_pwm
    if np.abs(min_pwm)<10 or np.abs(min_pwm)<10:
        M['calibrated'] = False
        return 49
    if current_pwm >= 0:
        p = int(99*(1.0 + current_pwm/max_pwm)/2.0)
    else:
        p = int(99*(1.0 - current_pwm/min_pwm)/2.0)
    if p > 110 or p < -10:
        M['calibrated'] = False
        return 49
    if p > 99:
        p = 99
    if p < 0:
        p = 0      
    return p







#        
##############################################################3
#