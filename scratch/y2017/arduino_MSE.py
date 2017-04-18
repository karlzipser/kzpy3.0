import threading
from kzpy3.utils import *

state_transition_timer = Timer(0)
n_avg_steer = 25
n_avg_motor = 15
n_avg_button = 15
n_avg_encoder = 15
##############################################################3
#
def setup(messages_dic,buttons=[0,1900,1700,1424,870]):

    M = messages_dic
    M['steer_max'] = buttons[4]
    M['motor_max'] = buttons[4]
    M['steer_min'] = buttons[1]
    M['motor_min'] = buttons[1]
    M['buttons'] = buttons

    for q in ['button_pwm_lst',
        'steer_pwm_lst',
        'motor_pwm_lst',
        'steer_pwm_write_lst',
        'motor_pwm_write_lst',
        'encoder_lst',
        'button_pwm_smooth_lst',
        'steer_pwm_smooth_lst',
        'motor_pwm_smooth_lst',
        'encoder_smooth_lst']:
        M[q] = []

    M['state'] = 2
    M['previous_state'] = 1
    M['steer_null'] = 1400
    M['motor_null'] = 1500


#        
##############################################################3
#

def run_loop(Arduinos,messages_dic,BUTTON_DELTA=50,n_lst_steps=100):
    M = messages_dic
    lock = threading.Lock()
    while M['Stop_Arduinos'] == False:

        if not serial_data_to_messages(Arduinos,M):
            continue

        buttons_to_state(Arduinos,M,BUTTON_DELTA,state_transition_timer)

        manage_list_lengths(M,n_lst_steps)

        smooth_raw_data(M,n_avg_steer,n_avg_motor,n_avg_button,n_avg_encoder,pid=(M['state']==3))
        
        if M['state'] == 4:
            process_state_4(M,n_lst_steps,state_transition_timer)
            continue

        if M['state'] == 3:
            if M['encoder_lst'][-1] > 1:
                M['motor_pwm_smooth_lst'][-1] -= 10*np.abs(M['encoder_lst'][-1]-0.75)/10.0
            elif M['encoder_lst'][-1] < 0.5:
                M['motor_pwm_smooth_lst'][-1] += 10*np.abs(M['encoder_lst'][-1]-0.75)/10.0
            print M['motor_pwm_smooth_lst'][-1],M['encoder_lst'][-1]
            M['external_write_str_B'] = d2n( '(', int(M['steer_pwm_smooth_lst'][-1]), ',', int(M['motor_pwm_smooth_lst'][-1]+10000), ')')
                #print M['external_write_str_B']
            Arduinos['MSE'].write(M['external_write_str_B'])

        M['raw_write_str'] = d2n( '(', int(M['steer_pwm_lst'][-1]), ',', int(M['motor_pwm_lst'][-1]+10000), ')')
        M['smooth_write_str'] = d2n( '(', int(M['steer_pwm_smooth_lst'][-1]), ',', int(M['motor_pwm_smooth_lst'][-1]+10000), ')')

        if M['state'] == 1:
            Arduinos['MSE'].write(M['raw_write_str'])
        elif M['state'] == 2:
            Arduinos['MSE'].write(M['smooth_write_str'])
        



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




def buttons_to_state(Arduinos,M,BUTTON_DELTA,state_transition_timer):
    for s in range(1,5):
        if np.abs(M['button_pwm_lst'][-1]-M['buttons'][s]) < BUTTON_DELTA:
            if M['state'] != s:
                M['previous_state'] = M['state']
                M['state'] = s
                state_transition_timer.reset()
                M['LED_signal'] = d2n('(',M['state']*100+M['state']*10+1001,')')
                Arduinos['SIG'].write(M['LED_signal'])
            return


def manage_list_lengths(M,n_lst_steps):
    for k in M:
        if type(M[k]) == list:        
            if len(M[k]) > 1.2 * n_lst_steps:
                M[k] = M[k][-n_lst_steps:]   



def smooth_raw_data(M,n_avg_steer,n_avg_motor,n_avg_button,n_avg_encoder,pid=False):
    if len(M['steer_pwm_lst']) >= n_avg_steer:
        M['steer_pwm_smooth_lst'].append(array(M['steer_pwm_lst'][-n_avg_steer:]).mean())
        if not pid:
            M['motor_pwm_smooth_lst'].append(array(M['motor_pwm_lst'][-n_avg_motor:]).mean())
        else:
            M['motor_pwm_smooth_lst'].append((array(M['motor_pwm_smooth_lst'][-n_avg_motor:]).mean()))
        M['button_pwm_smooth_lst'].append(array(M['button_pwm_lst'][-n_avg_button:]).mean())
        M['encoder_smooth_lst'].append(array(M['encoder_lst'][-n_avg_encoder:]).mean())
    else:
        M['steer_pwm_smooth_lst'].append(M['steer_pwm_lst'][-1])
        M['motor_pwm_smooth_lst'].append(M['motor_pwm_lst'][-1])
        M['button_pwm_smooth_lst'].append(M['button_pwm_lst'][-1])
        M['encoder_smooth_lst'].append(M['encoder_lst'][-1])



def process_state_4(M,n_lst_steps,state_transition_timer):
        if state_transition_timer.time() < 0.5:
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
                if M['steer_pwm_smooth_lst'][-1] > M['steer_max']:
                    M['steer_max'] = M['steer_pwm_smooth_lst'][-1]
                if M['motor_pwm_smooth_lst'][-1] > M['motor_max']:
                    M['motor_max'] = M['motor_pwm_smooth_lst'][-1]
                if M['steer_pwm_smooth_lst'][-1] < M['steer_min']:
                    M['steer_min'] = M['steer_pwm_smooth_lst'][-1]
                if M['motor_pwm_smooth_lst'][-1] < M['motor_min']:
                    M['motor_min'] = M['motor_pwm_smooth_lst'][-1]
            








#        
##############################################################3
#