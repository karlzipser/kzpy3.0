import threading
from kzpy3.utils import *



class State():
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        self.name = name
        self.number = number
        self.button_pwm_peak = button_pwm_peak
        self.Arduinos = Arduinos
        self.state_transition_timer = None
        self.M = M
    def enter(self):
        print('Entering '+self.name)
        self.state_transition_timer = Timer(0)
    def process(self):
        pass #print(self.name+' processing')
    def leave(self):
        self.state_transition_timer = None


class Run_State(State):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        State.__init__(self,name,number,button_pwm_peak,M,Arduinos)
    def enter(self):
        self.led()
        State.enter(self)
    def led(self):
        if self.number in [3,5,6,7,8]:
            n = 3
        else:
            n = self.number
        #print n
        LED_signal = d2n('(',n*100+n*10+1001,')')
        self.Arduinos['SIG'].write(LED_signal)

class Human_Control(Run_State):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Run_State.__init__(self,name,number,button_pwm_peak,M,Arduinos)
    def process(self):
        self.Arduinos['MSE'].write(self.M['raw_write_str'])

class Smooth_Human_Control(Human_Control):
    def process(self):
        self.Arduinos['MSE'].write(self.M['smooth_write_str'])

class Calibration_State(Run_State):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Run_State.__init__(self,name,number,button_pwm_peak,M,Arduinos)
    def leave(self):
        Run_State.leave(self)
        self.M['steer_pwm_lst'] = [self.M['steer_null']]
        self.M['motor_pwm_lst'] = [self.M['motor_null']]
        self.M['pid_motor_pwm'] = self.M['smooth_motor']


class Computer_Control(Run_State):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Run_State.__init__(self,name,number,button_pwm_peak,M,Arduinos)

class Net_Steer_Net_Motor(Computer_Control):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Computer_Control.__init__(self,name,number,button_pwm_peak,M,Arduinos)

class Net_Steer_Hum_Motor(Computer_Control):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Computer_Control.__init__(self,name,number,button_pwm_peak,M,Arduinos)

class Net_Steer_PID_Motor(Computer_Control):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Computer_Control.__init__(self,name,number,button_pwm_peak,M,Arduinos)
    def process(self):
        if self.M['smooth_motor'] < 5+self.M['pid_motor_pwm'] and  self.M['smooth_motor'] > self.M['motor_null']-5:
            self.M['PID'] = [1,2]
            pid_processing(self.M)
            self.M['pid_write_str'] = d2n( '(', int(self.M['smooth_steer']), ',', int(self.M['pid_motor_pwm']+10000), ')')
            Arduinos['MSE'].write(M['pid_write_str'])

class Hum_Steer_Net_Motor(Computer_Control):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Computer_Control.__init__(self,name,number,button_pwm_peak,M,Arduinos)

class Hum_Steer_Hum_Motor(Computer_Control):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Computer_Control.__init__(self,name,number,button_pwm_peak,M,Arduinos)





def buttons_to_state(Arduinos,M,BUTTON_DELTA):
    for s in [M['state_one'],M['state_two']]:
        if np.abs(M['button_pwm_lst'][-1] - s.button_pwm_peak) < BUTTON_DELTA:
            M['current_state'].name
            if M['current_state'] == s:
                return
            M['previous_state'] = M['current_state']
            M['current_state'] = s
            M['current_state'].enter()
            M['previous_state'].leave()
            return

    if np.abs(M['button_pwm_lst'][-1] - M['state_three'].button_pwm_peak) < BUTTON_DELTA:
        if M['current_state'] in [M['state_three'],M['state_five'],M['state_six'],M['state_seven'],M['state_eight']]:
            return
        M['previous_state'] = M['current_state']
        M['current_state'] = M['state_six']
        M['current_state'].enter()
        M['previous_state'].leave()

        return

    if np.abs(M['button_pwm_lst'][-1] - M['state_four'].button_pwm_peak) < BUTTON_DELTA:
        if M['current_state'] == M['state_four']:
            return
        M['previous_state'] = M['current_state']
        M['current_state'] = M['state_four']
        M['current_state'].enter()
        M['previous_state'].leave()






##############################################################3
#
def setup(M,Arduinos):

    M['state_transition_timer'] = Timer(0)
    M['n_avg_steer'] = 20
    M['n_avg_motor'] = 20
    M['n_avg_button'] = 15
    M['n_avg_encoder'] = 15
    M['steer_null'] = 1400
    M['motor_null'] = 1500
    M['steer_percent'] = 49
    M['motor_percent'] = 49
    M['steer_max'] = M['steer_null']+1
    M['motor_max'] = M['motor_null']+1
    M['steer_min'] = M['motor_null']-1
    M['motor_min'] = M['motor_null']-1
    M['buttons'] = [0,1900,1700,1424,870]
    M['set_null'] = False

    for q in ['button_pwm_lst',
        'steer_pwm_lst',
        'motor_pwm_lst',
        'steer_pwm_write_lst',
        'motor_pwm_write_lst',
        'encoder_lst']:
        M[q] = []

    #M['state'] = 2
    #M['previous_state'] = 1
    M['calibrated'] = False
    M['PID'] = [-1,-1]

    state_one = Human_Control('state 1',1,1900,M,Arduinos)
    state_two = Human_Control('state 2',2,1700,M,Arduinos)
    state_six = Net_Steer_PID_Motor('state 6',6,1424,M,Arduinos)
    state_three = Net_Steer_Hum_Motor('state 3',3,1424,M,Arduinos)
    state_five = Hum_Steer_Net_Motor('state 5',5,1424,M,Arduinos)
    state_seven = Hum_Steer_Hum_Motor('state 7',7,1424,M,Arduinos)
    state_eight = Net_Steer_PID_Motor('state 8',8,1424,M,Arduinos)
    state_four = Calibration_State('state 4',4,870,M,Arduinos)
    M['state_one'] = state_one
    M['state_two'] = state_two
    M['state_three'] = state_three
    M['state_four'] = state_four
    M['state_five'] = state_five
    M['state_six'] = state_six
    M['state_seven'] = state_seven
    M['state_eight'] = state_eight
    M['current_state'] = state_one


calibration_signal_timer = Timer(0.01)
#        
##############################################################3
#

def run_loop(Arduinos,M,BUTTON_DELTA=50,n_lst_steps=30):
    lock = threading.Lock()
    if 'MSE' not in Arduinos:
        M['Stop_Arduinos'] = True
        return

    

    while M['Stop_Arduinos'] == False:
        
        if not serial_data_to_messages(Arduinos,M):
            continue

        buttons_to_state(Arduinos,M,BUTTON_DELTA)

        manage_list_lengths(M,n_lst_steps)

        smooth_data(M)
        
        if M['current_state'] == M['state_four']:
            process_state_4(M,n_lst_steps)
            continue
        else:
            if not calibrated(Arduinos,M):
                continue

        M['steer_percent'] = pwm_to_percent(M,M['steer_null'],M['steer_pwm_lst'][-1],M['steer_max'],M['steer_min'])
        M['motor_percent'] = pwm_to_percent(M,M['motor_null'],M['motor_pwm_lst'][-1],M['motor_max'],M['motor_min'])


        M['raw_write_str'] = d2n( '(', int(M['steer_pwm_lst'][-1]), ',', int(M['motor_pwm_lst'][-1]+10000), ')')
        M['smooth_write_str'] = d2n( '(', int(M['smooth_steer']), ',', int(M['smooth_motor']+10000), ')')
        



        
        if M['current_state'] in [M['state_three'],M['state_five'],M['state_six'],M['state_seven']]:
            human_motor = False
            human_steer = False
            if np.abs(M['steer_percent'] - 49) > 8:
                human_steer = True
            if np.abs(M['motor_percent'] - 49) > 8:
                human_motor = True
            if human_motor and human_steer:
                if M['current_state'] != M['state_five']:
                    M['previous_state'] = M['current_state']
                    M['current_state'] = M['state_five']
                    M['current_state'].enter()
                    M['previous_state'].leave()
            elif human_motor and (not human_steer):
                if M['current_state'] != M['state_three']:
                    M['previous_state'] = M['current_state']
                    M['current_state'] = M['state_three']
                    M['current_state'].enter()
                    M['previous_state'].leave()
            elif (not human_motor) and human_steer:
                if M['current_state'] != M['state_seven']:
                    M['previous_state'] = M['current_state']
                    M['current_state'] = M['state_seven']
                    M['current_state'].enter()
                    M['previous_state'].leave()
            elif (not human_steer) and (not human_motor):
                if M['current_state'] != M['state_six']:
                    M['previous_state'] = M['current_state']
                    M['current_state'] = M['state_six']
                    M['current_state'].enter()
                    M['previous_state'].leave()

        M['current_state'].process()

        """


            #else:
            #M['PID'] = [-1,-1]
            #print((M['current_state'].number,M['steer_percent'],M['motor_percent'],M['state_transition_timer'].time()))
            Arduinos['MSE'].write(M['smooth_write_str'])
        """

#        
##############################################################3
#


def calibrated(Arduinos,M):
    if not M['calibrated']:
        if calibration_signal_timer.check():
            s = np.random.choice([1,2,3])
            LED_signal = d2n('(',s*100+s*10+1001,')')
            Arduinos['SIG'].write(LED_signal)
            calibration_signal_timer.reset()
        return False
    return True

def smooth_data(M):
    if len(M['steer_pwm_lst']) >= M['n_avg_steer']:
        M['smooth_steer'] = array(M['steer_pwm_lst'][-M['n_avg_steer']:]).mean()
    else:
        M['smooth_steer'] = M['steer_null']

    if len(M['motor_pwm_lst']) >= M['n_avg_motor']:  
        M['smooth_motor'] = array(M['motor_pwm_lst'][-M['n_avg_motor']:]).mean()
    else:
        M['smooth_motor'] = M['motor_null']




def pid_processing(M):
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


def serial_data_to_messages(Arduinos,M):
    read_str = Arduinos['MSE'].readline()
    try:
        exec('mse_input = list({0})'.format(read_str))
    except:
        return False
    if len(mse_input) == 5 and mse_input[0] == 'mse':
        M['button_pwm_lst'].append(mse_input[1])
        M['steer_pwm_lst'].append(mse_input[2])
        M['motor_pwm_lst'].append(mse_input[3])
        #M['steer_pwm_write_lst'].append(mse_input[4])
        #M['motor_pwm_write_lst'].append(mse_input[5])
        M['encoder_lst'].append(mse_input[4])#6])
        return True
    else:
        return False



def manage_list_lengths(M,n_lst_steps):
    for k in M:
        if type(M[k]) == list:        
            if len(M[k]) > 1.2 * n_lst_steps:
                M[k] = M[k][-n_lst_steps:]   







def process_state_4(M,n_lst_steps):
    M['calibrated'] = False
    if M['current_state'].state_transition_timer.time() < 0.5:
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