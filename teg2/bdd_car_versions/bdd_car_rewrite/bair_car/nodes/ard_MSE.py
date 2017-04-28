import threading
from kzpy3.utils import *
import std_msgs.msg
import rospy
import kzpy3.teg2.bdd_car_versions.bdd_car_rewrite.runtime_params
from kzpy3.teg2.bdd_car_versions.bdd_car_rewrite.runtime_params import *

lock = threading.Lock()

os.environ['STOP'] = 'False'

def apply_steer_pwm_gain(steer_pwm,M):
    return (steer_pwm-M['steer_null']) * M['steer_gain'] + M['steer_null']


def apply_motor_pwm_gain(motor_pwm,M):
    return (motor_pwm-M['motor_null']) * M['motor_gain'] + M['motor_null']

def mse_write_publish(M,Arduinos,steer_pwm,motor_pwm):
    steer_pwm = apply_steer_pwm_gain(steer_pwm,M)
    motor_pwm = apply_motor_pwm_gain(motor_pwm,M)
    write_str = d2n( '(', int(steer_pwm), ',', int(motor_pwm+10000), ')')
    Arduinos['MSE'].write(write_str)
    steer_percent = pwm_to_percent(M,M['steer_null'],steer_pwm,M['steer_max'],M['steer_min'])
    motor_percent = pwm_to_percent(M,M['motor_null'],motor_pwm,M['motor_max'],M['motor_min'])
    M['steer_pub'].publish(std_msgs.msg.Int32(steer_percent))
    M['motor_pub'].publish(std_msgs.msg.Int32(motor_percent))
    M['state_pub'].publish(std_msgs.msg.Int32(M['current_state'].number))

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
        #if self.number in [3,5,6,7,8]:
        #    n = 3
        #else:
        #    n = self.number
        #print n
        #LED_signal = d2n('(',n*100+n*10+1001,')')
        LED_signal = d2n('(',self.M['data_saving']*10000+self.number,')')
        self.Arduinos['SIG'].write(LED_signal)


class Human_Control(Run_State):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Run_State.__init__(self,name,number,button_pwm_peak,M,Arduinos)
    def process(self):
        mse_write_publish(self.M,self.Arduinos,self.M['steer_pwm_lst'][-1],self.M['motor_pwm_lst'][-1])


class Smooth_Human_Control(Human_Control):
    def process(self):
        mse_write_publish(self.M,self.Arduinos,self.M['smooth_steer'],self.M['smooth_motor'])



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
    def enter(self):
        Run_State.enter(self)


class PID_Motor(Computer_Control):
    def __init__(self,name,number,button_pwm_peak,M,Arduinos):
        Computer_Control.__init__(self,name,number,button_pwm_peak,M,Arduinos)
    def enter(self):
        Computer_Control.enter(self)
        if self.M['previous_state'] == None:
            self.M['pid_motor_pwm'] = self.M['smooth_motor']
        elif self.M['previous_state'].button_pwm_peak != self.button_pwm_peak: 
            self.M['pid_motor_pwm'] = self.M['smooth_motor']
    def process(self):
            if self.M['smooth_motor'] < 5+self.M['pid_motor_pwm'] and  self.M['smooth_motor'] > self.M['motor_null']-5:
                self.M['PID'] = self.M['PID_min_max']
                pid_processing(self.M)
                mse_write_publish(self.M,self.Arduinos,self.M['smooth_steer'],self.M['pid_motor_pwm'])
            else:
                mse_write_publish(self.M,self.Arduinos,self.M['smooth_steer'],self.M['smooth_motor'])


class Freeze(Run_State):
    def process(self):
        mse_write_publish(self.M,self.Arduinos,self.M['steer_null'],self.M['motor_null'])



class Hum_Steer_PID_Motor(PID_Motor):
    def process(self):
        self.M['PID'] = self.M['PID_min_max']
        pid_processing(self.M)
        mse_write_publish(self.M,self.Arduinos,self.M['smooth_steer'],self.M['pid_motor_pwm'])

class Net_Steer_PID_Motor(PID_Motor):
    def process(self):
        self.M['PID'] = self.M['PID_min_max']
        pid_processing(self.M)
        self.M['caffe_steer_pwm'] = percent_to_pwm(self.M['caffe_steer'],self.M['steer_null'],self.M['steer_max'],self.M['steer_min'])
        mse_write_publish(self.M,self.Arduinos,self.M['caffe_steer_pwm'],self.M['pid_motor_pwm'])

class Net_Steer_Hum_Motor(PID_Motor):
    def process(self):
        self.M['caffe_steer_pwm'] = percent_to_pwm(self.M['caffe_steer'],self.M['steer_null'],self.M['steer_max'],self.M['steer_min'])
        mse_write_publish(self.M,self.Arduinos,self.M['caffe_steer_pwm'],self.M['smooth_motor'])

class Aruco_Steer_Aruco_Motor(Computer_Control):
    def process(self):
        self.M['caffe_steer_pwm'] = percent_to_pwm(self.M['caffe_steer'],self.M['steer_null'],self.M['steer_max'],self.M['steer_min'])
        self.M['caffe_motor_pwm'] = percent_to_pwm(self.M['caffe_motor'],self.M['motor_null'],self.M['motor_max'],self.M['motor_min'])
        mse_write_publish(self.M,self.Arduinos,self.M['caffe_steer_pwm'],self.M['caffe_motor_pwm'])








def buttons_to_state(Arduinos,M,BUTTON_DELTA):

    if np.abs(M['button_pwm_lst'][-1] - M['state_four'].button_pwm_peak) < BUTTON_DELTA:
        if M['current_state'] == None:
            M['current_state'] = M['state_four']
            M['current_state'].enter()
            return
        if M['current_state'] == M['state_four']:
            return
        M['previous_state'] = M['current_state']
        M['current_state'] = M['state_four']
        M['current_state'].enter()
        M['previous_state'].leave()

    if M['current_state'] == None:
        return

    for s in [M['state_one'],M['state_two']]:
        if np.abs(M['button_pwm_lst'][-1] - s.button_pwm_peak) < BUTTON_DELTA:  
            if M['current_state'] == s:
                return
            M['previous_state'] = M['current_state']
            M['current_state'] = s
            M['current_state'].enter()
            M['previous_state'].leave()
            return



    if np.abs(M['button_pwm_lst'][-1] - M['state_three'].button_pwm_peak) < BUTTON_DELTA:
        if M['aruco_evasion_active'] == 1 and M['current_state'] != M['state_ten']:
           #print "HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            M['previous_state'] == M['current_state']
            M['current_state'] = M['state_ten']
            M['current_state'].enter()
            M['previous_state'].leave()
            return
        if M['current_state'] in [M['state_three'],M['state_five'],M['state_six'],M['state_seven'],M['state_eight'],M['state_nine']]:
            return
        M['previous_state'] = M['current_state']
        M['current_state'] = M['state_six']
        M['current_state'].enter()
        M['previous_state'].leave()
        return





##############################################################3
#
def setup(M,Arduinos):

    #M['state_transition_timer'] = Timer(0)
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
    M['aruco_evasion_active'] = 0
    #M['buttons'] = [0,1900,1700,1424,870]
    M['set_null'] = False
    #M['steer_gain'] = 1.0
    #M['motor_gain'] = 1.0

    for q in ['button_pwm_lst',
        'steer_pwm_lst',
        'motor_pwm_lst',
        'steer_pwm_write_lst',
        'motor_pwm_write_lst',
        'encoder_lst']:
        M[q] = []

    #M['state'] = 2
    M['previous_state'] = None
    M['calibrated'] = False
    M['PID'] = [-1,-1]

    state_one = Human_Control('state 1',1,1700,M,Arduinos)
    state_two = Smooth_Human_Control('state 2',2,1424,M,Arduinos)
    state_six = Net_Steer_PID_Motor('state 6',6,1900,M,Arduinos)
    state_three = Net_Steer_Hum_Motor('state 3',3,1900,M,Arduinos)
    state_five = Human_Control('state 5',5,1900,M,Arduinos)
    state_seven = Hum_Steer_PID_Motor('state 7',7,1900,M,Arduinos)
    state_eight = Net_Steer_PID_Motor('state 8',8,1900,M,Arduinos)
    state_nine = Freeze('state 9',9,1900,M,Arduinos)
    state_four = Calibration_State('state 4',4,870,M,Arduinos)
    state_ten = Aruco_Steer_Aruco_Motor('state 10',10,1900,M,Arduinos)
    """
    state_one = Human_Control('state 1',1,1900,M,Arduinos)
    state_two = Smooth_Human_Control('state 2',2,1700,M,Arduinos)
    state_six = Net_Steer_PID_Motor('state 6',6,1424,M,Arduinos)
    state_three = Net_Steer_Hum_Motor('state 3',3,1424,M,Arduinos)
    state_five = Human_Control('state 5',5,1424,M,Arduinos)
    state_seven = Hum_Steer_PID_Motor('state 7',7,1424,M,Arduinos)
    state_eight = Net_Steer_PID_Motor('state 8',8,1424,M,Arduinos)
    state_nine = Freeze('state 9',9,1424,M,Arduinos)
    state_four = Calibration_State('state 4',4,870,M,Arduinos)
    """

    M['state_one'] = state_one
    M['state_two'] = state_two
    M['state_three'] = state_three
    M['state_four'] = state_four
    M['state_five'] = state_five
    M['state_six'] = state_six
    M['state_seven'] = state_seven
    M['state_eight'] = state_eight
    M['state_nine'] = state_nine
    M['state_ten'] = state_ten

    M['current_state'] = None
    M['caffe_steer'] = 49
    M['caffe_steer_pwm'] = M['steer_null']
    M['n_lst_steps'] = 30
    M['steer_gain'] = steer_gain
    M['motor_gain'] = motor_gain
    M['acc2rd_threshold'] = acc2rd_threshold
    M['data_saving'] = 0
    print("MSE setup")

calibration_signal_timer = Timer(0.01)
#        
##############################################################3
#

def run_loop(Arduinos,M,BUTTON_DELTA=50,):


    lock = threading.Lock()
    if 'MSE' not in Arduinos:
        M['Stop_Arduinos'] = True
        return

    
    try:
        if os.environ['STOP'] == 'True':
            assert(False)
        while M['Stop_Arduinos'] == False or not rospy.is_shutdown():
            
            if not serial_data_to_messages(Arduinos,M):
                continue

            buttons_to_state(Arduinos,M,BUTTON_DELTA)

            if M['current_state'] == None:
                continue

            manage_list_lengths(M)

            smooth_data(M)
            
            if M['current_state'] == M['state_four']:
                #M['aruco_evasion_active'] = 0
                process_state_4(M)
                continue
            else:
                if not calibrated(Arduinos,M):
                    continue

            M['steer_percent'] = pwm_to_percent(M,M['steer_null'],M['steer_pwm_lst'][-1],M['steer_max'],M['steer_min'])
            M['motor_percent'] = pwm_to_percent(M,M['motor_null'],M['motor_pwm_lst'][-1],M['motor_max'],M['motor_min'])

            M['raw_write_str'] = d2n( '(', int(M['steer_pwm_lst'][-1]), ',', int(M['motor_pwm_lst'][-1]+10000), ')')
            M['smooth_write_str'] = d2n( '(', int(M['smooth_steer']), ',', int(M['smooth_motor']+10000), ')')


                            
            freeze = False
            if M['current_state'] in [M['state_three'],M['state_five'],M['state_six'],M['state_seven']]:
                print(M['motor_percent'],M['motor_freeze_threshold'],np.array(encoder_list[0:5]).mean(),np.array(encoder_list[-5:]).mean(),M['current_state'].state_transition_timer.time())

                if M['motor_percent'] > M['motor_freeze_threshold'] and np.array(encoder_list[0:5]).mean() > 1 and np.array(encoder_list[-5:]).mean()<0.1 and M['current_state'].state_transition_timer.time() > 1:
                    print("if M['motor_percent'] > M['motor_freeze_threshold']...")
                    freeze = True
                if 'acc' in M:
                    M['acc_lst_mean'] = array(M['acc_lst'][-10:]).mean(axis=0)
                    acc2rd = M['acc'][0]**2+M['acc'][2]**2
                    if acc2rd > M['acc2rd_threshold']:
                        print("if acc2rd > M['acc2rd_threshold']:")
                        freeze = True
                    if np.abs(M['acc_lst_mean'][0]) > M['acc_freeze_threshold_x']:
                        print("if M['acc_lst_mean'][0] > M['acc_freeze_threshold_x']:")
                        freeze = True
                    if M['acc_lst_mean'][1] > M['acc_freeze_threshold_y_max']:
                        print("if M['acc_lst_mean'][1] > M['acc_freeze_threshold_y_max']:")
                        freeze = True
                    if M['acc_lst_mean'][1] < M['acc_freeze_threshold_y_min']:
                        print("if M['acc_lst_mean'][1] > M['acc_freeze_threshold_y_min']:")
                        freeze = True
                    if np.abs(M['acc_lst_mean'][2]) > M['acc_freeze_threshold_z']:
                        print("if M['acc_lst_mean'][2] > M['acc_freeze_threshold_z']:")
                        freeze = True
                else:
                    print 'acc not in M'
                if freeze:
                    M['previous_state'] = M['current_state']
                    M['current_state'] = M['state_nine']
                    M['current_state'].enter()
                    M['previous_state'].leave()

            if False:
                if caf_motor > motor_freeze_threshold and np.array(encoder_list[0:3]).mean() > 1 and np.array(encoder_list[-3:]).mean()<0.2 and state_transition_time_s > 1:
                    freeze = True
            

                

            if M['current_state'] == M['state_nine']:
                #M['aruco_evasion_active'] = 0
                pass

            elif M['current_state'] in [M['state_three'],M['state_five'],M['state_six'],M['state_seven']]:
                #M['aruco_evasion_active'] = 0
                human_motor = False
                human_steer = False
                if np.abs(M['steer_percent'] - 49) > 5:
                    human_steer = True
                if np.abs(M['motor_percent'] - 49) > 5:
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

            M['state_pub'].publish(std_msgs.msg.Int32(M['current_state'].number))

            """


                #else:
                #M['PID'] = [-1,-1]
                #print((M['current_state'].number,M['steer_percent'],M['motor_percent'],M['state_transition_timer'].time()))
                Arduinos['MSE'].write(M['smooth_write_str'])
            """
    except Exception as e:
        print("********** Exception ***********************")
        print(e.message, e.args)
        os.environ['STOP'] = 'True'
        M['Stop_Arduinos'] = True
        rospy.signal_shutdown(d2s(e.message,e.args))
        LED_signal = d2n('(10000)')
        Arduinos['SIG'].write(LED_signal)

#        
##############################################################3
#


def calibrated(Arduinos,M):
    if not M['calibrated']:
        if calibration_signal_timer.check():
            s = np.random.choice([1,2,3,5,6,7,9])
            LED_signal = d2n('(',M['data_saving']*10000+s,')')
            Arduinos['SIG'].write(LED_signal)
            calibration_signal_timer.reset()
        return False
    return True

def smooth_data(M):
    if len(M['steer_pwm_lst']) >= M['n_avg_steer']:
        M['smooth_steer'] = array(M['steer_pwm_lst'][-M['n_avg_steer']:]).mean()
    else:
        M['smooth_steer'] = M['steer_pwm_lst'][-1]

    if len(M['motor_pwm_lst']) >= M['n_avg_motor']:  
        M['smooth_motor'] = array(M['motor_pwm_lst'][-M['n_avg_motor']:]).mean()
    else:
        M['smooth_motor'] = M['motor_pwm_lst'][-1]

    if len(M['encoder_lst']) >= M['n_avg_encoder']:  
        M['smooth_encoder'] = array(M['encoder_lst'][-M['n_avg_encoder']:]).mean()
    else:
        M['smooth_encoder'] = M['encoder_lst'][-1]




def pid_processing(M):
    if M['PID'][0] < 0:
        M['pid_motor_pwm'] = M['smooth_motor']
    else:
        if M['current_state'].state_transition_timer.time() > 0.0:
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
        lock.acquire()
        M['button_pwm_lst'].append(mse_input[1])
        M['steer_pwm_lst'].append(mse_input[2])
        M['motor_pwm_lst'].append(mse_input[3])
        #M['steer_pwm_write_lst'].append(mse_input[4])
        #M['motor_pwm_write_lst'].append(mse_input[5])
        M['encoder_lst'].append(mse_input[4])#6])
        M['encoder_pub'].publish(std_msgs.msg.Float32(M['encoder_lst'][-1]))
        lock.release()
        return True
    else:
        return False

    
     

def manage_list_lengths(M):
    lock.acquire()
    for k in M:
        if type(M[k]) == list:        
            if len(M[k]) > 1.2 * M['n_lst_steps']:
                M[k] = M[k][-M['n_lst_steps']:]
    lock.release() 



def process_state_4(M):
    M['calibrated'] = False
    if M['current_state'].state_transition_timer.time() < 0.5:
        M['set_null'] = False
    else:
        if M['set_null'] == False:
            M['steer_null'] = array(M['steer_pwm_lst'][-M['n_lst_steps']:]).mean()
            M['motor_null'] = array(M['motor_pwm_lst'][-M['n_lst_steps']:]).mean()
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
    #if p > 110 or p < -10:
    #    M['calibrated'] = False
    #    return 49
    if p > 99:
        p = 99
    if p < 0:
        p = 0      
    return p


def percent_to_pwm(percent,null_pwm,max_pwm,min_pwm):
    if percent >= 49:
        p = (percent-50)/50.0 * (max_pwm-null_pwm) + null_pwm
    else:
        p = (percent-50)/50.0 * (null_pwm-min_pwm) + null_pwm
    return p







#        
##############################################################3
#