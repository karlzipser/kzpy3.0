import threading
import rospy
from kzpy3.utils import *
import geometry_msgs.msg

imu_dic = {}
imu_dic['gyro'] = 'gyro_pub'
imu_dic['acc'] = 'acc_pub'
imu_dic['head'] = 'gyro_heading_pub'

lock = threading.Lock()

def setup(M,Arduinos):
    for m in ['gyro','acc','head']:
        for d in ['_x_lst','_y_lst','_z_lst']:
            M[m+d] = []

    M['n_avg_IMU'] = 10
    M['acc_lst'] = []





def run_loop(Arduinos,M):

    try:
        if os.environ['STOP'] == 'True':
            assert(False)
        while M['Stop_Arduinos'] == False or not rospy.is_shutdown():

            try:        
                read_str = Arduinos['IMU'].readline()
                #print read_str
                exec('imu_input = list({0})'.format(read_str))
                #print imu_input
                m = imu_input[0]
                M[m] = imu_input[1:4]
                if m == 'acc':
                    #print("if m == 'acc':")
                    M['acc_lst'].append(M['acc'])
                    if len(M['acc_lst']) > 1.5*M['n_avg_IMU']:
                        M['acc_lst'] = M['acc_lst'][-M['n_avg_IMU']:]
                M[imu_dic[m]].publish(geometry_msgs.msg.Vector3(*M[m]))
                """
                if m == 'acc':
                    ctr = 1
                    for n in ['acc_x_lst','acc_y_lst','acc_z_lst']:#,['acc_x_smooth','acc_y_smooth','acc_z_smooth']:
                        lock.aquire()
                        M[n].append(imu_input[ctr])
                        lock.release()
                        ctr += 1
                elif m == 'gyro':
                    ctr = 1
                    for n in ['gyro_x_lst','gyro_y_lst','gyro_z_lst']:
                        lock.aquire()
                        M[n].append(imu_input[ctr])
                        lock.release()
                        ctr += 1
                elif m == 'head':
                    ctr = 1
                    for n in ['head_x_lst','head_y_lst','head_z_lst']:
                        lock.aquire()
                        M[n].append(imu_input[ctr])
                        lock.release()
                        ctr += 1
                else:
                    print '***'+read_str + "*** is not imu"
                    continue
                
                if m == 'acc':
                    A = ['acc_x_lst','acc_y_lst','acc_z_lst']
                    B = ['acc_x_smooth','acc_y_smooth','acc_z_smooth']
                elif m == 'gyro':
                    A = ['gyro_x_lst','gyro_y_lst','gyro_z_lst']
                    B = ['gyro_x_smooth','gyro_y_smooth','gyro_z_smooth']
                elif m == 'head':
                    A = ['head_x_lst','head_y_lst','head_z_lst']
                    B = ['head_x_smooth','head_y_smooth','head_z_smooth']

                for n,o in zip([A,B]):
                    if len(M[n]) >= M['n_avg_IMU']:
                        M[o] = np.array(M[n][-M['n_avg_IMU']:]).mean()
                        print(d2n('************* M[',o,'] = ',M[o]))
                    else:
                        M[o] = M[n][-1]
                """

            except Exception as e:
                pass #print e
    except Exception as e:
        print("********** Exception ***********************")
        print(e.message, e.args)
        os.environ['STOP'] = 'True'
        LED_signal = d2n('(10000)')
        M['Stop_Arduinos'] = True
        Arduinos['SIG'].write(LED_signal)
        rospy.signal_shutdown(d2s(e.message,e.args))   






