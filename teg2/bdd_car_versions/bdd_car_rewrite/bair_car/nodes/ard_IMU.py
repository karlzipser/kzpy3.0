
from kzpy3.utils import *
import geometry_msgs.msg

imu_dic = {}
imu_dic['gyro'] = 'gyro_pub'
imu_dic['acc'] = 'acc_pub'
imu_dic['head'] = 'gyro_heading_pub'

def run_loop(Arduinos,M):

    while M['Stop_Arduinos'] == False:

        try:        
            read_str = Arduinos['IMU'].readline()
            #print read_str
            exec('imu_input = list({0})'.format(read_str))
            #print imu_input
            #print len(mse_input)
            if imu_input[0] in ['gyro','acc','head']:
                M[imu_input[0]] = imu_input[1:4]
                M[imu_dic[imu_input[0]]].publish(geometry_msgs.msg.Vector3(*M[imu_input[0]]))
            else:
                print '***'+read_str + "*** is not imu"
                continue

        except Exception as e:
            pass #print e
        






