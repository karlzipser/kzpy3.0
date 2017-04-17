#import os, serial, threading, Queue
#import threading

from kzpy3.utils import *

def run_loop(Arduinos,messages_dic):

    while messages_dic['Stop_Arduinos'] == False:

        try:        
            read_str = Arduinos['IMU'].readline()
            #print read_str
            exec('imu_input = list({0})'.format(read_str))
            #print imu_input
            #print len(mse_input)
            if imu_input[0] in ['gyro','acc','head']:
                messages_dic[imu_input[0]] = imu_input[1:4]
            else:
                print '***'+read_str + "*** is not imu"
                continue

        except Exception as e:
            pass #print e
        






