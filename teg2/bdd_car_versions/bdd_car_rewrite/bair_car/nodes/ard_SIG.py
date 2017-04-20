#import os, serial, threading, Queue
#import threading

from kzpy3.utils import *

"""
sudo chmod 666 /dev/ttyACM*

"""

def setup(M,Arduinos):
    LED_signal = d2n('(',3,')')
    Arduinos['SIG'].write(LED_signal)
    pass


def run_loop(Arduinos,M):

    while M['Stop_Arduinos'] == False or not rospy.is_shutdown():
        #M['Stop_Arduinos'] = True

        try:  

            read_str = Arduinos['SIG'].readline()
            #print read_str

            try:
                exec('sig_input = list({0})'.format(read_str))
                #print sig_input
            except:
                continue
            if sig_input[0] in ['GPS2']:
                M[sig_input[0]] = sig_input[1:] # This is just a placeholder for now.
            else:
                print '***'+read_str + "*** is not sig"
                continue

        except Exception as e:
            pass #print e
        






