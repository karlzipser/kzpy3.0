'''
Created on Apr 11, 2017

@author: picard
'''
import rosbag
from cv_bridge import CvBridge
import sys
import pickle
import os

class Bagfile_Handler(object):
    
    print("Reading Bagfilear")
    bag = None
    bag_access = None
    bridge = CvBridge()
    timestamp = 0
    
    evasion_file = None
    evasion_data = []
    old_evasion_data = []
    
    def __init__(self, bag_filepath):
        self.bag = rosbag.Bag(bag_filepath)
        head,tail = os.path.split(bag_filepath)
        self.evasion_file = open("evasion_" + tail +'.pkl', 'wb')
        self.bag_access = self.bag.read_messages(topics=['/bair_car/zed/left/image_rect_color']).__iter__()
        
    def __del__(self):
        self.bag.close()
        self.evasion_file.close()

    def get_image(self):
        try:
            topic, msg, t = self.bag_access.next()
            self.timestamp = self.bag_access.next().timestamp.to_time()
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            pickle.dump(self.evasion_data, self.evasion_file, pickle.HIGHEST_PROTOCOL) 
            self.evasion_file.close()
            self.bag.close()
            sys.exit(0)        
        return cv_image

    
    def fast_forward(self):
        try:
            for i in range(0,60):
                self.bag_access.next()
        except:
            pickle.dump(self.evasion_data, self.evasion_file, pickle.HIGHEST_PROTOCOL)
            self.evasion_file.close()
            self.bag.close()
            sys.exit(0)  