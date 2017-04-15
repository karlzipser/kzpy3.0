'''
Created on Apr 11, 2017

@author: picard
'''
import rosbag
from cv_bridge import CvBridge
import sys

class Bagfile_Handler(object):
    
    bag = None
    bag_access = None
    bridge = CvBridge()
    
    def __init__(self, bag_filepath):
        self.bag = rosbag.Bag(bag_filepath)
        self.bag_access = self.bag.read_messages(topics=['/bair_car/zed/left/image_rect_color']).__iter__()
        
    def __del__(self):
        self.bag.close()

    def get_image(self):
        topic, msg, t = self.bag_access.next() 
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        return t, cv_image

    
    def fast_forward(self):
        for i in range(0,60):
            self.bag_access.next()