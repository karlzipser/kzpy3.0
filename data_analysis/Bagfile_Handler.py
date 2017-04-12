'''
Created on Apr 11, 2017

@author: picard
'''
import rosbag
from cv_bridge import CvBridge

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
        return cv_image

#test = Bagfile_Handler("/home/picard/2ndDisk/caffe2_z2_color_direct_local_09Apr17_16h40m09s_Mr_Black/bair_car_2017-04-09-16-43-05_6.bag")

#cv2.imshow('frame',test.get_image())
#cv2.waitKey(6000)