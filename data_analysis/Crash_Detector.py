'''
Created on Apr 11, 2017

@author: Sascha Hornauer
'''
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
import aruco_tools.aruco_annotator
from aruco_tools.Video_Marker import Video_Marker
from Bagfile_Handler import Bagfile_Handler
from aruco_tools.aruco_angle_retriever import get_boundary_angle_min_distance

class Crash_Detector:
    
    detected_markers = {}
    area_visualizer = None # The choice if there is a visualizer or not is not yet implemented
    source_local_camera = False
    source_bagfile = True
    show_video = True
    crop = False # Should the input video be cropped to the left image input. If false this code might contain errors
    distance = None
    
    def __init__(self, arguments):
        
        bagfile_path = arguments[1]
        self.distance = arguments[2]
        show_video = arguments[3]
        if(show_video == "0"):
            self.show_video = False
        
        self.crop = False    
        bagfile_handler = Bagfile_Handler(bagfile_path)
        self.play_video(bagfile_handler, None)
        
    def detect_crash(self,cv_image,distance, bagfile_handler):
        averageAngle, min_distance, markers = get_boundary_angle_min_distance(cv_image, False, distance)
        if(min_distance < distance):
            bagfile_handler.data_for_pickle_file.append(bagfile_handler.timestamp)
            cv2.putText(cv_image, "BAAMM!", (300, 300), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 4)
            
        

    def play_video(self,bagfile_handler,capture_device):
          
        paused_video = False
        
        while True:
            if not paused_video:
                if(not bagfile_handler == None and capture_device == None):
                    cv_image = bagfile_handler.get_image()
                elif(not capture_device == None):
                    ret, cv_image = capture_device.read()
                   
                if cv_image is None:
                    print("Error reading cv_image! Wrong number of camera?")
                else:
                    self.detect_crash(cv_image, float(self.distance),bagfile_handler)
                            
            if(self.show_video):
                cv2.imshow('frame',cv_image)
                key = cv2.waitKey(1000/30) & 0xFF
                if key == ord('q'):
                    break
                if key == ord(' '):
                    paused_video = not paused_video
                if key == ord('w'):
                    bagfile_handler.fast_forward()
                

Crash_Detector(sys.argv)
