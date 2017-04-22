'''
Created on Apr 11, 2017

@author: Sascha Hornauer
'''
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
import aruco_annotator

from Video_Marker import Video_Marker
from Bagfile_Handler import Bagfile_Handler
from Area_Visualizer import Area_Visualizer
from Map import Map
from aruco_annotator import get_aruco_image

class Marker_Handler:
    
    detected_markers = {}
    area_visualizer = None # The choice if there is a visualizer or not is not yet implemented
    source_local_camera = True
    source_bagfile = False
    show_video = True
    crop = True # Should the input video be cropped to the left image input. If false this code might contain errors
    
    
    def __init__(self, arguments):
        
        if(self.source_bagfile):
        
            bagfile_handler = Bagfile_Handler(arguments)
            image_marker = Video_Marker(bagfile_handler,None) 
            self.area_visualizer = Area_Visualizer()
            self.play_video(bagfile_handler,None,image_marker)
            
        elif (self.source_local_camera):
            
            capture_device = cv2.VideoCapture(2)
            if(capture_device == None):
                print("Camera not found")
                sys.exit(1)
            image_marker = Video_Marker(None,capture_device) 
            self.area_visualizer = Area_Visualizer()
            self.play_video(None,capture_device,image_marker)

    

    def play_video(self,bagfile_handler,capture_device,image_marker):
          
        paused_video = False
        
        while True:
            if not paused_video:
                if(not bagfile_handler == None and capture_device == None):
                    image = bagfile_handler.get_image()
                elif(not capture_device == None):
                    ret, image = capture_device.read()
                   
                if image is None:
                    print("Error reading image! Wrong number of camera?")
                
                #DEBUG cv_image, markers, motor_cmd, steer_cmd, evasion_needed = image_marker.process_next_image(self.crop,None,image) 
                cv_image = get_aruco_image(image,True,True)
                
                
            if(self.show_video):
                cv2.imshow('frame',cv_image)
                key = cv2.waitKey(1000/30) & 0xFF
                if key == ord('q'):
                    break
                if key == ord(' '):
                    paused_video = not paused_video
                if key == ord('w'):
                    bagfile_handler.fast_forward()
                #if not paused_video:
                # DEBUG    self.area_visualizer.visualize_markers_center_line(markers)
            

Marker_Handler(sys.argv)
