'''
Created on Apr 11, 2017

@author: Sascha Hornauer
'''
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np

from Video_Marker import Video_Marker
from Bagfile_Handler import Bagfile_Handler
from Area_Visualizer import Area_Visualizer
from Map import Map

class Marker_Handler:
    
    detected_markers = {}
    area_visualizer = None
    source_local_camera = True
    show_video = True
    
    def __init__(self, arguments):
        
        
        bagfile_handler = Bagfile_Handler(arguments)
        #capture_device = cv2.VideoCapture(1)
        #image_marker = Video_Marker(None,capture_device) 
        image_marker = Video_Marker(bagfile_handler,None)
        self.area_visualizer = Area_Visualizer()
                    
        self.play_video(bagfile_handler,None,image_marker)
        #self.play_video(None,capture_device,image_marker)

    

    def play_video(self,bagfile_handler,capture_device,image_marker):
          
        paused_video = False
        
        while True:
            if not paused_video:
                if(not bagfile_handler == None and capture_device == None):
                    image = bagfile_handler.get_image()
                elif(not capture_device == None):
                    ret, image = capture_device.read()
                   
                
                cv_image, markers = image_marker.process_next_image(True,image) 
 
                
                
            if(self.show_video):
                cv2.imshow('frame',cv_image)
                key = cv2.waitKey(1000/30) & 0xFF
                if key == ord('q'):
                    break
                if key == ord(' '):
                    paused_video = not paused_video
                if key == ord('w'):
                    bagfile_handler.fast_forward()
                if not paused_video:
                    self.area_visualizer.visualize_markers_center_line(markers)
            

if len(sys.argv) < 2:
    print("Please provide the path to the bagfile")
else:
    print("Processing bag files")
    Marker_Handler(sys.argv[1])
