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

class Marker_Handler:
    
    detected_markers = {}
    area_visualizer = None
    
    def __init__(self, arguments):
        
        bagfile_handler = Bagfile_Handler(arguments)
        image_marker = Video_Marker() 
        self.area_visualizer = Area_Visualizer()

        # create persistent aruco marker dict                       
        self.play_video(bagfile_handler,image_marker)

    

    def play_video(self,bagfile_handler,image_marker):
          
        paused_video = False
        
        while True:
            if not paused_video:
                timestamp, image = bagfile_handler.get_image()
                gray,markers = image_marker.process_next_image(timestamp, image) 
            
            cv2.imshow('frame',gray)
            key = cv2.waitKey(1000/60) & 0xFF
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