'''
Created on Apr 11, 2017

@author: Sascha Hornauer
'''
import sys
import cv2


from Video_Marker import Video_Marker
from Bagfile_Handler import Bagfile_Handler
from Area_Visualizer import Area_Visualizer

class Marker_Handler:
    
    detected_markers = {}
    
    def __init__(self, arguments):
        
        bagfile_handler = Bagfile_Handler(arguments)
        image_marker = Video_Marker() 
        area_visualizer = Area_Visualizer(bagfile_handler,image_marker)

        # create persistent aruco marker dict       
                
        self.play_video(bagfile_handler,image_marker)

          

    def play_video(self,bagfile_handler,image_marker):
          
        paused_video = False
        while True:
            if not paused_video:
                gray,_ = image_marker.get_next_image(bagfile_handler.get_image()) 
            
            cv2.imshow('frame',gray)
            key = cv2.waitKey(1000/30) & 0xFF
            if key == ord('q'):
                break
            if key == ord(' '):
                paused_video = not paused_video


if len(sys.argv) < 2:
    print("Please provide the path to the bagfile")
else:
    print("Processing bag files")
    Marker_Handler(sys.argv[1])