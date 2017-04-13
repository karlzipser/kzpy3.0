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
    
    def __init__(self, arguments):
        
        bagfile_handler = Bagfile_Handler(arguments)
        image_marker = Video_Marker() 
        area_visualizer = Area_Visualizer(bagfile_handler,image_marker)

        # create persistent aruco marker dict                       
        self.play_video(bagfile_handler,image_marker)

    def play_video(self,bagfile_handler,image_marker):
          
        paused_video = False
        
        persistent_markers = {}
        
        while True:
            if not paused_video:
                gray,markers = image_marker.get_next_image(bagfile_handler.get_image()) 
            
            cv2.imshow('frame',gray)
            key = cv2.waitKey(1000/60) & 0xFF
            if key == ord('q'):
                break
            if key == ord(' '):
                paused_video = not paused_video
                
            
            img2 = np.ones((600,600,3), np.uint8)
            
            # This factor is rather arbitrary. The x,y results of the following
            # calculation are approximately in between 0 and 8 and accordingly this
            # factor is chosen
            scale_factor = 300.0 * (1.0/8.0)
            shift_factor = 300.0
            turn_factor = np.deg2rad(110.0/2.0)+np.pi/2
            
            if not paused_video:
                for marker in persistent_markers:
                    persistent_markers[marker]['confidence']=persistent_markers[marker]['confidence']/2.0
                persistent_markers.update(markers)
            
            for marker_id in persistent_markers:
                #x = marker['distance']
                #print(x)#xy_marker_positions.append(())
                marker_data = persistent_markers[marker_id]
                
                x,y = cv2.polarToCart(marker_data['distance'],marker_data['angle']-turn_factor)
                x[0] = x[0] * scale_factor + shift_factor
                y[0] = y[0] * scale_factor + shift_factor
                cv2.circle(img2,(x[0],y[0]),5,(0,0,255*marker_data['confidence']),-1)
                
                # draw viewport lines
                x_orig,y_orig = cv2.polarToCart(0.0,0.0-turn_factor)
                x_dest,y_dest = cv2.polarToCart(8.0,0.0-turn_factor)
                x_orig[0] = x_orig[0] * scale_factor + shift_factor
                y_orig[0] = y_orig[0] * scale_factor + shift_factor
                x_dest[0] = x_dest[0] * scale_factor + shift_factor
                y_dest[0] = y_dest[0] * scale_factor + shift_factor
                
                cv2.line(img2,(x_orig[0],y_orig[0]),(x_dest[0],y_dest[0]),(255,255,255),1)
                
                fov = np.deg2rad(110)
                    
                x_orig_max,y_orig_max = cv2.polarToCart(0.0,fov-turn_factor)
                x_dest_max,y_dest_max = cv2.polarToCart(8.0,fov-turn_factor)
                x_orig_max[0] = x_orig_max[0] * scale_factor + shift_factor
                y_orig_max[0] = y_orig_max[0] * scale_factor + shift_factor
                x_dest_max[0] = x_dest_max[0] * scale_factor + shift_factor
                y_dest_max[0] = y_dest_max[0] * scale_factor + shift_factor
                
                cv2.line(img2,(x_orig_max[0],y_orig_max[0]),(x_dest_max[0],y_dest_max[0]),(255,255,255),1)
                
                
            cv2.imshow('topView',img2)
            cv2.moveWindow('topView',800,0)

if len(sys.argv) < 2:
    print("Please provide the path to the bagfile")
else:
    print("Processing bag files")
    Marker_Handler(sys.argv[1])