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
from aruco_tools.aruco_angle_retriever import *



def get_angles_and_distance(cv_image, crop=False):
    
    markers = get_markers_in_image(cv_image, crop)
    angles_to_center = {}
    angles_surfaces = {}
    distances_marker = {}
    
    for marker in markers:
        angles_to_center[marker.marker_id] = get_angle_to_center(marker)
        angles_surfaces[marker.marker_id] = get_angle_surface(marker)
        distances_marker[marker.marker_id] = get_distance(marker)
    
    return angles_to_center, angles_surfaces, distances_marker, markers


class Angle_Dict_Creator:
    
    detected_markers = {}
    area_visualizer = None # The choice if there is a visualizer or not is not yet implemented
    source_local_camera = False
    source_bagfile = True
    show_video = True
    crop = False # Should the input video be cropped to the left image input. If false this code might contain errors

    
    def __init__(self, arguments):
        
        bagfile_path = arguments[1]
        show_video = arguments[2]
        if(show_video == "0"):
            self.show_video = False
        
        self.crop = False    
        bagfile_handler = Bagfile_Handler(bagfile_path)
        self.play_video(bagfile_handler, None)
        
        
        
    def calculate_data_to_dict(self,cv_image):
        
        marker_dict = {}
        
        angles_to_center, angles_surfaces, distances_marker, markers  = get_angles_and_distance(cv_image, False)

        
        for marker in markers:
            marker_id = marker.marker_id

            marker_dict[marker_id] = {'angle_to_center':angles_to_center[marker_id],'angle_of_surface':angles_surfaces[marker_id],'distance_to_center':distances_marker[marker_id]}

            if(self.show_video):            
            
                xy1 = tuple(marker.corners_xy[0][0])
                xy2 = tuple(marker.corners_xy[0][1])
                xy3 = tuple(marker.corners_xy[0][2])
                
                cv2.putText(cv_image, self.beautify_string(angles_to_center[marker_id],True), xy1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)
                cv2.putText(cv_image, self.beautify_string(angles_surfaces[marker_id],True), xy2, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(cv_image, self.beautify_string(distances_marker[marker_id],False), xy3, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 200), 1)
        
        return marker_dict
       
            
    def beautify_string(self,number,rad2deg):
        
        if rad2deg:
            return str(np.round(np.rad2deg(number),2))
        else:
            return str(np.round(number,2))
        

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
                    self.calculate_data_to_dict(cv_image)
                            
            if(self.show_video):
                cv2.imshow('frame',cv_image)
                key = cv2.waitKey(1000/30) & 0xFF
                if key == ord('q'):
                    break
                if key == ord(' '):
                    paused_video = not paused_video
                if key == ord('w'):
                    bagfile_handler.fast_forward()
                

Angle_Dict_Creator(sys.argv)
