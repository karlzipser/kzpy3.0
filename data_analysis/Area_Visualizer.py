'''
Created on Apr 12, 2017

@author: Sascha Hornauer
'''
import numpy as np
import sys
import cv2
from Marker import Marker


class Area_Visualizer(object):
    
    
    persistent_markers = {}
        
    def __init__(self):
        pass      

    def visualize_markers_center_line(self,markers):
        img2 = np.ones((600,600,3), np.uint8)
        
        # This factor is rather arbitrary. The x,y results of the following
        # calculation are approximately in between 0 and 8 and accordingly this
        # factor is chosen
        scale_factor = 300.0 * (1.0/8.0)
        shift_factor = 300
        turn_factor = np.deg2rad(110.0/2.0)+np.pi/4.0 # division by int gives integer
        
        # Reduce confidence for each marker in the persistent list, if there are any yet
        for marker_id in self.persistent_markers:
            self.persistent_markers[marker_id].confidence = self.persistent_markers[marker_id].confidence-0.1
        
        
        # Add markers to list, overwriting old markers and thereby increasing confidence levels
        for marker in markers:
            self.persistent_markers[str(marker.id)]=marker
        
        
        # Now draw lines onto new window
        for marker_id in self.persistent_markers:
            
            marker = self.persistent_markers[marker_id] 
            
            # Draw marker outline at the bottom of the screen
            xy1 = marker.corners_xy_pos[0]
            xy2 = marker.corners_xy_pos[1]

            cv2.line(img2,(xy1[0],xy1[1]+shift_factor),(xy2[0],xy2[1]+shift_factor),(0,0,255*marker.confidence),1)

            distance_a = marker.corners_distances_angles['distance']
            angle_a = marker.corners_distances_angles['angle']
            
            # Draw top view outline
            x_a,y_a = cv2.polarToCart(distance_a,angle_a-turn_factor)
            x_a[0] = x_a[0] * scale_factor + shift_factor
            y_a[0] = y_a[0] * scale_factor + shift_factor
            cv2.circle(img2,(x_a[0],y_a[0]),2,(0,0,255*marker.confidence),-1)
                
            #ircle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
                
            # Draw outer viewport lines
            '''
            x_orig,y_orig = cv2.polarToCart(0.0,0.0-turn_factor)
            x_dest,y_dest = cv2.polarToCart(8.0,0.0-turn_factor)
            x_orig[0] = x_orig[0] * scale_factor + shift_factor
            y_orig[0] = y_orig[0] * scale_factor + shift_factor
            x_dest[0] = x_dest[0] * scale_factor + shift_factor
            y_dest[0] = y_dest[0] * scale_factor + shift_factor
            
            cv2.line(img2,(x_orig[0],y_orig[0]),(x_dest[0],y_dest[0]),(255,255,255),1)
            
            fov = np.deg2rad(110/2.0)
                
            x_orig_max,y_orig_max = cv2.polarToCart(0.0,fov-turn_factor)
            x_dest_max,y_dest_max = cv2.polarToCart(8.0,fov-turn_factor)
            x_orig_max[0] = x_orig_max[0] * scale_factor + shift_factor
            y_orig_max[0] = y_orig_max[0] * scale_factor + shift_factor
            x_dest_max[0] = x_dest_max[0] * scale_factor + shift_factor
            y_dest_max[0] = y_dest_max[0] * scale_factor + shift_factor
            
            cv2.line(img2,(x_orig_max[0],y_orig_max[0]),(x_dest_max[0],y_dest_max[0]),(255,255,255),1)
            '''
            
        cv2.imshow('topView',img2)
        cv2.moveWindow('topView',700,0)
    
        
    def visualize_markers_four_corners(self,markers):
        img2 = np.ones((600,600,3), np.uint8)
        
        # This factor is rather arbitrary. The x,y results of the following
        # calculation are approximately in between 0 and 8 and accordingly this
        # factor is chosen
        scale_factor = 300.0 * (1.0/8.0)
        shift_factor = 300
        turn_factor = np.deg2rad(110.0/2.0)+np.pi/2
        
        # Reduce confidence for each marker in the persistent list, if there are any yet
        for marker_id in self.persistent_markers:
            self.persistent_markers[marker_id].confidence = self.persistent_markers[marker_id].confidence-0.1
        
        
        
        # Add markers to list, overwriting old markers and thereby increasing confidence levels
        for marker in markers:
            self.persistent_markers[str(marker.id)]=marker
        
        
        
        # Now draw lines onto new window
        for marker_id in self.persistent_markers:
            
            marker = self.persistent_markers[marker_id] 
            
            # Draw marker outline at the bottom of the screen
            xy1 = marker.corners_xy_pos[0]
            xy2 = marker.corners_xy_pos[1]
            xy3 = marker.corners_xy_pos[2]
            xy4 = marker.corners_xy_pos[3]
            
            cv2.line(img2,(xy1[0],xy1[1]+shift_factor),(xy2[0],xy2[1]+shift_factor),(0,0,255*marker.confidence),1)
            cv2.line(img2,(xy2[0],xy2[1]+shift_factor),(xy3[0],xy3[1]+shift_factor),(0,0,255*marker.confidence),1)
            cv2.line(img2,(xy3[0],xy3[1]+shift_factor),(xy4[0],xy4[1]+shift_factor),(0,0,255*marker.confidence),1)
            cv2.line(img2,(xy4[0],xy4[1]+shift_factor),(xy1[0],xy1[1]+shift_factor),(0,0,255*marker.confidence),1)

            distance_a = marker.corners_distances_angles[2]['distance']
            angle_a = marker.corners_distances_angles[2]['angle']
            
            distance_b = marker.corners_distances_angles[0]['distance']
            angle_b = marker.corners_distances_angles[0]['angle']
            
            
            # Draw top view outline
            x_a,y_a = cv2.polarToCart(distance_a,angle_a-turn_factor)
            x_a[0] = x_a[0] * scale_factor + shift_factor
            y_a[0] = y_a[0] * scale_factor + shift_factor
                
            x_b,y_b = cv2.polarToCart(distance_b,angle_b-turn_factor)
            x_b[0] = x_b[0] * scale_factor + shift_factor
            y_b[0] = y_b[0] * scale_factor + shift_factor
                
                
            cv2.line(img2,(x_a[0],y_a[0]),(x_b[0],y_b[0]),(0,0,255*marker.confidence),1)
            
            
            # Draw outer viewport lines
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
        cv2.moveWindow('topView',700,0)