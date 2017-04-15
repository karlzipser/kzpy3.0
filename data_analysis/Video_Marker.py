'''
Created on Apr 11, 2017

@author: Sascha Hornauer
'''
import cv2
import cv2.aruco as aruco
from Board import Board
import sys
import zed_parameter
import numpy as np
from _socket import AF_X25
from cv2 import polarToCart
import math
from Marker import Marker
from bakKzpy3.RPi3.drive_server_RPi import left_range


safety_distance = 2.0 # meter


class Video_Marker(object):
    
    capture_device = None
    board = Board()
    markers = []

    def __init__(self, capture_device=None):
        
        if capture_device != None:
            self.capture_device = capture_device
            
            capture_device.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
            capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        else:
            self.capture_device = capture_device    
            
       
    def process_next_image(self, timestamp, cv_image=None):
        
        if(self.capture_device == None):
            return self.mark_next_image(cv_image)
        else:
            ret, frame = self.capture_device.read()
            return self.mark_next_image(frame) 
         
    def read_next_image(self):
        return self.capture_device.read()


    def get_safe_commands(self, critical_dist_angle_pairs):
        
        max_left_steering_angle = np.deg2rad(-90)
        max_right_steering_angle = np.deg2rad(90)
        
        max_left_command = 0
        max_right_command = 100
        
        left_range = 50
        right_range = 50
        
        # Lets just say it is not straightforward to calculate an
        # average angle
        for crit_pair in critical_dist_angle_pairs:
            sum_sinuses = np.sin(crit_pair['angle'])
            sum_cosinuses = np.cos(crit_pair['angle'])
            
        average_angle = np.arctan(sum_sinuses/sum_cosinuses)    
            
        opposite_angle =( (average_angle + np.pi) + np.pi) % (2 * np.pi ) - np.pi 
        
        if opposite_angle < max_left_steering_angle:
            steering_command = max_left_command
        elif opposite_angle > max_right_steering_angled:
            steering_command = max_right_command

        # Opposite angle is within our steerable area
        # It is necessary to know if left or right to go though
        
        if opposite_angle < 0:
            steering_command = (opposite_angle / np.pi)*left_range              
        else:
            steering_command = (opposite_angle / np.pi)*right_range 
        
        
        return 0.0,0.0
    
    
    def mark_next_image(self, cv_image, crop=False):
                
        frame = cv_image
    
        if(crop):
            height, width, channel = cv_image.shape
            yMin = 0
            yMax = height
            xMin = 0
            xMax = width / 2
            frame = frame[yMin:yMax, xMin:xMax]  
        
        aruco_dict = self.board.get_dictionary()
        parameters = aruco.DetectorParameters_create()
    
        res = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        corners = res[0]
        ids = res[1]
        gray = frame
        markers = []
        
        evasion_needed = False
        
        if len(corners) > 0:
            gray = aruco.drawDetectedMarkers(frame, corners)            
            try:
                rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.20, zed_parameter.cameraMatrix, zed_parameter.distCoeffs)
            except:
                pass
            
            critical_dist_angle_pairs = []
            
            for i in range(0, len(rvec)):
                # Get two dictionaries with xy positions about the corners of one marker and calculate also distance and angle to them
                center_line_xy, center_line_dist_ang = self.get_marker_from_image(gray, rvec[i][0], tvec[i][0], zed_parameter.cameraMatrix, zed_parameter.distCoeffs)
                # They are drawn onto the current image
                self.drawPointAtSingleMarker(gray, center_line_xy, center_line_dist_ang)
                
                if(center_line_dist_ang['distance'] < safety_distance):
                    critical_dist_angle_pairs.append(center_line_dist_ang)
                    evasion_needed = True
                
                # Finally they are filled in the marker data object
                # PUT DICT SAVING HERE
                marker = Marker(ids[i], confidence=1.0, center_line_xy=center_line_xy, center_line_dist_ang=center_line_dist_ang)
                markers.append(marker)
            
            # If an evasion is needed draw onto the image safe values
            if(evasion_needed):
                safe_motor, safe_steer = self.get_safe_commands(critical_dist_angle_pairs)
                cv2.putText(gray, str(str(safe_motor) + "," + str(safe_steer)), [300,300], cv2.FONT_HERSHEY_SIMPLEX, 6, (255, 255, 255), 2)
            
        return gray, markers
    
    def get_center_line_xy(self,image,rvec,tvec,camMat,camDist):
        length = 0.2
        axisPoints = np.array([[-length/2.0, 0, 0],[length/2.0, 0, 0]])
        imgpts, jac = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
        
        # The points will be ordered according to their y axis so that even markers which are upside down
        # look the same to the algorithm
        
        if(int(imgpts[0][0][1]) < int(imgpts[1][0][1])):
            xy1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
            xy2 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
        else:
            xy2 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
            xy1 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
  
        return [xy1,xy2]
    
    def get_corners_xy(self, image, rvec, tvec, camMat, camDist):
        '''
        Returns four different points for each corner right now in a 
        strange order. This is a bug in general it works though.
        Working on a different method now so this have to be improved later
        '''
        length = 0.2
        axisPoints = np.array([[-length / 2, -length / 2, 0], [-length / 2, length / 2, 0], [length / 2, -length / 2, 0], [length / 2, length / 2, 0]])
        imgpts, jac = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
        
        xy1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
        xy2 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
        xy3 = (int(imgpts[2][0][0]), int(imgpts[2][0][1]))
        xy4 = (int(imgpts[3][0][0]), int(imgpts[3][0][1]))
        
        return [xy1, xy2, xy3, xy4]
        

    def order_corners(self, corners_xy):
        '''
        The order in which the corners are presented is not
        good to loop over so this is corrected so the corners
        are in a rectangle
        '''
        tmp = corners_xy[2]
        corners_xy[2] = corners_xy[3]
        corners_xy[3] = tmp
        
    
    def get_center_line_polar(self, rvec, tvec, center_line_xy, camMat, camDist):
        center_line_dist_ang = []
        
        # distance D from our camera.
        # object width in pixels P
        # Focal length F
        
        W = 0.2  # meter size of marker
      
        x = center_line_xy[0][0]
        x_ = center_line_xy[1][0]
        y = center_line_xy[0][1]
        y_ = center_line_xy[1][1]
        
        # The method returns the angle to point x,y    
        distance, angle = self.get_distance_and_angle_of_line(W, (x, y), (x_, y_), camMat, camDist)
        center_line_dist_ang = {'distance':distance, 'angle':angle}        
            
        return center_line_dist_ang
    
    def get_corners_polar(self, rvec, tvec, corners_xy, camMat, camDist):
        
        corners_dist_ang = {}
        
        # distance D from our camera.
        # object width in pixels P
        # Focal length F
        
        W = 0.2  # meter size of marker
      
        # The corners are not always ordered in the expected way. To standardise this
        # they are ordered in the next method
        self.order_corners(corners_xy)
        
        for i in range(0, len(corners_xy) - 1):
            
            # A complete line is needed for distance calculation
            # because we compare the size of that found line with
            # the known size of the line
            x = corners_xy[i][0]
            x_ = corners_xy[i + 1][0]
            y = corners_xy[i][1]
            y_ = corners_xy[i + 1][1]
            
            # The method returns the angle to point x,y    
            distance, angle = self.get_distance_and_angle_of_line(W, (x, y), (x_, y_), camMat, camDist)
        
            corners_dist_ang[i] = {'corner_id':i, 'distance':distance, 'angle':angle}
            
        
        # We do the calculation again for the final distance, we did not do before in the loop
        # which goes from the last to the first corner    
        x = corners_xy[3][0]
        x_ = corners_xy[0][0]
        y = corners_xy[3][1]
        y_ = corners_xy[0][1]
        # The method returns the angle to point x,y
        distance, angle = self.get_distance_and_angle_of_line(W, (x, y), (x_, y_), camMat, camDist)
        corners_dist_ang[3] = {'corner_id':3, 'distance':distance, 'angle':angle}        
            
        return corners_dist_ang
          
    def get_marker_from_image(self, image, rvec, tvec, camMat, camDist):
        '''
        Method to compute the position, distance and angle to the markers
        '''
        # Changed the approach to mark only the middle
        # corners_xy = self.get_corners_xy(image, rvec, tvec, camMat, camDist)
        
        center_line_xy = self.get_center_line_xy(image, rvec, tvec, camMat, camDist)
        
        # Now we need the distance and angle to determine the appropriate size of 
        # text, drawn onto the markers. 
        center_line_dist_ang = self.get_center_line_polar(rvec, tvec, center_line_xy, camMat, camDist)  
        
        return center_line_xy, center_line_dist_ang
    
    def drawPointAtSingleMarker(self, image, center_line_xy, center_line_dist_ang):
        '''
        This method draws single markers and their distances 
        '''        
        # Just draw something on one corner because otherwise the screen will be too cluttered
        xy1 = (center_line_xy[0][0], center_line_xy[1][1])
    
        # Again the distance and angle of the first
        # marker is used here because for the textsize, this is adequate  
        
        distance = center_line_dist_ang['distance']
        angle = center_line_dist_ang['angle']
          
        # 8 is arbitrary and here approximately the biggest measured distance
        text_zoomfactor = 1 - (distance / (12 - distance))
        if text_zoomfactor < 0.5:
            text_zoomfactor = 0.5
        if text_zoomfactor > 1:
            text_zoomfactor = 1
        
        cv2.putText(image, str(np.round(distance,2)), xy1, cv2.FONT_HERSHEY_SIMPLEX, text_zoomfactor, (255, 255, 255), 2)
        
        # cv2.putText(image,str(id),xy2,cv2.FONT_HERSHEY_SIMPLEX, text_zoomfactor, (0,255,0),2)

    def get_distance_and_angle_of_line(self, real_object_width_m, (px, py), (px_, py_), camMat, camDist):
        '''
        Returns distance to a line, given by px,py and px_,py_ and the angle in degree to 
        the point px,xy
        '''
                       
        # The focal length is averaged over fx and fy, which might decrease
        # the accuracy. This could be corrected in the future
        F = (camMat[0][0] + camMat[1][1]) / 2   
        
        P_x = (px_ - px)
        P_y = (py_ - py)     
        P = np.hypot(P_x, P_y)
        
        distance = (real_object_width_m * F) / P
        
        x_mid = camMat[0][2]
        y_mid = camMat[1][2]        
        
        # angle =  np.rad2deg(np.arctan2(y_mid - py, x_mid - px))
        angle = np.arctan2(y_mid - py, x_mid - px)
        
        return distance, angle
