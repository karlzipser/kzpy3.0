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

class Video_Marker(object):
    
    capture_device = None
    board = Board()
    markers = {}

    def __init__(self, capture_device=None):
        
        if capture_device != None:
            self.capture_device = capture_device
            
            capture_device.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
            capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        else:
            self.capture_device = capture_device    
            
       
    def get_next_image(self, cv_image=None):
        if(self.capture_device == None):
            return self.mark_next_image(cv_image)
        else:
            ret, frame = self.capture_device.read()
            return self.mark_next_image(frame) 
         
    def read_next_image(self):
        return self.capture_device.read()

    def mark_next_image(self, cv_image, crop=False):
                
        height, width, channel = cv_image.shape

        yMin = 0
        yMax = height
        xMin = 0
        xMax = width / 2
        # Capture frame-by-frame
        # ret, frame = self.capture_device.read()
        frame = cv_image
    
        if(crop):
            frame = frame[yMin:yMax, xMin:xMax]  
        
        aruco_dict = self.board.get_dictionary()
        parameters = aruco.DetectorParameters_create()
    
        res = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        corners = res[0]
        ids = res[1]
        gray = frame
        fov = 0
        markers = {}
        
        if len(corners) > 0:
            gray = aruco.drawDetectedMarkers(frame, corners)
            
            try:
                rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.20, zed_parameter.cameraMatrix, zed_parameter.distCoeffs)
            except:
                pass
            
            for i in range(0,len(rvec)):
                distance, angle, fov = self.drawPointAtSingleMarker(gray, rvec[i][0], tvec[i][0], zed_parameter.cameraMatrix, zed_parameter.distCoeffs, ids[i])
                # It is a bit uninuitive that these information are calculated in a "draw" function. Needs redesign
                markers[str(ids[i])]={'distance':distance,'angle':angle,'confidence':1.0}
            
        
    
        return gray, markers, fov
    
    
    
    
    def drawPointAtSingleMarker(self, image, rvec, tvec, camMat, camDist, id):
        '''
        This method draws single markers and their distances 
        '''
    
        length = 0.2
        
        axisPoints = np.array([[-length / 2, -length / 2, 0], [-length / 2, length / 2, 0], [length / 2, -length / 2, 0], [length / 2, length / 2, 0]])
    
        imgpts, jac = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
        
        xy1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
        xy2 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
        xy3 = (int(imgpts[2][0][0]), int(imgpts[2][0][1]))
        xy4 = (int(imgpts[3][0][0]), int(imgpts[3][0][1]))
        
        # distance D from our camera.
        # object width in pixels P
        # Focal length F
        
        W = 0.2  # meter size of marker
      
        x = xy1[0]
        x_ = xy2[0]
        y = xy1[1]
        y_ = xy2[1]
        
        distance, angle = self.get_distance_and_angle_of_line(W, (x, y), (x_, y_), camMat, camDist)
        distance = np.round(distance,3)
        angle = np.round(angle,3)
        
        # 8 is arbitrary and here approximately the biggest measured distance
        text_zoomfactor = 1 - (distance/(8-distance))
        if text_zoomfactor < 0.5:
            text_zoomfactor = 0.5
        if text_zoomfactor > 1:
            text_zoomfactor = 1
        
        cv2.putText(image, str(angle), xy1, cv2.FONT_HERSHEY_SIMPLEX, text_zoomfactor, (255, 255, 255), 2)
        #cv2.putText(image,str(id),xy2,cv2.FONT_HERSHEY_SIMPLEX, text_zoomfactor, (0,255,0),2)
        
      
        height, width, channel = image.shape
        
        fov = 2*np.arctan(width/(2*(camMat[0][0] + camMat[1][1]) / 2))
        
        return distance, angle, fov

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
        
       
        
        #angle =  np.rad2deg(np.arctan2(y_mid - py, x_mid - px))
        angle =  np.arctan2(y_mid - py, x_mid - px)
        
        return distance, angle
