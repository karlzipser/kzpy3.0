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

class Video_Marker(object):
    
    capture_device = None
    board = Board()

    def __init__(self, capture_device=None):
        
        if capture_device != None:
            self.capture_device = capture_device
            
            capture_device.set(cv2.CAP_PROP_FRAME_WIDTH,2560)
            capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
        else:
            self.capture_device = capture_device    
            
       
    def get_next_image(self, cv_image = None):
        if(self.capture_device == None):
            return self.mark_next_image(cv_image)
        else:
            ret, frame = self.capture_device.read()
            return self.mark_next_image(frame) 
         
    def read_next_image(self):
        return self.capture_device.read()

    def mark_next_image(self, cv_image, crop=False):
                
        height, width, channel  = cv_image.shape

        yMin=0
        yMax=height
        xMin=0
        xMax = width/2
        # Capture frame-by-frame
        #ret, frame = self.capture_device.read()
        frame = cv_image
    
        if(crop):
            frame = frame[yMin:yMax,xMin:xMax] #
        
        aruco_dict = self.board.get_dictionary()
        parameters =  aruco.DetectorParameters_create()
    
        res = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        corners = res[0]
        ids = res[1]
        gray = frame
        if len(corners)>0:
            gray = aruco.drawDetectedMarkers(frame, corners)
            
            try:
                rvec, tvec = aruco.estimatePoseSingleMarkers(corners,0.20,zed_parameter.cameraMatrix,zed_parameter.distCoeffs)
            except:
                pass
            
        #    try:
            self.drawPointAtSingleMarker(gray,rvec[0][0],tvec[0][0],zed_parameter.cameraMatrix,zed_parameter.distCoeffs)
        #    except:
        #        print(sys.exc_info()[1])
        #        pass
    
        return gray, res
    
    
    
    
    def drawPointAtSingleMarker(self,image,rvec,tvec,camMat,camDist):
        '''
        This method draws single markers and their distances 
        '''
    
        length = 0.2
            
        axisPoints = np.array([[-length/2, -length/2, 0],[-length/2, length/2, 0],[length/2, -length/2,0],[length/2,length/2, 0]])
    
        imgpts, jac  = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
        
        xy1 = (int(imgpts[0][0][0]),int(imgpts[0][0][1]))
        xy2 = (int(imgpts[1][0][0]),int(imgpts[1][0][1]))
        xy3 = (int(imgpts[2][0][0]),int(imgpts[2][0][1]))
        xy4 = (int(imgpts[3][0][0]),int(imgpts[3][0][1]))
        
        # distance D from our camera.
        # object width in pixels P
        # Focal length F
        
        W = 0.2 # meter size of marker
      
        x = xy1[0]
        x_ = xy2[0]
        y = xy1[1]
        y_ = xy2[1]
        
        distance= self.get_distance_of_line(W, (x, y), (x_, y_), camMat, camDist)

        cv2.putText(image,str(distance),xy1,cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255),2)
        #cv2.putText(image,str((F_y/P_y)),xy2,cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0),2)

    def get_distance_of_line(self, real_object_width_m, (px,py),(px_,py_),camMat,camDist):
                       
        # The focal length is averaged over fx and fy, which might decrease
        # the accuracy. This could be corrected in the future
        F = (camMat[0][0]+camMat[1][1])/2   
        P_x = (px_-px)
        P_y = (py_-py)     
        P = np.hypot(P_x,P_y)
        
        distance = (real_object_width_m * F) / P
        
        return distance

