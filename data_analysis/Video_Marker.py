'''
Created on Apr 11, 2017

@author: picard
'''
import cv2
import cv2.aruco as aruco
from Board import Board
import zed_parameter


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
            return mark_next_image(cv_image)
        else:
            ret, frame = self.capture_device.read()
            return mark_next_image(frame) 
         
    def read_next_image(self):
        return self.capture_device.read()

    def mark_next_image(self, cv_image):
                
        height, width, channel  = cv_image.shape

        yMin=0
        yMax=heigth
        xMin=0
        xMax = width/2
        # Capture frame-by-frame
        #ret, frame = self.capture_device.read()
        frame = cv_image
    
        frame = frame[yMin:yMax,xMin:xMax] # this is all there is to cropping
        
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
            
            try:
                drawPointAtSingleMarker(gray,rvec[0][0],tvec[0][0],zed_parameter.cameraMatrix,zed_parameter.distCoeffs)
            except:
                print(sys.exc_info()[1])
                pass
    
        return gray
        