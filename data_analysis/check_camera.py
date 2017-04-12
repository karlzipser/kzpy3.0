from __future__ import print_function
import numpy as np
import cv2
import sys
import cv2.aruco as aruco
from zed_parameter import *
from draw_markers import *

#arucoType = aruco.DICT_4X4_250 # Markers on the side boundary

#arucoBoard = aruco.CharthuucoBoard_create(4,5,0.7,0.5,aruco.Dictionary_get(arucoType))
#arucoBoard = aruco.CharucoBoard_create(2,2,0.7,0.5,aruco.Dictionary_get(arucoType))
#arucoBoard = aruco.CharucoBoard_create(9,5,0.7,0.5,aruco.Dictionary_get(arucoType))
#dictionary = cv2.aruco.getPredefinedDictionary(arucoType)

cap = cv2.VideoCapture(1)

# 3840x1080
# 2560x720
cap.set(cv2.CAP_PROP_FRAME_WIDTH,2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
#cap.set(cv2.CAP_PROP_FPS,30)
#cap.set(cv2.CAP_PROP_AUTOFOCUS,1)
#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,-1)
#cap.set(cv2.CAP_PROP_)
#cap.set(cv2.CAP_PROP_FOCUS,0)
#cap.set(cv2.CAP_PROP_FPS,100)

#arucoBoard = aruco.CharucoBoard_create(3,3,0.3,0.2,aruco.Dictionary_get(aruco.DICT_4X4_250))

rvec = None
tvec = None

retVal = False


width = cap.get(3)
heigth = cap.get(4)
    
yMin=0
yMax=heigth
xMin=0
xMax = width/2
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('output.avi',fourcc, 60.0, (2560,720))


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    frame = frame[yMin:yMax,xMin:xMax] # this is all there is to cropping
    
    aruco_dict = dictionary
    parameters =  aruco.DetectorParameters_create()

    res = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    
    corners = res[0]
    ids = res[1]
    gray = frame
    if len(corners)>0:
        gray = aruco.drawDetectedMarkers(frame, corners)
        
        ''' c++ cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec); 
        '''
        
        rvecOld = None
        tvecOld = None
        
        try:
            rvecOld = rvec
            tvecOld = tvec
        except:
            pass
            #rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs
        
        try:
            #print(cameraMatrix)
            rvec, tvec = aruco.estimatePoseSingleMarkers(corners,0.20,cameraMatrix,distCoeffs)
        except:
            pass
        
        if rvecOld == None:
            rvecOld = rvec
            tvecOld = tvec
        #print(rvec)
        
        try:
            drawPointAtSingleMarker(gray,rvec[0][0],tvec[0][0],rvecOld,tvecOld,cameraMatrix,distCoeffs)
        except:
            print(sys.exc_info()[1])
            pass
          
    #other_format = cv2.cvtColor(gray,cv2.COLOR_BGR2RGB)

    cv2.imshow('frame',gray)
    if cv2.waitKey(1000/30) & 0xFF == ord('q'):
        break
              


cap.release()
cv2.destroyAllWindows()
