import cv2
import numpy as np
from zed_parameter import *

def drawPointAtSingleMarker(image,rvec,tvec,camMat,camDist):
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
    
      
    W = 0.2 # meter size of marker
    F = (cameraMatrix[0][0]+cameraMatrix[1][1])/2

    x = xy1[0]
    x_ = xy2[0]
    y = xy1[1]
    y_ = xy2[1]
    
    P = np.hypot(x_-x,y_-y)
    

    D = (W * F) / P
     
    cv2.circle(image,xy1, 5, (0,0,255), -1)
    cv2.circle(image,xy2, 5, (0,0,255), -1)
    cv2.circle(image,xy3, 5, (0,0,255), -1)
    cv2.circle(image,xy4, 5, (0,0,255), -1)
    cv2.putText(image,str(D),xy1,cv2.FONT_HERSHEY_SIMPLEX, 1, 255,2)

    
