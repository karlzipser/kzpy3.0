import cv2
import numpy as np
from zed_parameter import *

def drawPointAtSingleMarker(image,rvec,tvec,camMat,camDist):
    #cv2.circle(image,(x,y), 40, (0,0,255), -1)
    #imgpts, jac = cv2.projectPoints(axis, rvec, tvec, mtx, dist)

    length = 0.2
    axisPoints = np.array([[-length/2, -length/2, 0],[-length/2, length/2, 0],[length/2, -length/2,0],[length/2,length/2, 0]])
    #print(rvec)
    imgpts, jac  = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
    
    x = imgpts[0][0][0]
    y = imgpts[0][0][1]

    fps = 30.0

    oldpts, jav = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
    xOld = oldpts[0][0][0]
    yOld = oldpts[0][0][1]
    
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
    
    diffX = x-xOld
    diffY = y-yOld
    #print(oldpts)
    v = (np.hypot(diffX,diffY)/fps)*50 
    
    psi = np.arctan2(diffY,diffX)
    #print(x)
    #answer = getXYFor(x,y,0.0,v,psi,2,a,sigmF)
    #cv2.circle(image,(int(answer.x),int(answer.y)), 5, (0,0,255), -1)    
    cv2.circle(image,xy1, 10, (0,0,255), -1)
    cv2.circle(image,xy2, 10, (0,0,255), -1)
    cv2.circle(image,xy3, 10, (0,0,255), -1)
    cv2.circle(image,xy4, 10, (0,0,255), -1)
    cv2.putText(image,str(D),xy1,cv2.FONT_HERSHEY_SIMPLEX, 2, 255,2)
    #cv2.putText(image,str(x_),xy2,cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255,2)
    #cv2.putText(image,str(cameraPose[0][2][1]),(int(imgpts[0][0][0])-100,int(imgpts[0][0][1])-50),cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255,2)
    #cv2.putText(image,str(cameraPose[0][2][2]),(int(imgpts[0][0][0])-100,int(imgpts[0][0][1])),cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255,2)

    
    #putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]]) -> img
    #cv2.circle(image,(200,200), 40, (0,0,255), -1)    
    
