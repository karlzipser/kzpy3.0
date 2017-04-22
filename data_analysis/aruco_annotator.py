import cv2.aruco as aruco
import cv2
from zed_parameter import Zed_Parameter
import numpy as np

if __name__ == '__main__':
    pass
    

zed_parameters = Zed_Parameter()

def get_aruco_image(cv_image, filled = False, crop = False):
        
    if(crop):
        height, width, channel = cv_image.shape
        yMin = 0
        yMax = height
        xMin = 0
        xMax = width / 2
        cv_image = cv_image[yMin:yMax, xMin:xMax]    
        
        
    marker_length = 0.2 # in meters    
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()
    
    corners, ids, rejected_points = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
    cv_image = aruco.drawDetectedMarkers(cv_image, corners)
    
    
    
    if(filled):
        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, marker_length, zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
        
        # We have now one rvec and tvec for each found marker
        if (not rvec == None) and (not tvec == None):
            for i in range(0, len(rvec)):
                fill_image(cv_image,corners,rvec[i],tvec[i],zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
    
    return cv_image
    
    
def fill_image(cv_image, corners,rvec,tvec,camMat,camDist):
    length = 0.2
    
    axisPoints = np.array([[-length/2.0,-length/2.0,0.0],[-length/2.0,length/2.0,0.0],[length/2.0,length/2.0,0.0],[length/2.0,-length/2.0,0.0]])
    imgpts, jac = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
    
    xy1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
    xy2 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
    xy3 = (int(imgpts[2][0][0]), int(imgpts[2][0][1]))
    xy4 = (int(imgpts[3][0][0]), int(imgpts[3][0][1]))
    polyPoints = np.array([[xy1,xy2,xy3,xy4]],dtype=np.int32)

    cv2.fillPoly(cv_image,polyPoints, (0,0,255))
        