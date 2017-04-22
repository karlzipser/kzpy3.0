import cv2.aruco as aruco
import cv2
from zed_parameter import Zed_Parameter
import numpy as np

zed_parameters = Zed_Parameter()

def get_aruco_image(cv_image, filled = False,color=(0,0,255), crop = False):
    '''
    Returns but also writes on the input image the place where the aruco markers
    are found.
    
    It is possible to fill the area of the marker entirely (filled)
    to choose the color (color = (RGB)) and to crop the input image
    by width/2.0 because that is sometimes handy when the whole ZED
    camera image from both cameras is used as input
    '''
        
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
    cv_image = aruco.drawDetectedMarkers(cv_image, corners, borderColor = color)
    
    
    if(filled):
        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, marker_length, zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
        
        # We have now one rvec and tvec for each found marker
        if (not rvec == None) and (not tvec == None):
            for i in range(0, len(rvec)):
                fill_image(cv_image,corners,rvec[i],tvec[i],zed_parameters.cameraMatrix, zed_parameters.distCoeffs,color)
    
    return cv_image
    
    
def fill_image(cv_image, corners,rvec,tvec,camMat,camDist,color):
    length = 0.2
    
    axisPoints = np.array([[-length/2.0,-length/2.0,0.0],[-length/2.0,length/2.0,0.0],[length/2.0,length/2.0,0.0],[length/2.0,-length/2.0,0.0]])
    imgpts, jac = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
    
    xy1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
    xy2 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
    xy3 = (int(imgpts[2][0][0]), int(imgpts[2][0][1]))
    xy4 = (int(imgpts[3][0][0]), int(imgpts[3][0][1]))
    polyPoints = np.array([[xy1,xy2,xy3,xy4]],dtype=np.int32)

    cv2.fillConvexPoly(cv_image,polyPoints, color)
    
    
if __name__ == '__main__':
    '''
    This code is purely to check if the code works standalone
    '''
    capture_device = cv2.VideoCapture(0)
    paused_video = False
    while True:
            if not paused_video:
                ret, image = capture_device.read()
                if image is None:
                    print("Error reading image! Wrong number of camera?")
                     
                cv_image = get_aruco_image(image,True,(255,0,0),True)
                cv2.imshow('frame',cv_image)
                key = cv2.waitKey(1000/30) & 0xFF
                
                if key == ord('q'):
                    break
                if key == ord(' '):
                    paused_video = not paused_video
                
    
        