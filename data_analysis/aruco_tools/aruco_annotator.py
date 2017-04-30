import cv2.aruco as aruco
import cv2
from kzpy3.data_analysis.zed_parameter import Zed_Parameter
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
        
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()
    
    corners, ids, rejected_points = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
    cv_image = aruco.drawDetectedMarkers(cv_image, corners, borderColor = color)
    

    if(filled):
        for i in range(0, len(corners)):
            fill_image(cv_image,corners[i],color)
    
    return cv_image
    
    
def fill_image(cv_image, corners,color):
    polyPoints = np.array(corners,dtype=np.int32)
    cv2.fillConvexPoly(cv_image,polyPoints, color)
    
    
if __name__ == '__main__':
    '''
    This code is purely to check if the code works standalone
    '''
    capture_device = cv2.VideoCapture(2)
    capture_device.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
    capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
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
                
    
        