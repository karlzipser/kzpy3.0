import cv2.aruco as aruco
import cv2
from zed_parameter import Zed_Parameter
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


zed_parameters = Zed_Parameter()

fig = plt.figure()
ax = plt.axes(projection='3d')

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
    
 


 
    marker_length = 0.2 # meter
    try:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
    except:
        rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, marker_length, zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
    
    if rvecs != None:       
        for i in range(0,len(rvecs)):
            
            rvec = rvecs[i]
            tvec = tvecs[i]
            
            R,strangeStuffIDontUnderstand = cv2.Rodrigues(rvec)
            
            cameraRotationVector,strangeStuffIDontUnderstand = cv2.Rodrigues(cv2.transpose(R))
        
            cameraTranslationVector = np.dot(cv2.transpose(-R),cv2.transpose(tvec))
   
            angle=np.arctan2(cameraTranslationVector[2],cameraTranslationVector[0])
            
            xy1 = cv2.polarToCart(30,angle[0]-np.pi/2.0)
            a = cv2.polarToCart(50,angle[0]-np.pi/2.0)
            
            cv2.line(cv_image,(xy1[0][0],xy1[1][0]),(a[0][0],a[1][0]),(255,255,255),1)
            


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
    capture_device = cv2.VideoCapture(0)
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
                
    
        