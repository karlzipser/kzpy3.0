import cv2.aruco as aruco
import cv2
from zed_parameter import Zed_Parameter
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


zed_parameters = Zed_Parameter()

fig = plt.figure()
ax = plt.axes(projection='3d')

def get_average_boundary_angle(cv_image, crop = False, max_distance_boundary = 4):
    '''
    Returns the average angle of the boundary next to the vehicle
    within the distance, defined by max_distance_boundary (in meter)
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
    
    averageAngle = None
    
    try:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
    except:
        rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, marker_length, zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
    
    if rvecs != None:       
        
        sum_sinuses = 0.0
        sum_cosinuses = 0.0
        
        for i in range(0,len(rvecs)):
            
            rvec = rvecs[i]
            tvec = tvecs[i]
            
            R,strangeStuffIDontUnderstand = cv2.Rodrigues(rvec)
            cameraRotationVector,strangeStuffIDontUnderstand = cv2.Rodrigues(cv2.transpose(R))
            cameraTranslationVector = np.dot(cv2.transpose(-R),cv2.transpose(tvec))
            
            angle=np.arctan2(cameraTranslationVector[2],cameraTranslationVector[0])
            
            top_left_corner = tuple(corners[i][0][0].astype(int))
            bottom_left_corner = tuple(corners[i][0][3].astype(int))
           
            # Corner 0 is the top left corner and corner 3 the bottom left. This can be used
            # to detect flipped markers
            markerFlipped = False
            if top_left_corner[1] > bottom_left_corner[1]:
                markerFlipped = True
            
            if markerFlipped:
                angle = np.pi - angle 
            
            # Some corrections to have the angle within reasonable values
            angle = angle - np.pi/2.0
            
            # Now get the distance to weight according to distance
            distance_marker = get_distance_of_line(0.2, top_left_corner, bottom_left_corner, zed_parameters.cameraMatrix,zed_parameters.distCoeffs)
            
            if distance_marker < max_distance_boundary:           
                
                distance_norm = 1.0-(max_distance_boundary-distance)/max_distance_boundary
                                            
                sum_sinuses = (sum_sinuses + np.sin(angle)) * distance_norm 
                sum_cosinuses = (sum_cosinuses + np.cos(angle)) * distance_norm
            
                
            
        
        average_angle = np.arctan(sum_sinuses / sum_cosinuses)
        
        if averageAngle == None:
            angleMessage = "Too far" 
        else:
            angleMessage = str(np.rad2deg(averageAngle))
            
        cv2.putText(cv_image,angleMessage, (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
        
        
    return cv_image


def get_distance_of_line(self, real_line_length, (px, py), (px_, py_), camMat, camDist):
        '''
        Returns distance to a line of known size, given by px,py and px_,py_ and the angle in degree to 
        the point px,xy
        '''
                       
        # The focal length is averaged over fx and fy, which might decrease
        # the accuracy. This could be corrected in the future
        F = (camMat[0][0] + camMat[1][1]) / 2.0
        
        P_x = (px_ - px)
        P_y = (py_ - py)     
        P = np.hypot(P_x, P_y)
        
        distance = (real_object_width_m * F) / P
        return distance
    
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
                
    
        