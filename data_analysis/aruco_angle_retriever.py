import cv2.aruco as aruco
import cv2
from zed_parameter import Zed_Parameter
import numpy as np
from numpy import average
from Marker import Marker

angleList = [0,0,0,0]
angleListIndex = 0
angleListLength = 4

zed_parameters = Zed_Parameter()

def get_boundary_angle_distance(cv_image, crop = False, max_distance_boundary = 2):
    '''
    Returns the average angle of the boundary next to the vehicle
    within the distance, defined by max_distance_boundary (in meter)
    '''
    markers = [] 
        
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
    cv_image = aruco.drawDetectedMarkers(cv_image, corners, borderColor = (0,255,0))
    
    marker_length = 0.2 # meter
    
    averageAngle = None
    minDistance = 99.0
    
    try:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
    except:
        rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, marker_length, zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
    
    if rvecs != None:       
        
        sum_sinuses = 0.0
        sum_cosinuses = 0.0
        distCounter = 0
        
        for i in range(0,len(rvecs)):
                        
            rvec = rvecs[i]
            tvec = tvecs[i]
            
            R,_ = cv2.Rodrigues(rvec)
            #cameraRotationVector,_ = cv2.Rodrigues(cv2.transpose(R))
            # Maybe needed later
            cameraTranslationVector = np.dot(cv2.transpose(-R),cv2.transpose(tvec))
            
            angle=np.arctan2(cameraTranslationVector[2],cameraTranslationVector[0])
            
            top_left_corner = tuple(corners[i][0][0].astype(int))
            bottom_left_corner = tuple(corners[i][0][3].astype(int))
           
            # Corner 0 is the top left corner and corner 3 the bottom left. This can be used
            # to detect flipped markers and change the angle accordingly
            if top_left_corner[1] > bottom_left_corner[1]:
                angle = np.pi - angle 
            
            # Some corrections to have the angle within reasonable values
            angle = angle - np.pi/2.0
        
            #cv2.putText(cv_image,str(np.round(np.rad2deg(angle[0]),2)), (top_left_corner[0],top_left_corner[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 1, (25,25,255),2)
            
            # Now get the distance to weight according to distance
            distance_marker = get_distance_of_line(0.2, top_left_corner, bottom_left_corner, zed_parameters.cameraMatrix, zed_parameters.distCoeffs)
            
            # Angle averaging is difficult because of the change around 0 and 2*pi
            if distance_marker < max_distance_boundary:           
                distCounter=distCounter + 1.0                
                distance_norm = (max_distance_boundary-distance_marker)/max_distance_boundary
                minDistance = min(minDistance, distance_marker)
                
                
                sum_sinuses = (sum_sinuses + np.sin(angle)) * distance_norm 
                sum_cosinuses = (sum_cosinuses + np.cos(angle)) * distance_norm
                
            #confidence,corners_xy,angle_to_top_left
            marker = Marker(ids[i],1.0,corners,angle,distance_marker)
            markers.append(marker)
                
        try:
            averageAngle = np.arctan(sum_sinuses / sum_cosinuses)
            #angleMessage1 = 'angle: ' + str(np.round(np.rad2deg(averageAngle[0]),2)) 
            #angleMessage2 = 'min distance: ' + str(np.round(minDistance,2))
            
            #cv2.putText(cv_image,angleMessage1, (50,300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
            #cv2.putText(cv_image,angleMessage2, (50,330), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
        
        except ZeroDivisionError:
            pass

    
    return averageAngle, minDistance, markers
    
    
    # Angles are averaged over the last 10 angles however, this is
    # done in a very simple fashion and should be also done with the
    # sin/cosin calculation
    
    # return cv_image

def get_distance_of_line(real_line_length, (px, py), (px_, py_), camMat, camDist):
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
        
        distance = (real_line_length * F) / P
        return distance
    
