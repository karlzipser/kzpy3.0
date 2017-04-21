'''aruco_steer,aruco_motor,aruco_only = aruco_code.do_aruco(left_list[-1],steer,motor)'''
from Map import Map
from Video_Marker import Video_Marker
import cv2
marker = Video_Marker()

def do_aruco(cv_image,steering_cmd,motor_cmd):
    
    crop = True
    
    max_motor = motor_cmd 
    
    cv_image, markers, safe_motor, safe_steer = marker.process_next_image(crop,cv_image) 
    
    cv2.imshow('frame 2',cv_image)
    cv2.moveWindow('frame 2',700,600)
    return safe_steer,safe_motor,True

