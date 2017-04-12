from __future__ import print_function
import numpy as np
import cv2
import sys
import cv2.aruco as aruco
import rosbag
from cv_bridge import CvBridge, CvBridgeError
from draw_markers import *
from Board import dictionary
from zed_parameter import *

bridge = CvBridge()
#bag = rosbag.Bag('/home/picard/hddPlatte/caffe2_z2_color_direct_local_05Apr17_22h10m13s_Mr_Yellow/bair_car_2017-04-05-22-11-00_1.bag')
#bag = rosbag.Bag('/home/picard/2ndDisk/caffe2_z2_color_direct_local_09Apr17_16h40m09s_Mr_Black/bair_car_2017-04-09-16-43-33_7.bag')
bag = rosbag.Bag('/media/picard/rosbags/direct_Fern_aruco_1_11Apr17_22h59m32s_Mr_Yellow/bair_car_2017-04-11-23-07-52_16.bag')

#ids = {}


for topic, msg, t in bag.read_messages(topics=['/bair_car/zed/right/image_rect_color']):#, '/bair_car/encoder']):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    aruco_dict = dictionary
    parameters =  aruco.DetectorParameters_create()

    # See which markers are there
    res = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

    corners = res[0]
    ids = res[1]
    
    #for i in range(0,len(ids)):
    #    ids[i]
        
    gray = cv_image
    if len(corners)>0:
        gray = aruco.drawDetectedMarkers(cv_image, corners)
        
        ''' c++ cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec); 
        '''
        
        rvecOld = None
        tvecOld = None
        
        try:
            rvecOld = rvec
            tvecOld = tvec
        except:
            pass
        

        rvec, tvec = aruco.estimatePoseSingleMarkers(corners,0.20,cameraMatrix,distCoeffs)
        
        if rvecOld == None:
            rvecOld = rvec
            tvecOld = tvec
         
           
        #drawPointAtSingleMarker(gray,rvec,tvec,rvecOld,tvecOld,cameraMatrix,distCoeffs)
       
    cv2.imshow('frame',gray)
    if cv2.waitKey(1000/30) & 0xFF == ord('q'):
        break
               

cv2.destroyAllWindows()
bag.close()
