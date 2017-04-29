'''
Created on Apr 12, 2017

@author: picard
'''
import pickle
import sys
import cv2
#import aruco_code

for i in range(1,len(sys.argv)):
    print("-----------------")
    print("File: " + sys.argv[i])
    file = open(sys.argv[i], "rb" )
    
    while True:
        try:
            mylist = pickle.load( file )
            print(mylist)
        except:
            break


#    print("#####################")


#cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
#while True:
    #ret, image = cap.read()
        #
    #cv2.imshow('frame',image)
    #key = cv2.waitKey(1000/30) & 0xFF
#    
    ###if key == ord('q'):
        #break
#    
    #aruco_steer,aruco_motor,aruco_only = aruco_code.do_aruco(image,55.0,60.0)
    
    
    