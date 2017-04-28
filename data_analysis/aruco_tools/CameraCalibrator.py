import time
import cv2
import cv2.aruco as aruco
import numpy as np
import pickle

arucoType = aruco.DICT_5X5_250
dictionary = cv2.aruco.getPredefinedDictionary(arucoType)
arucoBoard = aruco.CharucoBoard_create(9,6,0.7,0.5,aruco.Dictionary_get(arucoType))

cap = cv2.VideoCapture(1)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,2560)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)

width = cap.get(3)
heigth = cap.get(4)

yMin=0
yMax=heigth
xMin=0
xMax = width/2


allCorners = []
allIds = []
decimator = 0
for i in range(50):

    ret,frame = cap.read()
    frame = frame[yMin:yMax,xMin:xMax] # this is all there is to cropping

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary)

    if len(res[0])>0:
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,arucoBoard)
        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
            allCorners.append(res2[1])
            allIds.append(res2[2])

        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])
    #print i
    cv2.imshow('frame',gray)
    if cv2.waitKey(1000/5) & 0xFF == ord('q'):
        break
    decimator+=1

imsize = gray.shape

#Calibration fails for lots of reasons. Release the video if we do
print("Trying calibration")
retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,arucoBoard,imsize,None,None)
print(retval)
# Writing the calibration data so it must not be repeated each time

camera_values = {'retval':retval, 'cameraMatrix':cameraMatrix,'distCoeffs':distCoeffs,'rvecs':rvecs,'tvecs':tvecs}
file_out = open( "ZED_new_2.cal", "wb" );
pickle.dump( camera_values, file_out)
file_out.close()

print(camera_values)

cap.release()
cv2.destroyAllWindows()