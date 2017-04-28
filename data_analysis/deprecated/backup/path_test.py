import os
import cv2.aruco as aruco
try:
    user_paths = os.environ['PYTHONPATH'].split(os.pathsep)
except KeyError:
    user_paths = []
    
print user_paths