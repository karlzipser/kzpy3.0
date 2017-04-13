'''
Created on Apr 11, 2017

@author: Sascha Hornauer
'''
import sys
import cv2

from Video_Marker import Video_Marker
from Bagfile_Handler import Bagfile_Handler

class Marker_Handler:
    
    def __init__(self, arguments):
        bagfile_handler = Bagfile_Handler(arguments)
        image_marker = Video_Marker()
        
        
        while True:
            gray = image_marker.get_next_image(bagfile_handler.get_image())
            cv2.imshow('frame',gray)
            if cv2.waitKey(1000/30) & 0xFF == ord('q'):
                break

if len(sys.argv) < 2:
    print("Please provide the path to the bagfile")
else:
    print("Processing bag files")
    Marker_Handler(sys.argv[1])