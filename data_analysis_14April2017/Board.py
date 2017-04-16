import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image

class Board:
    
    dictionary = None
    aruco_type = None
    aruco_board = None
    board_image = None
    
    def __init__(self):
        #arucoType = aruco.DICT_6X6_100
        #arucoType = aruco.DICT_6X6_1000 # The single marker
        #arucoType = aruco.DICT_5X5_250
        self.aruco_type = aruco.DICT_4X4_250 # Markers on the side boundary

        #arucoBoard = aruco.CharthuucoBoard_create(4,5,0.7,0.5,aruco.Dictionary_get(arucoType))
        #arucoBoard = aruco.CharucoBoard_create(2,2,0.7,0.5,aruco.Dictionary_get(arucoType))
        #arucoBoard = aruco.CharucoBoard_create(9,5,0.7,0.5,aruco.Dictionary_get(arucoType))
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.aruco_type)
        self.aruco_board = aruco.CharucoBoard_create(9,5,0.7,0.5,aruco.Dictionary_get(self.aruco_type))
        self.board_image = self.aruco_board.draw((7014,4962))



    def get_marker_type(self):
        return self.aruco_type

    def get_dictionary(self):
        return self.dictionary

    def get_aruco_board(self):
        return self.aruco_board


#for i in range(131,230):
#    singleMarker = aruco.drawMarker(aruco.Dictionary_get(arucoType),i,1200)
#    cv2.imwrite("/home/picard/markers/id" + str(i) + ".jpg", singleMarker)
#cv2.imwrite("/home/picard/markers/charucoCal.jpg",board)

