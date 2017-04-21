'''
Created on Apr 11, 2017

@author: Sascha Hornauer
'''
import cv2
import cv2.aruco as aruco
from Board import Board
import sys
from zed_parameter import Zed_Parameter
import numpy as np
from cv2 import polarToCart
import math
from Marker import Marker
from Map import Map
from numpy import average



safety_distance = 1.5  # meter


class Video_Marker(object):
    map = Map()
    capture_device = None
    board = Board()
    markers = []
    bagfile_handler = None
    zed_parameters = Zed_Parameter()

    def __init__(self, bagfile_handler=None, capture_device=None):
        
        
        self.bagfile_handler = bagfile_handler
        
        
        if capture_device != None:
            self.capture_device = capture_device
            ''' Possible modes according to the zed manufacturer. This relates to the overall
            image. Our image is half of this width because of the cropping

            Video Mode    Frames per second    Output Resolution (side by side)
            2.2K          15                     4416x1242 ( NOT AVAILABLE ON JETSON TX1 )
            1080p         30                     3840x1080
            720p          60                     2560x720
            WVGA          100                    1344x376
            '''
            capture_device.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
            capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
        
       
    def process_next_image(self, crop, ar_params, cv_image=None):
        
            return self.mark_next_image(cv_image, ar_params, crop)

         
    def read_next_image(self):
        return self.capture_device.read()


    def get_safe_commands(self, critical_dist_angle_pairs, ar_params):
        '''
        Avert collision by steering at the opposite side of the average angle of all
        obstacles which are too near
        
        Change the motor command for all obstacles which are nearer than 1.5 meters
        and in the rough direction of movement proportionally until coming to a halt 
        if closer than 0.5 meter
        
        '''
        ''' Old settings
        motor_command = 49 # This is the resting command for stop
        max_left_steering_angle = np.deg2rad(-130)
        max_right_steering_angle = np.deg2rad(130)
        
        max_left_command = 100
        max_right_command = 0
        
        left_range = 50
        right_range = 50
        
        min_perceived_distance = 9999
        
        critical_distance = 1.5
        stop_distance = 0.5
        
        max_motor = 60
        min_motor = 49  # Full stop. Backwards is not considered
        '''
        motor_command = ar_params['ar_motor_command']
        max_left_steering_angle = ar_params['ar_max_left_steering_angle']
        max_right_steering_angle = ar_params['ar_max_right_steering_angle']
        
        max_left_command = ar_params['ar_max_left_command']
        max_right_command = ar_params['ar_max_right_command']
                
        left_range = ar_params['ar_left_range']
        right_range = ar_params['ar_right_range']
                
        min_perceived_distance = ar_params['ar_min_perceived_distance']
                
        critical_distance = ar_params['ar_critical_distance']
        stop_distance = ar_params['ar_stop_distance']
                
        max_motor = ar_params['ar_max_motor']
        min_motor = ar_params['ar_min_motor'] 
 
        
        # Which area in our viewport is considered "in front"
        # The viewport is at our angle calculation roughly in between -33 and 33 deg
        front_left_limit_deg = -25
        front_right_limit_deg = 25
        
        
        # Calculating the average angle to get away from the wall.
        # Lets just say it is not straightforward to calculate an
        # average angle
        for crit_pair in critical_dist_angle_pairs:
            sum_sinuses = np.sin(crit_pair['angle'])
            sum_cosinuses = np.cos(crit_pair['angle'])
            min_perceived_distance = min(crit_pair['distance'], min_perceived_distance)
            
        average_angle = np.arctan(sum_sinuses / sum_cosinuses)    
            
        opposite_angle = ((average_angle + np.pi) + np.pi) % (2 * np.pi) - np.pi 
        
        if opposite_angle < max_left_steering_angle:
            steering_command = max_left_command
        elif opposite_angle > max_right_steering_angle:
            steering_command = max_right_command
        else:
            # Opposite angle is within our steerable area
            # It is necessary to know if left or right to go though
            mid_steering_command = (max_right_command - max_left_command) / 2.0
            
            if opposite_angle < 0:
                steering_command = (opposite_angle / np.pi) * left_range
                           
            else:
                steering_command = (opposite_angle / np.pi) * right_range 
            
            # Finally change the mapping from -50,50 to 0,100
            steering_command = steering_command + mid_steering_command
            # steering_command = 
        
        
        # Next, calculate a safe motor command
        # If the average obstacle is in front of us....
        print("----" + str(average_angle)+ " , " + str(min_perceived_distance))
        
        if(np.deg2rad(front_left_limit_deg) < average_angle < np.deg2rad(front_right_limit_deg)):
            if(min_perceived_distance < stop_distance):
                motor_command = min_motor
            elif (min_perceived_distance < critical_distance):
                distance_norm = ((min_perceived_distance - stop_distance) / (critical_distance - stop_distance))
                motor_command = min_motor + distance_norm * (max_motor - min_motor)
                
                
        
        # safe_motor, safe_steer
        return motor_command, steering_command
    
    
    def mark_next_image(self, cv_image, ar_params, crop=False):
                
        frame = cv_image
    
        if(crop):
            height, width, channel = cv_image.shape
            yMin = 0
            yMax = height
            xMin = 0
            xMax = width / 2
            frame = frame[yMin:yMax, xMin:xMax]  
        
        aruco_dict = self.board.get_dictionary()
        parameters = aruco.DetectorParameters_create()
    
        res = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        corners = res[0]
        ids = res[1]
        gray = frame
        markers = []
        
        evasion_needed = False
        safe_steer = None
        safe_motor = None
        
        if len(corners) > 0:
            gray = aruco.drawDetectedMarkers(frame, corners)            
        
            # Quick fix to make my code compatible with the newer versions
            try:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.20, self.zed_parameters.cameraMatrix, self.zed_parameters.distCoeffs)
            except:
                rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.20, self.zed_parameters.cameraMatrix, self.zed_parameters.distCoeffs)
            
            
            
            critical_dist_angle_pairs = []
            
            for i in range(0, len(rvec)):
                
                # Get two dictionaries with xy positions about the corners of one marker and calculate also distance and angle to them
                center_line_xy, center_line_dist_ang = self.get_marker_from_image(gray, rvec[i][0], tvec[i][0], self.zed_parameters.cameraMatrix, self.zed_parameters.distCoeffs)
               
                
                # They are drawn onto the current image
                self.drawPointAtSingleMarker(gray, center_line_xy, center_line_dist_ang)
                
                if(center_line_dist_ang['distance'] < safety_distance):
                    critical_dist_angle_pairs.append(center_line_dist_ang)
                    evasion_needed = True
                
                # Finally they are filled in the marker data object

                marker = Marker(ids[i], confidence=1.0, center_line_xy=center_line_xy, center_line_dist_ang=center_line_dist_ang)
                markers.append(marker)
            
            if(evasion_needed and self.bagfile_handler == None):
                safe_motor, safe_steer = self.get_safe_commands(critical_dist_angle_pairs,ar_params)
                cv2.putText(gray, str(np.round(safe_motor, 2)) + "," + str(safe_steer), (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 4)
            
            # If an evasion is needed draw onto the image safe values
            if(evasion_needed and self.bagfile_handler != None):
                safe_motor, safe_steer = self.get_safe_commands(critical_dist_angle_pairs)
                self.bagfile_handler.evasion_data.append({'timestamp':self.bagfile_handler.timestamp, 'motor_command':safe_motor, 'steering_command':safe_steer})
                
                cv2.putText(gray, str(np.round(safe_motor, 2)) + "," + str(safe_steer), (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 4)
            
        return gray, markers, safe_motor, safe_steer, evasion_needed
    
    def get_center_line_xy(self, image, rvec, tvec, camMat, camDist):
        length = 0.2
        axisPoints = np.array([[-length / 2.0, 0, 0], [length / 2.0, 0, 0]])
        imgpts, jac = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
        
        # The points will be ordered according to their y axis so that even markers which are upside down
        # look the same to the algorself.ithm
        
        if(int(imgpts[0][0][1]) < int(imgpts[1][0][1])):
            xy1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
            xy2 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
        else:
            xy2 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
            xy1 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
  
        return [xy1, xy2]
    
    def get_corners_xy(self, image, rvec, tvec, camMat, camDist):
        '''
        Returns four different points for each corner right now in a 
        strange order. This is a bug in general it works though.
        Working on a different method now so this have to be improved later
        '''
        length = 0.2
        axisPoints = np.array([[-length / 2, -length / 2, 0], [-length / 2, length / 2, 0], [length / 2, -length / 2, 0], [length / 2, length / 2, 0]])
        imgpts, jac = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
        
        xy1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
        xy2 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
        xy3 = (int(imgpts[2][0][0]), int(imgpts[2][0][1]))
        xy4 = (int(imgpts[3][0][0]), int(imgpts[3][0][1]))
        
        return [xy1, xy2, xy3, xy4]
        

    def order_corners(self, corners_xy):
        '''
        The order in which the corners are presented is not
        good to loop over so this is corrected so the corners
        are in a rectangle
        '''
        tmp = corners_xy[2]
        corners_xy[2] = corners_xy[3]
        corners_xy[3] = tmp
        
    
    def get_center_line_polar(self, rvec, tvec, center_line_xy, camMat, camDist):
        center_line_dist_ang = []
        
        # distance D from our camera.
        # object width in pixels P
        # Focal length F
        
        W = 0.2  # meter size of marker

        x = center_line_xy[0][0]
        x_ = center_line_xy[1][0]
        y = center_line_xy[0][1]
        y_ = center_line_xy[1][1]
        
        # print center_line_xy
        # The method returns the angle to point x,y    
        distance, angle = self.get_distance_and_angle_of_line(W, (x, y), (x_, y_), camMat, camDist)
        center_line_dist_ang = {'distance':distance, 'angle':angle}        
            
        return center_line_dist_ang
    
    def get_corners_polar(self, rvec, tvec, corners_xy, camMat, camDist):
        
        corners_dist_ang = {}
        
        # distance D from our camera.
        # object width in pixels P
        # Focal length F
        
        W = 0.2  # meter size of marker
      
        # The corners are not always ordered in the expected way. To standardise this
        # they are ordered in the next method
        self.order_corners(corners_xy)
        
        for i in range(0, len(corners_xy) - 1):
            
            # A complete line is needed for distance calculation
            # because we compare the size of that found line with
            # the known size of the line
            x = corners_xy[i][0]
            x_ = corners_xy[i + 1][0]
            y = corners_xy[i][1]
            y_ = corners_xy[i + 1][1]
            
            # The method returns the angle to point x,y    
            distance, angle = self.get_distance_and_angle_of_line(W, (x, y), (x_, y_), camMat, camDist)
        
            corners_dist_ang[i] = {'corner_id':i, 'distance':distance, 'angle':angle}
            
        
        # We do the calculation again for the final distance, we did not do before in the loop
        # which goes from the last to the first corner    
        x = corners_xy[3][0]
        x_ = corners_xy[0][0]
        y = corners_xy[3][1]
        y_ = corners_xy[0][1]
        # The method returns the angle to point x,y
        distance, angle = self.get_distance_and_angle_of_line(W, (x, y), (x_, y_), camMat, camDist)
        corners_dist_ang[3] = {'corner_id':3, 'distance':distance, 'angle':angle}        
            
        return corners_dist_ang
          
    def get_marker_from_image(self, image, rvec, tvec, camMat, camDist):
        '''
        Method to compute the position, distance and angle to the markers
        '''
        # Changed the approach to mark only the middle
        # corners_xy = self.get_corners_xy(image, rvec, tvec, camMat, camDist)
        
        center_line_xy = self.get_center_line_xy(image, rvec, tvec, camMat, camDist)
        
        # Now we need the distance and angle to determine the appropriate size of 
        # text, drawn onto the markers. 
        center_line_dist_ang = self.get_center_line_polar(rvec, tvec, center_line_xy, camMat, camDist)  
        
        return center_line_xy, center_line_dist_ang
    
    def drawPointAtSingleMarker(self, image, center_line_xy, center_line_dist_ang):
        '''
        This method draws single markers and their distances 
        '''        
        # Just draw something on one corner because otherwise the screen will be too cluttered
        xy1 = (center_line_xy[0][0], center_line_xy[1][1])
    
        # Again the distance and angle of the first
        # marker is used here because for the textsize, this is adequate  
        
        distance = center_line_dist_ang['distance']
        angle = center_line_dist_ang['angle']
          
        # 8 is arbitrary and here approximately the biggest measured distance
        text_zoomfactor = 1 - (distance / (12 - distance))
        if text_zoomfactor < 0.5:
            text_zoomfactor = 0.5
        if text_zoomfactor > 1:
            text_zoomfactor = 1

                
        cv2.putText(image, str(np.round(distance, 2)), xy1, cv2.FONT_HERSHEY_SIMPLEX, text_zoomfactor, (255, 255, 255), 2)
        
        # cv2.putText(image,str(id),xy2,cv2.FONT_HERSHEY_SIMPLEX, text_zoomfactor, (0,255,0),2)

    def get_distance_and_angle_of_line(self, real_object_width_m, (px, py), (px_, py_), camMat, camDist):
        '''
        Returns distance to a line, given by px,py and px_,py_ and the angle in degree to 
        the point px,xy
        '''
                       
        # The focal length is averaged over fx and fy, which might decrease
        # the accuracy. This could be corrected in the future
        F = (camMat[0][0] + camMat[1][1]) / 2.0
        
        P_x = (px_ - px)
        P_y = (py_ - py)     
        P = np.hypot(P_x, P_y)
        
        distance = (real_object_width_m * F) / P
        
        x_mid = 1344 / 4.0
        y_mid = 376 / 4.0        

        angle = np.arctan((px - x_mid) / F)
        # print((px-x_mid))
        
        # angle =  np.rad2deg(np.arctan2(y_mid - py, x_mid - px))
        # angle = np.arctan2(y_mid - py, x_mid - px)
        # print(np.rad2deg(angle))
        # sys.exit(0
        return distance, angle
