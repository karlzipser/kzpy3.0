'''
Created on Apr 18, 2017

@author: picard
'''
import cv2
import numpy as np
from cv2 import getDefaultNewCameraMatrix
from dask.array.ufunc import angle


class Map(object):
    '''
    classdocs
    '''
    img2 = np.ones((600,600,3), np.uint8)
    markers = {}
    positions = {}
        
    def __init__(self):
        
        print("test")
        #cv2.imshow('map',self.img2)
        #cv2.moveWindow('map',1400,0)
        #key = cv2.waitKey() & 0xFF
        
    
    def experiment(self, image, rvec, tvec, cameraMatrix, distCoeffs,center_line_dist_ang):
        # project axis points
        
        
        
        length = 0.1
        axisPoints = np.array([[0.0,0.0,0.0], [length, 0.0,0.0], [0.0, length, 0.0], [0.0,0.0,length]])


        rvec_rodrigues, _ = cv2.Rodrigues(rvec)
        _, rvecs_trans_rodrigues = cv2.invert(rvec_rodrigues)
        rvecs_trans, _ = cv2.Rodrigues(rvecs_trans_rodrigues)
    
        imagePoints_new, jac = cv2.projectPoints(axisPoints, rvecs_trans, tvec, cameraMatrix, distCoeffs);
        
        imagePoints_new = imagePoints_new.astype(int)
        
        camMat = cameraMatrix
        camDist = distCoeffs

        cv2.line(image, tuple(imagePoints_new[0][0]), tuple(imagePoints_new[1][0]), (0, 0, 255), 3);
        cv2.line(image, tuple(imagePoints_new[0][0]),  tuple(imagePoints_new[2][0]), (0, 255, 0), 3);
        cv2.line(image,  tuple(imagePoints_new[0][0]),  tuple(imagePoints_new[3][0]), (255, 0, 0), 3);
       
        #gray = cv2.undistort(frame, self.zed_parameters.cameraMatrix, self.zed_parameters.distCoeffs )
       
        #cv2.putText(image, str(center_line_dist_ang['angle']) , (10,300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
        #cv2.putText(image, str(rvecs_trans) , (10,200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
        #print(distCoeffs)
        angle_a = center_line_dist_ang['angle']
        angle_b = rvecs_trans[2][0]
        # angle normalisation
        angle_c = (((angle_b + np.pi/6.0)/(2*np.pi/3.0))*np.pi/3.0)-(np.pi/6.0)
        print(np.rad2deg(angle_b))
        print(np.rad2deg(angle_a))
        
        angle_c = angle_b + angle_a
        
        x,y = cv2.polarToCart(center_line_dist_ang['distance'],angle_c)
        xa,ya = cv2.polarToCart(center_line_dist_ang['distance'],angle_a)
        xb,yb = cv2.polarToCart(center_line_dist_ang['distance'],angle_b)
        scale_factor = 300
        cv2.circle(self.img2,(x[0]*scale_factor+300,y[0]*scale_factor+300),2,(0,0,255),-1)
        cv2.circle(self.img2,(xa[0]*scale_factor,ya[0]*scale_factor),2,(0,255,0),-1)
        cv2.circle(self.img2,(xb[0]*scale_factor,yb[0]*scale_factor),2,(255,0,0),-1)
        #(img2,(x_a[0],y_a[0]),2,(0,0,255*marker.confidence),-1)
        cv2.imshow('map',self.img2)
        cv2.moveWindow('map',1400,0)

    '''
    def get_center_line_xy(self,rvec,tvec,camMat,camDist):
        length = 0.2
        axisPoints = np.array([[-length/2.0, 0, 0],[length/2.0, 0, 0]])
        imgpts, jac = cv2.projectPoints(axisPoints, rvec, tvec, camMat, camDist);
        
        # The points will be ordered according to their y axis so that even markers which are upside down
        # look the same to the algorself.ithm
        
        if(int(imgpts[0][0][1]) < int(imgpts[1][0][1])):
            xy1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
            xy2 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
        else:
            xy2 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
            xy1 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))
  
        return [xy1,xy2]
    
    
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
        
        #print center_line_xy
        # The method returns the angle to point x,y    
        distance, angle = self.get_distance_and_angle_of_line(W, (x, y), (x_, y_), camMat, camDist)
        center_line_dist_ang = {'distance':distance, 'angle':angle}        
            
        return center_line_dist_ang
    
    def get_distance_and_angle_of_line(self, real_object_width_m, (px, py), (px_, py_), camMat, camDist):
        
        #Returns distance to a line, given by px,py and px_,py_ and the angle in degree to 
        #the point px,xy
        
                       
        # The focal length is averaged over fx and fy, which might decrease
        # the accuracy. This could be corrected in the future
        F = (camMat[0][0] + camMat[1][1]) / 2.0
        
        P_x = (px_ - px)
        P_y = (py_ - py)     
        P = np.hypot(P_x, P_y)
        
        distance = (real_object_width_m * F) / P
       #print(distance)
       # print( (real_object_width_m * F) / P_x)
       # print(str(np.abs(px_-px)) + " and " + str(P_x))
       # print(str(np.abs(py_-py)) + " and " + str(P_y))
       # print( (real_object_width_m * F) / P_y)
       ## sys.exit(0)
        
        #x_mid = camMat[0][2]
        #y_mid = camMat[1][2]
        
        x_mid= 1344/4.0
        y_mid = 376/4.0        

        angle = np.arctan((px-x_mid)/F)
        #print((px-x_mid))
        
        # angle =  np.rad2deg(np.arctan2(y_mid - py, x_mid - px))
        #angle = np.arctan2(y_mid - py, x_mid - px)
        #print(np.rad2deg(angle))
        #sys.exit(0
        return distance, angle
    '''
    def addMarker(self,id ,distance,angle):
        
        '''
        if not id in self.markers:
            # the id is not yet in the dcit
            # generate some position for the first marker
            self.positions[id]={'x':np.random.uniform(200,400),'y':np.random.uniform(200,400)}
            
        
        
       
        self.markers[id]={'distance':distance,'angle':angle}
        
        x_vec,y_vec = cv2.polarToCart(distance,angle)
        x=x_vec*600.0*(1.0/8.0)
        y=y_vec*600.0*(1.0/8.0)
        
        pos_x = self.positions[id]['x']+x[0]
        pos_y = self.positions[id]['y']+y[0]
        #print(pos_x[0],pos_y[0])
        
        cv2.circle(self.img2,(np.int(pos_x[0]),np.int(pos_y[0])),3,(0,0,255),-1)
        '''
        '''
        print(id)
        # This is amazingly bad though will do for the time
        if int(id[1:3]) == 21:
            
            x_fix = 300
            y_fix = 300
            
            scale_factor = 100 # calculate from meters to pixel 
            
            #print(distance)
            #print(angle)
            x_perc, y_perc = cv2.polarToCart(distance*scale_factor,angle+np.pi/2.0)
            x_origin = x_fix-x_perc[0][0]
            y_origin = y_fix-y_perc[0][0]
            cv2.circle(self.img2,(int(x_origin),int(y_origin)),3,(0,0,255),-1)
        
            cv2.imshow('map',self.img2)
        '''
        
test = Map()