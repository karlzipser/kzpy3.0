'''
Created on Apr 13, 2017

@author: Sascha Hornauer
'''

class Marker(object):
    '''
    classdocs
    '''
    marker_id = None
    confidence = 1.0
    corners_xy = []
    angle_to_top_left = None
    distance_to_left_side = None
    
    

    def __init__(self,marker_id,confidence,corners_xy,angle_to_top_left,distance_to_left_side):
        '''
        Constructor
        '''
        self.marker_id = marker_id
        self.confidence = confidence
        self.corners_xy = corners_xy
        self.angle_to_top_left = angle_to_top_left
        self.distance_to_left_side = distance_to_left_side
        
    
    
    
    def __repr__(self):
        '''
        Quick string out method
        '''
        return str(self.id) + ","+str(self.confidence) + ","+str(self.corners_xy) + ","+str(self.angle_to_top_left) 