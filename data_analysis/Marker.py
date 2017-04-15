'''
Created on Apr 13, 2017

@author: Sascha Hornauer
'''

class Marker(object):
    '''
    classdocs
    '''
    id = -1.0
    confidence = 1.0
    center_line_xy = []
    center_line_dist_ang = []
    
    

    def __init__(self,id,confidence,center_line_xy,center_line_dist_ang):
        '''
        Constructor
        '''
        self.id = id
        self.confidence = confidence
        self.corners_xy_pos = center_line_xy
        self.corners_distances_angles = center_line_dist_ang
        
    
    
        
    
    def __repr__(self):
        return str(self.id) + ","+str(self.confidence) + ","+str(self.center_line_xy) + ","+str(self.center_line_dist_ang) 