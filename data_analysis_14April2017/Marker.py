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
    corners_xy_pos = []
    corners_distances_angles = {}
    
    

    def __init__(self,id,confidence,corners_xy_pos,corners_distances_angles):
        '''
        Constructor
        '''
        self.id = id
        self.confidence = confidence
        self.corners_xy_pos = corners_xy_pos
        self.corners_distances_angles = corners_distances_angles
        
    
    
        
    
    def __repr__(self):
        return str(self.id) + ","+str(self.confidence) + ","+str(self.corners_xy_pos) + ","+str(self.corners_distances_angles) 