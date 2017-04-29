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
    rvec = None
    tvec = None
    
    

    def __init__(self,marker_id,confidence,corners_xy,rvec,tvec):
        '''
        Constructor
        '''
        self.marker_id = marker_id
        self.confidence = confidence
        self.corners_xy = corners_xy
        self.rvec = rvec
        self.tvec = tvec
        
    
    
    
    def __repr__(self):
        '''
        Quick string out method
        '''
        return str(self.marker_id) + ","+str(self.confidence) + ","+str(self.corners_xy) 