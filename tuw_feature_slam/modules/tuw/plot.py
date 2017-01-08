'''
Created on Jan 6, 2017

@author: max
'''
from matplotlib.patches import Polygon
from matplotlib.patches import Ellipse
from tuw.geometry import transform_points
import numpy as np

class PoseArrow(Polygon):
    '''
    classdocs
    '''


    def __init__(self, size, color, tranparency, **kwargs):
        '''
        Constructor
        '''
        
        Polygon.__init__(self, np.array([[0,0]]), **kwargs)
        #self.shape = np.array([[0, size],
        #               [np.sin(2.35) * size, np.cos(2.35) * size], 
        #               [0, 0], 
        #               [np.sin(-2.35) * size, np.cos(-2.35) * size]]);
                       
        self.shape = np.array([[size, 0],
                       [np.cos(2.35) * size, np.sin(2.35) * size], 
                       [0, 0], 
                       [np.cos(-2.35) * size, np.sin(-2.35) * size]]);
        
        self.set_alpha(tranparency)
        self.set_edgecolor(color)
        self.set_facecolor('none')
    
    def set_pose(self, pose):
        xy = np.copy(self.shape)
        transform_points(self.shape, pose, xy)
        self.set_xy(xy);
        
class Landmark(Polygon):
    '''
    classdocs
    '''
    def __init__(self, size, color, tranparency, **kwargs):
        '''
        Constructor
        '''
        
        Polygon.__init__(self, np.array([[0,0]]), **kwargs)
        r = size/2;
        self.shape = np.array([[ 0,  0],
                               [ r,  0],
                               [ r,  r],
                               [-r,  r], 
                               [-r, -r], 
                               [ r, -r], 
                               [ r,  0]]);
        
        self.set_alpha(tranparency)
        self.set_edgecolor(color)
        self.set_facecolor('none')
    
    def set_pose(self, pose):
        xy = np.copy(self.shape)
        transform_points(self.shape, pose, xy)
        self.set_xy(xy);
                
                
class CovEllipse(Ellipse):
    '''
    classdocs
    '''
    def __init__(self, color, tranparency, **kwargs):
        '''
        Constructor
        '''
        
        Ellipse.__init__(self, np.array([0, 0]), 0, 0, 0, **kwargs)
        #self.P_ell.set_clip_box(ax.bbox)
        self.set_facecolor('none')
        self.set_edgecolor(color)
        self.set_alpha(tranparency)
    
    def set_cov(self, pose, cov):
        x = pose[0,0]
        y = pose[0,1]
        theta = pose[0,2]    
        eval, evec = np.linalg.eigh(cov[0:2,0:2])
        angle = np.arctan2(evec[0,1], evec[0,0]) 
         
        self.center = np.array([x, y])
        self.width, self.height = eval[0], eval[1]
        self.angle = np.rad2deg(angle)
                                        