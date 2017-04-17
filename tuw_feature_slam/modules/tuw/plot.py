'''
Created on Jan 6, 2017

@author: max
'''
from matplotlib.patches import Ellipse
from matplotlib.patches import Polygon

import numpy as np
from tuw.geometry import transform_points
from tuw.geometry import transform_poses

    
def transform_shape(src, tf):
    dx = tf.item(0)
    dy = tf.item(1)
    dtheta = tf.item(2)
    s = np.sin(dtheta)
    c = np.cos(dtheta)
    des = np.zeros(src.shape)
    for i in range(len(src)):
        des[i,0] =  c * src[i,0] + -s * src[i,1] + dx
        des[i,1] =  s * src[i,0] +  c * src[i,1] + dy
    return des
   
def transform_pose(src, base):
    s = np.sin(base.item(2))
    c = np.cos(base.item(2))
    des = np.zeros(src.shape)
    des[0,0] =  c * src.item(0) + -s * src.item(1) + base.item(0)
    des[1,0] =  s * src.item(0) +  c * src.item(1) + base.item(1)
    des[2,0] =      src.item(2) + base.item(2)
    return des
            
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
                       
        self.shape = np.matrix([[size, 0],
                       [np.cos(2.35) * size, np.sin(2.35) * size], 
                       [0, 0], 
                       [np.cos(-2.35) * size, np.sin(-2.35) * size]])
        
        self.set_alpha(tranparency)
        self.set_edgecolor(color)
        self.set_facecolor('none')
    
    def set_pose(self, pose):  
        self.set_xy(transform_shape(self.shape, pose))
        
        
    def set_ralative_pose(self, base, pose):
        xy = np.copy(self.shape)
        pose = pose.reshape(1,3);
        tf = np.zeros((1,3))
        transform_poses(pose, base, tf)
        transform_points(self.shape, tf, xy)
        self.set_xy(transform_shape(self.shape, tf))
        
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
        self.set_xy(transform_shape(self.shape, pose))
        #xy = np.copy(self.shape)
        #transform_points(self.shape, pose, xy)
        #self.set_xy(xy);
        
        
    def set_ralative_pose(self, base, pose):
        tf = transform_pose(pose, base)
        self.set_xy(transform_shape(self.shape, tf))
        
    def set_text(self, id):
        self.text_label = str(int(id))
        self.text_color = self.get_edgecolor()
        
    def draw(self, renderer):
        r = Polygon.draw(self, renderer)
            
        if hasattr(self, 'text_label'):
            if hasattr(self, 'ax_text'):
                self.ax_text.set_text(self.text_label)
                self.ax_text.set_position([self.xy[0,0], self.xy[0,1]] )
            else:
                self.ax_text = self.axes.text(self.xy[0,0], self.xy[0,1], self.text_label)
            self.ax_text.set_alpha((self.get_alpha() > 0))
            
            self.ax_text.set_color(self.text_color)
        return r
    
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
        x = pose.item(0)
        y = pose.item(1)
        theta = pose.item(2)
        eval, evec = np.linalg.eigh(cov[0:2,0:2])
        angle = np.arctan2(evec[0,1], evec[0,0]) 
         
        self.center = np.array([x, y])
        self.width, self.height = eval[0], eval[1]
        self.angle = np.rad2deg(angle)
                                        