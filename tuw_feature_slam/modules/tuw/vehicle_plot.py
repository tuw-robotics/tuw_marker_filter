'''
Created on Jan 6, 2017

@author: max
'''

import numpy as np
import math
from matplotlib.patches import Ellipse
from matplotlib.patches import Circle
from matplotlib.patches import Polygon
from tuw.plot import PoseArrow
from tuw.plot import Landmark
from tuw.plot import CovEllipse


class VehiclePlot:
    '''
    classdocs
    '''
    def __init__(self, ax, vehicle):
        '''
        Constructor
        '''            
        self.vehicle = vehicle;      
        self.PoseArrowOdom = PoseArrow(0.4, 'b', 0.4)
        ax.add_artist(self.PoseArrowOdom)
        self.PoseArrowTruePose = PoseArrow(0.4, 'k', 0.4)
        ax.add_artist(self.PoseArrowTruePose)
        self.PoseArrowRobot = PoseArrow(0.4, 'r', 0.4)
        self.CovEllipseRobot = CovEllipse('r', 0.4)
        ax.add_artist(self.PoseArrowRobot)
        ax.add_artist(self.CovEllipseRobot)
        self.LandmarkMap = [ Landmark(0.4, 'g', 0.0) for i in range(10)]
        for i in range(len(self.LandmarkMap)):
            ax.add_artist(self.LandmarkMap[i])
        self.LandmarkMarker = [ Landmark(0.4, 'b', 0.0) for i in range(10)]
        for i in range(len(self.LandmarkMarker)):
            ax.add_artist(self.LandmarkMarker[i])
        self.LandmarkSLAMMap = [ Landmark(0.4, 'm', 0.0) for i in range(10)]
        for i in range(len(self.LandmarkSLAMMap)):
            ax.add_artist(self.LandmarkSLAMMap[i])

        
                
    def set_odom(self, pose):
        self.vehicle.set_odom(pose)
        self.PoseArrowOdom.set_pose(self.vehicle.odom) 
        
    def define_map(self, m):
        self.vehicle.define_map(m)
        for i in range(len(self.LandmarkMap)):
            if i < len(m):
                self.LandmarkMap[i].set_pose(m[i,1:4].transpose())
                self.LandmarkMap[i].set_alpha(0.5)
                self.LandmarkMap[i].set_text(m[i,0])
            else: 
                self.LandmarkMap[i].set_alpha(0.0)        
        
    def measurments(self, z):
        self.vehicle.measurments(z) 
        for i in range(len(self.LandmarkMarker)):
            if i < len(z) and hasattr(self.vehicle, 'x') :
                self.LandmarkMarker[i].set_ralative_pose(self.vehicle.x, z[i,1:4].transpose())
                self.LandmarkMarker[i].set_alpha(0.5)
                self.LandmarkMarker[i].set_text(z[i,0])
            else: 
                self.LandmarkMarker[i].set_alpha(0.0)
        if hasattr(self.vehicle, 'm') :
            for i in range(len(self.LandmarkSLAMMap)):
                if i < len(self.vehicle.m):
                    self.LandmarkSLAMMap[i].set_pose(self.vehicle.m[i,1:4].transpose())
                    self.LandmarkSLAMMap[i].set_alpha(0.5)
                    self.LandmarkSLAMMap[i].set_text(self.vehicle.m[i,0])
                else: 
                    self.LandmarkSLAMMap[i].set_alpha(0.0)   
        
    
    def prediction(self, u):
        if self.vehicle.prediction(u) :
            self.PoseArrowRobot.set_pose(self.vehicle.x)  
            self.CovEllipseRobot.set_cov(self.vehicle.x, self.vehicle.P)       
        
    
        
            