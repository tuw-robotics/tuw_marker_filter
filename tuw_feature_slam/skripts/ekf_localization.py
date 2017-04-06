'''
Created on Jul 25, 2016

@author: Markus Bader
'''

import numpy as np
import matplotlib.pyplot as plt 
from matplotlib.pyplot import imshow, pause
import matplotlib.cbook
from matplotlib.patches import Ellipse
from matplotlib.patches import Circle
from matplotlib.patches import Polygon
from tuw.plot import PoseArrow
from tuw.plot import Landmark
from tuw.plot import CovEllipse

fig, ax = plt.subplots()
plt.draw()
plt.ion()
ax.axis('equal')
ax.grid(True)
ax.set_xticks(np.arange(-10, 10, 1))
ax.set_yticks(np.arange(-10, 10, 1))
ax.set_xlim([-12,12])
plt.xlabel('x')
ax.set_ylim([-12,12])
plt.ylabel('y')
plt.show()

class EKFLocalization:
    '''
    classdocs
    '''
    def __init__(self, filename):
        '''
        Constructor
        '''     
        self.filename = filename        
        self.PoseArrowOdom = PoseArrow(0.4, 'b', 0.4)
        ax.add_artist(self.PoseArrowOdom)
        self.PoseArrowTruePose = PoseArrow(0.4, 'k', 0.4)
        ax.add_artist(self.PoseArrowTruePose)
        self.PoseArrowRobot = PoseArrow(0.4, 'r', 0.4)
        self.CovEllipseRobot = CovEllipse('r', 0.4)
        ax.add_artist(self.PoseArrowRobot)
        ax.add_artist(self.CovEllipseRobot)

        self.dt = 0.1
        self.alpha = np.array([0.2, 0.05 , 0.1, 0.02])
        #self.x = np.matrix([[ 0, 0 , 0]]).transpose()
        self.P = np.matrix([[ 0.3, 0 , 0],[ 0, 0.3 , 0],[ 0, 0 , 0.1]])
        
        
                
    def set_odom(self, pose):
        if hasattr(self, 'x') == False: 
            self.x = pose
        self.odom = pose
        self.PoseArrowOdom.set_pose(self.odom) 
        
    def define_map(self, m):
        print "map"
        print m
        
    def correction(self, z):
        print "marker"
        print z
        
    
    def prediction(self, u):
        if hasattr(self, 'x') == False: 
            return
        dt = self.dt
        v = u[0,0]
        w = u[1,0]
        x = self.x[0,0]
        y = self.x[1,0]
        theta = self.x[2,0]
        s = np.sin(theta)
        c = np.cos(theta)
        s_dt = np.sin(theta + w * dt)
        c_dt = np.cos(theta + w * dt)
        
        M = np.matrix( [[self.alpha[0]*v*v + self.alpha[1]*w*w, 0], [0, self.alpha[2]*v*v + self.alpha[3]*w*w]]);
        G = np.matrix( [[0, 0, 0], [0, 0, 0], [0, 0, 0]]);
        V = np.matrix( [[0, 0], [0, 0]]);
        if(np.fabs(w) > 0.0):
            r = v/w;
            dx = -r * s + r * s_dt
            dy = +r * c - r * c_dt
            da = w * dt
            
            G = np.matrix( [[1, 0, -r * c + r * c_dt], 
                            [0, 1, -r * s + r * s_dt], 
                            [0, 0,             1]]);
            
            V = np.matrix( [[( -s + s_dt ) /w,  r * ( s - s_dt ) / w + r * c_dt * dt], 
                            [(  c - c_dt ) /w, -r * ( c - c_dt ) / w + r * s_dt * dt],
                            [               0,                                    dt]]);
        else:
            dx = v * np.cos(theta) * dt
            dy = v * np.sin(theta) * dt
            da = 0.0
            
            G = np.matrix( [[1, 0, -v * s * dt], 
                            [0, 1,  v * c * dt], 
                            [0, 0,           1]]);
            
            V = np.matrix( [[dt * c, -0.5 * dt * dt * v * s], 
                            [dt * s,  0.5 * dt * dt * v * c],
                            [     0,                     dt]]);
        
        self.x = self.x + np.matrix([[ dx, dy , da]]).transpose()  
        P_last = np.copy(self.P)
        self.P = G * P_last * G.transpose() + V * M * V.transpose()  
        self.PoseArrowRobot.set_pose(self.x)  
        self.CovEllipseRobot.set_cov(self.x, self.P)       
        
    def loop(self):
        skip = 50
        loop_counter = 0
        with open(self.filename) as f:
            for line in f:
                loop_counter = loop_counter + 1
                elements = line.split(':')
                header = elements[0].strip()                
                if('odom' == header):
                    pose = np.matrix(map(float, elements[1].split(","))).reshape(3,-1)
                    self.set_odom(pose)
                if('true_pose' == header):
                    data = np.matrix(map(float, elements[1].split(",")))
                    self.PoseArrowTruePose.set_pose(data) 
                if('cmd' == header):
                    data = np.matrix(map(float, elements[1].split(","))).reshape(2,-1)
                    self.prediction(data) 
                if('marker' == header):
                    t = np.matrix(map(int,   elements[1].split(",")))
                    #z = np.matrix(map(float, elements[2].split(","))).reshape(-1,4)                    
                    #self.correction(z)    
                if('map' == header):
                    t = np.array(map(int,   elements[1].split(",")))
                    #m = np.array(map(float, elements[2].split(","))).reshape(-1,4) 
                    #self.define_map(m)
                if( loop_counter % skip ) == 0:
                    plt.pause(0.001)  
                    print loop_counter
            

if __name__ == '__main__':
    node = EKFLocalization("/home/max/projects/catkin/tuw_marker/src/tuw_marker_filter/tuw_feature_slam/data/log01.txt")
    node.loop()
    print "Good-by"
