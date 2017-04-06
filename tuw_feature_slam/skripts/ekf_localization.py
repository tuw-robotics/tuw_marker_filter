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
        
        
    def prediction(self, data):
        print "cmd"
        print data
                
    def define_map(self, m):
        print "map"
        print m
        
    def define_map(self, m):
        print "map"
        print m
        
    def correction(self, z):
        print "marker"
        print z
        
    def loop(self):
        with open(self.filename) as f:
            for line in f:
                elements = line.split(':')
                header = elements[0].strip()                
                if('odom' == header):
                    data = np.array(map(float, elements[1].split(",")))
                    self.PoseArrowOdom.set_pose(data) 
                    plt.pause(0.001)           
                if('cmd' == header):
                    data = np.array(map(float, elements[1].split(",")))
                    self.prediction(data) 
                    
                if('marker' == header):
                    t = np.matrix(map(int,   elements[1].split(",")))
                    z = np.matrix(map(float, elements[2].split(","))).reshape(-1,4)                    
                    self.correction(z)
                if('map' == header):
                    t = np.array(map(int,   elements[1].split(",")))
                    m = np.array(map(float, elements[2].split(","))).reshape(-1,4) 
                    self.define_map(m)
                    
                #print line
                #pause(0.001)
            

if __name__ == '__main__':
    node = EKFLocalization("/home/max/projects/catkin/tuw_marker/src/tuw_marker_filter/tuw_feature_slam/data/log01.txt")
    node.loop()
    print "Good-by"