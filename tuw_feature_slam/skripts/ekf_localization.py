'''
Created on Jul 25, 2016

@author: Markus Bader
'''

import rospy
import tf
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
        print data
        self.PoseArrowRobot.set_pose(data)  
        pause(0.001)
        
    def define_map(self, m):
        print m
        
    def correction(self, z):
        print z
        
    def loop(self):
        with open(self.filename) as f:
            for line in f:
                data = line.split(':')
                header = data[0].strip()                
                if('odom' == header):
                    data = list(map(float, data[1].split(",")))
                    self.PoseArrowOdom.set_pose(data) 
                    plt.draw()
                    pause(0.001)
                if('marker' == header):
                    t = list(map(int, data[1].split(",")))
                    z = list(map(float, data[2].split(",")))
                    self.correction(id, z)
                if('map' == header):
                    t = list(map(int, data[1].split(",")[0:2]))
                    m = list(map(float, data[2].split(",")))
                    self.define_map(id, m)
                    
                #print line
                #pause(0.001)
            

if __name__ == '__main__':
    node = EKFLocalization("/home/max/projects/catkin/tuw_marker/src/tuw_marker_filter/tuw_feature_slam/data/log01.txt")
    node.loop()
    print "Good-by"