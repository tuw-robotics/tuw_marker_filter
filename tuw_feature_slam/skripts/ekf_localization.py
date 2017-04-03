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
        
    def define_map(self, id, m):
        print m
        
    def correction(self, id, z):
        print z
        
    def loop(self):
        with open(self.filename) as f:
            for line in f:
                data = line.split(':')
                if('odom' == data [0]):
                    data = list(map(float, data[1].split(",")))
                    self.prediction(data)
                if('marker' == data [0]):
                    t = list(map(int, data[1].split(",")[0:2]))
                    id = list(map(int, data[1].split(",")[2:3]))
                    data = list(map(float, data[1].split(",")[4:]))
                    self.correction(id, data)
                if('map' == data [0]):
                    t = list(map(int, data[1].split(",")[0:2]))
                    id = list(map(int, data[1].split(",")[2:3]))
                    data = list(map(float, data[1].split(",")[4:]))
                    self.define_map(id, data)
                    
                #print line
                #pause(0.001)
            

if __name__ == '__main__':
    node = EKFLocalization("/home/max/projects/catkin/tuw_marker/src/tuw_marker_filter/tuw_feature_slam/data/log01.txt")
    node.loop()
    print "Good-by"