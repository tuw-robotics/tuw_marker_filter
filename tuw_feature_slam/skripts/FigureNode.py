'''
Created on Jul 25, 2016

@author: Markus Bader
'''

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from marker_msgs.msg import MarkerDetection
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt 
from matplotlib.pyplot import imshow, pause
import warnings
import matplotlib.cbook
from matplotlib.patches import Ellipse
from matplotlib.patches import Circle
from matplotlib.patches import Polygon
from tuw.plot import PoseArrow
from tuw.plot import Landmark
from tuw.plot import CovEllipse
from tuw.geometry import transform
from tuw.geometry import convert_ros_pose_to_array
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)

fig, ax = plt.subplots()
plt.draw()
plt.ion()
ax.axis('equal')
ax.grid(True)
ax.set_xticks(np.arange(-10, 10, 1))
ax.set_yticks(np.arange(-10, 10, 1))
ax.set_xlim([-10,10])
plt.xlabel('x')
ax.set_ylim([-10,10])
plt.ylabel('y')
plt.show()

def normalize_angle(phases):
    return np.arctan2(np.sin(phases), np.cos(phases))


class FigureNode:
    '''
    classdocs
    '''
    def __init__(self, *args, **kwargs):
        '''
        Constructor
        '''
        self.alpha = np.array([0.2, 0.05 , 0.1, 0.02])
        self.xp = np.array([[ 0, 0 , 0]])
        self.P = np.matrix([[ 0.3, 0 , 0],[ 0, 0.3 , 0],[ 0, 0 , 0.1]])
        self.cmd = np.array([[0, 0, 0]])
        
              
        self.PlotOdom = PoseArrow(0.4, 'r', 0.4)        
        self.PlotPoseCov = CovEllipse('b', 0.4)
        self.PlotPose = PoseArrow(0.4, 'b', 1.0)
        self.Landmarks = [ Landmark(0.4, 'r', 0.0) for i in range(10)]
        
        
    def callbackOdom(self, odom):
        self.odom = convert_ros_pose_to_array(odom.pose.pose) 
        #rospy.loginfo("Odom " + np.array_str(self.odom))
        
    def callbackMarker(self, detection):
        self.markers = detection.markers
        #rospy.loginfo("marker")
        header = detection.header
        for i in range(len(self.Landmarks)):
            self.Landmarks[i].set_alpha(0.0)
        for i in range(len(detection.markers)):
            #rospy.loginfo("id: %s" + str(landmark.ids))
            y = convert_ros_pose_to_array(detection.markers[i].pose)
            yp = np.copy(y)
            transform(y, self.xp, yp)
            self.Landmarks[i].set_pose(yp)
            self.Landmarks[i].set_alpha(0.6)
            
            
            
        
        
    def callbackCmd(self, twist):
        if hasattr(self, 'last_callbackCmd'):
            now = rospy.Time.now() 
            duration =  now - self.last_callbackCmd
            self.last_callbackCmd = now
        else:
            if hasattr(self, 'odom'):
                self.xp = np.copy(self.odom)
                self.last_callbackCmd = rospy.Time.now()
            return
        
        dt = duration.to_sec()     
        if dt > 0.2 :
            rospy.loginfo("timing error: %s" + dt)
            return

        self.cmd = np.array([[dt, twist.linear.x, twist.angular.z]])
        self.predict_pose()
        
    def predict_pose(self):
        dt = self.cmd[0,0]
        v = self.cmd[0,1]
        w = self.cmd[0,2]
        x = self.xp[0,0]
        y = self.xp[0,1]
        theta = self.xp[0,2]
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
        
        self.xp = self.xp + np.array([[ dx, dy , da]])
        P_last = np.copy(self.P)
        self.P = G * P_last * G.transpose() + V * M * V.transpose() 
        
        
    def predict_landmark(self, landmark):
        x = self.xp[0,0]
        y = self.xp[0,1]
        theta = self.xp[0,2]
        
        #ax.scatter(self.odom[0,0], self.odom[0,1], c='r', alpha=0.5)
        
    def init_node(self):
        rospy.init_node('DiffControl', anonymous=True)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.callbackOdom)
        self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.callbackCmd)
        self.sub_marker = rospy.Subscriber("base_marker_detection", MarkerDetection, self.callbackMarker)
        
        
    def loop(self):

        rate = rospy.Rate(10) # 10hz
        ax.add_artist(self.PlotPoseCov)
        ax.add_artist(self.PlotPose)
        ax.add_artist(self.PlotOdom)
        for i in range(len(self.Landmarks)):
            ax.add_artist(self.Landmarks[i])
        while not rospy.is_shutdown():
            
            if hasattr(self, 'odom'):
                self.PlotOdom.set_pose(self.odom)                
            
            if True:                                        
                self.PlotPose.set_pose(self.xp)
                self.PlotPoseCov.set_cov(self.xp, self.P)
            
            plt.draw()
            pause(0.01)
            rate.sleep()
            

if __name__ == '__main__':
    node = FigureNode()
    node.init_node()
    node.loop()
    print "Good-by"