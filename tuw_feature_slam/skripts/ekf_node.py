'''
Created on Jul 25, 2016

@author: Markus Bader
'''

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from marker_msgs.msg import MarkerDetection
from marker_msgs.msg import MarkerWithCovarianceArray
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
from tuw.geometry import transform_poses
from tuw.geometry import convert_ros_pose_to_array
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)

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

def normalize_angle(phases):
    return np.arctan2(np.sin(phases), np.cos(phases))


class EKFNode:
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
        self.z = np.zeros((0,3))       # messung im roboter frame
        self.z_map = np.zeros((0,3))   # messung im map frame
        self.m_robot = np.zeros((0,3))   # map in roboter frame
                   
        
    def callbackMarkerMap(self, map):
        self.m = np.zeros((len(map.markers),3))
        self.m_id = np.zeros((len(map.markers)),dtype=np.int)
        for i in range(len(map.markers)):
            marker = map.markers[i].marker
            self.m[i,0:3] = convert_ros_pose_to_array(marker.pose)
            if len(marker.ids) > 0:
                self.m_id[i] = int(marker.ids[0])
            else :
                self.m_id[i] = int(-1)
        
        
    def callbackOdom(self, odom):
        self.odom = convert_ros_pose_to_array(odom.pose.pose) 
        
    def callbackMarker(self, detection):
        self.markers = detection.markers
        #rospy.loginfo("marker")
        header = detection.header        
        self.z = np.zeros((len(detection.markers),3))
        self.s = np.zeros((len(detection.markers)),dtype=np.int)
        for i in range(len(detection.markers)):
            #rospy.loginfo("id: " + str(detection.markers[i].ids))
            self.z[i,0:3] = convert_ros_pose_to_array(detection.markers[i].pose)
            if len(detection.markers[i].ids) > 0:
                self.s[i] = int(detection.markers[i].ids[0])
            else :
                self.s[i] = int(-1)
            #rospy.loginfo("id = " + str(self.s[i]))        
        self.correction_using_landmark()
        
        
        
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
        
        
    def correction_using_landmark(self):
        if hasattr(self, 'm') == False: return
        #rospy.loginfo("correction_using_landmark")   
        
        self.z_map = np.zeros((len(self.z),3))
        transform_poses(self.z, self.xp, self.z_map)
        self.m_robot = np.zeros((len(self.m),3))
        transform_poses(self.m, -self.xp, self.m_robot)
        self.matches = []
        if hasattr(self, 'm'):
            for i in range(len(self.s)):
                #q = self.z[i,0]*self.z[i,0] + self.z[i,1]*self.z[i,1];
                #r = np.sqrt(q);
                #theta = np.arctan2(self.z[i,1],self.z[i,0]);
                for j in range(len(self.m_id)):
                    #q_p = self.m_robot[i,0]*self.m_robot[i,0] + self.m_robot[i,1]*self.m_robot[i,1];
                    #r_p = np.sqrt(q_p);
                    #theta_p = np.arctan2(self.m_robot[i,1],self.m_robot[i,0]);
                    if self.s[i] == self.m_id[j]:
                        if(len(self.matches) == 0): 
                            self.matches = np.array([[i,j]])
                        else:
                            self.matches = np.concatenate(self.matches, np.array([[i,j]])) 
                        #S = 
        
        
        
    def drawOdom(self):
        if hasattr(self, 'PoseArrowOdom') == False:
            self.PoseArrowOdom = PoseArrow(0.4, 'r', 0.4)
            ax.add_artist(self.PoseArrowOdom)
            
        if hasattr(self, 'odom'):
            self.PoseArrowOdom.set_pose(self.odom)  
        
        
    def drawRobot(self):
        if hasattr(self, 'PoseArrowRobot') == False:
            self.PoseArrowRobot = PoseArrow(0.4, 'b', 0.4)
            self.CovEllipseRobot = CovEllipse('b', 0.4)
            ax.add_artist(self.PoseArrowRobot)
            ax.add_artist(self.CovEllipseRobot)
            
        if hasattr(self, 'xp'):
            self.PoseArrowRobot.set_pose(self.xp)  
            if hasattr(self, 'P'):
                self.CovEllipseRobot.set_cov(self.xp, self.P)
            
    def drawMarkerMap(self):
        if hasattr(self, 'LandmarkMap') == False:
            self.LandmarkMap = [ Landmark(0.4, 'g', 0.0) for i in range(10)]
            for i in range(len(self.LandmarkMap)):
                ax.add_artist(self.LandmarkMap[i])
        
        if hasattr(self, 'm'):
            for i in range(len(self.LandmarkMap)):
                if i < len(self.m):
                    self.LandmarkMap[i].set_pose(self.m[i,0:3])
                    self.LandmarkMap[i].set_alpha(0.5)
                    self.LandmarkMap[i].set_text(self.m_id[i])
                else: 
                    self.LandmarkMap[i].set_alpha(0.0)
            
    def drawMarker(self):
        #rospy.loginfo("drawMarker")  
        if hasattr(self, 'LandmarkMarker') == False:
            self.LandmarkMarker = [ Landmark(0.4, 'b', 0.0) for i in range(10)]
            for i in range(len(self.LandmarkMarker)):
                ax.add_artist(self.LandmarkMarker[i])
        
        if hasattr(self, 'z_map'):
            for i in range(len(self.LandmarkMarker)):
                if i < len(self.z_map): 
                    #self.LandmarkMarker[i].set_pose(self.z_map[i,0:3])
                    self.LandmarkMarker[i].set_ralative_pose(self.xp, self.z[i,0:3])
                    self.LandmarkMarker[i].set_alpha(0.5)
                    self.LandmarkMarker[i].set_text(self.s[i])
                else: 
                    self.LandmarkMarker[i].set_alpha(0.0)
    
                
    #def drawMarkerCorresponding(self):
        #rospy.loginfo("drawMarker")  
        #if hasattr(self, 'matches'): 
            
            #for i in range(len(self.matches.shape)):
                #j = row[1]
                #rospy.loginfo("drawMarkerCorresponding " + str(self.matches[i,:]))  
                #self.LandmarkMap[j].set_alpha(1.0)
            
        
    def init_node(self):
        rospy.init_node('EKF-Selflocalization', anonymous=True)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.callbackOdom)
        self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.callbackCmd)
        self.sub_marker = rospy.Subscriber("base_marker_detection", MarkerDetection, self.callbackMarker)
        self.sub_marker_map = rospy.Subscriber("marker_map", MarkerWithCovarianceArray, self.callbackMarkerMap)
        
        
    def loop(self):

        rate = rospy.Rate(5)
        while not rospy.is_shutdown(): 
            self.drawOdom()         
            self.drawRobot()        
            self.drawMarkerMap()   
            self.drawMarker()        
            self.drawMarkerCorresponding()
            plt.draw()
            pause(0.001)
            rate.sleep()
            

if __name__ == '__main__':
    node = EKFNode()
    node.init_node()
    node.loop()
    print "Good-by"