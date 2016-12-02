'''
Created on Jul 25, 2016

@author: Markus Bader
'''

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from marker_msgs.msg import Marker
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt 
from matplotlib.pyplot import imshow, pause
import warnings
import matplotlib.cbook
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)

fig, ax = plt.subplots()
plt.draw()
plt.ion()
ax.axis('equal')
ax.grid(True)
ax.set_xticks([-10,-5,0,5,10])
ax.set_yticks([-10,-5,0,5,10])
ax.set_xlim([-10,10])
ax.set_ylim([-10,10])
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
        self.xp = np.array([[ 0, 0 , 0]])
        self.P = np.matrix([[ 0, 0 , 0],[ 0, 0 , 0],[ 0, 0 , 0]])
        self.cmd = np.array([[ 0, 0]])
        
    def callbackOdom(self, odom):
        q = odom.pose.pose.orientation;
        euler = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.odom = np.array([[ odom.pose.pose.position.x, odom.pose.pose.position.y ,euler[2]]]);
        #rospy.loginfo("Odom " + np.array_str(self.odom))
        
    def callbackMarker(self, marker):
        self.marker = marker
        rospy.loginfo("marker")
        
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

        self.cmd = np.array([[twist.linear.x, twist.angular.z]])
        v = self.cmd[0,0]
        w = self.cmd[0,1]
        x = self.xp[0,0]
        y = self.xp[0,1]
        theta = self.xp[0,2]
        G = np.matrix( [[0, 0, 0], [0, 0, 0], [0, 0, 0]]);
        V = np.matrix( [[0, 0], [0, 0]]);
        if(np.fabs(w) > 0.0):
            r = v/w;
            dx = -r * np.sin(theta) + r * np.sin(theta + w * dt)
            dy = +r * np.cos(theta) - r * np.cos(theta + w * dt)
            da = w * dt
        else:
            dx = v * np.cos(theta) * dt
            dy = v * np.sin(theta) * dt
            da = 0.0
        
        self.xp = self.xp + np.array([[ dx, dy , da]])
        normalize_angle(self.xp [0,2])
        
        
        
    def init_node(self):
        rospy.init_node('DiffControl', anonymous=True)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.callbackOdom)
        self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.callbackCmd)
        self.sub_marker = rospy.Subscriber("base_marker_detection", Marker, self.callbackMarker)
        
        
    def loop(self):

        rate = rospy.Rate(5) # 10hz
        while not rospy.is_shutdown():
            
            if hasattr(self, 'odom'):
                ax.scatter(self.odom[0,0], self.odom[0,1], c='r', alpha=0.5)
            ax.scatter(self.xp[0,0], self.xp[0,1], c='b', alpha=0.5)
            
            if hasattr(self, 'marker'):
                for i in range(0, len(self.marker.markers)):
                    print i 
            plt.draw()
            pause(0.01)
            rate.sleep()
            

if __name__ == '__main__':
    node = FigureNode()
    node.init_node()
    node.loop()
    print "Good-by"