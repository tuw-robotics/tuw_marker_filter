'''
Created on Jul 25, 2016

@author: Markus Bader
'''

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
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

class FigureNode:
    '''
    classdocs
    '''
    def __init__(self, *args, **kwargs):
        '''
        Constructor
        '''
        self.start_offset = 5;
        self.xp = np.array([[ 0, 0 , 0]])
        self.P = np.matrix([[ 0, 0 , 0],[ 0, 0 , 0],[ 0, 0 , 0]])
        self.odom = np.array([[ 0, 0 , 0]])
        self.cmd = np.array([[ 0, 0]])
        
    def callbackOdom(self, odom):
        q = odom.pose.pose.orientation;
        euler = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.odom = np.array([[ odom.pose.pose.position.x, odom.pose.pose.position.y ,euler[2]]]);
        #rospy.loginfo("Odom " + np.array_str(self.odom))
        
    def callbackCmd(self, twist):
        if hasattr(self, 'cmd_last'):
            now = rospy.Time.now() 
            duration =  now - self.cmd_last
            self.cmd_last = now
        else:
            self.xp = np.copy(self.odom)
            self.cmd_last = rospy.Time.now()
            return
        
        dt = duration.to_sec()     
        if dt > 0.2 :
            rospy.loginfo("timing error: " + dt)
            return

        v = twist.linear.x
        w = twist.angular.z        
        self.cmd = np.array([[v, w]])
        x = self.xp[0,0]
        y = self.xp[0,1]
        theta = self.xp[0,2]
        G = np.matrix( [[0, 0, 0], [0, 0, 0], [0, 0, 0]]);
        V = np.matrix( [[0, 0], [0, 0]]);
        if(np.fabs(w) > 0.01):
            r = v/w;
            dx = -r * np.sin(theta) + r * np.sin(theta + w * dt)
            dy = +r * np.cos(theta) - r * np.cos(theta + w * dt)
            da = w * dt
        else:
            dx = v * np.sin(theta) * dt
            dy = v * np.cos(theta) * dt
            da = 0.0
        
        self.xp = self.xp + np.array([[ dx, dy , da]])
        self.xp [0,2] = ( self.xp [0,2] + np.pi) % (2 * np.pi ) - np.pi
        
        
        
        
        #rospy.loginfo("cmd: " + np.array_str(self.cmd))
        #rospy.loginfo("odom: " + np.array_str(self.odom))
        #rospy.loginfo("xp: " + np.array_str(self.xp))
        
        
    def init_node(self):
        rospy.init_node('DiffControl', anonymous=True)

        rospy.Subscriber("odom", Odometry, self.callbackOdom)
        rospy.Subscriber("cmd_vel", Twist, self.callbackCmd)
        
        
    def loop(self):

        rate = rospy.Rate(5) # 10hz
        while not rospy.is_shutdown():
            
            ax.scatter(self.odom[0,0], self.odom[0,1], c='r', alpha=0.5)
            ax.scatter(self.xp[0,0], self.xp[0,1], c='b', alpha=0.5)
            plt.draw()
            pause(0.01)
            rate.sleep()
            

if __name__ == '__main__':
    node = FigureNode()
    node.init_node()
    node.loop()
    print "Good-by"