'''
Created on Jul 25, 2016

@author: Markus Bader
'''

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry


class DemoNode:
    '''
    classdocs
    '''
    def __init__(self, *args, **kwargs):
        '''
        Constructor
        '''
        
    def callbackOdom(self, odom):
        q = odom.pose.pose.orientation;
        euler = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.x = np.array([[ odom.pose.pose.position.x, odom.pose.pose.position.y ,euler[2]]]);
        rospy.loginfo(rospy.get_caller_id() + "Odom  %s" + np.array_str(self.x))
        
        
    def init_node(self):
        rospy.init_node('DiffControl', anonymous=True)

        rospy.Subscriber("odom", Odometry, self.callbackOdom)
        
        
    def loop(self):

        # spin() simply keeps python from exiting until this node is stopped
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #rospy.loginfo(rospy.get_caller_id() + "loop")  
            rate.sleep()
            

if __name__ == '__main__':
    node = DemoNode()
    node.init_node()
    node.loop()
    print "Good-by"