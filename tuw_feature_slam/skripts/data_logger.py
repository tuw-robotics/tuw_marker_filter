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
from tuw.geometry import convert_ros_pose_to_array

class EKFNode:
    '''
    classdocs
    '''
    def __init__(self, *args, **kwargs):
        '''
        Constructor
        '''        
        
           
    def callbackMarkerMap(self, map): 
        return 0        
        
    def callbackOdom(self, odom):  
        #rospy.loginfo("callbackOdom")
        odom = convert_ros_pose_to_array(odom.pose.pose) 
        self.f.write('odom: %10.4f, %10.4f, %10.4f\n' % (odom[0,0], odom[0,1], odom[0,2]))
        
        return 0
        
    def callbackMarker(self, detection):  
        #rospy.loginfo("callbackMarker")  
        for i in range(len(detection.markers)):
            #rospy.loginfo("id: " + str(detection.markers[i].ids))
            m = convert_ros_pose_to_array(detection.markers[i].pose)
            id = -1
            if len(detection.markers[i].ids) > 0:
                id = int(detection.markers[i].ids[0])
            self.f.write('m: %10i, %10i, %3i, %10.4f, %10.4f, %10.4f\n' % (detection.header.stamp.secs, detection.header.stamp.nsecs, id, m[0,0], m[0,1], m[0,2])) 
        return 0
        
        
    def callbackCmd(self, twist): 
        #rospy.loginfo("callbackCmd")
        self.f.write('cmd:  %10.4f, %10.4f\n' % ( twist.linear.x, twist.angular.z))
        return 0
        
    def init_node(self):
        rospy.init_node('logger', anonymous=False)        
        self.t0  = rospy.Time.now()
        filename = rospy.get_param('logfile', '/tmp/log.txt')
        self.f = open(filename, 'w')        
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.callbackOdom)
        self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.callbackCmd)
        self.sub_marker = rospy.Subscriber("base_marker_detection", MarkerDetection, self.callbackMarker)
        self.sub_marker_map = rospy.Subscriber("marker_map", MarkerWithCovarianceArray, self.callbackMarkerMap)
        
    def loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown(): 
            self.f.flush()
            rate.sleep()
            
        self.f.close()
            

if __name__ == '__main__':
    node = EKFNode()
    node.init_node()
    node.loop()
    print "Good-by"