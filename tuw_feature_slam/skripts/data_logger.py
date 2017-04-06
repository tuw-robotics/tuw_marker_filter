'''
Created on Jul 25, 2016

@author: Markus Bader
'''

import rospy
import tf
import numpy as np 
import threading, time, random  
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
        self.my_mutex = threading.Lock()
           
    def callbackMarkerMap(self, map): 
        self.my_mutex.acquire()
        #rospy.loginfo("callbackMarkerMap")
        str = '{:<9}:{:>10d}, {:>10d}:'.format('map', map.header.stamp.secs, map.header.stamp.nsecs)
        self.f.write(str)
        for i in range(len(map.markers)):
            marker = map.markers[i].marker
            marker = map.markers[i].marker
            m = convert_ros_pose_to_array(marker.pose)
            id = int(marker.ids[0]) 
            str = '{:>10d}, {: f}, {: f}, {: f}'.format(id, m[0,0], m[0,1], m[0,2])
            #self.f.write('%3i, %10.4f, %10.4f, %10.4f\n' % (id, m[0,0], m[0,1], m[0,2]))
            if i < (len(map.markers) - 1): 
                self.f.write('%s;' %(str))
            else:
                self.f.write('%s\n' %(str))
                
        self.my_mutex.release()
        return 0        
    def callbackMarker(self, detection): 
        self.my_mutex.acquire() 
        #rospy.loginfo("callbackMarker")  
        str = '{:<9}:{:>10d}, {:>10d}:'.format('marker', detection.header.stamp.secs, detection.header.stamp.nsecs)
        self.f.write(str)
        for i in range(len(detection.markers)):
            #rospy.loginfo("id: " + str(detection.markers[i].ids))
            z = convert_ros_pose_to_array(detection.markers[i].pose)
            id = -1
            if len(detection.markers[i].ids) > 0:
                id = int(detection.markers[i].ids[0])
            str = '{:>10d}, {: f}, {: f}, {: f}'.format(id, z[0,0], z[0,1], z[0,2])
            if i < (len(detection.markers) - 1): 
                self.f.write('%s;' %(str))
            else:
                self.f.write('%s\n' %(str))
        self.my_mutex.release()
        return 0
        
    def callbackOdom(self, odom):  
        self.my_mutex.acquire()
        #rospy.loginfo("callbackOdom")
        pose = convert_ros_pose_to_array(odom.pose.pose) 
        str = '{:<9}: {: f}, {: f}, {: f}\n'.format('odom', pose[0,0], pose[0,1], pose[0,2])
        self.f.write(str)        
        self.my_mutex.release()
        return 0
    
    def callbackGroundTruth(self, odom):  
        self.my_mutex.acquire()
        #rospy.loginfo("callbackOdom")
        pose = convert_ros_pose_to_array(odom.pose.pose) 
        str = '{:<9}: {: f}, {: f}, {: f}\n'.format('true_pose', pose[0,0], pose[0,1], pose[0,2])
        self.f.write(str)        
        self.my_mutex.release()
        return 0
    
    def callbackCmd(self, twist): 
        self.my_mutex.acquire()
        #rospy.loginfo("callbackCmd")
        str = '{:<9}: {: f}, {: f}\n'.format('cmd', twist.linear.x, twist.angular.z)
        self.f.write(str)     
        self.my_mutex.release()   
        return 0
        
        
        
        
    def init_node(self):
        rospy.init_node('logger', anonymous=False)        
        self.t0  = rospy.Time.now()
        filename = rospy.get_param('logfile', '/tmp/log.txt')
        self.f = open(filename, 'w')        
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.callbackOdom)
        self.sub_true_pose = rospy.Subscriber("base_pose_ground_truth", Odometry, self.callbackGroundTruth)
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