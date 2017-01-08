'''
Created on Jan 8, 2017

@author: max
'''

import numpy as np
import tf

def transform_poses(src, tf, des):
    if len(tf.shape) == 1:
        dx = tf[0]
        dy = tf[1]
        dtheta = tf[2]
    else :   
        dx = tf[0,0]
        dy = tf[0,1]
        dtheta = tf[0,2]
    s = np.sin(dtheta);
    c = np.cos(dtheta);
    for i in range(len(src)):
        des[i,0] =  c * src[i,0] + -s * src[i,1] + dx
        des[i,1] = s * src[i,0] + c * src[i,1] + dy
        des[i,2] = src[i,2] + dtheta
        
def transform_points(src, tf, des):
    if len(tf.shape) == 1:
        dx = tf[0]
        dy = tf[1]
        dtheta = tf[2]
    else :   
        dx = tf[0,0]
        dy = tf[0,1]
        dtheta = tf[0,2]
    s = np.sin(dtheta);
    c = np.cos(dtheta);
    for i in range(len(src)):
        des[i,0] =  c * src[i,0] + -s * src[i,1] + dx
        des[i,1] = s * src[i,0] + c * src[i,1] + dy
    
def convert_ros_pose_to_array(src):
    q = src.orientation;
    p = src.position;
    euler = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
    return np.array([[ p.x, p.y , euler[2]]]);