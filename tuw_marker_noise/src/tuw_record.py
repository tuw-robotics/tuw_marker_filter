#!/usr/bin/env python

import os
import rospy
import tf

import math
import numpy

from marker_msgs.msg import MarkerDetection
from geometry_msgs.msg import PoseStamped

def get_vector_position(position):
    return (position.x, position.y, position.z)

def get_vector_orientation(orientation):
    return (orientation.x, orientation.y, orientation.z, orientation.w)

class tuw_record:
    def __init__(self):
        # get and store parameters
        self.input = rospy.get_param('~input')
        self.output_dir = rospy.get_param('~output_dir')
        self.odom = rospy.get_param('~frame_id_odom', 'odom')
        if self.output_dir[-1:] != '/':
          self.output_dir = self.output_dir + '/'
        self.output = self.output_dir + 'record.csv'

        # listen to transformations and subscribe to topics
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber('markers', MarkerDetection, self.cb_marker)

        self.markers = {}
        with open(self.input, 'r') as f:
            for line in f:
                if line == 'id;x;y;z;roll;pitch;yaw\n':
                    continue

                tmp = line.split(';')
                if len(tmp) != 7:
                    raise ValueError('Not a valid line: ' + line)

                # Note: + 0.15 and + math.pi/2 is because of the gazebo link offset
                msg = PoseStamped()
                msg.header.frame_id = rospy.resolve_name(self.odom)[1:]
                msg.pose.position.x = float(tmp[1]) + 0.15
                msg.pose.position.y = float(tmp[2])
                msg.pose.position.z = float(tmp[3]) + 0.15
                q = tf.transformations.quaternion_from_euler(float(tmp[4]) + math.pi/2, float(tmp[5]), float(tmp[6]))
                msg.pose.orientation.x = q[0]
                msg.pose.orientation.y = q[1]
                msg.pose.orientation.z = q[2]
                msg.pose.orientation.w = q[3]
                self.markers[int(tmp[0])] = msg

    def cb_marker(self, msg):
        try:
            # prepare output file
            with open(self.output, 'a+') as f:
                if os.path.getsize(self.output) == 0:
                    f.write('exp_length;exp_angle;exp_orientation;length;angle;orientation\n')

                for marker in msg.markers:
                    # get prediction
                    predmsg = self.tf_listener.transformPose(msg.header.frame_id, self.markers[marker.ids[0]])

                    # get cartesian & sperical representation of prediction translation
                    (x_, y_, z_) = (predmsg.pose.position.x, predmsg.pose.position.y, predmsg.pose.position.z)
                    l_ = math.sqrt(x_*x_ + z_*z_)
                    a_ = math.atan2(x_, z_)

                    # get euler representation of prediction orientation
                    (roll_, pitch_, yaw_) = tf.transformations.euler_from_quaternion(get_vector_orientation(predmsg.pose.orientation))

                    # get cartesian & sperical representation of observation translation
                    (x, y, z) = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
                    l = math.sqrt(x*x + z*z)
                    a = math.atan2(x, z)

                    # get euler representation of observation orientation
                    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(get_vector_orientation(marker.pose.orientation))

                    # write out transformed pose using euler representation
                    f.write('{};{};{};{};{};{}\n'.format(l_, a_, pitch_, l, a, pitch))

        except tf.Exception as ex:
            rospy.logwarn('[%s cb_marker] %s', rospy.get_name(), ex)

if __name__ == '__main__':
    # initialize ros node
    rospy.init_node('tuw_record', anonymous=True)

    # create tuw_record object
    record = tuw_record()

    # keep python from exiting until this node is stopped
    rospy.spin()

