#!/usr/bin/env python

import os
import rospy
import tf
import tf2_ros

import math
import numpy

from marker_msgs.msg import MarkerDetection

def get_vector_translation(translation):
    return (translation.x, translation.y, translation.z)

def get_vector_rotation(rotation):
    return (rotation.x, rotation.y, rotation.z, rotation.w)

class tuw_record:
    def __init__(self):
        # use tf2 instead of tf since support of static transformations
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        # get and store parameters
        self.input = rospy.get_param('~input')
        self.output_dir = rospy.get_param('~output_dir')
        self.odom = rospy.get_param('~frame_id_odom', 'odom')
        if self.output_dir[-1:] != '/':
          self.output_dir = self.output_dir + '/'
        self.output = self.output_dir + 'record.csv'

        # subscribe
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
                trans = (float(tmp[1]) + 0.15, float(tmp[2]), float(tmp[3]) + 0.15)
                rot = tf.transformations.quaternion_from_euler(float(tmp[4]) + math.pi/2, float(tmp[5]), float(tmp[6]))
                self.markers[int(tmp[0])] = tf.TransformerROS().fromTranslationRotation(trans, rot)

    def cb_marker(self, msg):
        try:
            # prepare transformation matrix
            stampedTransform = self.tf2_buffer.lookup_transform(msg.header.frame_id, rospy.resolve_name(self.odom)[1:], rospy.Time(), rospy.Duration(1.0))
            T = tf.TransformerROS().fromTranslationRotation(get_vector_translation(stampedTransform.transform.translation), get_vector_rotation(stampedTransform.transform.rotation))

            # prepare output file
            with open(self.output, 'a+') as f:
                if os.path.getsize(self.output) == 0:
                    f.write('exp_length;exp_angle;exp_orientation;length;angle;orientation\n')

                for marker in msg.markers:
                    # calculate prediction
                    F = numpy.dot(T, self.markers[marker.ids[0]])

                    # get cartesian & sperical representation of prediction translation
                    (x_, y_, z_) = tf.transformations.translation_from_matrix(F)
                    l_ = math.sqrt(x_*x_ + z_*z_)
                    a_ = math.atan2(x_, z_)

                    # get euler representation of prediction orientation
                    (roll_, pitch_, yaw_) = tf.transformations.euler_from_matrix(F)

                    # get cartesian & sperical representation of observation translation
                    (x, y, z) = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
                    l = math.sqrt(x*x + z*z)
                    a = math.atan2(x, z)

                    # get euler representation of observation orientation
                    quaternion = (marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w)
                    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

                    # write out transformed pose using euler representation
                    f.write('{};{};{};{};{};{}\n'.format(l_, a_, yaw_, l, a, yaw))

        except tf2_ros.TransformException as ex:
            rospy.logwarn('[%s cb_marker] %s', rospy.get_name(), ex)

if __name__ == '__main__':
    # initialize ros node
    rospy.init_node('tuw_record', anonymous=True)

    # create tuw_record object
    record = tuw_record()

    # keep python from exiting until this node is stopped
    rospy.spin()

