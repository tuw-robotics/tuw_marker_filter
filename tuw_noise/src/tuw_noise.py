#!/usr/bin/env python

import rospy
import tf
import numpy
import math
import matplotlib.pyplot as mplot

from mpl_toolkits.mplot3d import Axes3D
from marker_msgs.msg import MarkerDetection

class tuw_noise:
    def __init__(self, fig):
        # get and store parameters
        self.fig = fig
        self.sigma_radial = rospy.get_param('~sigma_radial')
        self.sigma_polar = rospy.get_param('~sigma_polar')
        self.sigma_azimuthal = rospy.get_param('~sigma_azimuthal')
        self.sigma_roll = rospy.get_param('~sigma_roll')
        self.sigma_pitch = rospy.get_param('~sigma_pitch')
        self.sigma_yaw = rospy.get_param('~sigma_yaw')
        self.plot_data = rospy.get_param('~plot_data')

        # initialize axes
        if self.plot_data:
            self.ax = fig.add_subplot(111, projection='3d')
            self.ax.set_title("Detected markers")
            self.ax.set_xlim(-8, 8, False, False)
            self.ax.set_ylim(-8, 8, False, False)
            self.ax.set_zlim( 0, 8, False, False)

        # subscribe and advertise
        rospy.Subscriber('marker', MarkerDetection, self.cb_marker)
        self.pub = rospy.Publisher('marker_noise', MarkerDetection, queue_size=10)

    def cb_marker(self, data):
        #  copy found markers and set standard deviation of noise
        noisy_data = data

        # renew axes
        if self.plot_data:
            self.ax.clear()
            #self.ax.set_autoscale_on(False)
            #self.ax.set_autoscalez_on(False)
            self.ax.autoscale(False)
            self.ax.set_xlabel("X axis")
            self.ax.set_ylabel("Y axis")
            self.ax.set_zlabel("Z axis")

        # add and visualize gaussian noise
        for marker in noisy_data.markers:
            # store original pose in cartesian coordinate system 
            p_o = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
            q_o = (marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w)

            # calculate original pose in spherical coordinate system
            radial = math.sqrt(p_o[0]*p_o[0] + p_o[1]*p_o[1] + p_o[2]*p_o[2])
            polar = math.acos(p_o[2]/radial)
            azimuthal = math.atan2(p_o[1], p_o[0])
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q_o)

            # add gaussian noise in spherical coordinate system
            if self.sigma_radial > 0:
                radial += numpy.random.normal(0,self.sigma_radial)
            if self.sigma_polar > 0:
                polar += numpy.random.normal(0, self.sigma_polar)
            if self.sigma_azimuthal > 0:
                azimuthal += numpy.random.normal(0, self.sigma_azimuthal)
            if self.sigma_roll > 0:
                roll += numpy.random.normal(0, self.sigma_roll)
            if self.sigma_pitch > 0:
                pitch += numpy.random.normal(0, self.sigma_pitch)
            if self.sigma_yaw > 0:
                yaw += numpy.random.normal(0, self.sigma_yaw)

            # calculate noisy pose in cartesian coordinate system
            tmp = math.fabs(radial * math.sin(polar))
            marker.pose.position.x = tmp * math.cos(azimuthal)
            marker.pose.position.y = tmp * math.sin(azimuthal)
            marker.pose.position.z = radial * math.cos(polar)
            tmp = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            marker.pose.orientation.x = tmp[0]
            marker.pose.orientation.y = tmp[1]
            marker.pose.orientation.z = tmp[2]
            marker.pose.orientation.w = tmp[3]

            # store noise pose in spherical coordinate system 
            p_n = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
            q_n = (marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w)

            if self.plot_data:
                # vector indicating the orientation
                o = numpy.array((1,1,1,1))

                # transform orientation vector accordingly
                R_o = tf.transformations.quaternion_matrix(q_o)
                R_n = tf.transformations.quaternion_matrix(q_n)
                o_o = numpy.dot(R_o, o)
                o_n = numpy.dot(R_n, o)

                # plot ids and arrows indicating the pose of original and noisy marker
                self.ax.text(p_o[0], p_o[1], p_o[2], marker.id, None)
                self.ax.quiver(p_o[0], p_o[1], p_o[2], o_o[0], o_o[1], o_o[2])
                self.ax.quiver(p_n[0], p_n[1], p_n[2], o_n[0], o_n[1], o_n[2])

        # publish and draw (noisy) markers
        self.pub.publish(noisy_data)
        if self.plot_data:
            self.fig.canvas.draw()

if __name__ == '__main__':
    # initialize ros node
    rospy.init_node('tuw_noise', anonymous=True)

    # get parameters
    plot_data = rospy.get_param('~plot_data')

    # create tuw_noise object
    fig = mplot.figure()
    noise = tuw_noise(fig)

    # keep python from exiting until this node is stopped
    if plot_data:
        mplot.show()
    else:
        rospy.spin()

