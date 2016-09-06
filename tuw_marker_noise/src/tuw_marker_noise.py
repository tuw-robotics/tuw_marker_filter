#!/usr/bin/env python

import rospy
import tf
import numpy
import math
import matplotlib.pyplot as mplot

from mpl_toolkits.mplot3d import Axes3D
from marker_msgs.msg import MarkerDetection

class tuw_marker_noise:
    def __init__(self, fig):
        # get and store parameters
        self.fig = fig
        self.plot_data = rospy.get_param('~plot_data')
        self.beta_1 = rospy.get_param('~beta_1')
        self.beta_2 = rospy.get_param('~beta_2')
        self.beta_3 = rospy.get_param('~beta_3')
        self.beta_4 = rospy.get_param('~beta_4')
        self.beta_5 = rospy.get_param('~beta_5')
        self.beta_6 = rospy.get_param('~beta_6')
        self.beta_7 = rospy.get_param('~beta_7')
        self.beta_8 = rospy.get_param('~beta_8')
        self.beta_9 = rospy.get_param('~beta_9')
        self.beta_10 = rospy.get_param('~beta_10')
        self.beta_11 = rospy.get_param('~beta_11')
        self.beta_12 = rospy.get_param('~beta_12')
        self.beta_13 = rospy.get_param('~beta_13')
        self.beta_14 = rospy.get_param('~beta_14')
        self.beta_15 = rospy.get_param('~beta_15')
        self.beta_16 = rospy.get_param('~beta_16')
        self.beta_17 = rospy.get_param('~beta_17')
        self.beta_18 = rospy.get_param('~beta_18')

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
            radial = math.sqrt(p_o[0]**2 + p_o[1]**2 + p_o[2]**2)
            polar = math.acos(p_o[2]/radial)
            azimuthal = math.atan2(p_o[1], p_o[0])
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q_o)

            # calculate noise
            sigma_radial =    math.sqrt(max(self.beta_1  * radial**2 + self.beta_2,  0) + max(min(self.beta_3  * azimuthal**2 + self.beta_4,  math.pi**2), 0) + max(min(self.beta_5  * yaw**2 + self.beta_6,  math.pi**2), 0))
            sigma_azimuthal = min(math.sqrt(max(self.beta_7  / radial**2 + self.beta_8,  0) + max(min(self.beta_9  * azimuthal**2 + self.beta_10, math.pi**2), 0) + max(min(self.beta_11 * yaw**2 + self.beta_12, math.pi**2), 0)), math.pi) 
            sigma_yaw =       min(math.sqrt(max(self.beta_13 * radial**2 + self.beta_14, 0) + max(min(self.beta_15 * azimuthal**2 + self.beta_16, math.pi**2), 0) + max(min(self.beta_17 * yaw**2 + self.beta_18, math.pi**2), 0)), math.pi)

            # add gaussian noise in spherical coordinate system
            radial += numpy.random.normal(0, sigma_radial)
            azimuthal += numpy.random.normal(0, sigma_azimuthal)
            yaw += numpy.random.normal(0, sigma_yaw)

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
                if len(marker.ids) > 0:
                    id = marker.ids[0]
                else:
                    id = '{}'
                self.ax.text(p_o[0], p_o[1], p_o[2], id, None)
                self.ax.quiver(p_o[0], p_o[1], p_o[2], o_o[0], o_o[1], o_o[2])
                self.ax.quiver(p_n[0], p_n[1], p_n[2], o_n[0], o_n[1], o_n[2])

        # publish and draw (noisy) markers
        self.pub.publish(noisy_data)
        if self.plot_data:
            self.fig.canvas.draw()

if __name__ == '__main__':
    # initialize ros node
    rospy.init_node('tuw_marker_noise', anonymous=True)

    # get parameters
    plot_data = rospy.get_param('~plot_data')

    # create tuw_marker_noise object
    fig = mplot.figure()
    noise = tuw_marker_noise(fig)

    # keep python from exiting until this node is stopped
    if plot_data:
        mplot.show()
    else:
        rospy.spin()

