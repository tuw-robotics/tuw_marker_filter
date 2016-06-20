#!/usr/bin/env python

import rospy
import yaml

from marker_msgs.msg import MarkerWithCovarianceArray

class tuw_marker_saver:
    def __init__(self):
        # get and store parameters
        self.mapfile = rospy.get_param('~mapfile', 'map.yaml')

        if self.mapfile[-5:] != '.yaml':
            self.mapfile = self.mapfile + '.yaml'

        # subscribe
        rospy.Subscriber('map', MarkerWithCovarianceArray, self.cb_map)

    def cb_map(self, msg):
        with open(self.mapfile, 'w') as f:
            yaml.dump(msg.markers, f)

if __name__ == '__main__':
    # initialize ros node
    rospy.init_node('tuw_marker_saver', anonymous=True)

    # create tuw_marker_saver object
    saver = tuw_marker_saver()

    # keep python from exiting until this node is stopped
    rospy.spin()

