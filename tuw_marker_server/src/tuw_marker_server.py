#!/usr/bin/env python

import rospy
import json
from marker_msgs.msg import MarkerWithCovarianceArray
from utils import MarkerWithCovarianceArraySerializer

class tuw_marker_server:
    def __init__(self):
        # get and store parameters
        self.mapfile = rospy.get_param('~mapfile')
        self.frame_id = rospy.get_param('~frame_id', 'map')

        # prepare message containing the marker map
        self.msg = MarkerWithCovarianceArray()
        self.msg.header.frame_id = self.frame_id
        self.msg.header.seq = 0;
        with open(self.mapfile, 'r') as f:
            mmsg = MarkerWithCovarianceArraySerializer.from_json(json.load(f))
            self.msg.markers = mmsg.markers

        # prepare publisher for the marker map message
        self.pub = rospy.Publisher('map', MarkerWithCovarianceArray, queue_size=10)

if __name__ == '__main__':
    # initialize ros node
    rospy.init_node('tuw_marker_server', anonymous=True)

    # create tuw_marker_server object
    server = tuw_marker_server()

    # publish with a rate of 10 Hz the marker map
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            server.msg.header.seq += 1
            server.msg.header.stamp = rospy.Time.now()
            server.pub.publish(server.msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
