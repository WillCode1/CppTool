#! /usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rf2o_laser_odometry.srv import *
import tf
import chardet

import sys
reload(sys)
sys.setdefaultencoding("utf-8")


class TestService:
    def __init__(self):
        rospy.Subscriber('/front_laser/scan_filtered', LaserScan, self.scan_callback)
        self.pub_pose = rospy.Publisher('/tracked_pose', Odometry, queue_size=10)
        rospy.wait_for_service('/rf2o_laser_odometry/track_odom')
        try:
            self.add_two_ints = rospy.ServiceProxy('/rf2o_laser_odometry/track_odom', track)
        except rospy.ServiceException, e:
            print "Service init failed: %s" % e

    def scan_callback(self, data):
        resp = self.add_two_ints(data)
        self.pub_pose.publish(resp.odom)

if __name__ == '__main__':
    rospy.init_node('test_track')
    obj = TestService()
    rospy.loginfo("node [test_track] started.")
    rospy.spin()
