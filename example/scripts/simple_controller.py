#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos, sin, pi
from geometry_msgs.msg import Point32

class simple_controller:
    def __init__(self):
        rospy.init_node("simple_controller", anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        while not rospy.is_shutdown():
            rospy.spin()

    def laser_callback(self, msg):
        for theta, r in enumerate(msg.ranges):
            print(theta, r)

if __name__ == '__main__':
    try:
        test_track=simple_controller()
    except rospy.ROSInterruptException:
        pass