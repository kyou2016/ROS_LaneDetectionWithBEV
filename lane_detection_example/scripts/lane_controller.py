#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import os, rospkg
import numpy as np
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from std_msgs.msg import Float64

from utils import purePursuit


class LINEController:

    def __init__(self):
	self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
	self.speed_value = 1000

    def pub_speed(self):
	self.speed_pub.publish(self.speed_value)


if __name__ == '__main__':
    rospy.init_node('line_controller', anonymous=True)

    ctrl_wecar = LINEController()

    ctrl_onlane = purePursuit(look_forward_distance=1.0)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

	if ctrl_onlane.lpath is not None:
	    ctrl_onlane.steering_angle()

	    ctrl_onlane.pub_cmd()
	    ctrl_wecar.pub_speed()

	    rate.sleep()
