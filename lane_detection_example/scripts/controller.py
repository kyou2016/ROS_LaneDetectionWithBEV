#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import os, rospkg
import numpy as np
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import purePursuit, PID_longitudinal


class Controller:

    def __init__(self):
        self.image_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_Callback)
        self.img_wlane = None

    def image_Callback(self, msg):
        
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # 주황색 차선, 흰색 차선 모두 검출되는 값 -> 실제 사용시 다른 값이 검출될 수 있음
        upper_lane = np.array([37, 255, 255])
        lower_lane = np.array([ 0,   0, 250])

        self.img_wlane = cv2.inRange(img_hsv, lower_lane, upper_lane)
        # self.img_wlane = cv2.cvtColor(self.img_wlane, cv2.COLOR_GRAY2BGR)

if __name__ == '__main__':
    rospy.init_node('wecar_controller', anonymous=True)

    ctrl_wecar = Controller()

    ctrl_onlane = purePursuit(look_forward_distance=1.0)
    ctrl_speed = PID_longitudinal()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        
        if ctrl_wecar.img_wlane is not None:
            ctrl_speed.calc_vel_cmd()
            ctrl_onlane.steering_angle()

            ctrl_speed.pub_speed_cmd()
            ctrl_onlane.pub_cmd()

            rate.sleep()