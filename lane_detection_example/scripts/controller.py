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
        self.img =None
        self.set_cam(1)

    def set_cam(self,_index):
        self.cam =cv2.VideoCapture(int(_index))

    def get_image(self):
        ret, img =self.cam.read()
        return ret , img 

    def get_bi_img(self):

        ret,img_bgr =self.get_image()

        img_hsv=cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)

        lower_wlane=np.array([0,0,220])
        upper_wlane=np.array([40,10,255])
        
        img_wlane=cv2.inRange(img_hsv,lower_wlane,upper_wlane)

        return img_wlane

        

if __name__ == '__main__':
    rospy.init_node('wecar_controller', anonymous=True)

    ctrl_wecar = Controller()

    ctrl_onlane = purePursuit(look_forward_distance=0.85)
    ctrl_speed = PID_longitudinal(speed_max=2000)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

	img = ctrl_wecar.get_bi_img()
        
        if img is not None:
            ctrl_speed.calc_vel_cmd()
            ctrl_onlane.steering_angle()

            ctrl_speed.pub_speed_cmd()
            if ctrl_onlane.is_path == True:
                ctrl_onlane.pub_cmd()

            rate.sleep()
