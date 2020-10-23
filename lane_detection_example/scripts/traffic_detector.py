#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


class TRAFFICDetector:

    def __init__(self):
        self.image_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_callback)

        self.traffic_pub = rospy.Publisher('/traffic_light', String, queue_size=1)

        self.traffic_msg = String()
        self.img_hsv = None

    def image_callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    def detect_signal(self):
        h = self.img_hsv.shape[0]
        w = self.img_hsv.shape[1]

        # hsv이미지에 적용할 h, s, v 값의 최대 최소값을 리스트로 저장
        upper_sig_red = np.array([ 20, 255, 255])
        lower_sig_red = np.array([  0,  20, 251])

        upper_sig_yellow = np.array([ 35, 255, 255])
        lower_sig_yellow = np.array([ 20,  20, 251])

        upper_sig_green = np.array([ 90, 255, 255])
        lower_sig_green = np.array([ 60,  20, 251])

        # hsv이미지를 h,s,v 값의 범위를 조절하여 신호등의 각 색깔별로 인식하게 변환. 
        img_red = cv2.inRange(self.img_hsv, lower_sig_red, upper_sig_red)
        img_yellow = cv2.inRange(self.img_hsv, lower_sig_yellow, upper_sig_yellow)
        img_green = cv2.inRange(self.img_hsv, lower_sig_green, upper_sig_green)

        # 신호등 이미지에 차선이 출력될 수 있으므로 이미지의 세로 밑에서 부터 2/3을 0(검정색)으로 바꿈
        # img_red[int(h/2):, :] = 0
        img_yellow[int(h/2.5):, :] = 0
        # img_green[int(h/2):, :] = 0

        # countNonZero(이미지)함수를 이용하여 이미지 안의 0이 아닌 값을 가진 픽셀 수를 카운트한다.
        pix_red = cv2.countNonZero(img_red)
        pix_yellow = cv2.countNonZero(img_yellow)
        pix_green = cv2.countNonZero(img_green)

        # 각 이미지의 픽셀 수 중 최대값을 pix_max로 넣는다.
        pix_max = np.max([pix_red, pix_yellow, pix_green])
        # 각 이미지의 픽셀 수 중 최대값을 가진 이미지의 인덱스(0:RED, 1:YELLOW, 2:GREEN)를 max_index로 넣는다.
        max_index = np.argmax([pix_red, pix_yellow, pix_green])

        # 어떤 이미지든지 최대 픽셀 수가 40개보다 많으면 max_index의 값에 따라 메세지의 내용을 입력한다.
        if pix_max >= 20:
            
            if max_index == 0:
                self.traffic_msg.data = "RED"
            elif max_index == 1:
                self.traffic_msg.data = "YELLOW"
            elif max_index == 2:
                self.traffic_msg.data = "GREEN"
        else: 
            # 최대 픽셀 수가 40개보다 적으면 신호등을 찾지 못한 것으로 메세지의 내용을 입력한다.
            self.traffic_msg.data = "NONE"

        print('max_pixel_value', pix_max, 'light', self.traffic_msg.data)

        # resize는 3개의 이미지를 붙여서 출력하면 너무 창이 커져서 이미지 사이즈를 줄이기 위해 사용
        # img_red = cv2.resize(img_red, (w/2, h/2))
        # img_yellow = cv2.resize(img_yellow, (w/2, h/2))
        # img_green = cv2.resize(img_green, (w/2, h/2))

        # img_concat = np.concatenate([img_red, img_yellow, img_green], axis=1)
        
        # cv2.imshow("Signal Window", img_concat)
        # cv2.waitKey(1)

    def pub_signal(self):
        # 인식한 신호등의 불빛을 메세지로 보낸다. topic: traffic_light
        self.traffic_pub.publish(self.traffic_msg)

    
if __name__ == '__main__':
    rospy.init_node('traffic_detector', anonymous=True)
    traffic_detector = TRAFFICDetector()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if traffic_detector.img_hsv is not None:
            traffic_detector.detect_signal()
            traffic_detector.pub_signal()
            rate.sleep()
