#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import os, rospkg
import numpy as np
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import STOPLineEstimator, BEVTransform


class STOPLineDetector:

    def __init__(self):
        self.image_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_Callback)
        self.image_sline = None

    def image_Callback(self, msg):

        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # 주황색 차선, 흰색 차선 모두 검출되는 값 -> 실제 사용시 다른 값이 검출될 수 있음
        upper_sline = np.array([37, 255, 255])
        lower_sline = np.array([ 0,   0, 250])

        self.image_sline = cv2.inRange(img_hsv, lower_sline, upper_sline)
        # self.image_sline = cv2.cvtColor(self.image_sline, cv2.COLOR_GRAY2BGR)


if __name__ == '__main__':
    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    # WeCar의 카메라의 파라미터 값을 적어놓은 json파일을 읽어와서 params_cam으로 저장
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('StopLine_Detector', anonymous=True)

    # WeCar의 카메라 이미지에서 차선에 관한 포인트들을 추출하기 위해 클래스를 선언
    bev_trans = BEVTransform(params_cam=params_cam)
    # 카메라 이미지에서 추출한 포인트로 차선이 정지선인지 판단 및 정지선까지의 거리를 계산하는 클래스를 선언
    stopline_estimator = STOPLineEstimator()

    stopline_detector = STOPLineDetector()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if stopline_detector.image_sline is not None:
            
            # 카메라 이미지에서 차선에 관한 포인트들을 추출
            lane_pts = bev_trans.recon_lane_pts(stopline_detector.image_sline)

            # 추출한 포인트에서 x 값을 받아옴 -> y값의 제한을 둬서 좌우 차선에 해당하는 포인트는 걸러냄
            stopline_estimator.get_x_points(lane_pts)
            # 정지선을 판단 및 정지선까지의 거리를 계산
            stopline_estimator.estimate_dist(30)
            # 정지선까지의 거리를 publish topic: /stop_line
            stopline_estimator.pub_sline()

            rate.sleep()
