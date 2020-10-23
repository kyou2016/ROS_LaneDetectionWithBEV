#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg
import json
import copy

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import BEVTransform, CURVEFit, STOPLineEstimator, purePursuit, draw_lane_img

class IMGParser:

    def __init__(self):
	self.set_cam(1)

    def set_cam(self,_index):
	self.cam = cv2.VideoCapture(int(_index))

    def get_image(self):
	ret, img_lane = self.cam.read()
	return ret, img_lane

    def get_bi_img(self):
	ret, img_bgr = self.get_image()	

	return img_bgr

    def createTrackbar(self):
        cv2.namedWindow('Original Window')
        cv2.createTrackbar('MAX H', 'Original Window', 37, 255, self.trackbar_callback)
        cv2.createTrackbar('MIN H', 'Original Window', 0, 255, self.trackbar_callback)
        cv2.createTrackbar('MAX S', 'Original Window', 35, 255, self.trackbar_callback)
        cv2.createTrackbar('MIN S', 'Original Window', 0, 255, self.trackbar_callback)
        cv2.createTrackbar('MAX V', 'Original Window', 255, 255, self.trackbar_callback)
        cv2.createTrackbar('MIN V', 'Original Window', 254, 255, self.trackbar_callback)

    def trackbar_callback(self, msg):
        pass
   


if __name__ == '__main__':
    look_forward_distance = 1.0
    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('Image_parser', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam,cut_size=0.55)
    image_parser.createTrackbar()

    # curve_learner = CURVEFit(order=3)
    # ctrl = purePursuit(look_forward_distance=look_forward_distance)
    # sline_detector = STOPLineEstimator()

    rate = rospy.Rate(20) #20hz
    img_bgr = None

    while not rospy.is_shutdown():
        
        # if image_parser.img_wlane is not None:

        #     # img_bev = bev_op.warp_bev_img(image_parser.img_wlane)        # BEV이미지 만들기
        #     lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)        # BEV에서 차선 포인트 추출하여 반환

        #     # x_pred, y_pred_l, y_pred_r = curve_learner.fit_curve(lane_pts)    # lane_pts를 이용하여 차선을 예측(predict)한다.

        #     # ctrl.steering_angle(x_pred, y_pred_l, y_pred_r)               # 예측한 차선 포인트들을 이용하여 조향각을 컨트롤
        #     # ctrl.pub_cmd()                                                # 차량의 속도와 조향각 값을 publish

        #     # xyl, xyr = bev_op.project_lane2img(x_pred, y_pred_l, y_pred_r)    # 예측한 차선 포인트들을 BEV이미지에 출력하기 위해 변환
            
        #     # img_bev_lane = draw_lane_img(img_warp, xyl[:, 0].astype(np.int32),   # 예측한 차선 포인트들을 BEV이미지에 출력
        #     #                                     xyl[:, 1].astype(np.int32),
        #     #                                     xyr[:, 0].astype(np.int32),
        #     #                                     xyr[:, 1].astype(np.int32),
        #     #                                     )

        #     sline_detector.get_x_points(lane_pts)   # lane_pts에서 x에 해당하는 포인트들을 추출한다.
        #     sline_detector.estimate_dist(30)        # x와 y를 범위를 지정하여 정지선이 나오면 정지선까지의 거리를 출력, 정지선이 없으면 지정한 최대 x 범위를 출력

        #     sline_detector.visualize_dist()         # 그래프가 출력되면 멈추기 때문에 컨트롤할 경우 주석으로 만들고 사용

        #     sline_detector.pub_sline()              # 정지선과 차량의 거리를 publish

        #     # cv2.imshow("Image Window", img_bev_lane)
        #     # cv2.waitKey(1)

        #     rate.sleep()
	img_bgr = image_parser.get_bi_img()
	if img_bgr is not None:
	    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

            max_H = cv2.getTrackbarPos('MAX H', 'Original Window')
            min_H = cv2.getTrackbarPos('MIN H', 'Original Window')
            max_S = cv2.getTrackbarPos('MAX S', 'Original Window')
            min_S = cv2.getTrackbarPos('MIN S', 'Original Window')
            max_V = cv2.getTrackbarPos('MAX V', 'Original Window')
            min_V = cv2.getTrackbarPos('MIN V', 'Original Window')

            upper_lane = np.array([max_H, max_S, max_V])
            lower_lane = np.array([min_H, min_S, min_V])

	    img_wlane = cv2.inRange(img_hsv, lower_lane, upper_lane)
	    cv2.imshow("Original Window", img_wlane)
            cv2.waitKey(1)
	    rate.sleep()
