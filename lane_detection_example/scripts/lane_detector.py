#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import os, rospkg
import numpy as np
import json
import copy

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils_copy import BEVTransform, CURVEFit, draw_lane_img, purePursuit, PID_longitudinal

class LINEDetector:

    def __init__(self):
	self.set_cam(1)

    def set_cam(self,_index):
	self.cam = cv2.VideoCapture(int(_index))

    def get_image(self):
	ret, img_lane = self.cam.read()
	return ret, img_lane

    def get_bi_img(self):
	ret, img_bgr = self.get_image()
	img = copy.deepcopy(img_bgr)
	img2 = copy.deepcopy(img_bgr)
	img[:int(0.5*480), :] = 0
	img2[:int(0.5*480), :] = 0

	self.outimg = np.concatenate([img, img2], axis=1)

	img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

	lower_lane = np.array([ 0,   0, 194]) # 0,   0, 250  0  0 220
	upper_lane = np.array([37, 35, 255]) # 37, 255, 255 40 10 255

	img_wlane = cv2.inRange(img_hsv, lower_lane, upper_lane)

	return img_wlane

if __name__ == '__main__':
    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    # WeCar의 카메라의 파라미터 값을 적어놓은 json파일을 읽어와서 params_cam으로 저장
    with open(os.path.join(currentPath, 'sensor/sensor_params_LG.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('Line_Detector', anonymous=True)

    # WeCar의 카메라 이미지를 Bird Eye View 이미지로 변환하기 위한 클래스를 선언
    bev_trans = BEVTransform(params_cam=params_cam,cut_size=0.5)
    # BEV로 변환된 이미지에서 추출한 포인트를 기반으로 RANSAC을 이용하여 차선을 예측하는 클래스를 선언
    pts_learner = CURVEFit(order=3, init_width=1.0)

    lane_detector = LINEDetector()

    ctrl_onlane = purePursuit(look_forward_distance=1.2)
    ctrl_speed = PID_longitudinal(speed_max=1000)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        #if lane_detector.img_lane is not None:
	img_wlane = lane_detector.get_bi_img()

	if img_wlane is not None:
            # 카메라 이미지를 BEV이미지로 변환
            img_bev = bev_trans.warp_bev_img(img_wlane)
            # 카메라 이미지에서 차선에 해당하는 포인트들을 추출
            lane_pts = bev_trans.recon_lane_pts(img_wlane)

            # 추출한 포인트를 기반으로 차선을 예측
            x_pred, y_pred_l, y_pred_r = pts_learner.fit_curve(lane_pts)

            # 예측한 차선을 Path로 변환하여 메세지의 데이터를 작성
            pts_learner.write_path_msg(x_pred, y_pred_l, y_pred_r)
            # 예측한 차선을 publish topic: /lane_path
            pts_learner.pub_path_msg()

            # 예측한 차선 포인트들을 이미지에 넣기 위해서 변환
            xyl, xyr = bev_trans.project_lane2img(x_pred, y_pred_l, y_pred_r)

            # 예측한 차선 포인트들을 BEV이미지에 넣기
            img_bev_line = draw_lane_img(img_bev, xyl[:, 0].astype(np.int32),   # 예측한 차선 포인트들을 BEV이미지에 출력
                                                  xyl[:, 1].astype(np.int32),
                                                  xyr[:, 0].astype(np.int32),
                                                  xyr[:, 1].astype(np.int32),
                                                  )
            
            ctrl_speed.calc_vel_cmd()
            ctrl_onlane.steering_angle()

            if ctrl_onlane.is_path == True:
		ctrl_speed.pub_speed_cmd()
                ctrl_onlane.pub_cmd()


            cv2.imshow('BEV Window', img_bev_line)
            #cv2.imshow('BEV Window', lane_detector.outimg)
	    #cv2.imshow('test', lane_detector.img2)
	    cv2.waitKey(1)

            rate.sleep()
