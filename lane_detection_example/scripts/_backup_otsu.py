#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg
import json
import copy as cp

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# from utils import BEVTransform, CURVEFit, STOPLineEstimator, purePursuit, draw_lane_img
from utils_copy import BEVTransform, CURVEFit, purePursuit, draw_lane_img, PID_longitudinal

class IMGParser:

    def __init__(self):       
        # self.image_sub = rospy.Subscriber('/image_jpg/compressed', CompressedImage, self.image_Callback)
        # self.image_sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.image_Callback)
        self.img_wlane = None
	self.set_cam(1)
    
    def set_cam(self, _index):
	self.cam = cv2.VideoCapture(int(_index))

    def get_image(self):
	ret, img_lane = self.cam.read()
	return ret, img_lane

    def get_bi_img(self):
	ret, img_bgr = self.get_image()
	img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
	h, s, v = cv2.split(img_hsv)
	img_v = cv2.inRange(v, 240, 255)
	self.img_wlane = cv2.threshold(img_v, 0, 250, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    def image_Callback(self, msg): 
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        # img_bgr = cv2.resize(img_bgr, dsize=(640, 480), interpolation=cv2.INTER_AREA)
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        # print(img_hsv.shape)

        upper_lane = np.array([37, 255, 255])
        lower_lane = np.array([0, 0, 250])
        h, s, v = cv2.split(img_hsv)

        # upper_lane = np.array([37, 255, 255])
        # lower_lane = np.array([ 0,   0, 250])
        # self.img_wlane = cv2.inRange(v, 250, 255)
        img_v = cv2.inRange(v, 240, 255)
        # print(img_hsv.shape)
        # img_grey = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        # img_grey = cv2.cvtColor(v, cv2.COLOR_BGR2GRAY)
        # subImg = img_grey[120:480,0:640 ]
        ret, self.img_wlane = cv2.threshold(img_v, 0, 250, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
     

        # cv2.imshow("OTSU", self.img_wlane)
        # cv2.waitKey(1)
        # self.img_wlane = cv2.Canny(self.img_wlane, 0, 255)

if __name__ == '__main__':
    look_forward_distance = 1.0
    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('Image_parser', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(xb=1.0,zb=1.0,params_cam=params_cam, cut_size=0.4, val_fov=1.6) #60:1.2 50:1.44 45:1.6 36:2 30:2.4
    # bev_op2 = BEVTransform2(xb=1.0,zb=1.0,params_cam=params_cam, cut_size=0.4, val_fov=1.6) #60:1.2 50:1.44 45:1.6 36:2 30:2.4)
    learn_op = CURVEFit(order=3, init_width=1.0)

    rate = rospy.Rate(20) #20hz

    while not rospy.is_shutdown():
        if image_parser.img_wlane is not None:
            # pass 
            img_bev = bev_op.warp_bev_img(image_parser.img_wlane)
            # img_bev2 = bev_op2.warp_bev_img(image_parser.img_wlane)

            lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

            x_pred, y_pred_l, y_pred_r = learn_op.fit_curve(lane_pts)
            learn_op.pub_path_msg()

            xyl, xyr = bev_op.project_lane2img(x_pred, y_pred_l, y_pred_r)

            img_lane = draw_lane_img(img_bev, xyl[:, 0].astype(np.int32),   # 예측한 차선 포인트들을 BEV이미지에 출력
                                                  xyl[:, 1].astype(np.int32),
                                                  xyr[:, 0].astype(np.int32),
                                                  xyr[:, 1].astype(np.int32),
                                                  )

            # img_concat = np.concatenate([bev_op.cut_img, img_bev], axis=1)

            cv2.imshow("OTSU", img_lane)
            # cv2.imshow("OTSU", img_concat)
            # cv2.imshow("OTSU", img_bev)
            cv2.waitKey(1)
            rate.sleep()
