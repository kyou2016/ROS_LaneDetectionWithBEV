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

class Otsu:

    def __init__(self):
	self.img_wlane = None       
	self.set_cam(1)
    
    def set_cam(self, _index):
	self.cam = cv2.VideoCapture(int(_index))

    def get_image(self):
	ret, img_lane = self.cam.read()
	return img_lane

    def get_bi_img(self):
	img_bgr = self.get_image()
	#self.pub_msg = img_bgr.copy()
	img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
	h, s, v = cv2.split(img_hsv)
	img_v = cv2.inRange(v, 240, 255)
	ret, self.img_wlane = cv2.threshold(img_v, 0, 250, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
	
	#self.pub_img.publish(self.pub_msg)
	# return self.img_wlane

if __name__ == '__main__':
    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_params_LG.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('OTSU', anonymous=True)

    image_parser = Otsu()
    bev_op = BEVTransform(xb=1.0, zb=1.0, params_cam=params_cam, cut_size=0.4, val_fov=1.6) #60:1.2 50:1.44 45:1.6 36:2 30:2.4
    
    learn_op = CURVEFit(order=3, init_width=0.8)

    ctrl_onlane = purePursuit(look_forward_distance=1.0)
    ctrl_speed = PID_longitudinal(speed_max=1000)

    rate = rospy.Rate(20) #20hz
    # img_wlane = None

    while not rospy.is_shutdown():

	image_parser.get_bi_img()

        if image_parser.img_wlane is not None:
            # pass 
            img_bev = bev_op.warp_bev_img(image_parser.img_wlane)
            # img_bev2 = bev_op2.warp_bev_img(image_parser.img_wlane)

            lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

            x_pred, y_pred_l, y_pred_r = learn_op.fit_curve(lane_pts)            

            xyl, xyr = bev_op.project_lane2img(x_pred, y_pred_l, y_pred_r)

            img_lane = draw_lane_img(img_bev, xyl[:, 0].astype(np.int32),   # 예측한 차선 포인트들을 BEV이미지에 출력
                                                  xyl[:, 1].astype(np.int32),
                                                  xyr[:, 0].astype(np.int32),
                                                  xyr[:, 1].astype(np.int32),
                                                  )

	    learn_op.write_path_msg(x_pred, y_pred_l, y_pred_r)
	    learn_op.pub_path_msg()


	    ctrl_speed.calc_vel_cmd()
	    ctrl_onlane.steering_angle()

	    if ctrl_onlane.is_path == True:
		ctrl_speed.pub_speed_cmd()
		ctrl_onlane.pub_cmd()

            # img_concat = np.concatenate([bev_op.cut_img, img_bev], axis=1)

            cv2.imshow("OTSU", img_lane)
            # cv2.imshow("OTSU", img_concat)
            #cv2.imshow("OTSU", img_bev)
            cv2.waitKey(1)
            rate.sleep()
