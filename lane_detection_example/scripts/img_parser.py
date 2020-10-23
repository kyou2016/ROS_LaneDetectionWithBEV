#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import BEVTransform, CURVEFit, STOPLineEstimator, purePursuit, draw_lane_img

class IMGParser:

    def __init__(self):       
        self.image_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_Callback)
        self.img_wlane = None

    def createTrackbar(self):
        cv2.namedWindow('Original Window')
        cv2.createTrackbar('MAX H', 'Original Window', 37, 255, self.trackbar_callback)
        cv2.createTrackbar('MIN H', 'Original Window', 0, 255, self.trackbar_callback)
        cv2.createTrackbar('MAX S', 'Original Window', 255, 255, self.trackbar_callback)
        cv2.createTrackbar('MIN S', 'Original Window', 0, 255, self.trackbar_callback)
        cv2.createTrackbar('MAX V', 'Original Window', 255, 255, self.trackbar_callback)
        cv2.createTrackbar('MIN V', 'Original Window', 250, 255, self.trackbar_callback)

    def trackbar_callback(self, msg):
        pass
    
    def image_Callback(self, msg): 
             
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)     

        self.createTrackbar()  

        max_H = cv2.getTrackbarPos('MAX H', 'Original Window')
        min_H = cv2.getTrackbarPos('MIN H', 'Original Window')
        max_S = cv2.getTrackbarPos('MAX S', 'Original Window')
        min_S = cv2.getTrackbarPos('MIN S', 'Original Window')
        max_V = cv2.getTrackbarPos('MAX V', 'Original Window')
        min_V = cv2.getTrackbarPos('MIN V', 'Original Window')

        upper_lane = np.array([max_H, max_S, max_V])
        lower_lane = np.array([min_H, min_S, min_V])

        h = img_bgr.shape[0]
        w = img_bgr.shape[1]

        # upper_lane = np.array([37, 255, 255])
        # lower_lane = np.array([0,    0, 250])

        upper_sig_red = np.array([ 20, 255, 255])
        lower_sig_red = np.array([  0,  20, 251])

        upper_sig_yellow = np.array([ 35, 255, 255])
        lower_sig_yellow = np.array([ 20,  20, 251])

        upper_sig_green = np.array([ 90, 255, 255])
        lower_sig_green = np.array([ 60,  20, 251])

        # hsv이미지를 h,s,v 값의 최대 최소 값을 조절하여 원하는 결과만 흑백이미지로 보이게 변환
        self.img_wlane = cv2.inRange(img_hsv, lower_lane, upper_lane) 

        # hsv이미지를 h,s,v 값의 범위를 조절하여 신호등의 각 색깔별로 인식하게 변환. 
        # resize는 3개의 이미지를 붙여서 출력하면 너무 창이 커져서 이미지 사이즈를 줄이기 위해 사용
        img_red = cv2.resize(cv2.inRange(img_hsv, lower_sig_red, upper_sig_red), (w/2, h/2))
        img_yellow = cv2.resize(cv2.inRange(img_hsv, lower_sig_yellow, upper_sig_yellow), (w/2, h/2))
        img_green = cv2.resize(cv2.inRange(img_hsv, lower_sig_green, upper_sig_green), (w/2, h/2))

        # 신호등 이미지에 차선이 출력될 수 있으므로 이미지의 세로 밑에서 부터 2/3을 0(검정색)으로 바꾸고 
        # 위에서 이미지 사이즈를 1/2로 줄였으므로 똑같이 1/2를 해준다.
        
        img_yellow[int(h/3/2):, :] = 0
        

        ## concat을 하려면 이미지의 채널이 같아야 하므로 cvtColor를 적용한다.
        # self.img_wlane = cv2.cvtColor(self.img_wlane, cv2.COLOR_GRAY2BGR)

        ## img_bgr과 img_hsv를 [세로, 가로, 채널] 중 인덱스가 1인 가로를 기준으로 붙임 -> 가로로 붙어있는 이미지가 만들어짐
        img_concat = np.concatenate([img_red, img_yellow, img_green], axis=1)
        cv2.imshow("Signal Window", img_concat)

        cv2.imshow("Original Window", self.img_wlane)

        # cv2.setMouseCallback('mouseRGB', self.mouseRGB)
        cv2.waitKey(1)

    # ## 마우스 클릭으로 클릭 좌표의 HSV값을 출력
    # def mouseRGB(self, event, x, y, flags, param):
    #     if event == cv2.EVENT_LBUTTONDOWN:
    #         colorH = self.img_concat[y,x,0]
    #         colorS = self.img_concat[y,x,1]
    #         colorV = self.img_concat[y,x,2]
    #         colors = self.img_concat[y,x]
    #         print("Hue: ", colorH)
    #         print("Saturation: ", colorS)
    #         print("Value: ", colorV)
    #         print("HSV Format: ", colors)
    #         print("Coordinates of pixel: X: ", x, "Y: ", y)


if __name__ == '__main__':
    look_forward_distance = 1.0
    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('Image_parser', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)

    curve_learner = CURVEFit(order=3)
    ctrl = purePursuit(look_forward_distance=look_forward_distance)
    sline_detector = STOPLineEstimator()

    rate = rospy.Rate(20) #20hz

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

        rospy.spin()