#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import math
import copy as cp

from std_msgs.msg import Float64, String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from sklearn import linear_model
import random

import matplotlib.pyplot as plt

def warp_image(img ,source_prop):

    image_size = (img.shape[1], img.shape[0])
    x = img.shape[1]
    y = img.shape[0]

    destination_points = np.float32([
        [0, y],
        [0, 0],
        [x, 0],
        [x, y]
    ])

    source_points = source_prop * np.float32([
        [x, y], 
        [x, y], 
        [x, y], 
        [x, y]
    ])

    perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)

    warped_img = cv2.warpPerspective(img, perspective_transform, image_size, flags=cv2.INTER_LINEAR)

    return warped_img

def translationMtx(x, y, z):
    M = np.array([
        [1,         0,          0,          x],
        [0,         1,          0,          y],
        [0,         0,          1,          z],
        [0,         0,          0,          1]
    ])

    return M

def rotationMtx(yaw, pitch, roll):
    R_x = np.array([
        [1,           0,                0,                0],
        [0,           math.cos(roll),   -math.sin(roll),  0],
        [0,           math.sin(roll),   math.cos(roll),   0],
        [0,           0,                0,                1]
    ])

    R_y = np.array([
        [math.cos(pitch),  0,           math.sin(pitch),  0],
        [0,                1,           0,                0],
        [-math.sin(pitch), 0,           math.cos(pitch),  0],
        [0,                0,           0,                1]
    ])

    R_z = np.array([
        [math.cos(yaw),    -math.sin(yaw),   0,           0],
        [math.sin(yaw),    math.cos(yaw),    0,           0],
        [0,                0,                1,           0],
        [0,                0,                0,           1]
    ])

    R = np.matmul(R_x, np.matmul(R_y, R_z))

    return R

def projectionMtx(params_cam):
    if params_cam["ENGINE"]=='UNITY':
        fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
        fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    else:
        fc_x = params_cam["WIDTH"]/2(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
        fc_y = params_cam["WIDTH"]/2(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    cx = params_cam['WIDTH']/2
    cy = params_cam["HEIGHT"]/2

    R_f = np.array([[fc_x, 0, cx],
                    [0, fc_y, cy]])

    return R_f

class BEVTransform:

    def __init__(self, params_cam, xb=1.0, zb=1.0, cut_size=0.5):
        self.xb = xb
        self.zb = zb
        self.cut_size = cut_size

        self.theta = np.deg2rad(params_cam["PITCH"])
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        if params_cam["ENGINE"]=="UNITY":
            self.alpha_r = np.deg2rad(params_cam["FOV"]/2)

            self.fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            self.alpha_c = np.arctan2(params_cam["WIDTH"]/2, self.fc_y)

            self.fc_x = self.fc_y
        else:
            self.alpha_r = np.deg2rad(params_cam["FOV"]/2)

            self.fc_y = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            self.alpha_c = np.arctan2(params_cam["WIDTH"]/2, self.fc_y)

            self.fc_x = self.fc_y

        self.h = params_cam["Z"]
        self.x = params_cam["X"]
        

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT_b2g = np.matmul(np.matmul(translationMtx(xb, 0, zb), rotationMtx(np.deg2rad(-90), 0, 0)), rotationMtx(0, 0, np.deg2rad(180)))

        self.build_tf(params_cam)

    def calc_Xv_Yu(self, U, V):
        Xv = self.h*(np.tan(self.theta)*(1-2*(V-1)/(self.m-1))*np.tan(self.alpha_r)-1)/(-np.tan(self.theta)+(1-2*(V-1)/(self.m-1))*np.tan(self.alpha_r))
        Yu = (1-2*(U-1)/(self.n-1))*Xv*np.tan(self.alpha_c)

        return Xv, Yu

    def build_tf(self, params_cam):
        v = np.array([params_cam["HEIGHT"]*self.cut_size, params_cam["HEIGHT"]]).astype(np.float32)
        u = np.array([0, params_cam["WIDTH"]]).astype(np.float32)

        U, V = np.meshgrid(u, v)

        Xv, Yu = self.calc_Xv_Yu(U, V)

        xyz_g = np.concatenate([
            Xv.reshape([1,-1]) + params_cam["X"],
            Yu.reshape([1,-1]),
            np.zeros_like(Yu.reshape([1,-1])),
            np.ones_like(Yu.reshape([1,-1]))], axis=0)

        xyz_bird = np.matmul(np.linalg.inv(self.RT_b2g), xyz_g)

        xc, yc, zc = xyz_bird[0, :].reshape([1, -1]), xyz_bird[1, :].reshape([1, -1]), xyz_bird[2, :].reshape([1, -1])

        self.proj_mtx = projectionMtx(params_cam)

        xn, yn = xc/zc, yc/zc

        xy1 = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xy1 = xy1[0:2, :].T

        src_pts = np.concatenate([U.reshape([-1, 1]), V.reshape([-1, 1])], axis=1).astype(np.float32)
        dst_pts = xy1.astype(np.float32)

        self.perspective_tf = cv2.getPerspectiveTransform(src_pts, dst_pts)

    def warp_bev_img(self, img):
        img_warp = cv2.warpPerspective(img, self.perspective_tf, (self.width, self.height), flags=cv2.INTER_LINEAR)

        return img_warp

    def recon_lane_pts(self, img):
        # cut_size만큼 위에서 이미지를 잘라서 포인트를 추출
        img[:int(self.cut_size*self.height), :] = 0

        if cv2.countNonZero(img) != 0:
            UV_mark = cv2.findNonZero(img).reshape([-1, 2])
            # U는 카메라에서 가로(x) V는 카메라에서 세로(y)
            U, V = UV_mark[:, 0].reshape([-1, 1]), UV_mark[:, 1].reshape([-1, 1])

            Xv, Yu = self.calc_Xv_Yu(U, V)

            xyz_g = np.concatenate([
                Xv.reshape([1, -1]) + self.x,
                Yu.reshape([1, -1]),
                np.zeros_like(Yu.reshape([1, -1])),
                np.ones_like(Yu.reshape([1, -1]))
            ], axis=0)
            # xyz_g[0]: 추출한 포인터의 x좌표, xyz_g[1]: 추출한 포인터의 y좌표
            xyz_g = xyz_g[:, xyz_g[0, :] >= 0]
        else:
            xyz_g = np.zeros((4, 10))

        return xyz_g

    def project_lane2img(self, x_pred, y_pred_l, y_pred_r):
        xyz_l_g = np.concatenate([
            x_pred.reshape([1, -1]),
            y_pred_l.reshape([1, -1]),
            np.zeros_like(y_pred_l.reshape([1, -1])),
            np.ones_like(y_pred_l.reshape([1, -1])),
        ], axis=0)

        xyz_r_g = np.concatenate([
            x_pred.reshape([1, -1]),
            y_pred_r.reshape([1, -1]),
            np.zeros_like(y_pred_r.reshape([1, -1])),
            np.ones_like(y_pred_r.reshape([1, -1]))
        ], axis=0)

        xyz_l_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_l_g)
        xyz_r_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_r_g)

        xyl = self.project_pts2img(xyz_l_b)
        xyr = self.project_pts2img(xyz_r_b)

        xyl = self.crop_pts(xyl)
        xyr = self.crop_pts(xyr)

        return xyl, xyr

    def project_pts2img(self, xyz_bird):

        xc, yc, zc = xyz_bird[0, :].reshape([1, -1]), xyz_bird[1, :].reshape([1, -1]), xyz_bird[2, :].reshape([1, -1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = xyi[0:2, :].T

        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0] >= 0, xyi[:, 0] < self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1] >= 0, xyi[:, 1] < self.height), :]

        return xyi

class CURVEFit:

    def __init__(self, order, init_width=0.5):
        # order: 다항식 차수, 
        # init_width: 초기 좌우 차선간의 폭, 
        # y_margin: 차선 픽셀 좌표를 모을 마진: +,- 0.2, 
        # dx: Regresss가 되면 0.1간격으로 차선을 예측, 
        # min_pts: 모으는 구간당 최소 픽셀 수
        self.order = order
        self.y_margin = 0.2
        self.x_range = 3
        self.dx = 0.1
        self.min_pts = 50        
        self.init_width = init_width

        self.lane_path = Path()

        self.ransac_left = linear_model.RANSACRegressor(base_estimator=linear_model.Ridge(alpha=2),
                                                        max_trials=5, 
                                                        #loss='absolute_loss',
                                                        min_samples=self.min_pts, 
                                                        residual_threshold=0.4)
        
        self.ransac_right = linear_model.RANSACRegressor(base_estimator=linear_model.Ridge(alpha=2),
                                                        max_trials=5, 
                                                        #loss='absolute_loss',
                                                        min_samples=self.min_pts, 
                                                        residual_threshold=0.4)

        self.path_pub = rospy.Publisher('/lane_path', Path, queue_size=1)

        self._init_model()

    def _init_model(self):
        self.lane_width = self.init_width

        X = np.stack([np.arange(0, 2, 0.02)**i for i in reversed(range(1, self.order+1))]).T
        y_l = 0.5*self.lane_width*np.ones_like(np.arange(0, 2, 0.02))
        y_r = -0.5*self.lane_width*np.ones_like(np.arange(0, 2, 0.02))

        self.ransac_left.fit(X, y_l)
        self.ransac_right.fit(X, y_r)

    def preprocess_pts(self, lane_pts):
        # recon_lane_pts에서 추출한 포인트들(차선인지 구분 안한 포인트들)을 일정 거리만큼 간격을 주고 랜덤으로 포인트를 선택하여
        # x_left, x_right, y_left, y_right로 변환
        idx_list = []
        
        for d in np.arange(0, self.x_range, self.dx):

            idx_full_list = np.where(np.logical_and(lane_pts[0, :]>=d, lane_pts[0, :]<d+0.1))[0].tolist()

            idx_list += random.sample(idx_full_list, np.minimum(50, len(idx_full_list)))

        lane_pts = lane_pts[:, idx_list]

        x_g = np.copy(lane_pts[0, :])
        y_g = np.copy(lane_pts[1, :])

        X_g = np.stack([x_g**i for i in reversed(range(1, self.order+1)) ]).T

        y_ransac_collect_r = self.ransac_right.predict(X_g)

        y_right = y_g[np.logical_and(y_g>=y_ransac_collect_r-self.y_margin, y_g<y_ransac_collect_r+self.y_margin)]
        x_right = x_g[np.logical_and(y_g>=y_ransac_collect_r-self.y_margin, y_g<y_ransac_collect_r+self.y_margin)]

        y_ransac_collect_l = self.ransac_left.predict(X_g)

        y_left = y_g[np.logical_and(y_g>=y_ransac_collect_l-self.y_margin, y_g<y_ransac_collect_l+self.y_margin)]
        x_left = x_g[np.logical_and(y_g>=y_ransac_collect_l-self.y_margin, y_g<y_ransac_collect_l+self.y_margin)]

        return x_left, y_left, x_right, y_right

    def fit_curve(self, lane_pts):     
        self.not_left = False
        self.not_right = False

        x_left, y_left, x_right, y_right = self.preprocess_pts(lane_pts)

        # y포인트들(여기서 y값은 가로범위)이 갯수가 0 일경우 초기화
        if len(y_left) <= 10 or len(y_right) <= 10:
            if len(y_left) == 0:
                # print('y_left point does not exist')
                self.not_left = True
            else:
                # print("y_right point does not exist")
                self.not_right = True
            self._init_model()
            x_left, y_left, x_right, y_right = self.preprocess_pts(lane_pts)

        # 차선 폭이 너무 졻게 계산되면 자동으로 초기화
        if self.lane_width < 0.01:
            # print("Too Short lane width")
            self._init_model()
            x_left, y_left, x_right, y_right = self.preprocess_pts(lane_pts)
        
        X_left = np.stack([x_left**i for i in reversed(range(1, self.order+1))]).T
        X_right = np.stack([x_right**i for i in reversed(range(1, self.order+1))]).T

        if y_left.shape[0]>=self.ransac_left.min_samples:
            self.ransac_left.fit(X_left, y_left)

        if y_right.shape[0]>=self.ransac_right.min_samples:
            self.ransac_right.fit(X_right, y_right)

        x_pred = np.arange(0, self.x_range, self.dx).astype(np.float32)
        X_pred = np.stack([x_pred**i for i in reversed(range(1, self.order+1))]).T

        y_pred_l = self.ransac_left.predict(X_pred)
        y_pred_r = self.ransac_right.predict(X_pred)
        
        if y_left.shape[0]>=self.ransac_left.min_samples and y_right.shape[0]>=self.ransac_right.min_samples:
            # if self.lane_width < self.init_width - self.init_width/2:
            #     # print("Too Short lane width")
            #     self.lane_width = self.init_width
            # else:
            #     self.lane_width = np.mean(y_pred_l - y_pred_r)
            self.lane_width = np.mean(np.abs(y_pred_l - y_pred_r))

        if y_left.shape[0]<self.ransac_left.min_samples:
            y_pred_l = cp.deepcopy(y_pred_r + self.lane_width)
        
        if y_right.shape[0]<self.ransac_right.min_samples:
            y_pred_r = cp.deepcopy(y_pred_l - self.lane_width)

        # # 예측한 각 y값들의 거리 평균이 0.1보다 작을경우 차선을 강제로 분리
        if1 , if2 = False, False
        if np.mean(np.abs(y_pred_l - y_pred_r)) <= 0.1:
            if1 = True if y_pred_l.all() < 0 and y_pred_r.all() < 0 else False
            if2 = True if y_pred_l.all() > 0 and y_pred_r.all() > 0 else False
            # 둘다 0보다 큰 값이면 차선이 왼쪽에 있다는 것
            if if1:
                y_pred_l = cp.deepcopy(y_pred_l + self.init_width)
            # 둘다 0보다 작은 값이면 차선이 오른쪽에 있다는 것
            if if2:
                y_pred_r = cp.deepcopy(y_pred_r - self.init_width)

        if self.not_left:
            y_pred_l = cp.deepcopy(y_pred_r)
            y_pred_r = y_pred_r - self.init_width
        elif self.not_right:
            y_pred_r = cp.deepcopy(y_pred_l)
            y_pred_l = y_pred_l + self.init_width

        print("lane: {:.2f} | notleft: {:s} | notright: {:s}".format(self.lane_width, str(self.not_left), str(self.not_right)))
        print("Over 0: {:s} | Under 0: {:s}".format(str(if1), str(if2)))

        return x_pred, y_pred_l, y_pred_r

    def write_path_msg(self, x_pred, y_pred_l, y_pred_r):
        self.lane_path = Path()

        self.lane_path.header.frame_id='/map'

        for i in range(len(x_pred)):
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x=x_pred[i]
            tmp_pose.pose.position.y=(-0.5)*(y_pred_l[i] + y_pred_r[i])
            tmp_pose.pose.position.z=0
            tmp_pose.pose.orientation.x=0
            tmp_pose.pose.orientation.y=0
            tmp_pose.pose.orientation.z=0
            tmp_pose.pose.orientation.w=1
            self.lane_path.poses.append(tmp_pose)

    def pub_path_msg(self):
        self.path_pub.publish(self.lane_path)

def draw_lane_img(img, leftx, lefty, rightx, righty):

    point_np = cv2.cvtColor(np.copy(img), cv2.COLOR_GRAY2BGR)

    for ctr in zip(leftx, lefty):
        point_np = cv2.circle(point_np, ctr, 5, (255,0,0), -1)

    for ctr in zip(rightx, righty):
        point_np = cv2.circle(point_np, ctr, 5, (0,0,255), -1)

    return point_np

class purePursuit:

    def __init__(self, look_forward_distance):
        self.is_look_forward_point = False
        self.is_path = False
        self.vehicle_length = 2

        self.lfd = look_forward_distance
        self.min_lfd = 0.7
        self.max_lfd = 1.2

        self.path_sub = rospy.Subscriber('/lane_path', Path, self.path_callback)

        self.position_pub = rospy.Publisher('/commands/servo/position2', Float64, queue_size=1)

        self.lpath = None

    def path_callback(self, msg):
        if self.is_path == False:
            self.is_path = True
        self.lpath = msg

    def steering_angle(self):    

        self.is_look_forward_point = False

        if self.is_path == True:
            for i in self.lpath.poses:

                path_point = i.pose.position

                if path_point.x > 0:

                    dis_pts = np.sqrt(np.square(path_point.x) + np.square(path_point.y))

                    if dis_pts >= self.lfd:

                        self.is_look_forward_point = True
                        break                   
            
            	theta = math.atan2(path_point.y, path_point.x)

            if self.is_look_forward_point:
                steering_deg = math.atan2((2*self.vehicle_length*math.sin(theta)), self.lfd)*180/math.pi

                self.steering = np.clip(steering_deg, -22, 22)/44+0.5
                print(self.steering)            
            else:
                self.steering = 0.5304
                print("No found forward point")

    def pub_cmd(self):

        self.position_pub.publish(self.steering)

class STOPLineEstimator:
    def __init__(self):
        self.n_bins = 30
        self.x_max = 3          #x는 최대 3미터까지만 취급
        self.y_max = 0.1        #y는 좌우 최대 0.1미터까지만 취급
        self.bins = np.linspace(0, self.x_max, self.n_bins)
        self.min_pts = 10

        self.sline_pub = rospy.Publisher('/stop_line', Float64, queue_size=1)

    def get_x_points(self, lane_pts):
        # BEV에서 주는 lane_pts에서 좌우 y_max안의 값만 취급
        self.x = lane_pts[0, np.logical_and(lane_pts[1, :] > -self.y_max, lane_pts[1, :] <= self.y_max)]

    def estimate_dist(self, quantile):
        if self.x.shape[0] > self.min_pts:
            self.d_stopline = np.clip(np.percentile(self.x, quantile), 0, self.x_max)
        else:
            self.d_stopline = self.x_max

        return self.d_stopline

    def visualize_dist(self):
        fig, ax = plt.subplots(figsize=(8, 4))
        n, bins, patchs = ax.hist(self.x, self.bins)
        ax.cla()

        bins = (bins[1:] + bins[:-1])/2
        
        n_cum = np.cumsum(n)
        n_cum = n_cum/n_cum[-1]

        plt.plot(bins, n_cum)
        plt.show()

    def pub_sline(self):
        self.sline_pub.publish(self.d_stopline)

class PID_longitudinal:

    def __init__(self, K=500, safe_dis=1.0, speed_max=4000):
        self.speed_max = speed_max
        self.safe_dis = safe_dis
        self.K = K

        self.sline_dis = None
        self.light = None

        self.stopline_sub = rospy.Subscriber('/stop_line', Float64, self.stopline_callback)
        self.light_sub = rospy.Subscriber('/traffic_light', String, self.traffic_light_callback)

        self.speed_pub = rospy.Publisher('/commands/motor/speed2', Float64, queue_size=1)

    def stopline_callback(self, msg):
        self.sline_dis = msg.data
    
    def traffic_light_callback(self, msg):
        self.light = msg.data

    def calc_vel_cmd(self):
        
        if self.sline_dis <= self.safe_dis*1.2 and self.sline_dis > self.safe_dis*0.5:
            if self.light == "RED":
                self.speed_value = 0
                print('STOP')
            elif self.light == "YELLOW":
                self.speed_value = np.clip(self.K*(self.sline_dis - self.safe_dis), 0, self.speed_max)
            else:
                self.speed_value = self.speed_max
        else:
            self.speed_value = self.speed_max
        
        

    def pub_speed_cmd(self):
        self.speed_pub.publish(self.speed_value)
