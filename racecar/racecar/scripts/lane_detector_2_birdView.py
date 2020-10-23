#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json
import time

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import BEVTransform, CURVEFit, draw_lane_img

class IMGParser:
    def __init__(self, currentPath):
        self.img = None
        #region fixed code cam set
        self.cameraFeed = False
        #self.videoPath = currentPath + "/scripts/drive_video/sample_drive.mp4"
        #self.videoPath = currentPath + "/scripts/drive_video/drive.avi"
        #self.videoPath = currentPath + "/scripts/drive_video/drive_dark.avi"
        self.videoPath = currentPath + "/scripts/drive_video/drive_high_angle.avi"
        self.cameraNo = 1
        self.cameraWidth = 1280
        self.cameraHeight = 720
        self.frameWidth = 640
        self.frameHeight = 360
        #endregion
        self.set_cam()
        
        #region Trackbar side
        self.trackbar_change_flag = False
        self.speed_change_flag = False
        self.trackbar_name = "Control Panel"
        #endregion

        #Warp init
        self.warp_arrays = np.float32([[0,0], [0,0], [0,0], [0,0]])
        #endregion

    def set_cam(self):
        if self.cameraFeed:
            self.cam = cv2.VideoCapture(self.cameraNo)
            self.cam.set(3, self.cameraWidth)
            self.cam.set(4, self.cameraHeight)
        else:
            self.cam = cv2.VideoCapture(self.videoPath)
        
    def get_image(self):
        ret, image = self.cam.read()

        if not ret:
            self.set_cam()
        else:
            image = cv2.resize(image, (self.frameWidth, self.frameHeight), interpolation = cv2.INTER_AREA)
        
        return ret, image

    def image_process(self, inpImage):
        # Apply HLS color filtering to filter out white lane lines
        #imgMat = inpImage
        imgMat = cv2.UMat(inpImage)
        hlsMat = cv2.cvtColor(imgMat, cv2.COLOR_BGR2HLS)
        lower_white = np.array([136, 164, 101]) #136 164 101
        upper_white = np.array([255, 255, 255])
        mask = cv2.inRange(imgMat, lower_white, upper_white)
        imgMat = cv2.bitwise_and(imgMat, hlsMat, mask = mask)
        
        # Convert image to grayscale, apply threshold, blur & extract edges
        imgMat = cv2.cvtColor(imgMat, cv2.COLOR_BGR2GRAY)
        ret, imgMat = cv2.threshold(imgMat, 50, 255, cv2.THRESH_BINARY)
        imgMat = cv2.GaussianBlur(imgMat,(3, 3), 0)
        imgMat = cv2.Canny(imgMat, 40, 60)
        
        return imgMat

    #region Image Warp, insult mark point. (DRAW POINT->Perspective->Perspective->Perspective)
    def perspectiveWarp_init(self, srcs):
        # Get image size
        img_w = self.frameWidth
        img_h = self.frameHeight

        img_size = (img_w, img_h)

        srcs *= img_size
        for x in range(0, 4):
            self.warp_arrays[x][0] = int(srcs[x][0])
            self.warp_arrays[x][1] = int(srcs[x][1])

        # Window to be shown
        dst = np.float32([[0, 0], [img_w, 0], [0, img_h], [img_w, img_h]])

        # Matrix to warp the image for birdseye window
        self.matrix = cv2.getPerspectiveTransform(self.warp_arrays, dst)
        # Inverse matrix to unwarp the image for final window
        self.minv = cv2.getPerspectiveTransform(dst, self.warp_arrays)

    def perspectiveWarp(self, inpImage, srcs):
        return cv2.warpPerspective(inpImage, self.matrix, (self.frameWidth, self.frameHeight))
    #endregion

    #region CODE BLOCK REGION -> IOELECTRON TRACKBAR CONTROL
    def trackbar_ctrl(self, x):
        self.trackbar_change_flag = True

    def speed_change(self, x):
        self.speed_change_flag = True
        
    def initializeTrackbars(self, intialTracbarVals, intialCarRef):
        #intializing trackbars for region of intrest
        cv2.namedWindow(self.trackbar_name, cv2.WINDOW_NORMAL)
        cv2.createTrackbar("Top-X", self.trackbar_name, intialTracbarVals[0], 100, self.trackbar_ctrl)
        cv2.createTrackbar("Top-Y", self.trackbar_name, intialTracbarVals[1], 100, self.trackbar_ctrl)
        cv2.createTrackbar("Bottom-X", self.trackbar_name, intialTracbarVals[2], 100, self.trackbar_ctrl)
        cv2.createTrackbar("Bottom-Y", self.trackbar_name, intialTracbarVals[3], 100, self.trackbar_ctrl)
        cv2.createTrackbar("Speed", self.trackbar_name, intialCarRef, 95, self.speed_change)
        cv2.resizeWindow(self.trackbar_name, 400, 520)
        
    def valTrackbars(self):
        #getting the values of ROI, python2 version float error must float
        widthTop = float(cv2.getTrackbarPos("Top-X", self.trackbar_name))
        heightTop = float(cv2.getTrackbarPos("Top-Y", self.trackbar_name))
        widthBottom = float(cv2.getTrackbarPos("Bottom-X", self.trackbar_name))
        heightBottom = float(cv2.getTrackbarPos("Bottom-Y", self.trackbar_name))

        src = np.float32([(widthTop/100, heightTop/100), (1-(widthTop/100), (heightTop/100)),
                        (widthBottom/100, heightBottom/100), (1-(widthBottom/100), heightBottom/100)])
        #src = np.float32([widthTop, heightTop/100, widthBottom/100, heightBottom, speed_lv])
        #src = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
        return src
    def valTrackbar_speed(self):
        return cv2.getTrackbarPos("Speed", self.trackbar_name)
    #endregion

if __name__ == '__main__':
    intialTracbarVals = [10,20,0,100]
    intialCarRef = 0

    rp= rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('lane_detector',  anonymous=True)

    image_parser = IMGParser(currentPath)
    bev_op = BEVTransform(params_cam=params_cam)
    curve_learner = CURVEFit(order=3)

    rate = rospy.Rate(30)

    prevTime = curTime = time.time()

    image_parser.initializeTrackbars(intialTracbarVals, intialCarRef)

    src = image_parser.valTrackbars()
    set_speed = image_parser.valTrackbar_speed()
    image_parser.perspectiveWarp_init(src)

    while not rospy.is_shutdown():
        curTime = time.time()
        
        fps = 1 / (curTime - prevTime)
        prevTime = curTime
        fps_str = "FPS: %0.1f" % fps

        ret, img_wlane = image_parser.get_image()
        if not ret: continue

        img_wlane = image_parser.image_process(img_wlane)
        
        #region trackbar read
        src = image_parser.valTrackbars()
        if image_parser.trackbar_change_flag:
            image_parser.trackbar_change_flag = False
            print("Recalculate perspectiveWarp.")
            image_parser.perspectiveWarp_init(src)
        
        if image_parser.speed_change_flag:
            image_parser.speed_change_flag = False
            set_speed = image_parser.valTrackbar_speed()
            print("Change the speed " + str(set_speed))
        #endregion

        birdView = image_parser.perspectiveWarp(img_wlane, src)

        cv2.putText(img_wlane, fps_str, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))
        
        cv2.imshow("Pipe Line 1", birdView)
        cv2.imshow(image_parser.trackbar_name, img_wlane)

        if cv2.waitKey(1) == 13: break

        continue
