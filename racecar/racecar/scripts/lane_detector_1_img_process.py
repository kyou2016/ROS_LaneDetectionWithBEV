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
        #region fixed code
        self.cameraFeed = True
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
        print(self.videoPath)

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

if __name__ == '__main__':
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

    while not rospy.is_shutdown():
        curTime = time.time()
        
        fps = 1 / (curTime - prevTime)
        prevTime = curTime
        fps_str = "FPS: %0.1f" % fps

        ret, img_wlane = image_parser.get_image()
        if not ret: continue

        img_wlane = image_parser.image_process(img_wlane)

        cv2.putText(img_wlane, fps_str, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))
        
        cv2.imshow("Image window", img_wlane)
        cv2.waitKey(1)

        continue
