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

        #region Warp init
        self.warp_arrays = np.float32([[0,0], [0,0], [0,0], [0,0]])
        #endregion

        #region sliding windows data
        self.left_a, self.left_b, self.left_c = [], [], []
        self.right_a, self.right_b, self.right_c = [], [], []
        #endregion

        #region lane curved data
        self.noOfArrayValues = 10 
        self.arrayCounter = 0
        self.arrayCurve = np.zeros([self.noOfArrayValues])
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
        imgMat = inpImage
        #imgMat = cv2.UMat(inpImage)
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

    def drawPoints(self, img, src):
        #drawing circles on frame
        img_size = np.float32([(self.frameWidth, self.frameHeight)])
        #src = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
        src = src * img_size
        for x in range( 0,4): 
            cv2.circle(img,(int(src[x][0]),int(src[x][1])),15,(0,255,0),cv2.FILLED)
        return img

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

    #region Sliding windows
    def get_hist(self, img):
        hist = np.sum(img[img.shape[0]//2:,:], axis=0)
        return hist

    def sliding_window(self, img, nwindows=15, margin=50, minpix=1, draw_windows=True):
        left_fit_ = np.empty(3)
        right_fit_ = np.empty(3)
        out_img = np.dstack((img, img, img)) * 255

        histogram = self.get_hist(img)
        # find peaks of left and right halves
        midpoint = int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Set height of windows
        window_height = np.int(img.shape[0] / nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img.shape[0] - (window + 1) * window_height
            win_y_high = img.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            if draw_windows == True:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
                            (100, 255, 255), 1)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high),
                            (100, 255, 255), 1)
                # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        if leftx.size and rightx.size:
            # Fit a second order polynomial to each
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)

            self.left_a.append(left_fit[0])
            self.left_b.append(left_fit[1])
            self.left_c.append(left_fit[2])

            self.right_a.append(right_fit[0])
            self.right_b.append(right_fit[1])
            self.right_c.append(right_fit[2])

            left_fit_[0] = np.mean(self.left_a[-10:])
            left_fit_[1] = np.mean(self.left_b[-10:])
            left_fit_[2] = np.mean(self.left_c[-10:])

            right_fit_[0] = np.mean(self.right_a[-10:])
            right_fit_[1] = np.mean(self.right_b[-10:])
            right_fit_[2] = np.mean(self.right_c[-10:])

            # Generate x and y values for plotting
            ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])

            left_fitx = left_fit_[0] * ploty ** 2 + left_fit_[1] * ploty + left_fit_[2]
            right_fitx = right_fit_[0] * ploty ** 2 + right_fit_[1] * ploty + right_fit_[2]

            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 100]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 100, 255]

            return out_img, (left_fitx, right_fitx), (left_fit_, right_fit_), ploty
        else:
            return img,(0,0),(0,0),0
    #endregion

    #region Curved angle calculate
    def get_curve(self, img, leftx, rightx):
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
        y_eval = np.max(ploty)
        ym_per_pix = float(1) / float(img.shape[0])  # meters per pixel in y dimension
        xm_per_pix = float(0.1) / float(img.shape[0])  # meters per pixel in x dimension

        # Fit new polynomials to x,y in world space
        left_fit_cr = np.polyfit(ploty * ym_per_pix, leftx * xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty * ym_per_pix, rightx * xm_per_pix, 2)
        # Calculate the new radii of curvature
        left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * left_fit_cr[0])
        right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * right_fit_cr[0])

        car_pos = float(img.shape[1]) / float(2)
        l_fit_x_int = left_fit_cr[0] * img.shape[0] ** 2 + left_fit_cr[1] * img.shape[0] + left_fit_cr[2]
        r_fit_x_int = right_fit_cr[0] * img.shape[0] ** 2 + right_fit_cr[1] * img.shape[0] + right_fit_cr[2]
        lane_center_position = float(r_fit_x_int + l_fit_x_int) / 2
        center = (car_pos - lane_center_position) * float(xm_per_pix) / 10
        # Now our radius of curvature is in meters
        #print(l_fit_x_int, r_fit_x_int, center)
        return (l_fit_x_int, r_fit_x_int, center)
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
        cv2.resizeWindow(self.trackbar_name, 700, 600)
        
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
        imgWarpPoints = img_wlane
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

        imgWarpPoints = image_parser.drawPoints(imgWarpPoints, src)
        
        imgSliding, curves, lanes, ploty = image_parser.sliding_window(birdView, nwindows=8, margin=40, draw_windows=True)

        try:
            lane_curve = 0
            curverad = image_parser.get_curve(imgWarpPoints, curves[0], curves[1])
            lane_curve = np.mean([curverad[0], curverad[1]])

            # ## Average
            currentCurve = lane_curve // 50
            if int(np.sum(image_parser.arrayCurve)) == 0: averageCurve = currentCurve
            else:
                averageCurve = np.sum(image_parser.arrayCurve) // image_parser.arrayCurve.shape[0]
            if abs(averageCurve-currentCurve) >200: image_parser.arrayCurve[image_parser.arrayCounter] = averageCurve
            else :image_parser.arrayCurve[image_parser.arrayCounter] = currentCurve
            image_parser.arrayCounter +=1
            if image_parser.arrayCounter >= image_parser.noOfArrayValues : image_parser.arrayCounter=0
            
            fps_str += " Current:%0.2f" % currentCurve
            fps_str += " Average:%0.2f" % averageCurve 
        except:
            fps_str += "Lane Error"
            pass

        cv2.putText(imgWarpPoints, fps_str, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))

        cv2.imshow("Pipe Line 1", imgSliding)
        cv2.imshow(image_parser.trackbar_name, imgWarpPoints)

        if cv2.waitKey(1) == 13: break

        continue
