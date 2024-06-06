#! /usr/bin/env python3

import cv2
import numpy as np

class Detection(object):
    def __init__(self):
        self.draw_img = True
        self.check_fps = True
        self.scale_percent = 40

        # ArUco info
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.camera_matrix = np.array([[381.36246688113556, 0, 320.5], [0, 381.36246688113556, 240.5], [0, 0, 1]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.aruco_size = 0.1778 # (m)

    def detectLane(self, input_image):

        # Resized Image
        resized_img = self.resize_img(input_image, self.scale_percent)

        # Thresholded Image
        thresh_img = self.findThreshold(resized_img)

        # Find cross track error and angle error
        cte, angle, output_image = self.calculateContours(thresh_img, resized_img)

        return cte, angle, output_image

    def resize_img(self, img, scale_percent):
        width = int(img.shape[1]*scale_percent/100)
        height = int(img.shape[0]*scale_percent/100)
        dim = (width, height)
        resized_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

        resized_img = resized_img[75:,:]
        return resized_img

    def findThreshold(self, img):

        # Convert input BGR image to HSV color space and take S-Channel (Saturation)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        s_channel = hsv[:,:,1]

        # Remove noise using 3x3 Kernel (Gaussian Filter)
        blurred = cv2.GaussianBlur(s_channel, (3,3), 1)

        # Binarize the copy image with global thresholding
        binarized_image = np.zeros_like(blurred)
        binarized_image[blurred>120] =1

        # Morphological Transformations (Erosion & Dilation)
        kernel = np.ones((3,3), dtype=np.uint8)
        #eroded_img = cv2.erode(binarized_image, kernel)
        dilated_img = cv2.dilate(binarized_image, kernel)

        return dilated_img

    def calculateContours(self, thresh_img, original_img):

        contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # ---------------------------------------------------------------------------
        # Originally developed by OutOfTheBots
        # (https://www.youtube.com/channel/UCCX7z2JENOSZ1mraOpyVyzg/about)
        if len(contours) > 0:
            blackbox = cv2.minAreaRect(contours[0])
            (x_min, y_min), (w_min, h_min), ang = blackbox
            if ang < -45: ang += 90
            if w_min < h_min and ang > 0: ang = (90-ang)*-1
            if w_min > h_min and ang < 0: ang = 90 + ang
            setpoint = thresh_img.shape[1]/2
            cte = -int(x_min - setpoint)
            angle = -int(ang)

            if self.draw_img:
                box = cv2.boxPoints(blackbox)
                box = np.int0(box)
                _ = cv2.drawContours(original_img, [box], 0, (255,0,0),1)
                ang_msg = "Angle Error = " + str(angle)
                err_msg = "Error = " + str(cte)
                cv2.putText(original_img, ang_msg, (130,25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,0), 1)
                cv2.putText(original_img, err_msg, (130,50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
                cv2.line(original_img, (int(x_min), 0), (int(x_min), thresh_img.shape[0]), (0,0,255), 1)
        # ---------------------------------------------------------------------------
        else:
            cte = None
            angle = None

        return cte, angle, original_img

    def detectMarkers(self, input_image):
        # 이미지에서 ArUco 마커를 감지
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(input_image, self.aruco_dict, parameters=self.parameters)

        # 감지한 ArUco 마커를 그림
        final_img = None
        if self.draw_img:
            final_img = cv2.aruco.drawDetectedMarkers(input_image.copy(), corners, ids)

        return corners, ids, final_img

    def estimatePoseSingleMarkers(self, corners, input_image):
        # Estimate pose of each marker
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.camera_matrix, self.dist_coeffs)

        final_img = None
        if self.draw_img:
            final_img = cv2.drawFrameAxes(input_image.copy(), self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.3)

        return rvec, tvec, final_img