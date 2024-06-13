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

    def detectLane(self, input_image, color = 'green'):

        # Resized Image
        resized_img = self.resize_img(input_image, self.scale_percent)

        # Thresholded Image
        thresh_img = self.findThreshold(resized_img, color)

        # Detect Intersection
        is_intersection = self.detectIntersection(thresh_img)

        # Find cross track error and angle error
        cte, angle, output_image = self.calculateContours(thresh_img, resized_img)

        if is_intersection:
            cte = 0
            angle = 0

        return cte, angle, output_image

    def resize_img(self, img, scale_percent):
        width = int(img.shape[1]*scale_percent/100)
        height = int(img.shape[0]*scale_percent/100)
        dim = (width, height)
        resized_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

        resized_img = resized_img[75:,:]
        return resized_img

    def findThreshold(self, img, color):

        # Convert input BGR image to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if color == 'green':
            # Define range for green color and threshold
            lower_green = np.array([40, 50, 50])
            upper_green = np.array([80, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
        elif color == 'red':
            # Define range for red color and threshold
            lower_red1 = np.array([0, 70, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

        # Remove noise using 3x3 Kernel (Gaussian Filter)
        blurred = cv2.GaussianBlur(mask, (3,3), 1)

        # Binarize the image
        binarized_image = np.zeros_like(blurred)
        binarized_image[blurred > 0] = 1

        # Morphological Transformations (Erosion & Dilation)
        kernel = np.ones((3,3), dtype=np.uint8)
        dilated_img = cv2.dilate(binarized_image, kernel)

        return dilated_img


    def detectIntersection(self, thresh_img):
        # Check for horizontal lines in the bottom half of the image
        height, width = thresh_img.shape
        bottom_half = thresh_img[height//2:, :]

        # Sum the rows to find horizontal lines
        row_sum = np.sum(bottom_half, axis=1)

        # If there are multiple significant lines, it could be an intersection
        horizontal_lines = np.sum(row_sum > width // 2)

        return horizontal_lines > 1

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