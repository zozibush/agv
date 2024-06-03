#! /usr/bin/env python3

import cv2
import numpy as np

# LaneDetection Class for finding cross road error from the undistorted image
class ArucoDetection(object):

    def __init__(self):
        self.draw_img = True

        # ArUco info
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.camera_matrix = np.array([[381.36246688113556, 0, 320.5], [0, 381.36246688113556, 240.5], [0, 0, 1]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.aruco_size = 0.1778 # (m)

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