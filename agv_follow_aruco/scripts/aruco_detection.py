#! /usr/bin/env python3

import cv2
import numpy as np

# LaneDetection Class for finding cross road error from the undistorted image
class ArucoDetection(object):

    def __init__(self):
        self.draw_img = True
        self.check_fps = True
        self.scale_percent = 40

        # ArUco의 사전을 선택
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        # ArUco의 파라미터를 기본값으로 설정
        self.parameters = cv2.aruco.DetectorParameters()

    def processImage(self, input_image):
        # 이미지에서 ArUco 마커를 감지
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(input_image, self.aruco_dict, parameters=self.parameters)

        # 감지한 ArUco 마커를 그림
        final_img = cv2.aruco.drawDetectedMarkers(input_image.copy(), corners, ids)
        return ids, final_img


    def resize_img(self, img, scale_percent):
        width = int(img.shape[1]*scale_percent/100)
        height = int(img.shape[0]*scale_percent/100)
        dim = (width, height)
        resized_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

        resized_img = resized_img[75:,:]
        return resized_img
