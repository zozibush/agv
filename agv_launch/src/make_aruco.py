#!/usr/bin/env python3

import cv2
import numpy as np

if __name__ == "__main__":
    board_type = cv2.aruco.DICT_6X6_250
    MARKER_SIZE = 400
    id_info = 1
    NEW_SIZE = (1778, 1778)

    arucoDict = cv2.aruco.getPredefinedDictionary(board_type)
    aruco_marker_img = cv2.aruco.generateImageMarker(arucoDict, id_info, MARKER_SIZE)

    aruco_marker_img_resized = cv2.resize(aruco_marker_img, NEW_SIZE)

    cv2.imshow("aruco_marker_img", aruco_marker_img_resized)
    cv2.waitKey(0)
