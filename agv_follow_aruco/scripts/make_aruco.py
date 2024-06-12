#!/usr/bin/env python3

import cv2
import numpy as np

if __name__ == "__main__":
    board_type = cv2.aruco.DICT_6X6_250
    MARKER_SIZE = 100

    arucoDict = cv2.aruco.getPredefinedDictionary(board_type)

    for id_info in range(0, 20):
        aruco_marker_img = cv2.aruco.generateImageMarker(arucoDict, id_info, MARKER_SIZE)

        # cv2.imshow("aruco_marker_img", aruco_marker_img)
        # cv2.waitKey(0)
        cv2.imwrite(f"aruco_marker_{id_info}.png", aruco_marker_img)

