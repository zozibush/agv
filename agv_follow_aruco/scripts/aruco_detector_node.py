#! /usr/bin/env python3

import rospy
import time
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from aruco_detection import ArucoDetection
import PIDController

class ArucoDetectFollower(object):

    def __init__(self):

        self.robot_name = rospy.get_param("robot_name", "AGV_OTA1")

        self.connect()

        # PID Controller
        self.Kp = 0.1
        self.Ki = 0.00001
        self.Kd = 0.05
        self.pid_x = PIDController.PID(self.Kp, self.Ki, self.Kd)

        self.find_aruco = None

        # how far is the robot from the aruco marker
        self.far_linear_x = 1 # (m)

        # Create ArUco detection object
        self.detection = ArucoDetection()

        self.default_speed = 0.1

    def connect(self):
        # Publisher for twist messages
        self.cmd_vel_pub = rospy.Publisher(f"/{self.robot_name}/cmd_vel", Twist, queue_size = 1)

        # Subscribe to camera messages
        self.image_sub = rospy.Subscriber(f"/{self.robot_name}/camera/image_raw/compressed", CompressedImage, self.camera_callback)

    # Main Callback Function
    def camera_callback(self, data):

        try:
            #### direct conversion to CV2 ####
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except:
            print("Error conversion to CV2")

        # Lane lines detection and process cross track error
        corners, ids, final_img = self.detection.detectMarkers(cv_image)

        cmd_vel = Twist()

        if self.find_aruco is not None or ids is not None:
            if ids is not None:
                self.find_aruco = {"id":ids[0][0], "corners": corners}

            # estimate aruco marker position
            rvec, tvec, final_img =self.detection.estimatePoseSingleMarkers(self.find_aruco["corners"], final_img)

            (rvec - tvec).any()
            tvec = tvec[0][0]
            tvec = [tvec[2], tvec[1], -tvec[0]] # transform (z, y, x) to (x, y, z)

            # PID
            cmd_vel.linear.x = -self.pid_x.compute(self.far_linear_x, tvec[0])

        else:
            cmd_vel.linear.x = self.default_speed

        self.cmd_vel_pub.publish(cmd_vel)

        if self.detection.draw_img:
            cv2.putText(final_img, f"robot_vel_x: {cmd_vel.linear.x:.6f}", (130,25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            cv2.putText(final_img, f"distance: {-self.pid_x.prev_error:.6f}", (130,50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            cv2.imshow("Original Image", final_img)
            cv2.waitKey(1)

    def clean_up(self):

        cv2.destroyAllWindows()

def main():

    rospy.init_node("aruco_detector_node", anonymous=True)

    aruco_detect_follower_object = ArucoDetectFollower()

    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():

        aruco_detect_follower_object.clean_up()
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()



if __name__ == '__main__':
    main()