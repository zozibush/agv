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

        # Publisher for twist messages
        self.cmd_vel_pub = rospy.Publisher("/AGV_OTA1/cmd_vel", Twist, queue_size = 1)

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

        # Undistorted image
        # undist = cv2.undistort(cv_image, self.mtx, self.dist, None, self.mtx)

        # Create lane detection object
        aruco_detection_object = ArucoDetection()

        # Lane lines detection and process cross track error
        ids, final_img = aruco_detection_object.processImage(cv_image)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0 if ids is not None else 0.2
        if not self.finish:
            self.finish = True if ids is not None else False
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