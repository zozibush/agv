#! /usr/bin/env python3

import rospy
import time
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from lane_detection import LaneDetection

class LaneDetectFollower(object):

    def __init__(self):

        self.robot_name = rospy.get_param("robot_name", "AGV_OTA1")

        self.connect()

        self.Kp = 0.001
        self.Ki = 0
        self.Kd = 0.01


        self.max_desire_linear_vel = 0.3 # m/s
        self.max_omega = 33.51 # rad/s
        self.last_cte = 0
        self.angle_const = 0.01

        self.counter = 1

    def connect(self):
        # Publisher for twist messages
        self.cmd_vel_pub = rospy.Publisher(f"/{self.robot_name}/cmd_vel", Twist, queue_size = 1)

        # Subscribe to camera messages
        self.image_sub = rospy.Subscriber(f"/{self.robot_name}/camera/image_raw/compressed", CompressedImage, self.camera_callback)

    # Main Callback Function
    def camera_callback(self, data):
        # Dividing frame rate by 3 (10fps)
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        try:
            #### direct conversion to CV2 ####
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except:
            print("Error conversion to CV2")

        # Undistorted image
        # undist = cv2.undistort(cv_image, self.mtx, self.dist, None, self.mtx)

        # Create lane detection object
        lane_detection_object = LaneDetection()

        # Lane lines detection and process cross track error
        cte, angle, final_img = lane_detection_object.processImage(cv_image)

        cmd_vel = Twist()

        if cte is not None and angle is not None:
            angular_z = self.Kp * cte + self.Kd * (cte - self.last_cte)
            self.last_cte = cte
            linear_x = self.max_desire_linear_vel
            angular_z = max(angular_z, -2.0) if angular_z < 0 else min(angular_z, 2.0)
        else:
            angular_z = self.Kp * self.last_cte * 1.9
            linear_x = self.max_desire_linear_vel
            angular_z = max(angular_z, -2.0) if angular_z < 0 else min(angular_z, 2.0)

        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel)

        if lane_detection_object.draw_img:
            cv2.imshow("Original Image", final_img)
            cv2.waitKey(1)

    def clean_up(self):

        cv2.destroyAllWindows()

def main():

    rospy.init_node("lane_detector_node", anonymous=True)

    lane_detect_follower_object = LaneDetectFollower()

    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():

        lane_detect_follower_object.clean_up()
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()



if __name__ == '__main__':
    main()