#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from agv_control.msg import Control
from PIDController import PID
from Detection import Detection

class AGVController:

    def __init__(self):

        self.robot_name = rospy.get_param("robot_name", "AGV_OTA1")

        self.connect()

        self.type = -1
        self.epsilon = 5e-2
        self.pause = False

        self.detection = Detection()
        self.pid_aruco = PID(0.1, 0.00001, 0.05)
        self.pid_lane = PID(0.0005, 0.0, 0.005)

        self.default_speed = 0.1

        # aruco
        self.far_linear_x = 0.7
        self.find_aruco = None

        # lane
        self.max_desire_linear_vel = 0.3 # m/s
        self.max_omega = 33.51 # rad/s
        self.last_cte = 0
        self.angle_const = 0.01

        # performance
        self.counter = 0

    def connect(self):
        # Publisher
        self.cmd_vel_pub = rospy.Publisher(f"/{self.robot_name}/cmd_vel", Twist, queue_size = 1)

        # Subscriber
        self.image_sub = rospy.Subscriber(f"/{self.robot_name}/camera/image_raw/compressed", CompressedImage, self.camera_callback)
        self.control_sub = rospy.Subscriber(f"/{self.robot_name}/control", Control, self.control_callback)
        self.pause_sub = rospy.Subscriber(f"/{self.robot_name}/pause", Bool, self.pause_callback)

    def pause_callback(self, data):
        self.pause = data.data

    def camera_callback(self, data):

        # Dividing frame rate by 3 (10fps)
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        cmd_vel = Twist()
        if self.type == -1 or self.pause:
            self.cmd_vel_pub.publish(cmd_vel)
            return
        try:
            #### direct conversion to CV2 ####
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except:
            print("Error conversion to CV2")

        corners, ids, final_img = self.detection.detectMarkers(cv_image)


        if self.type == 1: # follow aruco
            if ids is not None and ids[0][0] != self.marker_id:
                cmd_vel.linear.x = self.default_speed

            elif self.find_aruco is not None or ids is not None:
                if ids is not None:
                    self.find_aruco = {"id":ids[0][0], "corners": corners}

                # estimate aruco marker position
                rvec, tvec, final_img = self.detection.estimatePoseSingleMarkers(self.find_aruco["corners"], final_img)

                (rvec - tvec).any()
                tvec = tvec[0][0]
                tvec = [tvec[2], tvec[1], -tvec[0]] # transform (z, y, x) to (x, y, z)

                # PID
                error = tvec[0] - self.far_linear_x
                cmd_vel.linear.x = self.pid_aruco.compute(error)

            else:
                cmd_vel.linear.x = self.default_speed

            if self.pid_aruco.last_error != 0 and abs(self.pid_aruco.last_error) < self.epsilon:
                self.type = -1
                self.find_aruco = None
                self.pid_aruco.reset()
                self.pid_lane.reset()
                cmd_vel = Twist()

        elif self.type == 2: # follow lane

            if ids is not None and ids[0][0] == self.marker_id:
                self.type = 1
                return

            cte, angle, final_img = self.detection.detectLane(cv_image)

            if cte is not None and angle is not None:
                angular_z = self.pid_lane.compute(cte)
            else:
                angular_z = self.pid_lane.Kp * self.pid_lane.last_error * 1.9

            angular_z = max(angular_z, -2.0) if angular_z < 0 else min(angular_z, 2.0)
            cmd_vel.linear.x = self.default_speed
            cmd_vel.angular.z = angular_z

        if self.detection.draw_img:
            cv2.putText(final_img, f"distance: {self.pid_aruco.last_error:.6f}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            cv2.putText(final_img, f"cmd: ({cmd_vel.linear.x:.6f},0,{cmd_vel.linear.z:.6f})", (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            cv2.putText(final_img, f"type: {self.type}, marker_id: {self.marker_id}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            cv2.imshow("Original Image", final_img)
            cv2.waitKey(1)
        self.cmd_vel_pub.publish(cmd_vel)

    def control_callback(self, data):
        if data is None:
            return

        self.type = data.type
        self.marker_id = data.marker_id
        self.default_speed = data.vel

        if self.type != -1:
            rospy.loginfo(f"Received AGV control message: type={self.type}, marker_id={self.marker_id}, vel={self.default_speed:.2f}")

    def clean_up(self):
        cv2.destroyAllWindows()

    def move(self, x = 0, z = 0):
        cmd_vel = Twist()
        cmd_vel.linear.x = x
        cmd_vel.angular.z = z
        self.cmd_vel_pub.publish(cmd_vel)

    def turn(self, angle):
        self.move(0, angle)

def main():

    rospy.init_node("agv_controller_node", anonymous=True)

    agv_controller_node = AGVController()

    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():

        agv_controller_node.clean_up()
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()



if __name__ == '__main__':
    main()