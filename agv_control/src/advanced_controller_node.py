#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from agv_control.msg import AdvancedControl
from PIDController import PID
from Detection import Detection
import math

class AGVAdvancedController:

    def __init__(self):

        self.robot_name = rospy.get_param("robot_name", "AGV_OTA1")

        self.connect()

        self.type = -1 # -1 : stop, 1 : follow aruco, 2 : follow lane, 3 : turn
        self.epsilon = 1e-1
        self.pause = False
        self.path = None

        self.detection = Detection()
        self.pid_aruco_x = PID(0.1, 0.00001, 0.05)
        self.pid_aruco_z = PID(0.0005, 0.0, 0.005)
        self.pid_lane = PID(0.0005, 0.0, 0.005)

        self.default_speed = 0.2

        # aruco
        self.far_linear_x = 0.7
        self.find_aruco = None
        self.current_aruco_id = 0

        # lane
        self.max_desire_linear_vel = 0.3 # m/s
        self.max_omega = 33.51 # rad/s
        self.last_cte = 0
        self.angle_const = 0.01

        # performance
        self.counter = 0

        # env
        self.rate = 30 # camera frame rate
        self.start_time = None

        # rotate
        self.is_rotate = False
        self.rotate_duration = 5
        self.rotate_angle = math.pi / 2 / self.rotate_duration

        # forward
        self.is_forward = False
        self.forward_duration = 5
        self.forward_distance = 2.03 + 0.7 # 카메라 위치 + aruco 간의 거리

    def connect(self):
        # Publisher
        self.cmd_vel_pub = rospy.Publisher(f"/{self.robot_name}/cmd_vel", Twist, queue_size = 1)

        # Subscriber
        self.image_sub = rospy.Subscriber(f"/{self.robot_name}/camera/image_raw/compressed", CompressedImage, self.camera_callback)
        self.control_sub = rospy.Subscriber(f"/{self.robot_name}/advanced_control", AdvancedControl, self.advanced_control_callback)
        self.pause_sub = rospy.Subscriber(f"/{self.robot_name}/pause", Bool, self.pause_callback)

    def pause_callback(self, data):
        self.pause = data.data

    def camera_callback(self, data):

        # Dividing frame rate by 3 (10fps)
        # if self.counter % 3 != 0:
        #     self.counter += 1
        #     return
        # else:
        #     self.counter = 1

        # If no path, stop
        if self.path is None:
            self.move()
            return

        # If pause sign, stop
        if self.pause:
            self.move()
            return

        # If located in the center of aruco, rotate
        if self.is_rotate:
            if self.start_time is None:
                self.start_time = rospy.Time.now().to_sec()

            if self.turn == "r": # right
                self.move(0, -self.rotate_angle)
            else: # left and backward
                self.move(0, self.rotate_angle)

            k = 2 if self.turn == "b" else 1
            if rospy.Time.now().to_sec() - self.start_time >= self.rotate_duration * k:
                self.is_rotate = False
                self.type = 2 # follow lane
                self.start_time = None
            return

        # If arrived in aruco, forward
        if self.is_forward:
            if self.start_time is None:
                self.start_time = rospy.Time.now().to_sec()

            self.move(self.forward_distance / self.forward_duration, 0)

            if rospy.Time.now().to_sec() - self.start_time >= self.forward_duration:
                self.is_forward = False
                self.start_time = None
            return

        cmd_vel = Twist()

        try:
            #### direct conversion to CV2 ####
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except:
            print("Error conversion to CV2")

        if not self.current_aruco_id in self.path.keys():
            self.type = -1
            self.path = None
            rospy.loginfo("End of path")
            return

        lane_color = self.path[self.current_aruco_id]["lane_color"]
        next_aruco_id = self.path[self.current_aruco_id]["next_aruco_id"]
        self.turn = self.path[self.current_aruco_id]["turn"]


        corners, ids, final_img = self.detection.detectMarkers(cv_image)

        if self.type == 1: # follow aruco
            if ids is not None and ids[0][0] != next_aruco_id:
                cmd_vel.linear.x = self.default_speed

            elif self.find_aruco is not None or ids is not None:
                if ids is not None:
                    self.find_aruco = {"id":ids[0][0], "corners": corners}

                # estimate aruco marker position
                rvec, tvec, final_img = self.detection.estimatePoseSingleMarkers(self.find_aruco["corners"], final_img)

                tvec = tvec[0][0]
                tvec = [tvec[2], tvec[1], -tvec[0]] # transform (z, y, x) to (x, y, z)

                # PID
                cmd_vel.linear.x = self.pid_aruco_x.compute(tvec[0] - self.far_linear_x)
                cmd_vel.angular.z = self.pid_aruco_z.compute(tvec[2])

            else:
                cmd_vel.linear.x = self.default_speed

            # aruco에서 정지
            if self.pid_aruco_x.last_error != 0 and abs(self.pid_aruco_x.last_error) < self.epsilon:
                self.type = 3
                self.find_aruco = None
                self.pid_aruco_x.reset()
                self.pid_aruco_z.reset()
                self.pid_lane.reset()
                cmd_vel = Twist()

                self.current_aruco_id = next_aruco_id
                self.is_forward = True

        elif self.type == 2: # follow lane
            # lane_color에 따라 필터 변경하기

            if ids is not None and ids[0][0] == next_aruco_id:
                self.type = 1
                return

            cte, angle, final_img = self.detection.detectLane(cv_image, lane_color)

            if cte is not None and angle is not None:
                angular_z = self.pid_lane.compute(cte)
            else:
                angular_z = self.pid_lane.Kp * self.pid_lane.last_error * 1.9

            angular_z = max(angular_z, -2.0) if angular_z < 0 else min(angular_z, 2.0)
            cmd_vel.linear.x = self.default_speed
            cmd_vel.angular.z = angular_z

        elif self.type == 3: # turn
            if self.turn == "s":
                self.type = 2
            else:
                self.is_rotate = True
            return

        if self.detection.draw_img:
            cv2.putText(final_img, f"distance: {self.pid_aruco_x.last_error:.6f}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            cv2.putText(final_img, f"cmd: ({cmd_vel.linear.x:.6f},0,{cmd_vel.linear.z:.6f})", (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            cv2.putText(final_img, f"type: {self.type}, marker_id: {next_aruco_id}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            cv2.imshow("Original Image", final_img)
            cv2.waitKey(1)
        self.cmd_vel_pub.publish(cmd_vel)

    def advanced_control_callback(self, data):
        if data is None:
            return

        self.cmd = data.cmd
        self.dest = data.dest

        # dest에 따른 path 설정 => 현재 6번셀 하드코딩 되어있다.
        self.path = {
            1 : {"next_aruco_id": 3, "lane_color": "green", "turn": "s"},
            3 : {"next_aruco_id": 4, "lane_color": "green", "turn": "r"},
            4 : {"next_aruco_id": 6, "lane_color": "green", "turn": "l"}
        }
        self.current_aruco_id = 1
        self.type = 3 # turn

    def clean_up(self):
        cv2.destroyAllWindows()

    def move(self, x = 0, z = 0):
        cmd_vel = Twist()
        cmd_vel.linear.x = x
        cmd_vel.angular.z = z
        self.cmd_vel_pub.publish(cmd_vel)

def main():

    rospy.init_node("advanced_controller_node", anonymous=True)

    agv_controller_node = AGVAdvancedController()

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