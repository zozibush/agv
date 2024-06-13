#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import tf.transformations as tf
import math

class ObjectMover:
    def __init__(self):
        self.robot_name = rospy.get_param("robot_name", "AGV_OTA1")
        self.model_name = rospy.get_param("model_name", "prius_hybrid")

        self.odom_sub = rospy.Subscriber(f"/{self.robot_name}/odom", Odometry, self.odom_callback)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


    def odom_callback(self, data):
        new_state = ModelState()
        new_state.model_name = f"{self.robot_name}_{self.model_name}"
        new_state.pose = data.pose.pose
        new_state.pose.position.z += 0.3

        existing_quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

        roll = 0.0
        pitch = 0.0
        yaw = math.radians(90)

        additional_quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

        x, y, z, w = tf.quaternion_multiply(existing_quaternion, additional_quaternion)
        new_state.pose.orientation.x = x
        new_state.pose.orientation.y = y
        new_state.pose.orientation.z = z
        new_state.pose.orientation.w = w
        new_state.twist = data.twist.twist

        try:
            self.set_model_state(new_state)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('object_mover', anonymous=True)
    mover = ObjectMover()
    mover.run()