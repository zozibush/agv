#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

class ObstaclePause:
    def __init__(self):
        self.robot_name = rospy.get_param("robot_name", "AGV_OTA1")
        self.max_range = float('inf')
        self.ranges = [self.max_range for _ in range(self.sensors)]
        self.start_time = rospy.get_time()
        self.min_range_above_threshold = False
        self.range_threshold = 1.0 # meters to obstacle
        self.duration_threshold = 3.0  # seconds to pause

        self.connect()

    def connect(self):
        # Publish to the pause topic
        self.pause_pub = rospy.Publisher(f'/{self.robot_name}/pause', Bool, queue_size=1)
        self.pause_pub.publish(False) # Initialize the pause state

        # Subscribe to the IR sensors
        self.sensors_sub = []
        for i in range(self.sensors):
            topic_name = f'/{self.robot_name}/ir{i+1}'
            self.sensors_sub.append(rospy.Subscriber(topic_name, Range, self.callback, callback_args=i))

    def callback(self, data, sensor_id): # sensor_id begins 0
        self.ranges[sensor_id] = data.range

        if self.get_min_range() <= self.range_threshold:
            self.start_time = rospy.get_time()
            if not self.min_range_above_threshold:
                self.pause_pub.publish(True)
                self.min_range_above_threshold = True
        else:
            if self.min_range_above_threshold and rospy.get_time() - self.start_time >= self.duration_threshold:
                self.reset_ranges()
                self.min_range_above_threshold = False

    def get_min_range(self):
        return min(self.ranges)

    def reset_ranges(self):
        self.ranges = [self.max_range for _ in range(self.sensors)]
        self.pause_pub.publish(False)

if __name__ == '__main__':
    rospy.init_node('ObstaclePause')
    obstaclePause = ObstaclePause()
    rospy.spin()
