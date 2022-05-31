#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from geometry_msgs.msg import PoseStamped

class TestOdometry(Node):
    def __init__(self):
        super().__init__("test_odometry")

        self.name_robot="/robot0"
        self.topic_name=self.name_robot +"/current_pose"
        self.odometry_subscriber = self.create_subscription(
            PoseStamped, "curr_pos", self.callback_odometry, 10)
        self.get_logger().info("Test odometry running")


    def callback_odometry(self, msg):
        # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot0/cmd_vel
        msg=PoseStamped()
        print("Current pose of [" + self.name_robot + "]: x=" + str(msg.pose.position.x) + " y=" + str(msg.pose.position.y) + " z=" + str(msg.pose.position.z))


def main(args=None):
    rclpy.init(args=args)
    node = TestOdometry()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
