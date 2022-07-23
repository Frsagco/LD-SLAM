#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d



class TestOdometry(Node):
    def __init__(self):
        super().__init__("test_odometry")

        self.name_robot="/robot1"
        self.topic_name=self.name_robot +"/current_pose"
        self.odometry_subscriber = self.create_subscription(
            PoseStamped, "/curr_pos", self.callback_odometry, 30)
        self.get_logger().info("Test odometry running")
        self.x=[]
        self.y=[]
        self.z=[]

        self.count=0
        self.timer_ = self.create_timer(60, self.odometry_result)



    def callback_odometry(self, msg):
        # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot1/cmd_vel
        print("[" + str(self.count) +"]: Current pose of [" + self.name_robot + "]: x=" + str(msg.pose.position.x) + " y=" + str(msg.pose.position.y) + " z=" + str(msg.pose.position.z))
        self.x.append(msg.pose.position.x)
        self.y.append(msg.pose.position.y)
        self.z.append(msg.pose.position.z)
        
        self.count=self.count+1
        
    def odometry_result(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot3D(self.x, self.y, self.z, 'gray')
        plt.show()
            


def main(args=None):
    rclpy.init(args=args)
    node = TestOdometry()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
