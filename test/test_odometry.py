#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from sensor_msgs.msg import PointCloud2
import queue
import sys

def get_topic_point():
    if(not len(sys.argv) == 2):
        dir=input("Give me bag dataset folder path: ")
    else:
        dir=sys.argv[1]

    info=subprocess.run(['ros2', 'bag', 'info', dir], stdout=subprocess.PIPE).stdout.decode('utf-8')

    if len(info.splitlines()) > 8:
        topic=info.splitlines()[8].split(" ")[3]
    else:
        print("Give me a valid path. Abort.")
        return

    return topic

class TestOdometry(Node):
    def __init__(self):
        super().__init__("test_odometry")
        self.get_logger().info("Test odometry running")

        
        self.name_robot="/robot1"
        self.topic_point_=get_topic_point()

        self.x=[]
        self.y=[]
        self.z=[]
        self.point_cloud=queue.Queue()

        self.topic_name=self.name_robot +"/current_pose"
        self.odometry_subscriber = self.create_subscription(
            PoseStamped, "/curr_pos", self.callback_odometry, 100)
        self.point_cloud_subscriber = self.create_subscription(
            PointCloud2, self.topic_point_, self.callback_point_cloud, 300)
        self.point_cloud_publisher_ = self.create_publisher(PointCloud2, "input_cloud", 100)

        self.count_point=0
        self.count=0
        self.count_pub_point=0
        self.odometry_timer_ = self.create_timer(60, self.odometry_result)
        self.point_cloud_timer_ = self.create_timer(0.7, self.publish_point_cloud)


    def callback_odometry(self, msg):
        # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot1/cmd_vel
        print("[" + str(self.count) +"]: [" + str(self.count_point) +"]:" + "[" + str(self.count_pub_point) + "]:" + " Current pose of [" + self.name_robot + "]: x=" + str(msg.pose.position.x) + " y=" + str(msg.pose.position.y) + " z=" + str(msg.pose.position.z))
        self.x.append(msg.pose.position.x)
        self.y.append(msg.pose.position.y)
        self.z.append(msg.pose.position.z)
        
        self.count=self.count+1
        
    def callback_point_cloud(self, msg):
        self.count_point=self.count_point+1
        self.point_cloud.put(msg)
    

    def odometry_result(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot3D(self.x, self.y, self.z, 'gray')
        plt.show()
            
    def publish_point_cloud(self):
        if(not self.point_cloud.empty()):
            self.count_pub_point=self.count_pub_point+1

            self.point_cloud_publisher_.publish(self.point_cloud.get())

def main(args=None):
    rclpy.init(args=args)
    node = TestOdometry()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
