import os

import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.01},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'z_pose': 0.01}]

    urdf_file_name = 'turtlebot3_waffle.urdf'

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        urdf_model = infp.read()


    odometry = launch_ros.actions.Node(
        package='ld_slam',
        executable='odometry_node',
        output='screen'
        )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','/base_link','/velodyne']
        )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    #state_publisher_commands = []
    state_publisher_commands=LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'),

            launch_ros.actions.Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace='robot1/',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                            'robot_description': urdf_model}],
                ),
        ])


    return launch.LaunchDescription([
        odometry,
        tf,
       # state_publisher_commands,
    ])