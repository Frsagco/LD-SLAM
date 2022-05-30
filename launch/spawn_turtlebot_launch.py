from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import launch.actions
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():

    turtlebot = Node(
            package='ld_slam',
            executable='spawn_turtlebot.py',
            output='screen',
            arguments=[
                '--robot_sdf', launch.substitutions.LaunchConfiguration('robot_sdf'),
                '--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-z', launch.substitutions.LaunchConfiguration('z')]
        )


    return LaunchDescription([
        turtlebot
    ])
