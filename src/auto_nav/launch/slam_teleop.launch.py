"""Launch lidar + SLAM + rviz with the real Guido base stack.

Includes guido_lidar.launch.py (serial bridge, URDF, lidar) so that
keyboard_teleop can drive the robot while SLAM builds a map in rviz.

Usage:
    Terminal 1:  ros2 launch auto_nav slam_teleop.launch.py
    Terminal 2:  ros2 run guido_base keyboard_teleop
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    auto_nav_share = get_package_share_directory('auto_nav')
    guido_bringup_share = get_package_share_directory('guido_bringup')
    ldlidar_node_share = get_package_share_directory('ldlidar_node')

    slam_config = os.path.join(auto_nav_share, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(ldlidar_node_share, 'config', 'ldlidar_slam.rviz')

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(guido_bringup_share, 'launch', 'guido_lidar.launch.py')
        )
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        base_stack,
        slam_toolbox,
        rviz2,
    ])
