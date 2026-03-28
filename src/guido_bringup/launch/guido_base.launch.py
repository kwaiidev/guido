"""Launch Guido base control without the LiDAR stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    guido_bringup_share = get_package_share_directory('guido_bringup')
    guido_base_share = get_package_share_directory('guido_base')

    urdf_path = os.path.join(guido_bringup_share, 'urdf', 'guido.urdf.xml')
    with open(urdf_path, 'r') as handle:
        robot_desc = handle.read()

    bridge_config = os.path.join(guido_base_share, 'config', 'serial_bridge.yaml')

    guido_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='guido_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    serial_bridge = Node(
        package='guido_base',
        executable='serial_bridge',
        name='guido_serial_bridge',
        output='screen',
        parameters=[bridge_config],
    )

    return LaunchDescription([
        guido_state_publisher,
        serial_bridge,
    ])
