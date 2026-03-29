"""Launch the full Guido wheelchair stack.

Starts:
  1. robot_state_publisher  - publishes the Guido URDF TF tree
  2. guido_serial_bridge    - cmd_vel <-> Arduino serial <-> odom + TF
  3. ldlidar_bringup        - LDLidar driver (scan topic + ldlidar TF)
  4. lifecycle_manager      - auto-configures and activates the lidar node
"""

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

    lc_mgr_config = os.path.join(guido_bringup_share, 'config', 'lifecycle_mgr.yaml')
    ldlidar_config = os.path.join(guido_bringup_share, 'config', 'ldlidar.yaml')
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

    ldlidar_bringup = Node(
        package='ldlidar_node',
        executable='ldlidar_node',
        name='ldlidar_node',
        output='screen',
        parameters=[ldlidar_config],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[lc_mgr_config],
    )

    return LaunchDescription([
        guido_state_publisher,
        serial_bridge,
        ldlidar_bringup,
        lifecycle_manager,
    ])
