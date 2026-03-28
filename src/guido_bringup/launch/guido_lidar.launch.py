"""Launch the Guido wheelchair LiDAR stack.

Starts:
  1. robot_state_publisher  – publishes the Guido URDF (base_link -> ldlidar_base)
  2. ldlidar_bringup        – LDLidar component node (publishes ldlidar_base -> ldlidar_link)
  3. lifecycle_manager      – auto-configures and activates the lidar node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    guido_share = get_package_share_directory('guido_bringup')
    ldlidar_share = get_package_share_directory('ldlidar_node')

    urdf_path = os.path.join(guido_share, 'urdf', 'guido.urdf.xml')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    lidar_config = os.path.join(guido_share, 'config', 'ldlidar.yaml')
    lc_mgr_config = os.path.join(guido_share, 'config', 'lifecycle_mgr.yaml')

    guido_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='guido_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    ldlidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_share, 'launch', 'ldlidar_bringup.launch.py')
        ),
        launch_arguments={'node_name': 'ldlidar_node'}.items(),
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
        ldlidar_bringup,
        lifecycle_manager,
    ])
