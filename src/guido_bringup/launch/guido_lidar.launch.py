"""Launch the full Guido wheelchair stack.

Starts:
  1. robot_state_publisher  - publishes the Guido URDF TF tree
  2. guido_serial_bridge    - cmd_vel <-> Arduino serial <-> odom (+ TF if no EKF)
  3. ekf_filter_node        - optional: fuses /odom + /imu/data_raw, publishes odom TF
  4. ldlidar_component      - LDLidar driver in a component container (scan)
  5. lifecycle_manager      - auto-configures and activates the lidar node

The upstream ``ldlidar_node`` package does not ship a standalone executable; the driver is
``ldlidar::LdLidarComponent`` loaded into ``component_container_isolated``, same as
``ldlidar_bringup.launch.py`` in the LDLidar repo (without their duplicate lidar URDF publisher).

When ``use_ekf`` is true, the serial bridge sets ``publish_tf`` false so the EKF is the sole
publisher of odom -> base_footprint. Point Nav2 ``bt_navigator.odom_topic`` at
``/odometry/filtered`` in that configuration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    guido_bringup_share = get_package_share_directory('guido_bringup')
    guido_base_share = get_package_share_directory('guido_base')

    urdf_path = os.path.join(guido_bringup_share, 'urdf', 'guido.urdf.xml')
    with open(urdf_path, 'r') as handle:
        robot_desc = handle.read()

    lc_mgr_config = os.path.join(guido_bringup_share, 'config', 'lifecycle_mgr.yaml')
    ldlidar_config = os.path.join(guido_bringup_share, 'config', 'ldlidar.yaml')
    bridge_config = os.path.join(guido_base_share, 'config', 'serial_bridge.yaml')
    ekf_config = os.path.join(guido_bringup_share, 'config', 'ekf.yaml')

    use_ekf_str = LaunchConfiguration('use_ekf').perform(context).lower()
    use_ekf = use_ekf_str in ('true', '1', 'yes')

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
        parameters=[
            bridge_config,
            {'publish_tf': not use_ekf},
        ],
    )

    ldlidar_container = ComposableNodeContainer(
        name='ldlidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[],
        output='screen',
    )

    ldlidar_component = ComposableNode(
        package='ldlidar_component',
        plugin='ldlidar::LdLidarComponent',
        name='ldlidar_node',
        namespace='',
        parameters=[ldlidar_config],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    load_ldlidar = LoadComposableNodes(
        target_container=ldlidar_container,
        composable_node_descriptions=[ldlidar_component],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[lc_mgr_config],
    )

    nodes = [
        guido_state_publisher,
        serial_bridge,
        ldlidar_container,
        load_ldlidar,
        lifecycle_manager,
    ]

    if use_ekf:
        ekf_filter = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': False}],
        )
        nodes.insert(2, ekf_filter)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ekf',
            default_value='false',
            description='If true, run robot_localization EKF (/odometry/filtered, odom TF) '
                        'and disable serial_bridge TF.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
