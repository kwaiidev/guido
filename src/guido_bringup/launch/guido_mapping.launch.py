"""Bring up Guido's autonomy stack in live SLAM mapping mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    guido_bringup_share = get_package_share_directory('guido_bringup')
    guido_base_share = get_package_share_directory('guido_base')
    guido_navigation_share = get_package_share_directory('guido_navigation')
    ldlidar_share = get_package_share_directory('ldlidar_node')

    urdf_path = os.path.join(guido_bringup_share, 'urdf', 'guido.urdf.xml')
    with open(urdf_path, 'r') as handle:
        robot_description = handle.read()

    bridge_config = os.path.join(guido_base_share, 'config', 'serial_bridge.yaml')
    lidar_lifecycle_config = os.path.join(guido_bringup_share, 'config', 'lifecycle_mgr.yaml')
    ekf_config = os.path.join(guido_bringup_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(guido_bringup_share, 'config', 'slam_toolbox.yaml')
    perception_config = os.path.join(guido_navigation_share, 'config', 'lidar_perception.yaml')
    localization_config = os.path.join(guido_navigation_share, 'config', 'localization_bridge.yaml')
    planner_config = os.path.join(guido_navigation_share, 'config', 'planner.yaml')
    controller_config = os.path.join(guido_navigation_share, 'config', 'controller.yaml')
    command_mux_config = os.path.join(guido_navigation_share, 'config', 'command_mux.yaml')
    safety_config = os.path.join(guido_navigation_share, 'config', 'safety.yaml')
    mission_manager_config = os.path.join(guido_navigation_share, 'config', 'mission_manager.yaml')
    rviz_config = os.path.join(guido_bringup_share, 'rviz', 'guido_navigation.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_ekf = LaunchConfiguration('use_ekf')
    serial_publish_tf = LaunchConfiguration('serial_publish_tf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    allow_odom_fallback = LaunchConfiguration('allow_odom_fallback')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='guido_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
        ],
    )

    serial_bridge = Node(
        package='guido_base',
        executable='serial_bridge',
        name='guido_serial_bridge',
        output='screen',
        parameters=[
            bridge_config,
            {
                'publish_tf': serial_publish_tf,
                'cmd_topic': 'cmd_vel',
            },
        ],
    )

    ldlidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_share, 'launch', 'ldlidar_bringup.launch.py')
        ),
        launch_arguments={'node_name': 'ldlidar_node'}.items(),
    )

    lidar_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lidar_lifecycle_manager',
        output='screen',
        parameters=[lidar_lifecycle_config],
    )

    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    lidar_perception = Node(
        package='guido_navigation',
        executable='lidar_perception',
        name='guido_lidar_perception',
        output='screen',
        parameters=[perception_config],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )

    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    localization_bridge = Node(
        package='guido_navigation',
        executable='localization_bridge',
        name='guido_localization_bridge',
        output='screen',
        parameters=[
            localization_config,
            {'allow_odom_fallback': allow_odom_fallback},
        ],
    )

    mission_manager = Node(
        package='guido_navigation',
        executable='mission_manager',
        name='guido_mission_manager',
        output='screen',
        parameters=[mission_manager_config],
    )

    global_planner = Node(
        package='guido_navigation',
        executable='global_planner',
        name='guido_global_planner',
        output='screen',
        parameters=[planner_config],
    )

    trajectory_controller = Node(
        package='guido_navigation',
        executable='trajectory_controller',
        name='guido_trajectory_controller',
        output='screen',
        parameters=[controller_config],
    )

    command_mux = Node(
        package='guido_navigation',
        executable='command_mux',
        name='guido_command_mux',
        output='screen',
        parameters=[command_mux_config],
    )

    safety_monitor = Node(
        package='guido_navigation',
        executable='safety_monitor',
        name='guido_safety_monitor',
        output='screen',
        parameters=[safety_config],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='guido_rviz',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('serial_publish_tf', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('allow_odom_fallback', default_value='false'),
        robot_state_publisher,
        serial_bridge,
        ldlidar_bringup,
        lidar_lifecycle_manager,
        ekf_filter,
        lidar_perception,
        slam_toolbox,
        map_saver_server,
        localization_bridge,
        mission_manager,
        global_planner,
        trajectory_controller,
        command_mux,
        safety_monitor,
        rviz,
    ])
