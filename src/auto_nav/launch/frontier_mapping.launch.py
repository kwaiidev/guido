"""Launch SLAM + Nav2 infrastructure for frontier-based autonomous mapping.

Brings up the base stack (serial bridge, URDF, lidar), SLAM Toolbox,
Nav2, cmd_vel bridge to /cmd_vel, and map_saver.  The frontier_explorer node is NOT
included here because it needs interactive stdin access -- run it
in a separate terminal:

    Terminal 1:  ros2 launch auto_nav frontier_mapping.launch.py
    Terminal 2:  ros2 run auto_nav frontier_explorer
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    auto_nav_share = get_package_share_directory('auto_nav')
    guido_bringup_share = get_package_share_directory('guido_bringup')
    slam_config = os.path.join(auto_nav_share, 'config', 'slam_toolbox.yaml')
    nav2_config = os.path.join(auto_nav_share, 'config', 'nav2.yaml')
    # EKF publishes /odometry/filtered; match bt_navigator odom_topic to that topic.
    nav2_params_for_frontier = RewrittenYaml(
        source_file=nav2_config,
        root_key='',
        param_rewrites={'odom_topic': '/odometry/filtered'},
        convert_types=True,
    )
    rviz_config = os.path.join(auto_nav_share, 'config', 'frontier_mapping.rviz')
    fastrtps_profile = os.path.join(
        auto_nav_share, 'config', 'fastrtps_profile.xml')

    # Disable FastRTPS shared-memory transport; UDP-only is more reliable
    # on memory-constrained boards (Jetson) where SHM causes DDS discovery
    # failures between lifecycle managers and Nav2 servers.
    set_fastrtps = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_profile)

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                guido_bringup_share, 'launch', 'guido_lidar.launch.py')
        ),
        launch_arguments={'use_ekf': 'true'}.items(),
    )

    # No static odom->base_footprint: EKF publishes that transform for this launch.

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                auto_nav_share, 'launch', 'guido_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_for_frontier,
            'autostart': 'true',
        }.items(),
    )

    cmd_vel_bridge = Node(
        package='auto_nav',
        executable='cmd_vel_bridge',
        name='nav_cmd_vel_bridge',
        output='screen',
        parameters=[
            {'input_topic': '/cmd_vel_nav', 'output_topic': '/cmd_vel'},
        ],
    )

    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    map_saver_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_saver_server']},
        ],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        set_fastrtps,
        base_stack,
        slam_toolbox,
        nav2,
        cmd_vel_bridge,
        map_saver_server,
        TimerAction(period=12.0, actions=[map_saver_lifecycle_manager]),
        rviz2,
    ])
