"""Launch Guido navigation mode with SLAM localization, Nav2, and auto-nav nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_nav_share = get_package_share_directory('auto_nav')
    guido_share = get_package_share_directory('guido_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    waypoint_file = LaunchConfiguration('waypoint_file')
    active_map_id = LaunchConfiguration('active_map_id')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('map_file', default_value=''),
            DeclareLaunchArgument(
                'waypoint_file',
                default_value=os.path.expanduser('~/.guido/waypoints.yaml'),
            ),
            DeclareLaunchArgument('active_map_id', default_value=''),
            DeclareLaunchArgument(
                'nav2_params_file',
                default_value=os.path.join(auto_nav_share, 'config', 'nav2.yaml'),
            ),
            DeclareLaunchArgument(
                'slam_params_file',
                default_value=os.path.join(
                    auto_nav_share,
                    'config',
                    'slam_localization.yaml',
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(guido_share, 'launch', 'guido_lidar.launch.py')
                )
            ),
            Node(
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {
                        'map_file_name': map_file,
                        'use_sim_time': use_sim_time,
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                }.items(),
            ),
            Node(
                package='auto_nav',
                executable='auto_nav_command_node',
                name='auto_nav_command_node',
                output='screen',
                parameters=[
                    {
                        'mode': 'navigation',
                        'map_file': map_file,
                        'active_map_id': active_map_id,
                        'waypoint_file': waypoint_file,
                        'use_sim_time': use_sim_time,
                    }
                ],
            ),
            Node(
                package='auto_nav',
                executable='auto_nav_navigation_node',
                name='auto_nav_navigation_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )
