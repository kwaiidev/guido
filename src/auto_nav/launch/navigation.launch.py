import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    auto_nav_share = get_package_share_directory('auto_nav')
    guido_bringup_share = get_package_share_directory('guido_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    slam_config = os.path.join(auto_nav_share, 'config', 'slam_toolbox.yaml')
    nav2_config = os.path.join(auto_nav_share, 'config', 'nav2.yaml')
    waypoint_file = os.path.join(os.getcwd(), '.guido', 'waypoints.yaml')
    map_output_dir = os.path.join(os.getcwd(), '.guido', 'maps')

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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_config,
            'autostart': 'true',
        }.items(),
    )

    auto_nav_command = Node(
        package='auto_nav',
        executable='auto_nav_command_node',
        name='auto_nav_command_node',
        output='screen',
        parameters=[
            {
                'mode': 'navigation',
                'waypoint_file': waypoint_file,
                'map_output_dir': map_output_dir,
            }
        ],
    )

    auto_nav_navigation = Node(
        package='auto_nav',
        executable='auto_nav_navigation_node',
        name='auto_nav_navigation_node',
        output='screen',
    )

    return LaunchDescription(
        [
            base_stack,
            slam_toolbox,
            nav2,
            auto_nav_command,
            auto_nav_navigation,
        ]
    )
