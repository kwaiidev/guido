import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    auto_nav_share = get_package_share_directory('auto_nav')
    guido_bringup_share = get_package_share_directory('guido_bringup')

    slam_config = os.path.join(auto_nav_share, 'config', 'slam_toolbox.yaml')
    nav2_config = os.path.join(auto_nav_share, 'config', 'nav2.yaml')
    rviz_config = os.path.join(auto_nav_share, 'config', 'frontier_mapping.rviz')
    twist_mux_config = os.path.join(guido_bringup_share, 'config', 'twist_mux.yaml')
    # Match keyboard_teleop (~/.guido/waypoints.yaml).
    waypoint_file = os.path.join(os.path.expanduser('~'), '.guido', 'waypoints.yaml')

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
            os.path.join(auto_nav_share, 'launch', 'guido_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_config,
            'autostart': 'true',
        }.items(),
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[('cmd_vel_out', '/cmd_vel')],
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

    auto_nav_command = Node(
        package='auto_nav',
        executable='auto_nav_command_node',
        name='auto_nav_command_node',
        output='screen',
        parameters=[
            {
                'waypoint_file': waypoint_file,
            }
        ],
    )

    auto_nav_navigation = Node(
        package='auto_nav',
        executable='auto_nav_navigation_node',
        name='auto_nav_navigation_node',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription(
        [
            base_stack,
            slam_toolbox,
            nav2,
            twist_mux,
            map_saver_server,
            TimerAction(period=12.0, actions=[map_saver_lifecycle_manager]),
            auto_nav_command,
            auto_nav_navigation,
            rviz2,
        ]
    )
