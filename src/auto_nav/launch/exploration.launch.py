"""Launch Guido exploration mode with live mapping, Nav2, and auto-nav nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_nav_share = get_package_share_directory("auto_nav")
    guido_share = get_package_share_directory("guido_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    waypoint_file = LaunchConfiguration("waypoint_file")
    map_output_dir = LaunchConfiguration("map_output_dir")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value=os.path.expanduser("~/.guido/waypoints.yaml"),
            ),
            DeclareLaunchArgument(
                "map_output_dir",
                default_value=os.path.expanduser("~/.guido/maps"),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(auto_nav_share, "config", "nav2.yaml"),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(
                    auto_nav_share,
                    "config",
                    "slam_mapping.yaml",
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(guido_share, "launch", "guido_lidar.launch.py")
                )
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params_file,
                }.items(),
            ),
            Node(
                package="nav2_map_server",
                executable="map_saver_server",
                name="map_saver",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_saver",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_saver"],
                    }
                ],
            ),
            Node(
                package="auto_nav",
                executable="auto_nav_command_node",
                name="auto_nav_command_node",
                output="screen",
                parameters=[
                    {
                        "mode": "exploration",
                        "waypoint_file": waypoint_file,
                        "map_output_dir": map_output_dir,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="auto_nav",
                executable="auto_nav_navigation_node",
                name="auto_nav_navigation_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
