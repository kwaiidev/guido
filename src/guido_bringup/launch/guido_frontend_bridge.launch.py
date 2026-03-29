"""Launch the ROS-to-frontend telemetry bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('host', default_value='0.0.0.0'),
            DeclareLaunchArgument('port', default_value='8765'),
            Node(
                package='guido_navigation',
                executable='telemetry_bridge',
                name='guido_frontend_bridge',
                output='screen',
                parameters=[
                    {
                        'host': LaunchConfiguration('host'),
                        'port': LaunchConfiguration('port'),
                    }
                ],
            ),
        ]
    )
