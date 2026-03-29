"""Mission manager for waypoint goals, RViz goals, and voice commands."""

from __future__ import annotations

import math
from pathlib import Path
import re
from typing import Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class MissionManager(Node):
    def __init__(self):
        super().__init__('guido_mission_manager')

        default_waypoints = Path(get_package_share_directory('guido_navigation')) / 'config' / 'waypoints.yaml'

        self.declare_parameter('waypoints_file', str(default_waypoints))
        self.declare_parameter('voice_topic', '/voice/command_text')
        self.declare_parameter('rviz_goal_topic', '/goal_pose')
        self.declare_parameter('goal_topic', '/navigation/goal')
        self.declare_parameter('mode_topic', '/system/mode')
        self.declare_parameter('mission_status_topic', '/navigation/mission_status')

        waypoints_file = Path(str(self.get_parameter('waypoints_file').value))
        voice_topic = str(self.get_parameter('voice_topic').value)
        rviz_goal_topic = str(self.get_parameter('rviz_goal_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        mode_topic = str(self.get_parameter('mode_topic').value)
        mission_status_topic = str(self.get_parameter('mission_status_topic').value)

        self._goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self._mode_pub = self.create_publisher(String, mode_topic, 10)
        self._status_pub = self.create_publisher(String, mission_status_topic, 10)

        self.create_subscription(String, voice_topic, self._voice_cb, 10)
        self.create_subscription(PoseStamped, rviz_goal_topic, self._rviz_goal_cb, 10)

        self._waypoints = self._load_waypoints(waypoints_file)
        self.get_logger().info(f'Mission manager online with {len(self._waypoints)} named waypoints.')

    def _load_waypoints(self, path: Path) -> dict[str, dict[str, Any]]:
        if not path.exists():
            raise FileNotFoundError(f'Waypoint file not found: {path}')

        data = yaml.safe_load(path.read_text()) or {}
        return dict(data.get('waypoints', {}))

    def _voice_cb(self, message: String):
        text = self._normalize(message.data)
        if not text:
            return

        if any(word in text for word in ('stop', 'halt', 'cancel')):
            self._publish_mode('hold')
            self._publish_status('Mission stopped by voice command.')
            return

        if 'manual mode' in text or text == 'manual':
            self._publish_mode('manual')
            self._publish_status('Switched to manual mode.')
            return

        if 'auto mode' in text or 'resume autonomy' in text or text == 'auto':
            self._publish_mode('auto')
            self._publish_status('Switched to autonomy mode.')
            return

        waypoint_name = self._resolve_waypoint_name(text)
        if waypoint_name is None:
            self._publish_status(
                'Voice command not matched to a waypoint. Try "go to parking lot" or "go to waypoint 2".'
            )
            return

        self._dispatch_waypoint(waypoint_name, source='voice')

    def _rviz_goal_cb(self, message: PoseStamped):
        self._goal_pub.publish(message)
        self._publish_mode('auto')
        self._publish_status(
            f'Accepted RViz goal in frame {message.header.frame_id or "map"} at '
            f'({message.pose.position.x:.2f}, {message.pose.position.y:.2f}).'
        )

    def _resolve_waypoint_name(self, text: str) -> str | None:
        cleaned = text.replace('_', ' ')

        direct_match = re.search(r'(?:go to|drive to|navigate to|head to)\s+(.+)$', cleaned)
        if direct_match:
            cleaned = direct_match.group(1).strip()

        if cleaned in self._waypoints:
            return cleaned

        if re.fullmatch(r'waypoint\s+\d+', cleaned):
            candidate = cleaned.replace(' ', '_')
            if candidate in self._waypoints:
                return candidate

        for name, waypoint in self._waypoints.items():
            aliases = waypoint.get('aliases', [])
            normalized_aliases = {name.lower().replace('_', ' ')}
            normalized_aliases.update(alias.lower() for alias in aliases)
            if cleaned in normalized_aliases:
                return name

        return None

    def _dispatch_waypoint(self, waypoint_name: str, *, source: str):
        waypoint = self._waypoints[waypoint_name]

        goal = PoseStamped()
        goal.header.frame_id = str(waypoint.get('frame_id', 'map'))
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(waypoint['x'])
        goal.pose.position.y = float(waypoint['y'])
        yaw = float(waypoint.get('yaw', 0.0))
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self._goal_pub.publish(goal)
        self._publish_mode('auto')
        self._publish_status(
            f'Accepted {source} mission to {waypoint_name} at '
            f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}).'
        )

    def _publish_mode(self, mode: str):
        message = String()
        message.data = mode
        self._mode_pub.publish(message)

    def _publish_status(self, text: str):
        message = String()
        message.data = text
        self._status_pub.publish(message)
        self.get_logger().info(text)

    @staticmethod
    def _normalize(text: str) -> str:
        cleaned = ' '.join(text.strip().lower().split())
        cleaned = re.sub(r'[!?.,]+', ' ', cleaned)
        return re.sub(r'\s+', ' ', cleaned).strip()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
