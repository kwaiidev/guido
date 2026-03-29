from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from navigation.adapters import CommandBridgeAdapter
from navigation.health import HealthMonitor
from navigation.messages import decode_navigation_result
from navigation.messages import encode_navigation_request
from navigation.supervisor import NavigationSupervisor
from navigation.types import CommandContext
from navigation.types import Pose2D
from navigation.waypoints import WaypointStore


class AutoNavCommandNode(Node):
    def __init__(self):
        super().__init__('auto_nav_command_node')

        _declare_parameter_if_missing(self, 'use_sim_time', False)
        _declare_parameter_if_missing(self, 'map_file', '')
        _declare_parameter_if_missing(self, 'active_map_id', '')
        _declare_parameter_if_missing(self, 'waypoint_file', str(Path('.guido') / 'waypoints.yaml'))
        _declare_parameter_if_missing(self, 'navigation_timeout', 10.0)
        _declare_parameter_if_missing(self, 'scan_timeout', 0.5)
        _declare_parameter_if_missing(self, 'odom_timeout', 2.0)
        _declare_parameter_if_missing(self, 'tf_timeout', 1.0)
        _declare_parameter_if_missing(self, 'timer_period', 0.2)
        _declare_parameter_if_missing(self, 'command_topic', '/auto_nav/command')
        _declare_parameter_if_missing(self, 'request_topic', '/auto_nav/request')
        _declare_parameter_if_missing(self, 'result_topic', '/auto_nav/result')
        _declare_parameter_if_missing(self, 'status_topic', '/auto_nav/status')
        _declare_parameter_if_missing(self, 'scan_topic', '/ldlidar_node/scan')
        _declare_parameter_if_missing(self, 'odom_topic', '/odom')
        _declare_parameter_if_missing(self, 'map_frame', 'map')
        _declare_parameter_if_missing(self, 'base_frame', 'base_link')

        map_file = self.get_parameter('map_file').value
        requested_map_id = self.get_parameter('active_map_id').value
        self._active_map_id = _derive_map_id(requested_map_id=requested_map_id, map_file=map_file)

        waypoint_store = WaypointStore(Path(self.get_parameter('waypoint_file').value).expanduser())
        health_monitor = HealthMonitor(
            scan_timeout=float(self.get_parameter('scan_timeout').value),
            odom_timeout=float(self.get_parameter('odom_timeout').value),
            tf_timeout=float(self.get_parameter('tf_timeout').value),
        )
        self._supervisor = NavigationSupervisor(
            waypoint_store=waypoint_store,
            health_monitor=health_monitor,
            active_map_id=self._active_map_id,
            navigation_timeout=float(self.get_parameter('navigation_timeout').value),
        )
        self._adapter = CommandBridgeAdapter(self._supervisor)

        self._command_topic = self.get_parameter('command_topic').value
        self._request_topic = self.get_parameter('request_topic').value
        self._result_topic = self.get_parameter('result_topic').value
        self._status_topic = self.get_parameter('status_topic').value
        self._scan_topic = self.get_parameter('scan_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._request_pub = self.create_publisher(String, self._request_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self.create_subscription(String, self._command_topic, self._on_command, 10)
        self.create_subscription(String, self._result_topic, self._on_navigation_result, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        timer_period = float(self.get_parameter('timer_period').value)
        self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            "auto_nav command node ready with map id '{}'.".format(self._active_map_id)
        )

    def _on_command(self, msg: String) -> None:
        current_pose = self._lookup_current_pose_if_needed(msg.data)
        context = CommandContext(current_pose=current_pose)
        try:
            dispatch = self._adapter.handle_text_command(msg.data, context)
        except ValueError as exc:
            self._publish_status(str(exc))
            return
        self._publish_dispatch(dispatch)

    def _on_navigation_result(self, msg: String) -> None:
        try:
            result = decode_navigation_result(msg.data)
        except ValueError as exc:
            self._publish_status(f'Ignoring invalid navigation result payload: {exc}')
            return
        dispatch = self._adapter.handle_navigation_result(result)
        self._publish_dispatch(dispatch)

    def _on_scan(self, _: LaserScan) -> None:
        self._supervisor._health_monitor.record_scan()

    def _on_odom(self, _: Odometry) -> None:
        self._supervisor._health_monitor.record_odom()

    def _on_timer(self) -> None:
        self._refresh_tf_health()
        dispatch = self._adapter.tick()
        self._publish_dispatch(dispatch)

    def _refresh_tf_health(self) -> None:
        if self._lookup_current_pose(report_failure=False) is not None:
            self._supervisor._health_monitor.record_tf()

    def _lookup_current_pose_if_needed(self, raw_command: str) -> Optional[Pose2D]:
        normalized = ' '.join(raw_command.strip().split()).lower()
        if not normalized:
            return None
        command_name = normalized.split(' ', 1)[0]
        if command_name == 'save_waypoint':
            return self._lookup_current_pose(report_failure=True)
        return self._lookup_current_pose(report_failure=False)

    def _lookup_current_pose(self, report_failure: bool = True) -> Optional[Pose2D]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            if report_failure:
                self._publish_status(f'Unable to resolve current pose: {exc}')
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return Pose2D(x=translation.x, y=translation.y, yaw=yaw)

    def _publish_dispatch(self, dispatch) -> None:
        if dispatch is None:
            return
        for message in dispatch.status_messages:
            self._publish_status(message)
        for request in dispatch.requests:
            outbound = String()
            outbound.data = encode_navigation_request(request)
            self._request_pub.publish(outbound)

    def _publish_status(self, message: str) -> None:
        outbound = String()
        outbound.data = message
        self._status_pub.publish(outbound)
        self.get_logger().info(message)


def _derive_map_id(requested_map_id: str, map_file: str) -> str:
    if requested_map_id:
        return requested_map_id
    if map_file:
        return Path(map_file).stem
    return 'live_map'


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    del x, y
    siny_cosp = 2.0 * (w * z)
    cosy_cosp = 1.0 - 2.0 * (z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _declare_parameter_if_missing(node: Node, name: str, default_value) -> None:
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)


def main(args=None):
    rclpy.init(args=args)
    node = AutoNavCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
