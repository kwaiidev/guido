"""Expose RViz-relevant ROS telemetry over a simple HTTP JSON API."""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
from typing import Any
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from urllib.parse import urlparse

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:
    import yaml
except ImportError:  # pragma: no cover - depends on system packages
    yaml = None


def _now() -> float:
    return time.time()


def _yaw_from_quaternion(orientation) -> float:
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return float(math.atan2(siny_cosp, cosy_cosp))


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('guido_frontend_telemetry_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/navigation/filtered_scan')
        self.declare_parameter('path_topic', '/navigation/global_path')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('fallback_odom_topic', '/odom')
        self.declare_parameter('status_topic', '/auto_nav/status')
        self.declare_parameter('waypoints_file', str(Path.home() / '.guido' / 'waypoints.yaml'))

        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            'started_at': _now(),
            'map': None,
            'map_version': 0,
            'scan': None,
            'path': None,
            'robot': None,
            'status': None,
            'waypoints': [],
            'topic_seen_at': {},
        }
        self._waypoints_file = Path(str(self.get_parameter('waypoints_file').value)).expanduser()
        self._waypoints_mtime_ns: int | None = None

        map_topic = str(self.get_parameter('map_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        fallback_odom_topic = str(self.get_parameter('fallback_odom_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 5)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_subscription(PathMessage, path_topic, self._on_path, 10)
        self.create_subscription(Odometry, odom_topic, self._on_primary_odom, 10)
        if fallback_odom_topic != odom_topic:
            self.create_subscription(Odometry, fallback_odom_topic, self._on_fallback_odom, 10)
        self.create_subscription(String, status_topic, self._on_status, 20)

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self._server = _TelemetryHttpServer((host, port), _RequestHandler, self)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name='guido-telemetry-http',
            daemon=True,
        )
        self._server_thread.start()

        self._refresh_waypoints()
        self.create_timer(1.0, self._refresh_waypoints)
        self.get_logger().info(f'Frontend telemetry bridge serving http://{host}:{port}')

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        return super().destroy_node()

    def build_summary_payload(self) -> dict[str, Any]:
        with self._lock:
            topic_seen_at = dict(self._state['topic_seen_at'])
            payload = {
                'serverTime': _now(),
                'startedAt': self._state['started_at'],
                'map': None,
                'robot': self._state['robot'],
                'scan': self._state['scan'],
                'path': self._state['path'],
                'status': self._state['status'],
                'waypoints': list(self._state['waypoints']),
                'freshness': _freshness_report(topic_seen_at),
            }
            if self._state['map'] is not None:
                payload['map'] = {
                    'version': self._state['map_version'],
                    'frameId': self._state['map']['frameId'],
                    'width': self._state['map']['width'],
                    'height': self._state['map']['height'],
                    'resolution': self._state['map']['resolution'],
                    'origin': self._state['map']['origin'],
                }
            return payload

    def build_map_payload(self) -> dict[str, Any]:
        with self._lock:
            if self._state['map'] is None:
                return {'version': 0, 'map': None}
            return {
                'version': self._state['map_version'],
                'map': self._state['map'],
            }

    def _on_map(self, message: OccupancyGrid) -> None:
        with self._lock:
            self._state['map_version'] += 1
            self._state['map'] = {
                'frameId': message.header.frame_id,
                'width': int(message.info.width),
                'height': int(message.info.height),
                'resolution': float(message.info.resolution),
                'origin': {
                    'x': float(message.info.origin.position.x),
                    'y': float(message.info.origin.position.y),
                },
                'data': [int(value) for value in message.data],
            }
            self._state['topic_seen_at']['map'] = _now()

    def _on_scan(self, message: LaserScan) -> None:
        ranges = []
        max_points = 180
        total = len(message.ranges)
        step = max(1, total // max_points)
        for index in range(0, total, step):
            ranges.append(round(float(message.ranges[index]), 3))

        with self._lock:
            self._state['scan'] = {
                'frameId': message.header.frame_id,
                'angleMin': round(float(message.angle_min), 4),
                'angleIncrement': round(float(message.angle_increment * step), 4),
                'rangeMin': round(float(message.range_min), 3),
                'rangeMax': round(float(message.range_max), 3),
                'ranges': ranges,
            }
            self._state['topic_seen_at']['scan'] = _now()

    def _on_path(self, message: PathMessage) -> None:
        points = []
        poses = message.poses
        step = max(1, len(poses) // 240)
        for pose_stamped in poses[::step]:
            points.append(
                {
                    'x': round(float(pose_stamped.pose.position.x), 3),
                    'y': round(float(pose_stamped.pose.position.y), 3),
                }
            )
        with self._lock:
            self._state['path'] = {
                'frameId': message.header.frame_id,
                'points': points,
            }
            self._state['topic_seen_at']['path'] = _now()

    def _on_primary_odom(self, message: Odometry) -> None:
        self._update_robot(message, source='filtered')

    def _on_fallback_odom(self, message: Odometry) -> None:
        self._update_robot(message, source='raw')

    def _update_robot(self, message: Odometry, *, source: str) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        robot = {
            'source': source,
            'frameId': message.header.frame_id,
            'childFrameId': message.child_frame_id,
            'x': round(float(pose.position.x), 3),
            'y': round(float(pose.position.y), 3),
            'yaw': round(_yaw_from_quaternion(pose.orientation), 3),
            'linear': round(float(twist.linear.x), 3),
            'angular': round(float(twist.angular.z), 3),
        }
        seen_at = _now()
        with self._lock:
            current = self._state['robot']
            if current is None or source == 'filtered' or current.get('source') != 'filtered':
                self._state['robot'] = robot
            self._state['topic_seen_at'][f'odom:{source}'] = seen_at

    def _on_status(self, message: String) -> None:
        with self._lock:
            self._state['status'] = {
                'text': message.data.strip(),
            }
            self._state['topic_seen_at']['status'] = _now()

    def _refresh_waypoints(self) -> None:
        if not self._waypoints_file.exists():
            with self._lock:
                self._state['waypoints'] = []
            self._waypoints_mtime_ns = None
            return

        stat = self._waypoints_file.stat()
        if self._waypoints_mtime_ns == stat.st_mtime_ns:
            return

        self._waypoints_mtime_ns = stat.st_mtime_ns
        records = _load_waypoints(self._waypoints_file)
        with self._lock:
            self._state['waypoints'] = records


class _TelemetryHttpServer(ThreadingHTTPServer):
    def __init__(self, address, handler, bridge: TelemetryBridge):
        super().__init__(address, handler)
        self.bridge = bridge


class _RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/state':
            self._send_json(self.server.bridge.build_summary_payload())
            return
        if parsed.path == '/api/map':
            self._send_json(self.server.bridge.build_map_payload())
            return
        if parsed.path == '/health':
            self._send_json({'ok': True, 'time': _now()})
            return
        self.send_error(HTTPStatus.NOT_FOUND, 'Not found')

    def log_message(self, fmt, *args):
        del fmt, args

    def _send_json(self, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload).encode('utf-8')
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


def _freshness_report(topic_seen_at: dict[str, float]) -> dict[str, dict[str, Any]]:
    now = _now()
    report: dict[str, dict[str, Any]] = {}
    for name, seen_at in topic_seen_at.items():
        age = max(0.0, now - seen_at)
        threshold = 1.5 if 'odom' in name else 2.5
        report[name] = {
            'ageSec': round(age, 2),
            'fresh': age <= threshold,
        }
    return report


def _load_waypoints(path: Path) -> list[dict[str, Any]]:
    raw = path.read_text(encoding='utf-8').strip()
    if not raw:
        return []
    if yaml is not None:
        data = yaml.safe_load(raw)
    else:
        import json as std_json

        data = std_json.loads(raw)
    if not isinstance(data, dict):
        return []
    section = data.get('waypoints', {})
    if isinstance(section, list):
        items = section
    elif isinstance(section, dict):
        items = []
        for key in sorted(section.keys()):
            record = section[key]
            if isinstance(record, dict):
                items.append({'name': record.get('name', key), **record})
    else:
        return []

    waypoints = []
    for item in items:
        if not isinstance(item, dict):
            continue
        pose = item.get('pose')
        if isinstance(pose, dict):
            x = float(pose.get('x', 0.0))
            y = float(pose.get('y', 0.0))
            yaw = float(pose.get('yaw', 0.0))
        else:
            x = float(item.get('x', 0.0))
            y = float(item.get('y', 0.0))
            yaw = float(item.get('yaw', 0.0))
        waypoints.append(
            {
                'name': str(item.get('name', 'waypoint')),
                'x': round(x, 3),
                'y': round(y, 3),
                'yaw': round(yaw, 3),
            }
        )
    return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
