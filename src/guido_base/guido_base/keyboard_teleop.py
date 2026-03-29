"""Keyboard teleop with waypoint saving and RViz marker publishing."""

import math
from pathlib import Path
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from visualization_msgs.msg import Marker, MarkerArray

try:
    import yaml
except ImportError:
    yaml = None


WAYPOINT_DIR = Path.home() / '.guido'
WAYPOINT_FILE = WAYPOINT_DIR / 'waypoints.yaml'

HELP = """\
Guido keyboard teleop
---------------------
w / up    : forward
s / down  : backward
a / left  : turn left
d / right : turn right
space / x : stop
q / z     : increase / decrease linear speed
e / c     : increase / decrease angular speed
p         : save waypoint at current position
l         : list saved waypoints
CTRL-C    : quit
"""


def _load_waypoints() -> list:
    if not WAYPOINT_FILE.exists():
        return []
    raw = WAYPOINT_FILE.read_text(encoding='utf-8').strip()
    if not raw:
        return []
    if yaml is not None:
        data = yaml.safe_load(raw)
    else:
        import json
        data = json.loads(raw)
    if not isinstance(data, dict):
        return []
    wps = data.get('waypoints', [])
    return wps if isinstance(wps, list) else []


def _save_waypoints(waypoints: list) -> None:
    WAYPOINT_DIR.mkdir(parents=True, exist_ok=True)
    data = {'waypoints': waypoints}
    if yaml is not None:
        text = yaml.safe_dump(data, sort_keys=False)
    else:
        import json
        text = json.dumps(data, indent=2)
    WAYPOINT_FILE.write_text(text, encoding='utf-8')


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('guido_keyboard_teleop')

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)

        self._pub = self.create_publisher(Twist, self._cmd_topic, 1)
        self._marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self._current = Twist()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._waypoints = _load_waypoints()
        self._wp_counter = len(self._waypoints)
        self._last_marker_time = 0.0

    def _print_status(self):
        self.get_logger().info(
            f'linear_speed={self._linear_speed:.2f} angular_speed={self._angular_speed:.2f}'
        )

    def _publish_current(self):
        self._pub.publish(self._current)

    def _set_motion(self, linear_x: float, angular_z: float):
        self._current = Twist()
        self._current.linear.x = linear_x
        self._current.angular.z = angular_z
        self._publish_current()

    def _get_robot_pose(self):
        """Look up robot pose in the map frame. Returns (x, y, yaw) or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = _yaw_from_quaternion(t.transform.rotation)
            return x, y, yaw
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def _save_waypoint(self):
        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().error('Cannot save waypoint: TF not available')
            return
        x, y, yaw = pose
        self._wp_counter += 1
        name = f'wp_{self._wp_counter}'
        wp = {'name': name, 'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        self._waypoints.append(wp)
        _save_waypoints(self._waypoints)
        self.get_logger().info(f'Saved {name} at x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
        self._publish_markers()

    def _list_waypoints(self):
        if not self._waypoints:
            self.get_logger().info('No waypoints saved')
            return
        self.get_logger().info(f'{len(self._waypoints)} waypoint(s):')
        for wp in self._waypoints:
            self.get_logger().info(
                f"  {wp['name']}: x={wp['x']:.3f} y={wp['y']:.3f} yaw={wp['yaw']:.3f}"
            )

    def _publish_markers(self):
        msg = MarkerArray()

        if not self._waypoints:
            delete_all = Marker()
            delete_all.action = Marker.DELETEALL
            msg.markers.append(delete_all)
            self._marker_pub.publish(msg)
            return

        for i, wp in enumerate(self._waypoints):
            sphere = Marker()
            sphere.header.frame_id = self._map_frame
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoint_spheres'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(wp['x'])
            sphere.pose.position.y = float(wp['y'])
            sphere.pose.position.z = 0.08
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.15
            sphere.scale.y = 0.15
            sphere.scale.z = 0.15
            sphere.color.r = 0.2
            sphere.color.g = 1.0
            sphere.color.b = 0.2
            sphere.color.a = 0.9
            msg.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self._map_frame
            label.header.stamp = sphere.header.stamp
            label.ns = 'waypoint_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(wp['x'])
            label.pose.position.y = float(wp['y'])
            label.pose.position.z = 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = wp['name']
            msg.markers.append(label)

        self._marker_pub.publish(msg)

    def _maybe_publish_markers(self):
        """Publish markers at ~1 Hz so RViz picks them up even if opened late."""
        now = time.monotonic()
        if now - self._last_marker_time >= 1.0:
            self._last_marker_time = now
            self._publish_markers()

    def _handle_key(self, key: str):
        if key in ('w', '\x1b[A'):
            self._set_motion(self._linear_speed, 0.0)
        elif key in ('s', '\x1b[B'):
            self._set_motion(-self._linear_speed, 0.0)
        elif key in ('a', '\x1b[D'):
            self._set_motion(0.0, self._angular_speed)
        elif key in ('d', '\x1b[C'):
            self._set_motion(0.0, -self._angular_speed)
        elif key in (' ', 'x'):
            self._set_motion(0.0, 0.0)
        elif key == 'q':
            self._linear_speed *= 1.1
            self._print_status()
        elif key == 'z':
            self._linear_speed *= 0.9
            self._print_status()
        elif key == 'e':
            self._angular_speed *= 1.1
            self._print_status()
        elif key == 'c':
            self._angular_speed *= 0.9
            self._print_status()
        elif key == 'p':
            self._save_waypoint()
        elif key == 'l':
            self._list_waypoints()

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(HELP)
        self._print_status()
        try:
            tty.setcbreak(sys.stdin.fileno())
            timeout = 1.0 / self._publish_rate_hz
            while rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], timeout)
                if ready:
                    key = self._read_key()
                    if key == '\x03':
                        raise KeyboardInterrupt
                    self._handle_key(key)
                self._publish_current()
                self._maybe_publish_markers()
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self._set_motion(0.0, 0.0)

    @staticmethod
    def _read_key() -> str:
        key = sys.stdin.read(1)
        if key != '\x1b':
            return key

        sequence = key
        for _ in range(2):
            ready, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not ready:
                break
            sequence += sys.stdin.read(1)
        return sequence


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
