"""ROS 2 serial bridge for the Guido wheelchair Arduino motor controller.

Subscribes to /cmd_vel (Twist), converts to differential-drive PWM commands,
sends them over serial to the Arduino. Reads odometry back from the Arduino
and publishes /odom + broadcasts the odom → base_footprint TF.

Arduino protocol:
  TX (Jetson → Arduino): "L<int> R<int>\n"   e.g. "L150 R-150\n" or "STOP\n"
  RX (Arduino → Jetson): "ODOM <x> <y> <theta> <vLin> <vAng>\n"
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import serial


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SerialBridge(Node):

    def __init__(self):
        super().__init__('guido_serial_bridge')

        # -- Declare parameters --
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.17)
        self.declare_parameter('max_wheel_vel', 0.5)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self._port_name = self.get_parameter('serial_port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._wheel_base = self.get_parameter('wheel_base').value
        self._max_wheel_vel = self.get_parameter('max_wheel_vel').value
        self._cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # -- Serial port --
        self._serial = None
        self._connect_serial()

        # -- Publishers / subscribers --
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10
        )

        # -- Safety watchdog: stop motors if no cmd_vel received --
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_cb)

        # -- Serial read thread --
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info(
            f'Serial bridge started on {self._port_name} @ {self._baudrate}'
        )

    # ── Serial connection ────────────────────────────────────────────

    def _connect_serial(self):
        try:
            self._serial = serial.Serial(
                self._port_name,
                self._baudrate,
                timeout=0.1,
            )
            self.get_logger().info(f'Connected to {self._port_name}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {self._port_name}: {e}')
            self._serial = None

    def _write_serial(self, msg: str):
        if self._serial is None or not self._serial.is_open:
            self._connect_serial()
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.write(msg.encode('ascii'))
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial write error: {e}')
                self._serial = None

    # ── cmd_vel → motor commands ─────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        v_left = linear_x - (angular_z * self._wheel_base / 2.0)
        v_right = linear_x + (angular_z * self._wheel_base / 2.0)

        pwm_left = int(self._clamp(v_left / self._max_wheel_vel * 255.0, -255, 255))
        pwm_right = int(self._clamp(v_right / self._max_wheel_vel * 255.0, -255, 255))

        self._write_serial(f'L{pwm_left} R{pwm_right}\n')

    # ── Safety watchdog ──────────────────────────────────────────────

    def _watchdog_cb(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self._cmd_timeout:
            self._write_serial('STOP\n')

    # ── Serial read → odom publisher ─────────────────────────────────

    def _serial_read_loop(self):
        while rclpy.ok():
            if self._serial is None or not self._serial.is_open:
                rclpy.spin_once(self, timeout_sec=1.0)
                self._connect_serial()
                continue
            try:
                raw = self._serial.readline().decode('ascii', errors='replace').strip()
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read error: {e}')
                self._serial = None
                continue

            if not raw.startswith('ODOM '):
                continue

            parts = raw.split()
            if len(parts) != 6:
                continue

            try:
                x = float(parts[1])
                y = float(parts[2])
                theta = float(parts[3])
                v_lin = float(parts[4])
                v_ang = float(parts[5])
            except ValueError:
                continue

            stamp = self.get_clock().now().to_msg()
            self._publish_odom(stamp, x, y, theta, v_lin, v_ang)
            self._broadcast_tf(stamp, x, y, theta)

    def _publish_odom(self, stamp, x, y, theta, v_lin, v_ang):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quaternion(theta)

        msg.twist.twist.linear.x = v_lin
        msg.twist.twist.angular.z = v_ang

        self._odom_pub.publish(msg)

    def _broadcast_tf(self, stamp, x, y, theta):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        q = yaw_to_quaternion(theta)
        t.transform.rotation = q
        self._tf_broadcaster.sendTransform(t)

    # ── Util ─────────────────────────────────────────────────────────

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def destroy_node(self):
        self._write_serial('STOP\n')
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
