"""ROS 2 serial bridge for the Guido wheelchair Arduino motor controller.

Subscribes to /cmd_vel (Twist), converts to differential-drive PWM commands,
sends them over serial to the Arduino. Reads odometry and IMU back from the
Arduino and publishes /odom plus the odom -> base_footprint TF.

Arduino protocol:
  TX (Jetson -> Arduino): "L<int> R<int>\n" or "STOP\n"
  RX (Arduino -> Jetson):
    "ODOM <x> <y> <theta> <vLin> <vAng>\n"
    "IMU <ax> <ay> <az> <gx> <gy> <gz>\n"
"""

import math
import threading
import time

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    return quaternion


class SerialBridge(Node):
    def __init__(self):
        super().__init__('guido_serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.17)
        self.declare_parameter('max_wheel_vel', 0.5)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('imu_topic', 'imu/data_raw')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('left_pwm_scale', 1.0)
        self.declare_parameter('right_pwm_scale', 1.0)
        self.declare_parameter('left_min_abs_pwm', 0)
        self.declare_parameter('right_min_abs_pwm', 0)

        self._port_name = self.get_parameter('serial_port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._wheel_base = self.get_parameter('wheel_base').value
        self._max_wheel_vel = self.get_parameter('max_wheel_vel').value
        self._cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._imu_topic = str(self.get_parameter('imu_topic').value)
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._imu_frame = str(self.get_parameter('imu_frame').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._invert_left = bool(self.get_parameter('invert_left').value)
        self._invert_right = bool(self.get_parameter('invert_right').value)
        self._left_pwm_scale = float(self.get_parameter('left_pwm_scale').value)
        self._right_pwm_scale = float(self.get_parameter('right_pwm_scale').value)
        self._left_min_abs_pwm = int(self.get_parameter('left_min_abs_pwm').value)
        self._right_min_abs_pwm = int(self.get_parameter('right_min_abs_pwm').value)

        self._serial = None
        self._connect_serial()

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._imu_pub = self.create_publisher(Imu, self._imu_topic, 20)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cmd_sub = self.create_subscription(Twist, self._cmd_topic, self._cmd_vel_cb, 10)

        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_cb)

        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info(
            f'Serial bridge started on {self._port_name} @ {self._baudrate} '
            f'(cmd_topic={self._cmd_topic}, publish_tf={self._publish_tf})'
        )

    def _connect_serial(self):
        try:
            self._serial = serial.Serial(
                self._port_name,
                self._baudrate,
                timeout=0.1,
            )
            self.get_logger().info(f'Connected to {self._port_name}')
        except serial.SerialException as exc:
            self.get_logger().error(f'Cannot open {self._port_name}: {exc}')
            self._serial = None

    def _write_serial(self, message: str):
        if self._serial is None or not self._serial.is_open:
            self._connect_serial()
        if self._serial is None or not self._serial.is_open:
            return

        try:
            self._serial.write(message.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().warning(f'Serial write error: {exc}')
            self._serial = None

    def _cmd_vel_cb(self, message: Twist):
        self._last_cmd_time = self.get_clock().now()

        linear_x = message.linear.x
        angular_z = message.angular.z

        v_left = linear_x - (angular_z * self._wheel_base / 2.0)
        v_right = linear_x + (angular_z * self._wheel_base / 2.0)

        pwm_left = int(self._clamp(v_left / self._max_wheel_vel * 255.0, -255, 255))
        pwm_right = int(self._clamp(v_right / self._max_wheel_vel * 255.0, -255, 255))

        pwm_left = self._apply_wheel_calibration(
            pwm_left,
            invert=self._invert_left,
            scale=self._left_pwm_scale,
            min_abs_pwm=self._left_min_abs_pwm,
        )
        pwm_right = self._apply_wheel_calibration(
            pwm_right,
            invert=self._invert_right,
            scale=self._right_pwm_scale,
            min_abs_pwm=self._right_min_abs_pwm,
        )

        self._write_serial(f'L{pwm_left} R{pwm_right}\n')

    def _watchdog_cb(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self._cmd_timeout:
            self._write_serial('STOP\n')

    def _serial_read_loop(self):
        while rclpy.ok():
            if self._serial is None or not self._serial.is_open:
                time.sleep(1.0)
                self._connect_serial()
                continue

            try:
                raw = self._serial.readline().decode('ascii', errors='replace').strip()
            except serial.SerialException as exc:
                self.get_logger().warning(f'Serial read error: {exc}')
                self._serial = None
                continue

            if raw.startswith('ODOM '):
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
                if self._publish_tf:
                    self._broadcast_tf(stamp, x, y, theta)
            elif raw.startswith('IMU '):
                parts = raw.split()
                if len(parts) != 7:
                    continue

                try:
                    ax = float(parts[1])
                    ay = float(parts[2])
                    az = float(parts[3])
                    gx = float(parts[4])
                    gy = float(parts[5])
                    gz = float(parts[6])
                except ValueError:
                    continue

                self._publish_imu(self.get_clock().now().to_msg(), ax, ay, az, gx, gy, gz)

    def _publish_odom(self, stamp, x, y, theta, v_lin, v_ang):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame

        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation = yaw_to_quaternion(theta)
        message.twist.twist.linear.x = v_lin
        message.twist.twist.angular.z = v_ang

        self._odom_pub.publish(message)

    def _broadcast_tf(self, stamp, x, y, theta):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = yaw_to_quaternion(theta)
        self._tf_broadcaster.sendTransform(transform)

    def _publish_imu(self, stamp, ax, ay, az, gx, gy, gz):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self._imu_frame
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration.x = ax
        message.linear_acceleration.y = ay
        message.linear_acceleration.z = az
        message.angular_velocity.x = gx
        message.angular_velocity.y = gy
        message.angular_velocity.z = gz
        message.angular_velocity_covariance[0] = 0.02
        message.angular_velocity_covariance[4] = 0.02
        message.angular_velocity_covariance[8] = 0.02
        message.linear_acceleration_covariance[0] = 0.2
        message.linear_acceleration_covariance[4] = 0.2
        message.linear_acceleration_covariance[8] = 0.2
        self._imu_pub.publish(message)

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def _apply_wheel_calibration(self, pwm, *, invert, scale, min_abs_pwm):
        if invert:
            pwm = -pwm

        pwm = pwm * scale
        if pwm == 0:
            return 0

        sign = 1 if pwm > 0 else -1
        magnitude = min(255, abs(pwm))
        if 0 < magnitude < min_abs_pwm:
            magnitude = min_abs_pwm

        return int(sign * magnitude)

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
