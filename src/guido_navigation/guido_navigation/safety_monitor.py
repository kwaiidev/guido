"""Safety supervisor that independently gates all motor commands."""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('guido_safety_monitor')

        self.declare_parameter('input_cmd_topic', '/cmd_vel_request')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/navigation/filtered_scan')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('localization_ok_topic', '/navigation/localization_ok')
        self.declare_parameter('active_mode_topic', '/system/active_mode')
        self.declare_parameter('emergency_topic', '/navigation/emergency_stop')
        self.declare_parameter('safety_status_topic', '/navigation/safety_status')
        self.declare_parameter('reaction_time_sec', 0.25)
        self.declare_parameter('max_braking_decel_mps2', 1.20)
        self.declare_parameter('static_margin_m', 0.60)
        self.declare_parameter('lateral_margin_m', 0.45)
        self.declare_parameter('reverse_margin_m', 0.35)
        self.declare_parameter('scan_timeout_sec', 0.40)
        self.declare_parameter('odom_timeout_sec', 0.40)
        self.declare_parameter('localization_timeout_sec', 0.50)

        self._reaction_time_sec = float(self.get_parameter('reaction_time_sec').value)
        self._max_braking_decel_mps2 = float(self.get_parameter('max_braking_decel_mps2').value)
        self._static_margin_m = float(self.get_parameter('static_margin_m').value)
        self._lateral_margin_m = float(self.get_parameter('lateral_margin_m').value)
        self._reverse_margin_m = float(self.get_parameter('reverse_margin_m').value)
        self._scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self._odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self._localization_timeout_sec = float(self.get_parameter('localization_timeout_sec').value)

        input_cmd_topic = str(self.get_parameter('input_cmd_topic').value)
        output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        localization_ok_topic = str(self.get_parameter('localization_ok_topic').value)
        active_mode_topic = str(self.get_parameter('active_mode_topic').value)
        emergency_topic = str(self.get_parameter('emergency_topic').value)
        safety_status_topic = str(self.get_parameter('safety_status_topic').value)

        self._cmd_pub = self.create_publisher(Twist, output_cmd_topic, 10)
        self._emergency_pub = self.create_publisher(Bool, emergency_topic, 10)
        self._status_pub = self.create_publisher(String, safety_status_topic, 10)

        self.create_subscription(Twist, input_cmd_topic, self._cmd_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(Bool, localization_ok_topic, self._localization_ok_cb, 10)
        self.create_subscription(String, active_mode_topic, self._active_mode_cb, 10)

        self._requested_cmd = Twist()
        self._latest_scan: LaserScan | None = None
        self._latest_odom: Odometry | None = None
        self._localization_ok = False
        self._last_localization_stamp = self.get_clock().now()
        self._active_mode = 'hold'
        self._last_reason = 'Waiting for data.'

        self.create_timer(0.05, self._evaluate)
        self.get_logger().info('Safety monitor online.')

    def _cmd_cb(self, message: Twist):
        self._requested_cmd = message

    def _scan_cb(self, message: LaserScan):
        self._latest_scan = message

    def _odom_cb(self, message: Odometry):
        self._latest_odom = message

    def _localization_ok_cb(self, message: Bool):
        self._localization_ok = bool(message.data)
        self._last_localization_stamp = self.get_clock().now()

    def _active_mode_cb(self, message: String):
        self._active_mode = message.data.strip().lower()

    def _evaluate(self):
        now = self.get_clock().now()
        emergency = False
        reason = 'Safe to drive.'

        if self._latest_scan is None or self._scan_age_sec(now) > self._scan_timeout_sec:
            emergency = True
            reason = 'LiDAR scan timeout.'
        elif self._latest_odom is None or self._odom_age_sec(now) > self._odom_timeout_sec:
            emergency = True
            reason = 'Odometry timeout.'
        elif self._active_mode == 'auto' and (
            (not self._localization_ok)
            or self._localization_age_sec(now) > self._localization_timeout_sec
        ):
            emergency = True
            reason = 'Localization unavailable.'
        elif self._would_collide(self._requested_cmd):
            emergency = True
            reason = 'Obstacle inside computed stopping distance.'

        self._publish_emergency(emergency, reason)
        if emergency:
            self._cmd_pub.publish(Twist())
        else:
            self._cmd_pub.publish(self._requested_cmd)

    def _would_collide(self, command: Twist) -> bool:
        if self._latest_scan is None:
            return True

        commanded_speed = abs(float(command.linear.x))
        current_speed = abs(float(self._latest_odom.twist.twist.linear.x)) if self._latest_odom else 0.0
        speed = max(commanded_speed, current_speed)
        stop_distance = (
            speed * self._reaction_time_sec
            + (speed * speed) / max(1e-3, 2.0 * self._max_braking_decel_mps2)
            + self._static_margin_m
        )

        moving_forward = command.linear.x >= 0.0
        best_distance = math.inf
        angle = self._latest_scan.angle_min

        for sample in self._latest_scan.ranges:
            if not math.isfinite(sample):
                angle += self._latest_scan.angle_increment
                continue

            x_coord = sample * math.cos(angle)
            y_coord = sample * math.sin(angle)

            if moving_forward and x_coord <= 0.0:
                angle += self._latest_scan.angle_increment
                continue
            if not moving_forward and x_coord >= 0.0:
                angle += self._latest_scan.angle_increment
                continue

            lateral_limit = self._lateral_margin_m + 0.6 * speed
            if abs(y_coord) > lateral_limit:
                angle += self._latest_scan.angle_increment
                continue

            distance_along_path = abs(x_coord)
            best_distance = min(best_distance, distance_along_path)
            angle += self._latest_scan.angle_increment

        reverse_limit = self._reverse_margin_m if not moving_forward else 0.0
        threshold = stop_distance if moving_forward else reverse_limit
        return best_distance <= threshold

    def _publish_emergency(self, emergency: bool, reason: str):
        emergency_message = Bool()
        emergency_message.data = emergency
        self._emergency_pub.publish(emergency_message)

        if reason != self._last_reason:
            status = String()
            status.data = reason
            self._status_pub.publish(status)
            self.get_logger().info(reason)
            self._last_reason = reason

    def _scan_age_sec(self, now) -> float:
        if self._latest_scan is None:
            return math.inf
        stamp = self._latest_scan.header.stamp
        return self._stamp_age_sec(now, stamp)

    def _odom_age_sec(self, now) -> float:
        if self._latest_odom is None:
            return math.inf
        stamp = self._latest_odom.header.stamp
        return self._stamp_age_sec(now, stamp)

    def _localization_age_sec(self, now) -> float:
        return (now - self._last_localization_stamp).nanoseconds / 1e9

    @staticmethod
    def _stamp_age_sec(now, stamp) -> float:
        message_time = rclpy.time.Time.from_msg(stamp)
        return (now - message_time).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
