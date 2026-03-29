"""Command multiplexer between manual teleop and autonomous control."""

from __future__ import annotations

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CommandMux(Node):
    def __init__(self):
        super().__init__('guido_command_mux')

        self.declare_parameter('default_mode', 'manual')
        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('autonomy_topic', '/cmd_vel_autonomy')
        self.declare_parameter('mode_topic', '/system/mode')
        self.declare_parameter('active_mode_topic', '/system/active_mode')
        self.declare_parameter('output_topic', '/cmd_vel_request')
        self.declare_parameter('emergency_topic', '/navigation/emergency_stop')
        self.declare_parameter('manual_timeout_sec', 0.75)
        self.declare_parameter('autonomy_timeout_sec', 0.50)

        self._mode = str(self.get_parameter('default_mode').value).lower()
        self._manual_timeout_sec = float(self.get_parameter('manual_timeout_sec').value)
        self._autonomy_timeout_sec = float(self.get_parameter('autonomy_timeout_sec').value)
        self._emergency_active = False

        manual_topic = str(self.get_parameter('manual_topic').value)
        autonomy_topic = str(self.get_parameter('autonomy_topic').value)
        mode_topic = str(self.get_parameter('mode_topic').value)
        active_mode_topic = str(self.get_parameter('active_mode_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        emergency_topic = str(self.get_parameter('emergency_topic').value)

        self._output_pub = self.create_publisher(Twist, output_topic, 10)
        self._active_mode_pub = self.create_publisher(String, active_mode_topic, 10)

        self.create_subscription(Twist, manual_topic, self._manual_cb, 10)
        self.create_subscription(Twist, autonomy_topic, self._autonomy_cb, 10)
        self.create_subscription(String, mode_topic, self._mode_cb, 10)
        self.create_subscription(Bool, emergency_topic, self._emergency_cb, 10)

        self._manual_cmd = Twist()
        self._autonomy_cmd = Twist()
        now = self.get_clock().now()
        self._manual_stamp = now
        self._autonomy_stamp = now

        self.create_timer(0.05, self._publish_active_command)
        self.get_logger().info(f'Command mux online, default mode={self._mode}.')

    def _manual_cb(self, message: Twist):
        self._manual_cmd = message
        self._manual_stamp = self.get_clock().now()

    def _autonomy_cb(self, message: Twist):
        self._autonomy_cmd = message
        self._autonomy_stamp = self.get_clock().now()

    def _mode_cb(self, message: String):
        requested = message.data.strip().lower()
        if requested in {'manual', 'auto', 'hold'}:
            self._mode = requested

    def _emergency_cb(self, message: Bool):
        self._emergency_active = bool(message.data)

    def _publish_active_command(self):
        active_mode = 'hold' if self._emergency_active else self._mode
        command = Twist()
        now = self.get_clock().now()

        if active_mode == 'manual':
            if self._age_seconds(now, self._manual_stamp) <= self._manual_timeout_sec:
                command = self._manual_cmd
        elif active_mode == 'auto':
            if self._age_seconds(now, self._autonomy_stamp) <= self._autonomy_timeout_sec:
                command = self._autonomy_cmd

        self._output_pub.publish(command)

        active_message = String()
        active_message.data = active_mode
        self._active_mode_pub.publish(active_message)

    @staticmethod
    def _age_seconds(now, then) -> float:
        return (now - then).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
