"""Minimal keyboard teleop that publishes Twist commands on a configurable topic."""

import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


HELP = """\
Guido keyboard teleop
---------------------
w : forward
s : backward
a : turn left
d : turn right
arrow up/down/left/right : same as w/s/a/d
space/x : stop
q : increase linear speed
z : decrease linear speed
e : increase angular speed
c : decrease angular speed
CTRL-C : quit
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('guido_keyboard_teleop')

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._cmd_topic = str(self.get_parameter('cmd_topic').value)

        self._pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._current = Twist()

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
