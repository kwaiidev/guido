"""Minimal keyboard teleop that publishes Twist commands on /cmd_vel."""

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

        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
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
        if key == 'w':
            self._set_motion(self._linear_speed, 0.0)
        elif key == 's':
            self._set_motion(-self._linear_speed, 0.0)
        elif key == 'a':
            self._set_motion(0.0, self._angular_speed)
        elif key == 'd':
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
                    key = sys.stdin.read(1)
                    if key == '\x03':
                        raise KeyboardInterrupt
                    self._handle_key(key)
                self._publish_current()
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self._set_motion(0.0, 0.0)


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
