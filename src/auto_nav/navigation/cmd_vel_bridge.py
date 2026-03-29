"""Forward Nav2 velocity commands to the same /cmd_vel topic as keyboard teleop."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('nav_cmd_vel_bridge')
        self.declare_parameter('input_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel')

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)

        # Default QoS (reliable) matches Nav2 velocity_smoother / behavior_server.
        self._pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._forward, 10)

        self.get_logger().info(
            f'Bridging {in_topic} -> {out_topic} (teleop / serial_bridge path)')

    def _forward(self, msg: Twist):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
