import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.publish_tf)
        self.publish_static()

    def publish_static(self):
        # base_footprint -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0
        self.static_br.sendTransform(t)

    def publish_tf(self):
        # odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(FakeOdom())

main()
