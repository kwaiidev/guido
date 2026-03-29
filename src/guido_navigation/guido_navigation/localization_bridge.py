"""Publish a unified navigation pose from the active TF tree."""

from __future__ import annotations

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, LookupException, TransformException, TransformListener

from .common import Pose2D, make_pose_stamped, yaw_from_quaternion


class LocalizationBridge(Node):
    def __init__(self):
        super().__init__('guido_localization_bridge')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('pose_topic', '/navigation/pose')
        self.declare_parameter('localization_ok_topic', '/navigation/localization_ok')
        self.declare_parameter('allow_odom_fallback', False)

        self._global_frame = str(self.get_parameter('global_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._allow_odom_fallback = bool(self.get_parameter('allow_odom_fallback').value)

        pose_topic = str(self.get_parameter('pose_topic').value)
        localization_ok_topic = str(self.get_parameter('localization_ok_topic').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self._ok_pub = self.create_publisher(Bool, localization_ok_topic, 10)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(1.0 / publish_rate_hz, self._timer_cb)

        self.get_logger().info(
            f'Localization bridge online: global_frame={self._global_frame} '
            f'base_frame={self._base_frame}'
        )

    def _timer_cb(self):
        pose_message = self._lookup_pose(self._global_frame)
        if pose_message is None and self._allow_odom_fallback:
            pose_message = self._lookup_pose(self._odom_frame)

        ok_message = Bool()
        ok_message.data = pose_message is not None and pose_message.header.frame_id == self._global_frame
        self._ok_pub.publish(ok_message)

        if pose_message is not None:
            self._pose_pub.publish(pose_message)

    def _lookup_pose(self, frame_id: str) -> PoseStamped | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except (LookupException, TransformException):
            return None

        pose = Pose2D(
            x=float(transform.transform.translation.x),
            y=float(transform.transform.translation.y),
            yaw=yaw_from_quaternion(transform.transform.rotation),
        )
        return make_pose_stamped(
            frame_id=frame_id,
            stamp=transform.header.stamp,
            pose=pose,
        )


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
