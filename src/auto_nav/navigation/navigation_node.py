"""ROS node that bridges internal navigation requests to Nav2 actions."""

from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from navigation.adapters import NavigationBridgeAdapter
from navigation.messages import decode_navigation_request
from navigation.messages import encode_navigation_result
from navigation.types import NavigationRequest


class AutoNavNavigationNode(Node):
    """Forward supervisor requests to Nav2 and relay action results back."""

    def __init__(self):
        super().__init__('auto_nav_navigation_node')

        _declare_parameter_if_missing(self, 'use_sim_time', False)
        _declare_parameter_if_missing(self, 'request_topic', '/auto_nav/request')
        _declare_parameter_if_missing(self, 'result_topic', '/auto_nav/result')
        _declare_parameter_if_missing(self, 'status_topic', '/auto_nav/status')
        _declare_parameter_if_missing(self, 'cmd_vel_topic', '/cmd_vel')
        _declare_parameter_if_missing(self, 'map_frame', 'map')
        _declare_parameter_if_missing(self, 'navigate_action_name', '/navigate_to_pose')

        self._request_topic = self.get_parameter('request_topic').value
        self._result_topic = self.get_parameter('result_topic').value
        self._status_topic = self.get_parameter('status_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._navigate_action_name = self.get_parameter('navigate_action_name').value

        self._result_pub = self.create_publisher(String, self._result_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(String, self._request_topic, self._on_request, 10)

        self._action_client = ActionClient(self, NavigateToPose, self._navigate_action_name)
        self._adapter = NavigationBridgeAdapter(self)
        self._goal_handle = None
        self._current_waypoint_name = None

        self.get_logger().info('auto_nav navigation bridge is ready.')

    def _on_request(self, msg: String) -> None:
        try:
            request = decode_navigation_request(msg.data)
        except ValueError as exc:
            self._publish_status(f'Ignoring invalid navigation request payload: {exc}')
            failure = self._adapter.map_result('failed', None, str(exc))
            self._publish_navigation_result(failure)
            return

        try:
            status_message = self._adapter.handle_request(request)
        except Exception as exc:  # pragma: no cover - depends on ROS runtime
            self._publish_status(str(exc))
            failure = self._adapter.map_result('failed', request.waypoint_name, str(exc))
            self._publish_navigation_result(failure)
            return

        self._publish_status(status_message)

    def start_navigation(self, request: NavigationRequest) -> None:
        if request.pose is None:
            raise ValueError('Navigation requests require a target pose.')
        if self._goal_handle is not None:
            raise RuntimeError('Nav2 is already executing a goal.')
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError('Nav2 action server is unavailable.')

        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose_stamped(request)
        self._current_waypoint_name = request.waypoint_name or 'unknown'
        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def cancel_navigation(self, reason: str = '') -> None:
        del reason
        if self._goal_handle is None:
            self._publish_status('No active Nav2 goal to cancel.')
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_cancel_response)

    def stop_navigation(self, reason: str = '') -> None:
        del reason
        self.cancel_navigation('stop requested')
        self._publish_zero_twist()

    def _build_pose_stamped(self, request: NavigationRequest) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self._map_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = request.pose.x
        pose_stamped.pose.position.y = request.pose.y
        pose_stamped.pose.orientation.z = math.sin(request.pose.yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(request.pose.yaw / 2.0)
        return pose_stamped

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - depends on ROS runtime
            failure = self._adapter.map_result('failed', self._current_waypoint_name, str(exc))
            self._publish_navigation_result(failure)
            self._publish_status(f'Nav2 goal submission failed: {exc}')
            return

        if not goal_handle.accepted:
            failure = self._adapter.map_result(
                'failed',
                self._current_waypoint_name,
                "Nav2 rejected goal '{}'.".format(self._current_waypoint_name or 'unknown'),
            )
            self._publish_navigation_result(failure)
            return

        self._goal_handle = goal_handle
        self._publish_status("Nav2 accepted goal '{}'.".format(self._current_waypoint_name))
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future) -> None:
        waypoint_name = self._current_waypoint_name
        self._goal_handle = None
        self._current_waypoint_name = None
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - depends on ROS runtime
            failure = self._adapter.map_result('failed', waypoint_name, str(exc))
            self._publish_navigation_result(failure)
            self._publish_status(f'Nav2 result retrieval failed: {exc}')
            return

        status_label = _goal_status_to_label(result.status)
        detail = ''
        if getattr(result, 'result', None) is not None:
            detail = getattr(result.result, 'error_msg', '') or getattr(result.result, 'message', '')
        mapped = self._adapter.map_result(status_label, waypoint_name, detail)
        self._publish_navigation_result(mapped)

    def _on_cancel_response(self, future) -> None:
        try:
            cancel_response = future.result()
        except Exception as exc:  # pragma: no cover - depends on ROS runtime
            self._publish_status(f'Nav2 cancel request failed: {exc}')
            return

        goals_canceling = getattr(cancel_response, 'goals_canceling', [])
        if goals_canceling:
            self._publish_status('Nav2 acknowledged the cancel request.')
        else:
            self._publish_status('Nav2 cancel request completed with no active goals.')

    def _publish_zero_twist(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _publish_navigation_result(self, result) -> None:
        outbound = String()
        outbound.data = encode_navigation_result(result)
        self._result_pub.publish(outbound)

    def _publish_status(self, message: str) -> None:
        outbound = String()
        outbound.data = message
        self._status_pub.publish(outbound)
        self.get_logger().info(message)


def _goal_status_to_label(status_code) -> str:
    if status_code == GoalStatus.STATUS_SUCCEEDED:
        return 'succeeded'
    if status_code == GoalStatus.STATUS_CANCELED:
        return 'canceled'
    return 'failed'


def _declare_parameter_if_missing(node: Node, name: str, default_value) -> None:
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)


def main(args=None):
    rclpy.init(args=args)
    node = AutoNavNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
