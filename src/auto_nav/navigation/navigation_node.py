"""ROS node that bridges internal navigation requests to Nav2 actions."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from .adapters import NavigationBridgeAdapter
from .messages import decode_navigation_request, encode_navigation_result
from .types import NavigationRequest, NavigationResult


class AutoNavNavigationNode(Node):
    """Forward supervisor requests to Nav2 and relay action results back."""

    def __init__(self) -> None:
        super().__init__('auto_nav_navigation_node')

        _declare_parameter_if_missing(self, 'use_sim_time', False)
        self.declare_parameter('request_topic', '/auto_nav/request')
        self.declare_parameter('result_topic', '/auto_nav/result')
        self.declare_parameter('status_topic', '/auto_nav/status')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('navigate_action_name', '/navigate_to_pose')

        request_topic = str(self.get_parameter('request_topic').value)
        result_topic = str(self.get_parameter('result_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        action_name = str(self.get_parameter('navigate_action_name').value)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._adapter = NavigationBridgeAdapter(self)
        self._result_publisher = self.create_publisher(String, result_topic, 10)
        self._status_publisher = self.create_publisher(String, status_topic, 10)
        self._cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(String, request_topic, self._on_request, 10)

        self._goal_handle = None
        self._active_waypoint_name: Optional[str] = None

        self.get_logger().info('auto_nav navigation bridge is ready.')

    def _on_request(self, msg: String) -> None:
        try:
            request = decode_navigation_request(msg.data)
        except ValueError as exc:
            self._publish_status('Ignoring invalid navigation request payload: {}'.format(exc))
            return

        try:
            status_message = self._adapter.handle_request(request)
        except ValueError as exc:
            self._publish_status(str(exc))
            failure = self._adapter.map_result(
                status='failed',
                waypoint_name=request.waypoint_name,
                detail=str(exc),
            )
            self._publish_navigation_result(failure)
            return

        self._publish_status(status_message)

    def start_navigation(self, request: NavigationRequest) -> None:
        if request.pose is None:
            raise ValueError('Navigation requests require a target pose.')

        if self._goal_handle is not None:
            raise ValueError('Nav2 is already executing a goal.')

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            raise ValueError('Nav2 action server is unavailable.')

        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose_stamped(request)

        send_future = self._nav_client.send_goal_async(goal)
        self._active_waypoint_name = request.waypoint_name
        send_future.add_done_callback(self._on_goal_response)

    def cancel_navigation(self, reason: str = '') -> None:
        if self._goal_handle is None:
            self._publish_status('No active Nav2 goal to cancel.')
            return

        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_cancel_response)
        if reason:
            self._publish_status(reason)

    def stop_navigation(self, reason: str = '') -> None:
        if reason:
            self._publish_status(reason)
        self._publish_zero_twist()
        self.cancel_navigation(reason='stop requested')

    def _build_pose_stamped(self, request: NavigationRequest) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self._map_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = request.pose.x
        pose_stamped.pose.position.y = request.pose.y
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.z = math.sin(request.pose.yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(request.pose.yaw / 2.0)
        return pose_stamped

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - requires live ROS action failure.
            failure = self._adapter.map_result(
                status='failed',
                waypoint_name=self._active_waypoint_name,
                detail='Nav2 goal submission failed: {}'.format(exc),
            )
            self._active_waypoint_name = None
            self._publish_navigation_result(failure)
            return

        if goal_handle is None or not goal_handle.accepted:
            failure = self._adapter.map_result(
                status='failed',
                waypoint_name=self._active_waypoint_name,
                detail="Nav2 rejected goal '{}'.".format(
                    self._active_waypoint_name or 'unknown'
                ),
            )
            self._active_waypoint_name = None
            self._publish_navigation_result(failure)
            return

        self._goal_handle = goal_handle
        self._publish_status(
            "Nav2 accepted goal '{}'.".format(self._active_waypoint_name or 'unknown')
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future) -> None:
        waypoint_name = self._active_waypoint_name
        self._goal_handle = None
        self._active_waypoint_name = None
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - requires live ROS action failure.
            failure = self._adapter.map_result(
                status='failed',
                waypoint_name=waypoint_name,
                detail='Nav2 result retrieval failed: {}'.format(exc),
            )
            self._publish_navigation_result(failure)
            return

        mapped = self._adapter.map_result(
            status=_goal_status_to_label(result.status),
            waypoint_name=waypoint_name,
        )
        self._publish_navigation_result(mapped)

    def _on_cancel_response(self, future) -> None:
        try:
            cancel_response = future.result()
        except Exception as exc:  # pragma: no cover - requires live ROS action failure.
            self._publish_status('Nav2 cancel request failed: {}'.format(exc))
            return
        if cancel_response.goals_canceling:
            self._publish_status('Nav2 acknowledged the cancel request.')
            return
        self._publish_status('Nav2 cancel request completed with no active goals.')

    def _publish_zero_twist(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def _publish_navigation_result(self, result: NavigationResult) -> None:
        outbound = String()
        outbound.data = encode_navigation_result(result)
        self._result_publisher.publish(outbound)
        self._publish_status(result.message)

    def _publish_status(self, message: str) -> None:
        outbound = String()
        outbound.data = message
        self._status_publisher.publish(outbound)
        self.get_logger().info(message)


def _goal_status_to_label(status_code: int) -> str:
    if status_code == GoalStatus.STATUS_SUCCEEDED:
        return 'succeeded'
    if status_code == GoalStatus.STATUS_CANCELED:
        return 'canceled'
    return 'failed'


def _declare_parameter_if_missing(node: Node, name: str, default_value) -> None:
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoNavNavigationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
