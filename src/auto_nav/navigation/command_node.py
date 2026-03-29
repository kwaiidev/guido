from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

from geometry_msgs.msg import Point
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from navigation.adapters import CommandBridgeAdapter
from navigation.frontiers import extract_frontier_clusters
from navigation.frontiers import rank_frontier_goals
from navigation.frontiers import select_frontier_goal
from navigation.health import HealthMonitor
from navigation.messages import decode_navigation_result
from navigation.messages import encode_navigation_request
from navigation.supervisor import NavigationSupervisor
from navigation.types import CommandContext
from navigation.types import FrontierParams
from navigation.types import OccupancyGridSnapshot
from navigation.types import OperatingMode
from navigation.types import Pose2D
from navigation.waypoints import WaypointStore


_BLACKLIST_RADIUS_METERS = 0.5


class AutoNavCommandNode(Node):
    def __init__(self):
        super().__init__('auto_nav_command_node')

        _declare_parameter_if_missing(self, 'use_sim_time', False)
        _declare_parameter_if_missing(self, 'mode', 'navigation')
        _declare_parameter_if_missing(self, 'map_file', '')
        _declare_parameter_if_missing(self, 'active_map_id', '')
        _declare_parameter_if_missing(self, 'waypoint_file', str(Path('.guido') / 'waypoints.yaml'))
        _declare_parameter_if_missing(self, 'map_output_dir', str(Path('.guido') / 'maps'))
        _declare_parameter_if_missing(self, 'map_save_service', '/map_saver/save_map')
        _declare_parameter_if_missing(self, 'navigation_timeout', 10.0)
        _declare_parameter_if_missing(self, 'scan_timeout', 0.5)
        _declare_parameter_if_missing(self, 'odom_timeout', 2.0)
        _declare_parameter_if_missing(self, 'tf_timeout', 1.0)
        _declare_parameter_if_missing(self, 'timer_period', 0.2)
        _declare_parameter_if_missing(self, 'command_topic', '/auto_nav/command')
        _declare_parameter_if_missing(self, 'request_topic', '/auto_nav/request')
        _declare_parameter_if_missing(self, 'result_topic', '/auto_nav/result')
        _declare_parameter_if_missing(self, 'status_topic', '/auto_nav/status')
        _declare_parameter_if_missing(self, 'frontier_marker_topic', '/auto_nav/frontier_markers')
        _declare_parameter_if_missing(self, 'scan_topic', '/ldlidar_node/scan')
        _declare_parameter_if_missing(self, 'odom_topic', '/odom')
        _declare_parameter_if_missing(self, 'map_topic', '/map')
        _declare_parameter_if_missing(self, 'cmd_vel_topic', '/cmd_vel')
        _declare_parameter_if_missing(self, 'map_frame', 'map')
        _declare_parameter_if_missing(self, 'odom_frame', 'odom')
        _declare_parameter_if_missing(self, 'base_frame', 'base_link')
        _declare_parameter_if_missing(self, 'info_gain_weight', 0.3)
        _declare_parameter_if_missing(self, 'info_gain_radius', 8)
        _declare_parameter_if_missing(self, 'min_cluster_size', 10)
        _declare_parameter_if_missing(self, 'base_timeout', 15.0)
        _declare_parameter_if_missing(self, 'expected_speed', 0.12)
        _declare_parameter_if_missing(self, 'timeout_safety_factor', 2.5)
        _declare_parameter_if_missing(self, 'empty_cycle_threshold', 3)
        _declare_parameter_if_missing(self, 'coverage_threshold', 0.0)

        self._mode = OperatingMode(self.get_parameter('mode').value)
        map_file = self.get_parameter('map_file').value
        requested_map_id = self.get_parameter('active_map_id').value
        self._active_map_id = _derive_map_id(requested_map_id=requested_map_id, map_file=map_file)

        waypoint_store = WaypointStore(Path(self.get_parameter('waypoint_file').value).expanduser())
        frontier_params = FrontierParams(
            min_cluster_size=int(self.get_parameter('min_cluster_size').value),
            blacklist_radius=_BLACKLIST_RADIUS_METERS,
            info_gain_weight=float(self.get_parameter('info_gain_weight').value),
            info_gain_radius=int(self.get_parameter('info_gain_radius').value),
        )
        health_monitor = HealthMonitor(
            scan_timeout=float(self.get_parameter('scan_timeout').value),
            odom_timeout=float(self.get_parameter('odom_timeout').value),
            tf_timeout=float(self.get_parameter('tf_timeout').value),
        )
        self._supervisor = NavigationSupervisor(
            waypoint_store=waypoint_store,
            health_monitor=health_monitor,
            active_map_id=self._active_map_id,
            navigation_timeout=float(self.get_parameter('navigation_timeout').value),
            mode=self._mode.value,
            map_output_dir=Path(self.get_parameter('map_output_dir').value).expanduser(),
            frontier_params=frontier_params,
            base_timeout=float(self.get_parameter('base_timeout').value),
            expected_speed=float(self.get_parameter('expected_speed').value),
            timeout_safety_factor=float(self.get_parameter('timeout_safety_factor').value),
            empty_cycle_threshold=int(self.get_parameter('empty_cycle_threshold').value),
            coverage_threshold=float(self.get_parameter('coverage_threshold').value),
        )
        self._adapter = CommandBridgeAdapter(self._supervisor)

        self._command_topic = self.get_parameter('command_topic').value
        self._request_topic = self.get_parameter('request_topic').value
        self._result_topic = self.get_parameter('result_topic').value
        self._status_topic = self.get_parameter('status_topic').value
        self._frontier_marker_topic = self.get_parameter('frontier_marker_topic').value
        self._scan_topic = self.get_parameter('scan_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._map_topic = self.get_parameter('map_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._request_pub = self.create_publisher(String, self._request_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._frontier_pub = self.create_publisher(MarkerArray, self._frontier_marker_topic, 1)

        self.create_subscription(String, self._command_topic, self._on_command, 10)
        self.create_subscription(String, self._result_topic, self._on_navigation_result, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, 10)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_map: Optional[OccupancyGridSnapshot] = None
        self._map_save_in_flight = False
        self._map_save_client = self.create_client(
            SaveMap,
            self.get_parameter('map_save_service').value,
        )

        timer_period = float(self.get_parameter('timer_period').value)
        self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            "auto_nav command node ready in '{}' mode with map id '{}'.".format(
                self._mode.value,
                self._active_map_id,
            )
        )

    def _on_command(self, msg: String) -> None:
        current_pose = self._lookup_current_pose_if_needed(msg.data)
        context = CommandContext(
            current_pose=current_pose,
            map_available=self._latest_map is not None,
            cmd_vel_ready=self._cmd_vel_ready(),
        )
        try:
            dispatch = self._adapter.handle_text_command(msg.data, context)
        except ValueError as exc:
            self._publish_status(str(exc))
            return
        self._publish_dispatch(dispatch)
        self._after_supervisor_update()

    def _on_navigation_result(self, msg: String) -> None:
        try:
            result = decode_navigation_result(msg.data)
        except ValueError as exc:
            self._publish_status(f'Ignoring invalid navigation result payload: {exc}')
            return
        dispatch = self._adapter.handle_navigation_result(result)
        self._publish_dispatch(dispatch)
        self._after_supervisor_update()

    def _on_scan(self, _: LaserScan) -> None:
        self._supervisor._health_monitor.record_scan()

    def _on_odom(self, _: Odometry) -> None:
        self._supervisor._health_monitor.record_odom()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = OccupancyGridSnapshot(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            data=tuple(int(value) for value in msg.data),
        )
        self._publish_frontier_markers()

    def _on_timer(self) -> None:
        self._refresh_tf_health()
        dispatch = self._adapter.tick()
        self._publish_dispatch(dispatch)
        self._maybe_plan_exploration()
        self._maybe_start_map_save()
        self._publish_frontier_markers()

    def _refresh_tf_health(self) -> None:
        if self._lookup_current_pose(report_failure=False) is not None:
            self._supervisor._health_monitor.record_tf()

    def _maybe_plan_exploration(self) -> None:
        if self._latest_map is None:
            return
        current_pose = self._lookup_current_pose(report_failure=False)
        dispatch = self._adapter.plan_exploration(current_pose, self._latest_map)
        self._publish_dispatch(dispatch)
        self._after_supervisor_update()

    def _lookup_current_pose_if_needed(self, raw_command: str) -> Optional[Pose2D]:
        normalized = ' '.join(raw_command.strip().split()).lower()
        if not normalized:
            return None
        command_name = normalized.split(' ', 1)[0]
        if command_name in {'save_waypoint', 'start_exploration'}:
            return self._lookup_current_pose(report_failure=True)
        return self._lookup_current_pose(report_failure=False)

    def _lookup_current_pose(self, report_failure: bool = True) -> Optional[Pose2D]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            if report_failure:
                self._publish_status(f'Unable to resolve current pose: {exc}')
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return Pose2D(x=translation.x, y=translation.y, yaw=yaw)

    def _publish_dispatch(self, dispatch) -> None:
        if dispatch is None:
            return
        for message in dispatch.status_messages:
            self._publish_status(message)
        for request in dispatch.requests:
            outbound = String()
            outbound.data = encode_navigation_request(request)
            self._request_pub.publish(outbound)

    def _after_supervisor_update(self) -> None:
        self._publish_frontier_markers()

    def _maybe_start_map_save(self) -> None:
        pending = self._supervisor.begin_pending_map_save()
        if pending is None or self._map_save_in_flight:
            return
        if not self._map_save_client.wait_for_service(timeout_sec=0.1):
            self._publish_status(
                "map save service '{}' is unavailable".format(
                    self.get_parameter('map_save_service').value
                )
            )
            return

        request = SaveMap.Request()
        if hasattr(request, 'map_topic'):
            request.map_topic = self._map_topic
        if hasattr(request, 'map_url'):
            request.map_url = pending.path
        if hasattr(request, 'image_format'):
            request.image_format = 'pgm'
        if hasattr(request, 'map_mode'):
            request.map_mode = 'trinary'
        if hasattr(request, 'free_thresh'):
            request.free_thresh = 0.25
        if hasattr(request, 'occupied_thresh'):
            request.occupied_thresh = 0.65

        self._publish_status("Saving exploration map to '{}'.".format(pending.path))
        self._map_save_in_flight = True
        future = self._map_save_client.call_async(request)
        future.add_done_callback(self._on_map_save_complete)

    def _on_map_save_complete(self, future) -> None:
        self._map_save_in_flight = False
        success = False
        detail = 'map saver reported failure'
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - depends on ROS runtime
            detail = str(exc)
        else:
            if hasattr(response, 'result'):
                success = bool(response.result)
            elif hasattr(response, 'success'):
                success = bool(response.success)
            else:
                success = bool(response)
            detail = getattr(response, 'message', '') or getattr(response, 'detail', '') or detail
        self._publish_status(self._supervisor.apply_map_save_result(success, detail))

    def _cmd_vel_ready(self) -> bool:
        return self.count_subscribers(self._cmd_vel_topic) > 0

    def _publish_status(self, message: str) -> None:
        outbound = String()
        outbound.data = message
        self._status_pub.publish(outbound)
        self.get_logger().info(message)

    def _publish_frontier_markers(self) -> None:
        markers = MarkerArray()
        markers.markers.append(self._make_delete_all_marker())

        if self._latest_map is None:
            self._frontier_pub.publish(markers)
            return

        stamp = self.get_clock().now().to_msg()
        current_pose = self._lookup_current_pose(report_failure=False)
        highlighted_cells = set(self._supervisor.active_exploration_frontier_cells)
        clusters = extract_frontier_clusters(
            self._latest_map,
            self._supervisor._frontier_params.min_cluster_size,
        )
        markers.markers.extend(self._build_cluster_markers(clusters, highlighted_cells, stamp))

        if current_pose is not None:
            ranked_goals = rank_frontier_goals(
                self._latest_map,
                current_pose,
                blacklist_points=self._supervisor.exploration_blacklist,
                params=self._supervisor._frontier_params,
            )
            markers.markers.extend(self._build_score_markers(ranked_goals, stamp))
            preview_goal = ranked_goals[0] if ranked_goals else None
            selected_goal_pose = self._supervisor.active_exploration_goal_pose
            selected_reference_pose = self._supervisor.active_exploration_frontier_reference
            if preview_goal is not None and selected_goal_pose is None:
                selected_goal_pose = preview_goal.target_pose
                selected_reference_pose = preview_goal.reference_pose
            if selected_goal_pose is not None:
                markers.markers.append(self._build_goal_marker(selected_goal_pose, stamp))
            if selected_reference_pose is not None:
                markers.markers.append(self._build_reference_marker(selected_reference_pose, stamp))
            if current_pose is not None and selected_goal_pose is not None:
                markers.markers.append(
                    self._build_goal_line_marker(current_pose, selected_goal_pose, stamp)
                )

        markers.markers.append(
            self._build_coverage_text_marker(self._supervisor.exploration_coverage * 100.0, stamp)
        )
        markers.markers.append(
            self._build_blacklist_marker(self._supervisor.exploration_blacklist, stamp)
        )
        self._frontier_pub.publish(markers)

    def _build_cluster_markers(self, clusters, highlighted_cells, stamp) -> list[Marker]:
        markers = []
        point_scale = max(self._latest_map.resolution, 0.05)
        for index, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = self._map_frame
            marker.header.stamp = stamp
            marker.ns = 'frontier_clusters'
            marker.id = index
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = point_scale
            marker.scale.y = point_scale
            marker.color.a = 0.9
            marker.color.r = 0.15
            marker.color.g = 0.8 if not set(cluster.cells) & highlighted_cells else 1.0
            marker.color.b = 0.2
            marker.points = [
                _point(*self._latest_map.cell_center(row, col), 0.02)
                for row, col in cluster.cells
            ]
            markers.append(marker)
        return markers

    def _build_score_markers(self, ranked_goals, stamp) -> list[Marker]:
        markers = []
        for index, goal in enumerate(ranked_goals[:10]):
            marker = Marker()
            marker.header.frame_id = self._map_frame
            marker.header.stamp = stamp
            marker.ns = 'frontier_scores'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.18
            marker.color.a = 0.95
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.pose.position = _point(goal.cluster.centroid_x, goal.cluster.centroid_y, 0.18)
            marker.text = 'd={:.1f} ig={:.0f} s={:.2f}'.format(
                goal.distance_to_robot,
                goal.information_gain,
                goal.score,
            )
            markers.append(marker)
        return markers

    def _build_coverage_text_marker(self, coverage_pct: float, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = 'frontier_coverage'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.22
        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.pose.position = _point(0.0, 0.0, 0.5)
        marker.text = 'Coverage: {:.1f}%'.format(coverage_pct)
        return marker

    def _build_blacklist_marker(self, blacklist_points, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = 'frontier_blacklist'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = _BLACKLIST_RADIUS_METERS * 2.0
        marker.scale.y = _BLACKLIST_RADIUS_METERS * 2.0
        marker.scale.z = 0.05
        marker.color.a = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.points = [_point(pose.x, pose.y, 0.02) for pose in blacklist_points]
        return marker

    def _build_reference_marker(self, pose: Pose2D, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = 'frontier_reference'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = _point(pose.x, pose.y, 0.08)
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.a = 0.85
        marker.color.r = 1.0
        marker.color.g = 0.65
        marker.color.b = 0.0
        return marker

    def _build_goal_marker(self, pose: Pose2D, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = 'frontier_goal'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = _point(pose.x, pose.y, 0.05)
        marker.pose.orientation.z = math.sin(pose.yaw / 2.0)
        marker.pose.orientation.w = math.cos(pose.yaw / 2.0)
        marker.scale.x = 0.35
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.a = 0.95
        marker.color.r = 0.1
        marker.color.g = 0.55
        marker.color.b = 1.0
        return marker

    def _build_goal_line_marker(self, current_pose: Pose2D, goal_pose: Pose2D, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = 'frontier_goal_line'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.a = 0.85
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.points = [
            _point(current_pose.x, current_pose.y, 0.05),
            _point(goal_pose.x, goal_pose.y, 0.05),
        ]
        return marker

    def _make_delete_all_marker(self) -> Marker:
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker


def _derive_map_id(requested_map_id: str, map_file: str) -> str:
    if requested_map_id:
        return requested_map_id
    if map_file:
        return Path(map_file).stem
    return 'live_map'


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    del x, y
    siny_cosp = 2.0 * (w * z)
    cosy_cosp = 1.0 - 2.0 * (z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _point(x: float, y: float, z: float) -> Point:
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


def _declare_parameter_if_missing(node: Node, name: str, default_value) -> None:
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)


def main(args=None):
    rclpy.init(args=args)
    node = AutoNavCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
