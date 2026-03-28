from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Point
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .adapters import CommandBridgeAdapter
from .frontiers import extract_frontier_clusters, rank_frontier_goals, select_frontier_goal
from .health import HealthMonitor
from .messages import decode_navigation_result, encode_navigation_request
from .supervisor import NavigationSupervisor
from .types import (
    CommandContext,
    FrontierParams,
    OccupancyGridSnapshot,
    OperatingMode,
    Pose2D,
)
from .waypoints import WaypointStore

_BLACKLIST_RADIUS_METERS = 0.5


class AutoNavCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("auto_nav_command_node")

        _declare_parameter_if_missing(self, "use_sim_time", False)
        self.declare_parameter("mode", OperatingMode.NAVIGATION.value)
        self.declare_parameter("map_file", "")
        self.declare_parameter("active_map_id", "")
        self.declare_parameter(
            "waypoint_file",
            str(Path.home() / ".guido" / "waypoints.yaml"),
        )
        self.declare_parameter("map_output_dir", str(Path.home() / ".guido" / "maps"))
        self.declare_parameter("map_save_service", "/map_saver/save_map")
        self.declare_parameter("navigation_timeout", 10.0)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("odom_timeout", 2.0)
        self.declare_parameter("tf_timeout", 1.0)
        self.declare_parameter("timer_period", 0.2)
        self.declare_parameter("command_topic", "/auto_nav/command")
        self.declare_parameter("request_topic", "/auto_nav/request")
        self.declare_parameter("result_topic", "/auto_nav/result")
        self.declare_parameter("status_topic", "/auto_nav/status")
        self.declare_parameter("frontier_marker_topic", "/auto_nav/frontier_markers")
        self.declare_parameter("scan_topic", "/ldlidar_node/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")

        # Frontier / exploration tuning parameters.
        self.declare_parameter("info_gain_weight", 0.3)
        self.declare_parameter("info_gain_radius", 8)
        self.declare_parameter("min_cluster_size", 10)
        self.declare_parameter("base_timeout", 15.0)
        self.declare_parameter("expected_speed", 0.12)
        self.declare_parameter("timeout_safety_factor", 2.5)
        self.declare_parameter("empty_cycle_threshold", 3)
        self.declare_parameter("coverage_threshold", 0.0)
        self.declare_parameter("base_frame", "base_link")

        self._mode = OperatingMode(str(self.get_parameter("mode").value))
        self._map_file = str(self.get_parameter("map_file").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._map_topic = str(self.get_parameter("map_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._map_save_service = str(self.get_parameter("map_save_service").value)
        self._active_map_id = _derive_map_id(
            requested_map_id=str(self.get_parameter("active_map_id").value),
            map_file=self._map_file,
        )

        frontier_params = FrontierParams(
            min_cluster_size=int(self.get_parameter("min_cluster_size").value),
            blacklist_radius=_BLACKLIST_RADIUS_METERS,
            info_gain_weight=float(self.get_parameter("info_gain_weight").value),
            info_gain_radius=int(self.get_parameter("info_gain_radius").value),
        )
        self._frontier_params = frontier_params

        waypoint_store = WaypointStore(
            Path(str(self.get_parameter("waypoint_file").value)).expanduser()
        )
        health_monitor = HealthMonitor(
            scan_timeout=float(self.get_parameter("scan_timeout").value),
            odom_timeout=float(self.get_parameter("odom_timeout").value),
            tf_timeout=float(self.get_parameter("tf_timeout").value),
        )
        supervisor = NavigationSupervisor(
            waypoint_store=waypoint_store,
            health_monitor=health_monitor,
            active_map_id=self._active_map_id,
            navigation_timeout=float(self.get_parameter("navigation_timeout").value),
            mode=self._mode.value,
            map_output_dir=Path(str(self.get_parameter("map_output_dir").value)),
            frontier_params=frontier_params,
            base_timeout=float(self.get_parameter("base_timeout").value),
            expected_speed=float(self.get_parameter("expected_speed").value),
            timeout_safety_factor=float(
                self.get_parameter("timeout_safety_factor").value
            ),
            empty_cycle_threshold=int(
                self.get_parameter("empty_cycle_threshold").value
            ),
            coverage_threshold=float(
                self.get_parameter("coverage_threshold").value
            ),
        )

        self._health_monitor = health_monitor
        self._supervisor = supervisor
        self._adapter = CommandBridgeAdapter(supervisor)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_map: Optional[OccupancyGridSnapshot] = None
        self._exploration_selection_pending = False
        self._map_save_future = None

        command_topic = str(self.get_parameter("command_topic").value)
        request_topic = str(self.get_parameter("request_topic").value)
        result_topic = str(self.get_parameter("result_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        frontier_marker_topic = str(
            self.get_parameter("frontier_marker_topic").value
        )
        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

        self._request_publisher = self.create_publisher(String, request_topic, 10)
        self._status_publisher = self.create_publisher(String, status_topic, 10)
        self._frontier_marker_publisher = self.create_publisher(
            MarkerArray,
            frontier_marker_topic,
            10,
        )
        self._map_save_client = self.create_client(SaveMap, self._map_save_service)
        self.create_subscription(String, command_topic, self._on_command, 10)
        self.create_subscription(String, result_topic, self._on_navigation_result, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, 10)
        self.create_timer(
            float(self.get_parameter("timer_period").value), self._on_timer
        )

        self.get_logger().info(
            "auto_nav command node ready in '{}' mode with map id '{}'.".format(
                self._mode.value,
                self._supervisor.active_map_id,
            )
        )

    def _on_command(self, msg: String) -> None:
        current_pose = self._lookup_current_pose_if_needed(msg.data)
        context = CommandContext(
            current_pose=current_pose,
            map_available=self._latest_map is not None,
            cmd_vel_ready=self._cmd_vel_ready(),
        )
        dispatch = self._adapter.handle_text_command(msg.data, context=context)
        self._publish_dispatch(dispatch)
        if self._mode == OperatingMode.EXPLORATION:
            self._schedule_exploration_selection()
        self._after_supervisor_update()

    def _on_navigation_result(self, msg: String) -> None:
        try:
            result = decode_navigation_result(msg.data)
        except ValueError as exc:
            self._publish_status(
                "Ignoring invalid navigation result payload: {}".format(exc)
            )
            return

        dispatch = self._adapter.handle_navigation_result(result)
        self._publish_dispatch(dispatch)
        if self._mode == OperatingMode.EXPLORATION:
            self._schedule_exploration_selection()
        self._after_supervisor_update()

    def _on_scan(self, _: LaserScan) -> None:
        self._health_monitor.record_scan()

    def _on_odom(self, _: Odometry) -> None:
        self._health_monitor.record_odom()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = OccupancyGridSnapshot(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            data=tuple(int(value) for value in msg.data),
        )
        if self._mode == OperatingMode.EXPLORATION:
            self._schedule_exploration_selection()
            self._publish_frontier_markers()

    def _on_timer(self) -> None:
        self._refresh_tf_health()
        dispatch = self._adapter.tick()
        if dispatch is not None:
            self._publish_dispatch(dispatch)
            if self._mode == OperatingMode.EXPLORATION:
                self._schedule_exploration_selection()
            self._after_supervisor_update()

        if self._mode == OperatingMode.EXPLORATION:
            self._maybe_plan_exploration()

    def _refresh_tf_health(self) -> None:
        try:
            self._tf_buffer.lookup_transform(
                self._map_frame,
                self._odom_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return

        self._health_monitor.record_tf()

    def _maybe_plan_exploration(self) -> None:
        if not self._exploration_selection_pending:
            return
        if not self._supervisor.exploration_active:
            self._exploration_selection_pending = False
            return
        if self._supervisor.has_active_goal or self._latest_map is None:
            return

        current_pose = self._lookup_current_pose()
        if current_pose is None:
            return

        self._exploration_selection_pending = False
        dispatch = self._adapter.plan_exploration(
            current_pose=current_pose,
            occupancy_grid=self._latest_map,
        )
        if dispatch is None:
            return

        self._publish_dispatch(dispatch)
        self._after_supervisor_update()

    def _lookup_current_pose_if_needed(self, raw_command: str) -> Optional[Pose2D]:
        normalized = " ".join(raw_command.strip().split())
        if not normalized:
            return None

        command_name = normalized.split(" ", 1)[0].lower()
        if command_name not in {"save_waypoint", "start_exploration"}:
            return None
        return self._lookup_current_pose()

    def _lookup_current_pose(self, report_failure: bool = True) -> Optional[Pose2D]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            if report_failure:
                self._publish_status("Unable to resolve current pose: {}".format(exc))
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return Pose2D(x=translation.x, y=translation.y, yaw=yaw)

    def _publish_dispatch(self, dispatch) -> None:
        for message in dispatch.status_messages:
            self._publish_status(message)

        for request in dispatch.requests:
            outbound = String()
            outbound.data = encode_navigation_request(request)
            self._request_publisher.publish(outbound)

    def _after_supervisor_update(self) -> None:
        self._maybe_start_map_save()
        self._publish_frontier_markers()

    def _schedule_exploration_selection(self) -> None:
        if (
            self._mode == OperatingMode.EXPLORATION
            and self._supervisor.exploration_active
            and not self._supervisor.has_active_goal
        ):
            self._exploration_selection_pending = True

    def _maybe_start_map_save(self) -> None:
        if self._map_save_future is not None:
            return

        pending = self._supervisor.begin_pending_map_save()
        if pending is None:
            return

        if not self._map_save_client.wait_for_service(timeout_sec=1.0):
            self._publish_status(
                self._supervisor.apply_map_save_result(
                    False,
                    detail="map save service '{}' is unavailable".format(
                        self._map_save_service
                    ),
                )
            )
            return

        map_output = Path(pending.path).expanduser()
        try:
            map_output.parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self._publish_status(
                self._supervisor.apply_map_save_result(False, detail=str(exc))
            )
            return

        request = SaveMap.Request()
        request.map_topic = self._map_topic
        request.map_url = pending.path
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        self._publish_status(
            "Saving exploration map to '{}'.".format(pending.path)
        )
        self._map_save_future = self._map_save_client.call_async(request)
        self._map_save_future.add_done_callback(self._on_map_save_complete)

    def _on_map_save_complete(self, future) -> None:
        self._map_save_future = None
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - requires live ROS service failure.
            self._publish_status(
                self._supervisor.apply_map_save_result(False, detail=str(exc))
            )
            return

        success = bool(getattr(response, "result", False))
        detail = "" if success else "map saver reported failure"
        self._publish_status(self._supervisor.apply_map_save_result(success, detail))

    def _cmd_vel_ready(self) -> bool:
        try:
            return self.count_subscribers(self._cmd_vel_topic) > 0
        except AttributeError:  # pragma: no cover - defensive for ROS test doubles.
            return False

    def _publish_status(self, message: str) -> None:
        outbound = String()
        outbound.data = message
        self._status_publisher.publish(outbound)
        self.get_logger().info(message)

    def _publish_frontier_markers(self) -> None:
        if self._mode != OperatingMode.EXPLORATION:
            return

        markers = MarkerArray()
        markers.markers.append(self._make_delete_all_marker())

        if self._latest_map is None:
            self._frontier_marker_publisher.publish(markers)
            return

        current_pose = self._lookup_current_pose(report_failure=False)
        if current_pose is None:
            self._frontier_marker_publisher.publish(markers)
            return

        stamp = self.get_clock().now().to_msg()
        highlighted_cells = self._supervisor.active_exploration_frontier_cells
        selected_goal_pose = self._supervisor.active_exploration_goal_pose
        selected_reference_pose = self._supervisor.active_exploration_frontier_reference

        if self._supervisor.exploration_active and selected_goal_pose is None:
            preview_goal = select_frontier_goal(
                self._latest_map,
                robot_pose=current_pose,
                blacklist_points=self._supervisor.exploration_blacklist,
                params=self._frontier_params,
            )
            if preview_goal is not None:
                highlighted_cells = preview_goal.cluster.cells
                selected_goal_pose = preview_goal.target_pose
                selected_reference_pose = preview_goal.reference_pose

        clusters = extract_frontier_clusters(self._latest_map)
        markers.markers.extend(
            self._build_cluster_markers(
                clusters=clusters,
                highlighted_cells=highlighted_cells,
                stamp=stamp,
            )
        )

        # Frontier score annotations.
        ranked_goals = rank_frontier_goals(
            self._latest_map,
            robot_pose=current_pose,
            blacklist_points=self._supervisor.exploration_blacklist,
            params=self._frontier_params,
        )
        markers.markers.extend(
            self._build_score_markers(ranked_goals=ranked_goals, stamp=stamp)
        )

        blacklist_points = self._supervisor.exploration_blacklist
        if blacklist_points:
            markers.markers.append(
                self._build_blacklist_marker(
                    blacklist_points=blacklist_points,
                    stamp=stamp,
                )
            )

        if selected_reference_pose is not None:
            markers.markers.append(
                self._build_reference_marker(
                    pose=selected_reference_pose,
                    stamp=stamp,
                )
            )

        if selected_goal_pose is not None:
            markers.markers.append(
                self._build_goal_marker(
                    pose=selected_goal_pose,
                    stamp=stamp,
                )
            )
            markers.markers.append(
                self._build_goal_line_marker(
                    current_pose=current_pose,
                    goal_pose=selected_goal_pose,
                    stamp=stamp,
                )
            )

        # Coverage text overlay.
        if self._supervisor.exploration_active:
            markers.markers.append(
                self._build_coverage_text_marker(
                    coverage_pct=self._supervisor.exploration_coverage * 100.0,
                    stamp=stamp,
                )
            )

        self._frontier_marker_publisher.publish(markers)

    def _build_cluster_markers(
        self,
        clusters,
        highlighted_cells,
        stamp,
    ) -> list[Marker]:
        markers = []
        point_scale = max(self._latest_map.resolution * 0.75, 0.08)

        for index, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = self._map_frame
            marker.header.stamp = stamp
            marker.ns = "frontier_clusters"
            marker.id = index
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = point_scale
            marker.scale.y = point_scale

            if cluster.cells == highlighted_cells:
                marker.color.r = 0.12
                marker.color.g = 0.82
                marker.color.b = 0.38
                marker.color.a = 0.95
            else:
                marker.color.r = 1.0
                marker.color.g = 0.55
                marker.color.b = 0.1
                marker.color.a = 0.7

            for row, col in cluster.cells:
                cell_x, cell_y = self._latest_map.cell_center(row, col)
                marker.points.append(_point(cell_x, cell_y, 0.05))

            markers.append(marker)

        return markers

    def _build_score_markers(
        self,
        ranked_goals,
        stamp,
    ) -> list[Marker]:
        markers = []
        for index, goal in enumerate(ranked_goals):
            marker = Marker()
            marker.header.frame_id = self._map_frame
            marker.header.stamp = stamp
            marker.ns = "frontier_scores"
            marker.id = 20_000 + index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            cx = goal.cluster.centroid_x
            cy = goal.cluster.centroid_y
            marker.pose.position = _point(cx, cy, 0.25)
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.18
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.9
            marker.text = "d={:.1f} ig={:.0f} s={:.2f}".format(
                goal.distance_to_robot,
                goal.information_gain,
                goal.score,
            )
            markers.append(marker)
        return markers

    def _build_coverage_text_marker(
        self,
        coverage_pct: float,
        stamp,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = "frontier_coverage"
        marker.id = 10_100
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = _point(0.0, 0.0, 1.5)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.35
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 0.4
        marker.color.a = 0.95
        marker.text = "Coverage: {:.1f}%".format(coverage_pct)
        return marker

    def _build_blacklist_marker(self, blacklist_points, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = "frontier_blacklist"
        marker.id = 10_000
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = _BLACKLIST_RADIUS_METERS * 2.0
        marker.scale.y = _BLACKLIST_RADIUS_METERS * 2.0
        marker.scale.z = 0.08
        marker.color.r = 0.9
        marker.color.g = 0.15
        marker.color.b = 0.15
        marker.color.a = 0.35
        marker.points = [_point(pose.x, pose.y, 0.02) for pose in blacklist_points]
        return marker

    def _build_reference_marker(self, pose: Pose2D, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = "frontier_reference"
        marker.id = 10_001
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = _point(pose.x, pose.y, 0.05)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.95
        marker.color.b = 0.2
        marker.color.a = 0.9
        return marker

    def _build_goal_marker(self, pose: Pose2D, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = "frontier_goal"
        marker.id = 10_002
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = _point(pose.x, pose.y, 0.08)
        marker.pose.orientation.z = math.sin(pose.yaw / 2.0)
        marker.pose.orientation.w = math.cos(pose.yaw / 2.0)
        marker.scale.x = 0.45
        marker.scale.y = 0.14
        marker.scale.z = 0.14
        marker.color.r = 0.1
        marker.color.g = 0.55
        marker.color.b = 1.0
        marker.color.a = 0.95
        return marker

    def _build_goal_line_marker(
        self,
        current_pose: Pose2D,
        goal_pose: Pose2D,
        stamp,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = "frontier_goal_line"
        marker.id = 10_003
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.r = 0.1
        marker.color.g = 0.75
        marker.color.b = 0.95
        marker.color.a = 0.8
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
    return "live_map"


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoNavCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
