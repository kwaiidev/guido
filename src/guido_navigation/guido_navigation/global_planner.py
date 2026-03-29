"""Hybrid-A*-style global planner for Guido autonomous navigation."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .common import (
    Pose2D,
    clamp,
    copy_grid_data,
    distance_xy,
    grid_offset,
    grid_to_world,
    iter_point_cloud_xy,
    pose2d_from_pose,
    quaternion_from_yaw,
    world_to_grid,
    wrap_angle,
)


@dataclass
class SearchNode:
    x: float
    y: float
    yaw: float
    direction: int
    steer: float


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('guido_global_planner')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/navigation/pose')
        self.declare_parameter('goal_topic', '/navigation/goal')
        self.declare_parameter('point_cloud_topic', '/navigation/points')
        self.declare_parameter('path_topic', '/navigation/global_path')
        self.declare_parameter('planner_status_topic', '/navigation/planner_status')
        self.declare_parameter('planning_rate_hz', 2.0)
        self.declare_parameter('step_size_m', 0.50)
        self.declare_parameter('heading_bins', 72)
        self.declare_parameter('max_steering_deg', 28.0)
        self.declare_parameter('steering_samples', 5)
        self.declare_parameter('wheelbase_m', 0.80)
        self.declare_parameter('footprint_radius_m', 0.45)
        self.declare_parameter('lethal_cost_threshold', 50)
        self.declare_parameter('goal_position_tolerance_m', 0.45)
        self.declare_parameter('goal_yaw_tolerance_rad', 0.40)
        self.declare_parameter('max_iterations', 25000)
        self.declare_parameter('reverse_enabled', False)
        self.declare_parameter('smooth_passes', 3)
        self.declare_parameter('dynamic_obstacle_radius_m', 0.40)

        self._step_size_m = float(self.get_parameter('step_size_m').value)
        self._heading_bins = int(self.get_parameter('heading_bins').value)
        self._max_steering_rad = math.radians(float(self.get_parameter('max_steering_deg').value))
        self._steering_samples = int(self.get_parameter('steering_samples').value)
        self._wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self._footprint_radius_m = float(self.get_parameter('footprint_radius_m').value)
        self._lethal_cost_threshold = int(self.get_parameter('lethal_cost_threshold').value)
        self._goal_position_tolerance_m = float(self.get_parameter('goal_position_tolerance_m').value)
        self._goal_yaw_tolerance_rad = float(self.get_parameter('goal_yaw_tolerance_rad').value)
        self._max_iterations = int(self.get_parameter('max_iterations').value)
        self._reverse_enabled = bool(self.get_parameter('reverse_enabled').value)
        self._smooth_passes = int(self.get_parameter('smooth_passes').value)
        self._dynamic_obstacle_radius_m = float(self.get_parameter('dynamic_obstacle_radius_m').value)

        map_topic = str(self.get_parameter('map_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        point_cloud_topic = str(self.get_parameter('point_cloud_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        planner_status_topic = str(self.get_parameter('planner_status_topic').value)
        planning_rate_hz = float(self.get_parameter('planning_rate_hz').value)

        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self._status_pub = self.create_publisher(String, planner_status_topic, 10)

        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, 10)
        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)
        self.create_subscription(PoseStamped, goal_topic, self._goal_cb, 10)
        self.create_subscription(PointCloud2, point_cloud_topic, self._point_cloud_cb, 10)

        self._map: OccupancyGrid | None = None
        self._pose: Pose2D | None = None
        self._goal: Pose2D | None = None
        self._frame_id = 'map'
        self._dynamic_points: list[tuple[float, float]] = []

        self.create_timer(1.0 / planning_rate_hz, self._plan_once)
        self.get_logger().info('Global planner online.')

    def _map_cb(self, message: OccupancyGrid):
        self._map = message
        self._frame_id = message.header.frame_id or 'map'

    def _pose_cb(self, message: PoseStamped):
        self._pose = pose2d_from_pose(message.pose)

    def _goal_cb(self, message: PoseStamped):
        self._goal = pose2d_from_pose(message.pose)

    def _point_cloud_cb(self, message: PointCloud2):
        self._dynamic_points = list(iter_point_cloud_xy(message))

    def _plan_once(self):
        if self._map is None or self._pose is None or self._goal is None:
            return

        overlay = self._build_overlay_costmap()
        path = self._run_hybrid_a_star(overlay)
        if not path:
            self._publish_status('Planner failed to find a feasible path.')
            return

        smoothed = self._smooth_path(path)
        self._path_pub.publish(self._to_path_message(smoothed))
        self._publish_status(f'Planner published path with {len(smoothed)} poses.')

    def _build_overlay_costmap(self) -> list[int]:
        if self._map is None or self._pose is None:
            return []

        overlay = copy_grid_data(self._map)
        resolution = float(self._map.info.resolution)
        width = int(self._map.info.width)
        height = int(self._map.info.height)
        origin_x = float(self._map.info.origin.position.x)
        origin_y = float(self._map.info.origin.position.y)
        dynamic_radius_cells = max(1, int(math.ceil(self._dynamic_obstacle_radius_m / resolution)))

        cos_yaw = math.cos(self._pose.yaw)
        sin_yaw = math.sin(self._pose.yaw)
        for x_local, y_local in self._dynamic_points:
            x_world = self._pose.x + x_local * cos_yaw - y_local * sin_yaw
            y_world = self._pose.y + x_local * sin_yaw + y_local * cos_yaw
            cell = world_to_grid(
                x_world,
                y_world,
                origin_x=origin_x,
                origin_y=origin_y,
                resolution=resolution,
                width=width,
                height=height,
            )
            if cell is None:
                continue
            gx, gy = cell
            for dy in range(-dynamic_radius_cells, dynamic_radius_cells + 1):
                for dx in range(-dynamic_radius_cells, dynamic_radius_cells + 1):
                    nx = gx + dx
                    ny = gy + dy
                    if nx < 0 or ny < 0 or nx >= width or ny >= height:
                        continue
                    if math.hypot(dx, dy) > dynamic_radius_cells:
                        continue
                    overlay[grid_offset(nx, ny, width)] = 100

        return overlay

    def _run_hybrid_a_star(self, overlay: list[int]) -> list[Pose2D]:
        assert self._map is not None
        assert self._pose is not None
        assert self._goal is not None

        start = SearchNode(
            x=self._pose.x,
            y=self._pose.y,
            yaw=self._pose.yaw,
            direction=1,
            steer=0.0,
        )

        start_key = self._state_key(start)
        frontier: list[tuple[float, float, tuple[int, int, int, int]]] = []
        heapq.heappush(frontier, (0.0, 0.0, start_key))

        best_cost: dict[tuple[int, int, int, int], float] = {start_key: 0.0}
        states: dict[tuple[int, int, int, int], SearchNode] = {start_key: start}
        parents: dict[tuple[int, int, int, int], tuple[int, int, int, int] | None] = {
            start_key: None
        }

        iterations = 0
        while frontier and iterations < self._max_iterations:
            _, current_cost, current_key = heapq.heappop(frontier)
            current_state = states[current_key]
            iterations += 1

            if self._goal_reached(current_state):
                return self._reconstruct_path(current_key, parents, states)

            for next_state in self._expand(current_state):
                next_key = self._state_key(next_state)
                travel_cost = self._transition_cost(current_state, next_state)
                new_cost = current_cost + travel_cost

                if new_cost >= best_cost.get(next_key, float('inf')):
                    continue
                if self._is_in_collision(next_state, overlay):
                    continue

                best_cost[next_key] = new_cost
                states[next_key] = next_state
                parents[next_key] = current_key
                priority = new_cost + self._heuristic(next_state)
                heapq.heappush(frontier, (priority, new_cost, next_key))

        return []

    def _expand(self, state: SearchNode) -> list[SearchNode]:
        steer_values = []
        if self._steering_samples <= 1:
            steer_values = [0.0]
        else:
            for index in range(self._steering_samples):
                alpha = -1.0 + 2.0 * index / (self._steering_samples - 1)
                steer_values.append(alpha * self._max_steering_rad)

        directions = [1]
        if self._reverse_enabled:
            directions.append(-1)

        next_states: list[SearchNode] = []
        for direction in directions:
            for steer in steer_values:
                yaw_delta = direction * self._step_size_m / max(1e-3, self._wheelbase_m) * math.tan(steer)
                next_yaw = wrap_angle(state.yaw + yaw_delta)
                next_x = state.x + direction * self._step_size_m * math.cos(state.yaw)
                next_y = state.y + direction * self._step_size_m * math.sin(state.yaw)
                next_states.append(
                    SearchNode(
                        x=next_x,
                        y=next_y,
                        yaw=next_yaw,
                        direction=direction,
                        steer=steer,
                    )
                )
        return next_states

    def _state_key(self, state: SearchNode) -> tuple[int, int, int, int]:
        assert self._map is not None
        resolution = float(self._map.info.resolution)
        origin_x = float(self._map.info.origin.position.x)
        origin_y = float(self._map.info.origin.position.y)
        gx = int(round((state.x - origin_x) / resolution))
        gy = int(round((state.y - origin_y) / resolution))
        yaw_bin = int(round(((wrap_angle(state.yaw) + math.pi) / (2.0 * math.pi)) * self._heading_bins)) % self._heading_bins
        direction_bin = 1 if state.direction >= 0 else 0
        return gx, gy, yaw_bin, direction_bin

    def _transition_cost(self, current: SearchNode, nxt: SearchNode) -> float:
        steer_change = abs(nxt.steer - current.steer)
        reverse_penalty = 2.0 if nxt.direction < 0 else 0.0
        return (
            self._step_size_m
            + 0.4 * abs(nxt.steer)
            + 0.8 * steer_change
            + reverse_penalty
        )

    def _heuristic(self, state: SearchNode) -> float:
        assert self._goal is not None
        distance = distance_xy(state.x, state.y, self._goal.x, self._goal.y)
        heading_error = abs(wrap_angle(self._goal.yaw - state.yaw))
        return distance + 0.2 * heading_error

    def _goal_reached(self, state: SearchNode) -> bool:
        assert self._goal is not None
        return (
            distance_xy(state.x, state.y, self._goal.x, self._goal.y) <= self._goal_position_tolerance_m
            and abs(wrap_angle(self._goal.yaw - state.yaw)) <= self._goal_yaw_tolerance_rad
        )

    def _is_in_collision(self, state: SearchNode, overlay: list[int]) -> bool:
        assert self._map is not None
        resolution = float(self._map.info.resolution)
        width = int(self._map.info.width)
        height = int(self._map.info.height)
        origin_x = float(self._map.info.origin.position.x)
        origin_y = float(self._map.info.origin.position.y)
        radius_cells = max(1, int(math.ceil(self._footprint_radius_m / resolution)))

        for sample_index in range(5):
            fraction = sample_index / 4.0
            sample_x = state.x - fraction * self._step_size_m * math.cos(state.yaw)
            sample_y = state.y - fraction * self._step_size_m * math.sin(state.yaw)
            center = world_to_grid(
                sample_x,
                sample_y,
                origin_x=origin_x,
                origin_y=origin_y,
                resolution=resolution,
                width=width,
                height=height,
            )
            if center is None:
                return True
            cx, cy = center
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    nx = cx + dx
                    ny = cy + dy
                    if nx < 0 or ny < 0 or nx >= width or ny >= height:
                        return True
                    if math.hypot(dx, dy) > radius_cells:
                        continue
                    if overlay[grid_offset(nx, ny, width)] >= self._lethal_cost_threshold:
                        return True
        return False

    def _reconstruct_path(
        self,
        goal_key,
        parents,
        states,
    ) -> list[Pose2D]:
        ordered: list[Pose2D] = []
        key = goal_key
        while key is not None:
            state = states[key]
            ordered.append(Pose2D(x=state.x, y=state.y, yaw=state.yaw))
            key = parents[key]
        ordered.reverse()
        return ordered

    def _smooth_path(self, path: list[Pose2D]) -> list[Pose2D]:
        if len(path) < 3:
            return path

        smoothed = [Pose2D(point.x, point.y, point.yaw) for point in path]
        for _ in range(self._smooth_passes):
            for index in range(1, len(smoothed) - 1):
                prev_point = smoothed[index - 1]
                curr_point = smoothed[index]
                next_point = smoothed[index + 1]
                curr_point.x = 0.25 * prev_point.x + 0.50 * curr_point.x + 0.25 * next_point.x
                curr_point.y = 0.25 * prev_point.y + 0.50 * curr_point.y + 0.25 * next_point.y

        for index in range(len(smoothed) - 1):
            current = smoothed[index]
            nxt = smoothed[index + 1]
            current.yaw = math.atan2(nxt.y - current.y, nxt.x - current.x)
        smoothed[-1].yaw = self._goal.yaw if self._goal else smoothed[-1].yaw
        return smoothed

    def _to_path_message(self, points: list[Pose2D]) -> Path:
        path = Path()
        path.header.frame_id = self._frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.orientation = quaternion_from_yaw(point.yaw)
            path.poses.append(pose)

        return path

    def _publish_status(self, text: str):
        message = String()
        message.data = text
        self._status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
