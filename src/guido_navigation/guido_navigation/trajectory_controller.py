"""Pure-pursuit style local controller with speed regulation."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node

from .common import Pose2D, clamp, grid_offset, pose2d_from_pose, world_to_grid


class TrajectoryController(Node):
    def __init__(self):
        super().__init__("guido_trajectory_controller")

        self.declare_parameter("pose_topic", "/navigation/pose")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("path_topic", "/navigation/global_path")
        self.declare_parameter("costmap_topic", "/navigation/local_costmap")
        self.declare_parameter("cmd_topic", "/cmd_vel_autonomy")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("lookahead_base_m", 0.70)
        self.declare_parameter("lookahead_gain", 1.20)
        self.declare_parameter("min_lookahead_m", 0.60)
        self.declare_parameter("max_lookahead_m", 2.50)
        self.declare_parameter("max_speed_mps", 1.00)
        self.declare_parameter("min_speed_mps", 0.15)
        self.declare_parameter("max_yaw_rate_rps", 0.90)
        self.declare_parameter("max_accel_mps2", 0.50)
        self.declare_parameter("max_decel_mps2", 0.90)
        self.declare_parameter("curvature_speed_gain", 2.20)
        self.declare_parameter("obstacle_slowdown_distance_m", 3.00)
        self.declare_parameter("obstacle_stop_distance_m", 0.90)
        self.declare_parameter("goal_tolerance_m", 0.35)
        self.declare_parameter("goal_slowdown_distance_m", 1.50)

        self._lookahead_base_m = float(self.get_parameter("lookahead_base_m").value)
        self._lookahead_gain = float(self.get_parameter("lookahead_gain").value)
        self._min_lookahead_m = float(self.get_parameter("min_lookahead_m").value)
        self._max_lookahead_m = float(self.get_parameter("max_lookahead_m").value)
        self._max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self._min_speed_mps = float(self.get_parameter("min_speed_mps").value)
        self._max_yaw_rate_rps = float(self.get_parameter("max_yaw_rate_rps").value)
        self._max_accel_mps2 = float(self.get_parameter("max_accel_mps2").value)
        self._max_decel_mps2 = float(self.get_parameter("max_decel_mps2").value)
        self._curvature_speed_gain = float(
            self.get_parameter("curvature_speed_gain").value
        )
        self._obstacle_slowdown_distance_m = float(
            self.get_parameter("obstacle_slowdown_distance_m").value
        )
        self._obstacle_stop_distance_m = float(
            self.get_parameter("obstacle_stop_distance_m").value
        )
        self._goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        self._goal_slowdown_distance_m = float(
            self.get_parameter("goal_slowdown_distance_m").value
        )

        pose_topic = str(self.get_parameter("pose_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        costmap_topic = str(self.get_parameter("costmap_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Path, path_topic, self._path_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, costmap_topic, self._costmap_cb, 10)
        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)

        self._pose: Pose2D | None = None
        self._path: list[Pose2D] = []
        self._costmap: OccupancyGrid | None = None
        self._current_speed = 0.0
        self._last_cmd = Twist()
        self._control_period = 1.0 / control_rate_hz
        self.create_timer(self._control_period, self._control_loop)

        self.get_logger().info("Trajectory controller online.")

    def _pose_cb(self, message):
        self._pose = pose2d_from_pose(message.pose)

    def _odom_cb(self, message: Odometry):
        self._current_speed = float(message.twist.twist.linear.x)

    def _path_cb(self, message: Path):
        self._path = [pose2d_from_pose(pose.pose) for pose in message.poses]

    def _costmap_cb(self, message: OccupancyGrid):
        self._costmap = message

    def _control_loop(self):
        if self._pose is None or len(self._path) < 2:
            self._publish_stop()
            return

        target = self._find_target_point()
        if target is None:
            self._publish_stop()
            return

        dx_world = target.x - self._pose.x
        dy_world = target.y - self._pose.y
        cos_yaw = math.cos(-self._pose.yaw)
        sin_yaw = math.sin(-self._pose.yaw)
        x_local = dx_world * cos_yaw - dy_world * sin_yaw
        y_local = dx_world * sin_yaw + dy_world * cos_yaw
        lookahead = max(1e-3, math.hypot(x_local, y_local))

        if self._distance_to_goal() <= self._goal_tolerance_m:
            self._publish_stop()
            return

        curvature = 2.0 * y_local / max(lookahead * lookahead, 1e-3)
        desired_speed = self._compute_desired_speed(curvature)
        desired_speed = self._apply_accel_limits(desired_speed)

        command = Twist()
        command.linear.x = desired_speed
        command.angular.z = clamp(
            desired_speed * curvature,
            -self._max_yaw_rate_rps,
            self._max_yaw_rate_rps,
        )
        self._last_cmd = command
        self._cmd_pub.publish(command)

    def _find_target_point(self) -> Pose2D | None:
        assert self._pose is not None
        lookahead_distance = clamp(
            self._lookahead_base_m + self._lookahead_gain * abs(self._current_speed),
            self._min_lookahead_m,
            self._max_lookahead_m,
        )

        nearest_index = 0
        nearest_distance = float("inf")
        for index, point in enumerate(self._path):
            distance = math.hypot(point.x - self._pose.x, point.y - self._pose.y)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index

        for point in self._path[nearest_index:]:
            distance = math.hypot(point.x - self._pose.x, point.y - self._pose.y)
            if distance >= lookahead_distance:
                return point
        return self._path[-1] if self._path else None

    def _compute_desired_speed(self, curvature: float) -> float:
        curvature_scale = 1.0 / (1.0 + self._curvature_speed_gain * abs(curvature))
        obstacle_scale = 1.0
        obstacle_distance = self._costmap_forward_distance()
        if obstacle_distance <= self._obstacle_stop_distance_m:
            return 0.0
        if obstacle_distance < self._obstacle_slowdown_distance_m:
            obstacle_scale = obstacle_distance / max(
                self._obstacle_slowdown_distance_m, 1e-3
            )

        goal_distance = self._distance_to_goal()
        goal_scale = 1.0
        if goal_distance < self._goal_slowdown_distance_m:
            goal_scale = max(
                goal_distance / max(self._goal_slowdown_distance_m, 1e-3), 0.25
            )

        speed = self._max_speed_mps * curvature_scale * obstacle_scale * goal_scale
        if speed <= 0.0:
            return 0.0
        return clamp(speed, self._min_speed_mps, self._max_speed_mps)

    def _apply_accel_limits(self, desired_speed: float) -> float:
        current = float(self._last_cmd.linear.x)
        max_increase = self._max_accel_mps2 * self._control_period
        max_decrease = self._max_decel_mps2 * self._control_period

        if desired_speed > current:
            return min(desired_speed, current + max_increase)
        return max(desired_speed, current - max_decrease)

    def _costmap_forward_distance(self) -> float:
        if self._costmap is None:
            return float("inf")

        resolution = float(self._costmap.info.resolution)
        width = int(self._costmap.info.width)
        height = int(self._costmap.info.height)
        origin_x = float(self._costmap.info.origin.position.x)
        origin_y = float(self._costmap.info.origin.position.y)

        best = float("inf")
        for x_coord in self._frange(
            0.2, self._obstacle_slowdown_distance_m, resolution
        ):
            for y_coord in self._frange(-0.5, 0.5, resolution):
                cell = world_to_grid(
                    x_coord,
                    y_coord,
                    origin_x=origin_x,
                    origin_y=origin_y,
                    resolution=resolution,
                    width=width,
                    height=height,
                )
                if cell is None:
                    continue
                gx, gy = cell
                if self._costmap.data[grid_offset(gx, gy, width)] >= 70:
                    best = min(best, x_coord)
        return best

    def _distance_to_goal(self) -> float:
        assert self._pose is not None
        if not self._path:
            return float("inf")
        goal = self._path[-1]
        return math.hypot(goal.x - self._pose.x, goal.y - self._pose.y)

    def _publish_stop(self):
        self._last_cmd = Twist()
        self._cmd_pub.publish(self._last_cmd)

    @staticmethod
    def _frange(start: float, stop: float, step: float):
        current = start
        while current <= stop:
            yield current
            current += step


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
