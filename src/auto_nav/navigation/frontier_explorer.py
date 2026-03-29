"""Frontier-based autonomous exploration with human approval.

Detects unexplored map regions, proposes goals via RViz markers,
waits for terminal y/n approval, then drives via Nav2.
"""

from __future__ import annotations

from collections import deque
from enum import Enum, auto
from functools import partial
import math
import os
import random
import select
import subprocess
import sys
import termios
import time
import tty
from typing import List, Optional, Tuple

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class _State(Enum):
    DETECTING = auto()
    WAITING_APPROVAL = auto()
    NAVIGATING = auto()
    COMPLETE = auto()


Frontier = Tuple[float, float, int]

_NEIGHBORS_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
_NEIGHBORS_8 = _NEIGHBORS_4 + [(-1, -1), (-1, 1), (1, -1), (1, 1)]

# Ignore stale async preview callbacks (wrong generation, state, or frontier).
_PREVIEW_XY_EPS = 0.02

_HELP = """\
Frontier Explorer
-----------------
Detects unexplored map regions and proposes navigation goals.

  y   approve proposed frontier
  n   skip (blacklist) proposed frontier
  c   cancel current navigation
  s   save map & quit
  CTRL-C  quit without saving
"""


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('tick_hz', 5.0)
        self.declare_parameter('detect_cooldown_sec', 2.0)
        self.declare_parameter('nav2_startup_delay_sec', 25.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_save_path',
                               os.path.expanduser('~/guido/maps/frontier_map'))
        self.declare_parameter('goal_retreat_m', 0.3)
        self.declare_parameter('goal_snap_radius_m', 1.0)
        # Must exceed Nav2 xy_goal_tolerance + SLAM jitter; re-clamped in _send_goal.
        self.declare_parameter('min_goal_dist_from_robot_m', 0.55)

        self._min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self._blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self._tick_hz = float(self.get_parameter('tick_hz').value)
        self._detect_cooldown = float(self.get_parameter('detect_cooldown_sec').value)
        nav2_startup_delay = float(self.get_parameter('nav2_startup_delay_sec').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._map_save_path = str(self.get_parameter('map_save_path').value)

        self._goal_retreat_m = float(self.get_parameter('goal_retreat_m').value)
        self._goal_snap_radius_m = float(self.get_parameter('goal_snap_radius_m').value)
        self._min_goal_dist_from_robot_m = float(
            self.get_parameter('min_goal_dist_from_robot_m').value)

        self._map: Optional[OccupancyGrid] = None
        self._state = _State.DETECTING
        self._blacklist: List[Tuple[float, float]] = []
        self._proposal: Optional[Frontier] = None
        self._selected_centroid: Optional[Tuple[float, float]] = None
        self._all_frontiers: List[Frontier] = []
        self._goal_handle = None
        self._printed_waiting_for_map = False
        self._detect_after = time.monotonic() + nav2_startup_delay
        self._last_marker_time = 0.0
        self._reject_count = 0
        self._preview_gen = 0

        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/explorer/frontiers', 10)
        self._preview_path_pub = self.create_publisher(
            Path, '/explorer/preview_path', 10)
        self._nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self._path_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # -- ROS callbacks ----------------------------------------------------

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    # -- Helpers ----------------------------------------------------------

    def _get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return None

    def _enter_detecting(self, cooldown: Optional[float] = None):
        self._state = _State.DETECTING
        self._proposal = None
        self._selected_centroid = None
        self._clear_preview_path()
        cd = cooldown if cooldown is not None else self._detect_cooldown
        self._detect_after = time.monotonic() + cd

    # -- Frontier detection -----------------------------------------------

    def _detect_frontiers(self) -> List[Frontier]:
        assert self._map is not None
        data = self._map.data
        w = int(self._map.info.width)
        h = int(self._map.info.height)
        res = float(self._map.info.resolution)
        ox = float(self._map.info.origin.position.x)
        oy = float(self._map.info.origin.position.y)

        frontier_set: set[Tuple[int, int]] = set()
        for row in range(h):
            row_offset = row * w
            for col in range(w):
                if data[row_offset + col] != 0:
                    continue
                for dr, dc in _NEIGHBORS_4:
                    nr, nc = row + dr, col + dc
                    if 0 <= nr < h and 0 <= nc < w and data[nr * w + nc] == -1:
                        frontier_set.add((row, col))
                        break

        visited: set[Tuple[int, int]] = set()
        clusters: List[Frontier] = []
        for cell in frontier_set:
            if cell in visited:
                continue
            cells: List[Tuple[int, int]] = []
            queue: deque[Tuple[int, int]] = deque([cell])
            visited.add(cell)
            while queue:
                cr, cc = queue.popleft()
                cells.append((cr, cc))
                for dr, dc in _NEIGHBORS_8:
                    nb = (cr + dr, cc + dc)
                    if nb in frontier_set and nb not in visited:
                        visited.add(nb)
                        queue.append(nb)

            if len(cells) < self._min_frontier_size:
                continue

            sum_x = sum_y = 0.0
            for r, c in cells:
                sum_x += ox + (c + 0.5) * res
                sum_y += oy + (r + 0.5) * res
            n = len(cells)
            clusters.append((sum_x / n, sum_y / n, n))

        return clusters

    def _is_blacklisted(self, cx: float, cy: float) -> bool:
        return any(math.hypot(cx - bx, cy - by) < self._blacklist_radius
                   for bx, by in self._blacklist)

    def _select_frontier(
        self, frontiers: List[Frontier], robot_xy: Tuple[float, float],
    ) -> Optional[Frontier]:
        candidates = [(cx, cy, size) for cx, cy, size in frontiers
                      if not self._is_blacklisted(cx, cy)]
        if not candidates:
            return None
        return random.choice(candidates)

    def _world_xy_to_rc(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self._map is None:
            return None
        ox = float(self._map.info.origin.position.x)
        oy = float(self._map.info.origin.position.y)
        res = float(self._map.info.resolution)
        c = int((x - ox) / res)
        r = int((y - oy) / res)
        return (r, c)

    def _snap_to_known_free(self, cx: float, cy: float) -> Tuple[float, float]:
        """Nudge goal onto a known-free /map cell near the target."""
        if self._map is None:
            return (cx, cy)
        data = self._map.data
        w, h = int(self._map.info.width), int(self._map.info.height)
        res = float(self._map.info.resolution)
        ox = float(self._map.info.origin.position.x)
        oy = float(self._map.info.origin.position.y)
        rc = self._world_xy_to_rc(cx, cy)
        if rc is None:
            return (cx, cy)
        r0, c0 = rc
        if 0 <= r0 < h and 0 <= c0 < w and data[r0 * w + c0] == 0:
            return (cx, cy)
        rad = max(2, int(self._goal_snap_radius_m / res))
        best: Optional[Tuple[int, int]] = None
        best_d2: Optional[float] = None
        for r in range(max(0, r0 - rad), min(h, r0 + rad + 1)):
            row_off = r * w
            for c in range(max(0, c0 - rad), min(w, c0 + rad + 1)):
                if data[row_off + c] != 0:
                    continue
                d2 = float((r - r0) ** 2 + (c - c0) ** 2)
                if best_d2 is None or d2 < best_d2:
                    best = (r, c)
                    best_d2 = d2
        if best is None:
            return (cx, cy)
        r, c = best
        return (ox + (c + 0.5) * res, oy + (r + 0.5) * res)

    def _adjust_goal_toward_robot(
        self, cx: float, cy: float, robot_xy: Tuple[float, float],
    ) -> Tuple[float, float]:
        """Pull centroid toward the robot so Nav2 goals sit deeper in free space."""
        rx, ry = robot_xy
        dx, dy = cx - rx, cy - ry
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return (cx, cy)
        mind = self._min_goal_dist_from_robot_m
        retreat = min(self._goal_retreat_m, d * 0.4)
        if d - retreat < mind:
            retreat = max(0.0, d - mind)
        s = retreat / d
        return (cx - dx * s, cy - dy * s)

    def _push_goal_outward(
        self, gx: float, gy: float, robot_xy: Tuple[float, float],
        toward_xy: Tuple[float, float],
    ) -> Tuple[float, float]:
        """If goal is too close to base, move it along robot->frontier direction."""
        rx, ry = robot_xy
        mind = self._min_goal_dist_from_robot_m
        dx, dy = gx - rx, gy - ry
        d = math.hypot(dx, dy)
        if d >= mind:
            return (gx, gy)
        fdx, fdy = toward_xy[0] - rx, toward_xy[1] - ry
        fd = math.hypot(fdx, fdy)
        if fd > 1e-6:
            ux, uy = fdx / fd, fdy / fd
        elif d > 1e-6:
            ux, uy = dx / d, dy / d
        else:
            return (gx, gy)
        return (rx + ux * mind, ry + uy * mind)

    def _clamp_goal_for_nav2(self, x: float, y: float) -> Tuple[float, float]:
        """Re-apply min distance using current TF (SLAM can move pose before 'y')."""
        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            return (x, y)
        rx, ry = robot_xy
        mind = self._min_goal_dist_from_robot_m
        dx, dy = x - rx, y - ry
        d = math.hypot(dx, dy)
        if d >= mind:
            return (x, y)
        ref = self._selected_centroid if self._selected_centroid is not None else (x, y)
        fdx, fdy = ref[0] - rx, ref[1] - ry
        fd = math.hypot(fdx, fdy)
        if fd > 1e-6:
            ux, uy = fdx / fd, fdy / fd
        elif d > 1e-6:
            ux, uy = dx / d, dy / d
        else:
            ux, uy = 1.0, 0.0
        return (rx + ux * mind, ry + uy * mind)


    # -- RViz markers -----------------------------------------------------

    def _publish_markers(self):
        now = time.monotonic()
        if now - self._last_marker_time < 1.0:
            return
        self._last_marker_time = now

        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        delete = Marker()
        delete.header.frame_id = self._map_frame
        delete.header.stamp = stamp
        delete.action = Marker.DELETEALL
        msg.markers.append(delete)
        for i, (cx, cy, size) in enumerate(self._all_frontiers):
            is_selected = (
                self._selected_centroid is not None
                and math.hypot(cx - self._selected_centroid[0],
                               cy - self._selected_centroid[1]) < 0.02)
            blacklisted = self._is_blacklisted(cx, cy)

            sphere = Marker()
            sphere.header.frame_id = self._map_frame
            sphere.header.stamp = stamp
            sphere.ns = 'frontier_centroids'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = cx
            sphere.pose.position.y = cy
            sphere.pose.position.z = 0.1
            sphere.pose.orientation.w = 1.0
            s = 0.3 if is_selected else 0.15
            sphere.scale.x = s
            sphere.scale.y = s
            sphere.scale.z = s
            if is_selected:
                sphere.color.r, sphere.color.g, sphere.color.b = 0.0, 1.0, 0.0
            elif blacklisted:
                sphere.color.r, sphere.color.g, sphere.color.b = 0.5, 0.5, 0.5
            else:
                sphere.color.r, sphere.color.g, sphere.color.b = 0.2, 0.4, 1.0
            sphere.color.a = 0.9
            msg.markers.append(sphere)

            if is_selected:
                label = Marker()
                label.header.frame_id = self._map_frame
                label.header.stamp = stamp
                label.ns = 'frontier_labels'
                label.id = i
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = cx
                label.pose.position.y = cy
                label.pose.position.z = 0.4
                label.pose.orientation.w = 1.0
                label.scale.z = 0.15
                label.color.r = label.color.g = label.color.b = 1.0
                label.color.a = 1.0
                label.text = f'GOAL ({size} cells)'
                msg.markers.append(label)

        self._marker_pub.publish(msg)

    # -- Path preview -----------------------------------------------------

    def _clear_preview_path(self):
        empty = Path()
        empty.header.frame_id = self._map_frame
        empty.header.stamp = self.get_clock().now().to_msg()
        self._preview_path_pub.publish(empty)

    def _compute_preview_path(self, x: float, y: float):
        if not self._path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Planner server not available for preview')
            return

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = self._map_frame
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False

        self._preview_gen += 1
        gen = self._preview_gen
        future = self._path_client.send_goal_async(goal)
        future.add_done_callback(
            partial(self._on_preview_goal_response, gen=gen, tx=x, ty=y))

    def _on_preview_goal_response(
            self, future, gen: int, tx: float, ty: float):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Planner rejected preview path request')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._on_preview_result, gen=gen, tx=tx, ty=ty))

    def _on_preview_result(self, future, gen: int, tx: float, ty: float):
        if gen != self._preview_gen:
            return
        if self._state != _State.WAITING_APPROVAL:
            return
        if self._proposal is None:
            return
        cx, cy, _ = self._proposal
        if (abs(cx - tx) > _PREVIEW_XY_EPS
                or abs(cy - ty) > _PREVIEW_XY_EPS):
            return

        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._preview_path_pub.publish(result.result.path)
            self.get_logger().info('Preview path published')
        else:
            self.get_logger().warn('Preview path computation failed')
            self._blacklist.append((cx, cy))
            print('  No valid path to frontier — auto-blacklisting.')
            self._enter_detecting(cooldown=0.5)

    # -- Nav2 action ------------------------------------------------------

    def _send_goal(self, x: float, y: float):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not available')
            print('  Nav2 action server not available. Retrying...')
            self._enter_detecting(cooldown=3.0)
            return

        x, y = self._clamp_goal_for_nav2(x, y)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending Nav2 goal ({x:.2f}, {y:.2f})')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._reject_count += 1
            self.get_logger().warn(
                f'Nav2 rejected goal (attempt {self._reject_count})')
            if self._reject_count >= 3:
                print(f'  Nav2 rejected goal {self._reject_count} times. '
                      f'Blacklisting frontier.')
                if self._proposal:
                    self._blacklist.append(
                        (self._proposal[0], self._proposal[1]))
                self._reject_count = 0
                self._enter_detecting()
            else:
                print(f'  Nav2 rejected goal (attempt {self._reject_count}/3). '
                      f'Nav2 may still be starting. Retrying in 5s...')
                self._enter_detecting(cooldown=5.0)
            return

        self._reject_count = 0
        self._goal_handle = goal_handle
        self.get_logger().info('Nav2 accepted goal, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        self._goal_handle = None
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached frontier!')
            print('\n  Reached frontier. Re-detecting...')
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation canceled.')
            print('\n  Navigation canceled.')
        else:
            self.get_logger().warn(f'Navigation failed (status {result.status})')
            print(f'\n  Navigation failed (status {result.status}). Blacklisting.')
            if self._proposal:
                self._blacklist.append((self._proposal[0], self._proposal[1]))
        self._enter_detecting()

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self.get_logger().info('Cancelling Nav2 goal')

    # -- Map saving -------------------------------------------------------

    def _save_map(self):
        save_dir = os.path.dirname(self._map_save_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        print(f'\n  Saving map to {self._map_save_path} ...')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', self._map_save_path,
                 '--ros-args', '-p', 'save_map_timeout:=10000'],
                timeout=30, check=False)
            print(f'  Map saved to {self._map_save_path}.pgm / .yaml')
        except Exception as exc:
            print(f'  Map save error: {exc}')

    # -- Keyboard input ---------------------------------------------------

    def _handle_key(self, key: str):
        if key == 's':
            if self._state == _State.NAVIGATING:
                self._cancel_goal()
            self._save_map()
            raise KeyboardInterrupt

        if self._state == _State.WAITING_APPROVAL:
            if key == 'y' and self._proposal is not None:
                cx, cy, size = self._proposal
                print(f'  Approved! Navigating to ({cx:.2f}, {cy:.2f})...')
                self._clear_preview_path()
                self._state = _State.NAVIGATING
                self._send_goal(cx, cy)
            elif key == 'n':
                if self._proposal:
                    self._blacklist.append(
                        (self._proposal[0], self._proposal[1]))
                    print('  Skipped. Blacklisting frontier.')
                self._enter_detecting(cooldown=0.5)

        elif self._state == _State.NAVIGATING:
            if key == 'c':
                self._cancel_goal()
                print('  Cancelling navigation...')

    # -- Main loop --------------------------------------------------------

    def _tick(self):
        self._publish_markers()

        if self._state != _State.DETECTING:
            return

        if self._map is None:
            if not self._printed_waiting_for_map:
                print('  Waiting for /map ...')
                self._printed_waiting_for_map = True
            return
        self._printed_waiting_for_map = False

        remaining = self._detect_after - time.monotonic()
        if remaining > 0:
            if remaining > 1.0 and int(remaining) % 5 == 0:
                self.get_logger().info(
                    f'Waiting {remaining:.0f}s for Nav2 to initialize...')
            return

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn('TF not available yet')
            return

        self._all_frontiers = self._detect_frontiers()
        self._proposal = self._select_frontier(self._all_frontiers, robot_xy)

        if self._proposal is not None:
            pcx, pcy, psz = self._proposal
            self._selected_centroid = (pcx, pcy)
            ax, ay = self._adjust_goal_toward_robot(pcx, pcy, robot_xy)
            sx, sy = self._snap_to_known_free(ax, ay)
            sx, sy = self._push_goal_outward(sx, sy, robot_xy, (pcx, pcy))
            sx, sy = self._snap_to_known_free(sx, sy)
            sx, sy = self._push_goal_outward(sx, sy, robot_xy, (pcx, pcy))
            self._proposal = (sx, sy, psz)

        if self._proposal is None:
            if self._blacklist:
                print('  All remaining frontiers blacklisted. Clearing...')
                self._blacklist.clear()
                self._detect_after = time.monotonic() + 0.5
                return
            self._state = _State.COMPLETE
            print('\n  No frontiers remain. Exploration complete!')
            self._save_map()
            raise KeyboardInterrupt

        cx, cy, size = self._proposal
        dist = math.hypot(cx - robot_xy[0], cy - robot_xy[1])
        n_total = len(self._all_frontiers)
        n_ok = sum(1 for fx, fy, _ in self._all_frontiers
                   if not self._is_blacklisted(fx, fy))

        print(f'\n  {n_total} frontier(s) found, {n_ok} available.')
        print(f'  Proposed: ({cx:.2f}, {cy:.2f})  '
              f'dist={dist:.1f}m  size={size} cells')
        print('  [y] approve  [n] skip  [s] save map & quit')
        self._state = _State.WAITING_APPROVAL
        self._last_marker_time = 0.0  # force immediate marker publish
        self._compute_preview_path(cx, cy)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(_HELP)
        try:
            tty.setcbreak(sys.stdin.fileno())
            timeout = 1.0 / self._tick_hz
            while rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], timeout)
                if ready:
                    key = sys.stdin.read(1)
                    if key == '\x03':
                        raise KeyboardInterrupt
                    self._handle_key(key)
                rclpy.spin_once(self, timeout_sec=0.0)
                self._tick()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
