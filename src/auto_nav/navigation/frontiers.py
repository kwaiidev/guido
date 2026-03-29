from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import math
from typing import Iterable
from typing import Sequence
from typing import Tuple

from navigation.types import FrontierParams
from navigation.types import OccupancyGridSnapshot
from navigation.types import Pose2D


_NEIGHBORS_4 = ((-1, 0), (0, -1), (0, 1), (1, 0))
_NEIGHBORS_8 = (
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)
_DEFAULT_PARAMS = FrontierParams()


@dataclass(frozen=True)
class FrontierCluster:
    cells: Tuple[Tuple[int, int], ...]
    centroid_x: float
    centroid_y: float

    @property
    def size(self) -> int:
        return len(self.cells)

    def as_pose(self) -> Pose2D:
        return Pose2D(self.centroid_x, self.centroid_y, 0.0)

    @property
    def centroid_pose(self) -> Pose2D:
        return self.as_pose()


@dataclass(frozen=True)
class FrontierGoal:
    cluster: FrontierCluster
    target_row: int
    target_col: int
    target_pose: Pose2D
    distance_to_robot: float
    information_gain: float = 0.0
    score: float = 0.0

    @property
    def reference_pose(self) -> Pose2D:
        return self.cluster.centroid_pose


def extract_frontier_clusters(
    grid: OccupancyGridSnapshot,
    min_cluster_size: int = 10,
) -> Tuple[FrontierCluster, ...]:
    frontier_cells = _find_frontier_cells(grid)
    visited = set()
    clusters = []

    for start in sorted(frontier_cells):
        if start in visited:
            continue

        cluster_cells = []
        queue = deque([start])
        visited.add(start)
        while queue:
            cell = queue.popleft()
            cluster_cells.append(cell)
            row, col = cell
            for d_row, d_col in _NEIGHBORS_8:
                neighbor = (row + d_row, col + d_col)
                if neighbor in frontier_cells and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        if len(cluster_cells) < min_cluster_size:
            continue

        centroid_x, centroid_y = _cluster_centroid(grid, cluster_cells)
        clusters.append(
            FrontierCluster(
                cells=tuple(sorted(cluster_cells)),
                centroid_x=centroid_x,
                centroid_y=centroid_y,
            )
        )

    return tuple(sorted(clusters, key=lambda cluster: (cluster.centroid_x, cluster.centroid_y)))


def rank_frontier_goals(
    grid: OccupancyGridSnapshot,
    robot_pose: Pose2D,
    blacklist_points: Sequence[Pose2D] = (),
    min_cluster_size: int = 10,
    blacklist_radius: float = 0.5,
    params: FrontierParams | None = None,
) -> Tuple[FrontierGoal, ...]:
    params = params or _DEFAULT_PARAMS
    min_cluster_size = params.min_cluster_size if min_cluster_size == 10 else min_cluster_size
    blacklist_radius = (
        params.blacklist_radius if blacklist_radius == 0.5 else blacklist_radius
    )

    robot_cell = _resolve_robot_cell(grid, robot_pose)
    if robot_cell is None:
        return tuple()

    reachable_free_cells = _reachable_free_cells(grid, robot_cell)
    if not reachable_free_cells:
        return tuple()

    goals = []
    for cluster in extract_frontier_clusters(grid, min_cluster_size):
        centroid_pose = cluster.centroid_pose
        if _is_blacklisted(centroid_pose, blacklist_points, blacklist_radius):
            continue

        goal_cell = _select_goal_cell_for_cluster(
            grid,
            cluster,
            robot_pose,
            reachable_free_cells,
        )
        if goal_cell is None:
            continue

        path_cost = estimate_path_cost(grid, robot_cell, goal_cell)
        if path_cost is None:
            continue

        target_row, target_col = goal_cell
        target_x, target_y = grid.cell_center(target_row, target_col)
        yaw = math.atan2(cluster.centroid_y - target_y, cluster.centroid_x - target_x)
        target_pose = Pose2D(target_x, target_y, yaw)
        distance_to_robot = path_cost * grid.resolution
        information_gain = _compute_information_gain(grid, cluster, params.info_gain_radius)
        score = distance_to_robot - params.info_gain_weight * information_gain * grid.resolution
        goals.append(
            FrontierGoal(
                cluster=cluster,
                target_row=target_row,
                target_col=target_col,
                target_pose=target_pose,
                distance_to_robot=distance_to_robot,
                information_gain=information_gain,
                score=score,
            )
        )

    goals.sort(
        key=lambda goal: (
            goal.score,
            goal.distance_to_robot,
            -goal.cluster.size,
            goal.cluster.centroid_x,
            goal.cluster.centroid_y,
        )
    )
    return tuple(goals)


def select_frontier_goal(
    grid: OccupancyGridSnapshot,
    robot_pose: Pose2D,
    blacklist_points: Sequence[Pose2D] = (),
    min_cluster_size: int = 10,
    blacklist_radius: float = 0.5,
    params: FrontierParams | None = None,
) -> FrontierGoal | None:
    goals = rank_frontier_goals(
        grid,
        robot_pose,
        blacklist_points=blacklist_points,
        min_cluster_size=min_cluster_size,
        blacklist_radius=blacklist_radius,
        params=params,
    )
    return goals[0] if goals else None


def select_frontier_cluster(
    grid: OccupancyGridSnapshot,
    robot_pose: Pose2D,
    blacklist_points: Sequence[Pose2D] = (),
    min_cluster_size: int = 10,
    blacklist_radius: float = 0.5,
    params: FrontierParams | None = None,
) -> FrontierCluster | None:
    goal = select_frontier_goal(
        grid,
        robot_pose,
        blacklist_points=blacklist_points,
        min_cluster_size=min_cluster_size,
        blacklist_radius=blacklist_radius,
        params=params,
    )
    return goal.cluster if goal is not None else None


def compute_coverage(grid: OccupancyGridSnapshot) -> float:
    """Return the fraction of mapped cells that are free (0.0–1.0).

    Coverage = free_cells / (free_cells + unknown_cells).
    Returns 1.0 when there are no unknown cells left.
    """

    free_cells = sum(1 for value in grid.data if value == 0)
    unknown_cells = sum(1 for value in grid.data if value == -1)
    if unknown_cells == 0:
        return 1.0
    return free_cells / float(free_cells + unknown_cells)


def estimate_path_cost(
    grid: OccupancyGridSnapshot,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> float | None:
    """A* on the occupancy grid; returns path cost in cells or None."""

    if grid.value(*start) != 0 or grid.value(*goal) != 0:
        return None

    queue = [(0.0, 0.0, start)]
    best_cost = {start: 0.0}

    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current == goal:
            return current_cost
        if current_cost > best_cost.get(current, float('inf')):
            continue

        row, col = current
        for d_row, d_col in _NEIGHBORS_8:
            n_row = row + d_row
            n_col = col + d_col
            if not grid.in_bounds(n_row, n_col):
                continue
            if grid.value(n_row, n_col) != 0:
                continue

            step_cost = math.hypot(d_row, d_col)
            next_cost = current_cost + step_cost
            neighbor = (n_row, n_col)
            if next_cost >= best_cost.get(neighbor, float('inf')):
                continue

            best_cost[neighbor] = next_cost
            heuristic = math.hypot(goal[0] - n_row, goal[1] - n_col)
            heapq.heappush(queue, (next_cost + heuristic, next_cost, neighbor))

    return None


def _compute_information_gain(
    grid: OccupancyGridSnapshot,
    cluster: FrontierCluster,
    radius: int,
) -> float:
    """Count unknown cells within *radius* cells of any cluster cell."""

    if radius <= 0:
        return 0.0

    unknown_cells = set()
    for row, col in cluster.cells:
        row_min = max(0, row - radius)
        row_max = min(grid.height, row + radius + 1)
        col_min = max(0, col - radius)
        col_max = min(grid.width, col + radius + 1)
        for n_row in range(row_min, row_max):
            for n_col in range(col_min, col_max):
                if max(abs(n_row - row), abs(n_col - col)) > radius:
                    continue
                if grid.value(n_row, n_col) == -1:
                    unknown_cells.add((n_row, n_col))

    return float(len(unknown_cells))


def _find_frontier_cells(grid: OccupancyGridSnapshot) -> set[Tuple[int, int]]:
    frontier_cells = set()
    for row in range(grid.height):
        for col in range(grid.width):
            if grid.value(row, col) == -1 and _has_adjacent_free_cell(grid, row, col):
                frontier_cells.add((row, col))
    return frontier_cells


def _has_adjacent_free_cell(grid: OccupancyGridSnapshot, row: int, col: int) -> bool:
    for d_row, d_col in _NEIGHBORS_8:
        n_row = row + d_row
        n_col = col + d_col
        if not grid.in_bounds(n_row, n_col):
            continue
        if grid.value(n_row, n_col) == 0:
            return True
    return False


def _cluster_centroid(
    grid: OccupancyGridSnapshot,
    cells: Iterable[Tuple[int, int]],
) -> Tuple[float, float]:
    xs = []
    ys = []
    for row, col in cells:
        center_x, center_y = grid.cell_center(row, col)
        xs.append(center_x)
        ys.append(center_y)
    return sum(xs) / len(xs), sum(ys) / len(ys)


def _is_blacklisted(
    centroid_pose: Pose2D,
    blacklist_points: Sequence[Pose2D],
    blacklist_radius: float,
) -> bool:
    return any(
        centroid_pose.planar_distance_to(blacklisted) <= blacklist_radius
        for blacklisted in blacklist_points
    )


def _resolve_robot_cell(
    grid: OccupancyGridSnapshot,
    robot_pose: Pose2D,
) -> Tuple[int, int] | None:
    cell = grid.world_to_cell(robot_pose.x, robot_pose.y)
    if cell is not None and grid.value(*cell) == 0:
        return cell

    if cell is None:
        search_rows = range(grid.height)
        search_cols = range(grid.width)
    else:
        row, col = cell
        search_rows = range(max(0, row - 3), min(grid.height, row + 4))
        search_cols = range(max(0, col - 3), min(grid.width, col + 4))

    closest = None
    closest_cost = float('inf')
    for row in search_rows:
        for col in search_cols:
            if grid.value(row, col) != 0:
                continue
            center_x, center_y = grid.cell_center(row, col)
            distance = math.hypot(center_x - robot_pose.x, center_y - robot_pose.y)
            if distance < closest_cost:
                closest = (row, col)
                closest_cost = distance
    return closest


def _reachable_free_cells(
    grid: OccupancyGridSnapshot,
    start_cell: Tuple[int, int],
) -> set[Tuple[int, int]]:
    if grid.value(*start_cell) != 0:
        return set()

    reachable = {start_cell}
    queue = deque([start_cell])
    while queue:
        row, col = queue.popleft()
        for d_row, d_col in _NEIGHBORS_4:
            neighbor = (row + d_row, col + d_col)
            n_row, n_col = neighbor
            if neighbor in reachable or not grid.in_bounds(n_row, n_col):
                continue
            if grid.value(n_row, n_col) != 0:
                continue
            reachable.add(neighbor)
            queue.append(neighbor)
    return reachable


def _select_goal_cell_for_cluster(
    grid: OccupancyGridSnapshot,
    cluster: FrontierCluster,
    robot_pose: Pose2D,
    reachable_free_cells: set[Tuple[int, int]],
) -> Tuple[int, int] | None:
    centroid_pose = cluster.centroid_pose
    candidates = []
    for row, col in _border_free_cells(grid, cluster.cells):
        if (row, col) not in reachable_free_cells:
            continue
        center_x, center_y = grid.cell_center(row, col)
        centroid_distance = math.hypot(center_x - centroid_pose.x, center_y - centroid_pose.y)
        robot_distance = math.hypot(center_x - robot_pose.x, center_y - robot_pose.y)
        candidates.append((centroid_distance, robot_distance, row, col))
    if not candidates:
        return None
    candidates.sort()
    _, _, row, col = candidates[0]
    return row, col


def _border_free_cells(
    grid: OccupancyGridSnapshot,
    frontier_cells: Iterable[Tuple[int, int]],
) -> set[Tuple[int, int]]:
    free_cells = set()
    for row, col in frontier_cells:
        for d_row, d_col in _NEIGHBORS_8:
            neighbor = (row + d_row, col + d_col)
            n_row, n_col = neighbor
            if not grid.in_bounds(n_row, n_col):
                continue
            if grid.value(n_row, n_col) == 0:
                free_cells.add(neighbor)
    return free_cells
