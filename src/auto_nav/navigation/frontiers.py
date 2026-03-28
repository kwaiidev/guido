from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

from .types import OccupancyGridSnapshot, Pose2D

_NEIGHBORS_4 = (
    (-1, 0),
    (0, -1),
    (0, 1),
    (1, 0),
)

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


@dataclass(frozen=True)
class FrontierCluster:
    cells: Tuple[Tuple[int, int], ...]
    centroid_x: float
    centroid_y: float

    @property
    def size(self) -> int:
        return len(self.cells)

    def as_pose(self, robot_pose: Pose2D) -> Pose2D:
        yaw = math.atan2(self.centroid_y - robot_pose.y, self.centroid_x - robot_pose.x)
        return Pose2D(x=self.centroid_x, y=self.centroid_y, yaw=yaw)

    def centroid_pose(self) -> Pose2D:
        return Pose2D(x=self.centroid_x, y=self.centroid_y, yaw=0.0)


@dataclass(frozen=True)
class FrontierGoal:
    cluster: FrontierCluster
    target_row: int
    target_col: int
    target_pose: Pose2D
    distance_to_robot: float

    @property
    def reference_pose(self) -> Pose2D:
        return self.cluster.centroid_pose()


def extract_frontier_clusters(
    grid: OccupancyGridSnapshot,
    min_cluster_size: int = 10,
) -> Tuple[FrontierCluster, ...]:
    frontier_cells = _find_frontier_cells(grid)
    visited = set()
    clusters = []

    for start_cell in frontier_cells:
        if start_cell in visited:
            continue

        cluster_cells = []
        queue = deque([start_cell])
        visited.add(start_cell)

        while queue:
            row, col = queue.popleft()
            cluster_cells.append((row, col))
            for row_offset, col_offset in _NEIGHBORS_8:
                neighbor = (row + row_offset, col + col_offset)
                if neighbor in visited or neighbor not in frontier_cells:
                    continue
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

    return tuple(clusters)


def rank_frontier_goals(
    grid: OccupancyGridSnapshot,
    robot_pose: Pose2D,
    blacklist_points: Sequence[Pose2D] = (),
    min_cluster_size: int = 10,
    blacklist_radius: float = 0.5,
) -> Tuple[FrontierGoal, ...]:
    robot_cell = _resolve_robot_cell(grid, robot_pose)
    if robot_cell is None:
        return ()

    reachable_free_cells = _reachable_free_cells(grid, robot_cell)
    candidates = []

    for cluster in extract_frontier_clusters(grid, min_cluster_size=min_cluster_size):
        centroid_pose = cluster.centroid_pose()
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

        goal_x, goal_y = grid.cell_center(*goal_cell)
        goal_pose = Pose2D(
            x=goal_x,
            y=goal_y,
            yaw=math.atan2(cluster.centroid_y - goal_y, cluster.centroid_x - goal_x),
        )
        distance_to_robot = robot_pose.planar_distance_to(goal_pose)
        candidates.append(
            (
                distance_to_robot,
                centroid_pose.planar_distance_to(goal_pose),
                FrontierGoal(
                    cluster=cluster,
                    target_row=goal_cell[0],
                    target_col=goal_cell[1],
                    target_pose=goal_pose,
                    distance_to_robot=distance_to_robot,
                ),
            )
        )

    candidates.sort(key=lambda item: (item[0], item[1]))
    return tuple(item[2] for item in candidates)


def select_frontier_goal(
    grid: OccupancyGridSnapshot,
    robot_pose: Pose2D,
    blacklist_points: Sequence[Pose2D] = (),
    min_cluster_size: int = 10,
    blacklist_radius: float = 0.5,
) -> FrontierGoal | None:
    goals = rank_frontier_goals(
        grid,
        robot_pose=robot_pose,
        blacklist_points=blacklist_points,
        min_cluster_size=min_cluster_size,
        blacklist_radius=blacklist_radius,
    )
    if not goals:
        return None
    return goals[0]


def select_frontier_cluster(
    grid: OccupancyGridSnapshot,
    robot_pose: Pose2D,
    blacklist_points: Sequence[Pose2D] = (),
    min_cluster_size: int = 10,
    blacklist_radius: float = 0.5,
) -> FrontierCluster | None:
    selected_goal = select_frontier_goal(
        grid,
        robot_pose=robot_pose,
        blacklist_points=blacklist_points,
        min_cluster_size=min_cluster_size,
        blacklist_radius=blacklist_radius,
    )
    if selected_goal is None:
        return None
    return selected_goal.cluster


def _find_frontier_cells(grid: OccupancyGridSnapshot) -> set[Tuple[int, int]]:
    cells = set()
    for row in range(grid.height):
        for col in range(grid.width):
            if grid.value(row, col) != -1:
                continue
            if _has_adjacent_free_cell(grid, row, col):
                cells.add((row, col))
    return cells


def _has_adjacent_free_cell(grid: OccupancyGridSnapshot, row: int, col: int) -> bool:
    for row_offset, col_offset in _NEIGHBORS_8:
        neighbor_row = row + row_offset
        neighbor_col = col + col_offset
        if not grid.in_bounds(neighbor_row, neighbor_col):
            continue
        if grid.value(neighbor_row, neighbor_col) == 0:
            return True
    return False


def _cluster_centroid(
    grid: OccupancyGridSnapshot,
    cells: Iterable[Tuple[int, int]],
) -> Tuple[float, float]:
    xs = []
    ys = []
    for row, col in cells:
        cell_x, cell_y = grid.cell_center(row, col)
        xs.append(cell_x)
        ys.append(cell_y)
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def _is_blacklisted(
    centroid_pose: Pose2D,
    blacklist_points: Sequence[Pose2D],
    blacklist_radius: float,
) -> bool:
    for point in blacklist_points:
        if centroid_pose.planar_distance_to(point) <= blacklist_radius:
            return True
    return False


def _resolve_robot_cell(
    grid: OccupancyGridSnapshot,
    robot_pose: Pose2D,
) -> Tuple[int, int] | None:
    start_cell = grid.world_to_cell(robot_pose.x, robot_pose.y)
    if start_cell is None:
        return None
    if grid.value(*start_cell) == 0:
        return start_cell

    best_candidate = None
    max_radius = 3
    for radius in range(1, max_radius + 1):
        for row in range(start_cell[0] - radius, start_cell[0] + radius + 1):
            for col in range(start_cell[1] - radius, start_cell[1] + radius + 1):
                if not grid.in_bounds(row, col) or grid.value(row, col) != 0:
                    continue
                cell_x, cell_y = grid.cell_center(row, col)
                distance = math.hypot(cell_x - robot_pose.x, cell_y - robot_pose.y)
                if best_candidate is None or distance < best_candidate[0]:
                    best_candidate = (distance, (row, col))
        if best_candidate is not None:
            return best_candidate[1]

    return None


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
        for row_offset, col_offset in _NEIGHBORS_4:
            neighbor_row = row + row_offset
            neighbor_col = col + col_offset
            neighbor = (neighbor_row, neighbor_col)
            if not grid.in_bounds(neighbor_row, neighbor_col):
                continue
            if neighbor in reachable or grid.value(neighbor_row, neighbor_col) != 0:
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
    candidates = []
    centroid_pose = cluster.centroid_pose()

    for candidate in _border_free_cells(grid, cluster.cells):
        if candidate not in reachable_free_cells:
            continue
        cell_x, cell_y = grid.cell_center(*candidate)
        distance_to_robot = math.hypot(cell_x - robot_pose.x, cell_y - robot_pose.y)
        distance_to_centroid = math.hypot(
            cell_x - centroid_pose.x,
            cell_y - centroid_pose.y,
        )
        candidates.append(
            (
                distance_to_robot,
                distance_to_centroid,
                candidate[0],
                candidate[1],
            )
        )

    if not candidates:
        return None

    candidates.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
    return (candidates[0][2], candidates[0][3])


def _border_free_cells(
    grid: OccupancyGridSnapshot,
    frontier_cells: Iterable[Tuple[int, int]],
) -> set[Tuple[int, int]]:
    free_cells = set()
    for row, col in frontier_cells:
        for row_offset, col_offset in _NEIGHBORS_8:
            neighbor_row = row + row_offset
            neighbor_col = col + col_offset
            if not grid.in_bounds(neighbor_row, neighbor_col):
                continue
            if grid.value(neighbor_row, neighbor_col) == 0:
                free_cells.add((neighbor_row, neighbor_col))
    return free_cells
