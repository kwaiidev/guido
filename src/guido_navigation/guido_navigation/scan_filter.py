"""Pure helpers for LiDAR keepout filtering and scan post-processing."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class KeepoutBox:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self):
        if self.x_min > self.x_max:
            raise ValueError('keepout x_min must be <= x_max')
        if self.y_min > self.y_max:
            raise ValueError('keepout y_min must be <= y_max')

    def contains(self, x_coord: float, y_coord: float) -> bool:
        return self.x_min <= x_coord <= self.x_max and self.y_min <= y_coord <= self.y_max

    @property
    def center_x(self) -> float:
        return 0.5 * (self.x_min + self.x_max)

    @property
    def center_y(self) -> float:
        return 0.5 * (self.y_min + self.y_max)

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min


def mask_keepout_ranges(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    keepout: KeepoutBox,
    enabled: bool,
) -> list[float]:
    filtered_ranges: list[float] = []
    angle = angle_min

    for sample in ranges:
        value = float(sample)
        if enabled and math.isfinite(value):
            x_coord = value * math.cos(angle)
            y_coord = value * math.sin(angle)
            if keepout.contains(x_coord, y_coord):
                value = math.inf
        filtered_ranges.append(value)
        angle += angle_increment

    return filtered_ranges


def scan_ranges_to_points(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    min_range_m: float,
    max_range_m: float,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    angle = angle_min

    for sample in ranges:
        value = float(sample)
        if math.isfinite(value) and min_range_m <= value <= max_range_m:
            points.append((value * math.cos(angle), value * math.sin(angle)))
        angle += angle_increment

    return points


def clear_keepout_cells(
    grid: np.ndarray,
    *,
    keepout: KeepoutBox,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> None:
    height, width = grid.shape
    gx_min = max(0, int(math.floor((keepout.x_min - origin_x) / resolution)) - 1)
    gx_max = min(width - 1, int(math.ceil((keepout.x_max - origin_x) / resolution)))
    gy_min = max(0, int(math.floor((keepout.y_min - origin_y) / resolution)) - 1)
    gy_max = min(height - 1, int(math.ceil((keepout.y_max - origin_y) / resolution)))

    if gx_min > gx_max or gy_min > gy_max:
        return

    for gy in range(gy_min, gy_max + 1):
        y_coord = origin_y + (gy + 0.5) * resolution
        if y_coord < keepout.y_min or y_coord > keepout.y_max:
            continue
        for gx in range(gx_min, gx_max + 1):
            x_coord = origin_x + (gx + 0.5) * resolution
            if keepout.contains(x_coord, y_coord):
                grid[gy, gx] = 0


def cluster_sequential_points(
    points: Sequence[tuple[float, float]],
    *,
    tolerance_m: float,
    min_cluster_points: int,
) -> list[dict[str, float]]:
    if not points:
        return []

    clusters: list[list[tuple[float, float]]] = []
    current_cluster = [points[0]]

    for previous, current in zip(points, points[1:]):
        if math.hypot(current[0] - previous[0], current[1] - previous[1]) <= tolerance_m:
            current_cluster.append(current)
        else:
            clusters.append(current_cluster)
            current_cluster = [current]
    clusters.append(current_cluster)

    results: list[dict[str, float]] = []
    for cluster in clusters:
        if len(cluster) < min_cluster_points:
            continue
        xs = [point[0] for point in cluster]
        ys = [point[1] for point in cluster]
        center_x = float(sum(xs) / len(xs))
        center_y = float(sum(ys) / len(ys))
        results.append(
            {
                'x': center_x,
                'y': center_y,
                'radius': float(
                    max(
                        math.hypot(x_coord - center_x, y_coord - center_y)
                        for x_coord, y_coord in cluster
                    )
                ),
            }
        )
    return results
