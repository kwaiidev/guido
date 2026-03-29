import math

import numpy as np

from guido_navigation.scan_filter import (
    KeepoutBox,
    clear_keepout_cells,
    cluster_sequential_points,
    mask_keepout_ranges,
    scan_ranges_to_points,
)


KEEP_OUT = KeepoutBox(
    x_min=-0.50,
    x_max=0.50,
    y_min=-0.45,
    y_max=0.45,
)


def test_masks_beam_inside_keepout():
    filtered_ranges = mask_keepout_ranges(
        [0.25, 1.20],
        angle_min=0.0,
        angle_increment=0.0,
        keepout=KEEP_OUT,
        enabled=True,
    )

    assert math.isinf(filtered_ranges[0])
    assert abs(filtered_ranges[1] - 1.20) < 1e-9


def test_preserves_beam_just_outside_keepout():
    filtered_ranges = mask_keepout_ranges(
        [0.51],
        angle_min=0.0,
        angle_increment=0.0,
        keepout=KEEP_OUT,
        enabled=True,
    )

    assert abs(filtered_ranges[0] - 0.51) < 1e-9


def test_clears_keepout_cells_in_costmap():
    grid = np.full((20, 20), fill_value=100, dtype=np.int8)
    clear_keepout_cells(
        grid,
        keepout=KEEP_OUT,
        origin_x=-1.0,
        origin_y=-1.0,
        resolution=0.1,
    )

    assert grid[10, 10] == 0
    assert grid[15, 15] == 100


def test_cluster_centroids_stay_outside_keepout():
    filtered_ranges = mask_keepout_ranges(
        [0.20, 0.22, 0.24, 0.80, 0.82, 0.84],
        angle_min=0.0,
        angle_increment=0.01,
        keepout=KEEP_OUT,
        enabled=True,
    )
    points = scan_ranges_to_points(
        filtered_ranges,
        angle_min=0.0,
        angle_increment=0.01,
        min_range_m=0.10,
        max_range_m=15.0,
    )
    clusters = cluster_sequential_points(
        points,
        tolerance_m=0.35,
        min_cluster_points=3,
    )

    assert clusters
    assert all(not KEEP_OUT.contains(cluster['x'], cluster['y']) for cluster in clusters)
