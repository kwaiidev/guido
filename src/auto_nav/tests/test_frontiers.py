from navigation.frontiers import (
    extract_frontier_clusters,
    select_frontier_cluster,
    select_frontier_goal,
)
from navigation.types import OccupancyGridSnapshot, Pose2D


def _build_grid(width, height, default=100):
    return [[default for _ in range(width)] for _ in range(height)]


def _snapshot(rows, resolution=1.0):
    height = len(rows)
    width = len(rows[0])
    data = tuple(value for row in rows for value in row)
    return OccupancyGridSnapshot(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        data=data,
    )


def test_extract_frontier_clusters_finds_8_connected_unknown_boundary():
    rows = _build_grid(width=8, height=12, default=100)
    for row in range(12):
        for col in range(5):
            rows[row][col] = 0
        rows[row][5] = -1

    clusters = extract_frontier_clusters(_snapshot(rows))

    assert len(clusters) == 1
    assert clusters[0].size == 12
    assert round(clusters[0].centroid_x, 2) == 5.5
    assert round(clusters[0].centroid_y, 2) == 6.0


def test_extract_frontier_clusters_rejects_small_clusters():
    rows = _build_grid(width=6, height=8, default=100)
    for row in range(8):
        for col in range(3):
            rows[row][col] = 0
        rows[row][3] = -1

    clusters = extract_frontier_clusters(_snapshot(rows))

    assert clusters == ()


def test_select_frontier_cluster_picks_nearest_non_blacklisted_centroid():
    rows = _build_grid(width=25, height=12, default=100)
    for row in range(12):
        for col in range(5):
            rows[row][col] = 0
        rows[row][5] = -1
        for col in range(15, 20):
            rows[row][col] = 0
        rows[row][20] = -1

    for col in range(4, 20):
        rows[11][col] = 0

    grid = _snapshot(rows)
    robot_pose = Pose2D(x=1.0, y=6.0, yaw=0.0)

    selected = select_frontier_cluster(grid, robot_pose=robot_pose)

    assert selected is not None
    assert round(selected.centroid_x, 2) == 5.5

    blacklisted = select_frontier_cluster(
        grid,
        robot_pose=robot_pose,
        blacklist_points=(Pose2D(x=5.5, y=6.0, yaw=0.0),),
    )

    assert blacklisted is not None
    assert round(blacklisted.centroid_x, 2) == 20.5


def test_select_frontier_goal_uses_reachable_free_space_pose():
    rows = _build_grid(width=8, height=12, default=100)
    for row in range(12):
        for col in range(5):
            rows[row][col] = 0
        rows[row][5] = -1

    grid = _snapshot(rows)

    selected = select_frontier_goal(
        grid,
        robot_pose=Pose2D(x=1.0, y=6.0, yaw=0.0),
    )

    assert selected is not None
    assert grid.value(selected.target_row, selected.target_col) == 0
    assert round(selected.target_pose.x, 2) == 4.5
    assert round(selected.reference_pose.x, 2) == 5.5


def test_select_frontier_goal_skips_disconnected_clusters():
    rows = _build_grid(width=20, height=12, default=100)
    for row in range(12):
        for col in range(5):
            rows[row][col] = 0
        rows[row][5] = -1
        for col in range(10, 15):
            rows[row][col] = 0
        rows[row][15] = -1

    grid = _snapshot(rows)

    selected = select_frontier_goal(
        grid,
        robot_pose=Pose2D(x=1.0, y=6.0, yaw=0.0),
        blacklist_points=(Pose2D(x=5.5, y=6.0, yaw=0.0),),
    )

    assert selected is None
