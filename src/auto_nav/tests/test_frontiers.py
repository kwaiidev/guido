from navigation.frontiers import (
    extract_frontier_clusters,
    rank_frontier_goals,
    select_frontier_cluster,
    select_frontier_goal,
)
from navigation.types import FrontierParams, OccupancyGridSnapshot, Pose2D


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


def test_information_gain_scores_higher_for_larger_unknown_region():
    """Cluster adjacent to more unknown cells should have higher info gain."""
    rows = _build_grid(width=30, height=12, default=-1)  # mostly unknown

    # Cluster A at col=5: small free area + frontier.
    for row in range(12):
        for col in range(5):
            rows[row][col] = 0
        rows[row][5] = -1

    # Cluster B at col=20: large free area + frontier, less unknown beyond.
    for row in range(12):
        for col in range(15, 20):
            rows[row][col] = 0
        rows[row][20] = -1
        # Fill more known space beyond cluster B.
        for col in range(21, 30):
            rows[row][col] = 100  # occupied → not unknown

    # Connect clusters via bottom row so both are reachable (up to col 20).
    for col in range(4, 21):
        rows[11][col] = 0

    grid = _snapshot(rows)
    robot_pose = Pose2D(x=1.0, y=6.0, yaw=0.0)

    goals = rank_frontier_goals(
        grid,
        robot_pose=robot_pose,
        params=FrontierParams(info_gain_weight=0.5),
    )
    assert len(goals) >= 2

    # Cluster A is at col=5, cluster B at col=20.
    a_ig = next(g.information_gain for g in goals if g.cluster.cells[0][1] == 5)
    b_ig = next(g.information_gain for g in goals if g.cluster.cells[0][1] == 20)

    # Cluster A has more unknown cells behind it (-1) vs cluster B (occupied 100).
    assert a_ig > b_ig


def test_rank_with_info_gain_weight_reorders_frontiers():
    """With high info gain weight, farther cluster with more unknown area ranks first."""
    rows = _build_grid(width=30, height=12, default=-1)

    # Near cluster at col=5: small free area.
    for row in range(12):
        for col in range(5):
            rows[row][col] = 0
        rows[row][5] = -1

    # Far cluster at col=20: surrounded by lots of unknown.
    for row in range(12):
        for col in range(15, 20):
            rows[row][col] = 0
        rows[row][20] = -1

    # Connect via bottom row up to col 20.
    for col in range(4, 21):
        rows[11][col] = 0

    # Make Cluster A border mostly occupied (low info gain). leave bottom row 11 free.
    for row in range(11):
        for col in range(6, 10):
            rows[row][col] = 100

    grid = _snapshot(rows)
    robot_pose = Pose2D(x=1.0, y=6.0, yaw=0.0)

    # Pure distance: cluster A should be first (closer).
    goals_dist = rank_frontier_goals(
        grid,
        robot_pose=robot_pose,
        params=FrontierParams(info_gain_weight=0.0),
    )
    assert len(goals_dist) >= 2
    assert goals_dist[0].cluster.cells[0][1] == 5  # Cluster A is closer

    # High info gain weight: cluster B should be first (more unknown).
    goals_ig = rank_frontier_goals(
        grid,
        robot_pose=robot_pose,
        params=FrontierParams(info_gain_weight=0.8),
    )
    assert len(goals_ig) >= 2
    assert goals_ig[0].cluster.cells[0][1] == 20  # Cluster B has more info gain


def test_frontier_params_defaults_match_existing_behavior():
    """Default FrontierParams should produce same results as calling without params."""
    rows = _build_grid(width=8, height=12, default=100)
    for row in range(12):
        for col in range(5):
            rows[row][col] = 0
        rows[row][5] = -1

    grid = _snapshot(rows)
    robot = Pose2D(x=1.0, y=6.0, yaw=0.0)

    goal_no_params = select_frontier_goal(grid, robot_pose=robot)
    goal_default_params = select_frontier_goal(
        grid, robot_pose=robot, params=FrontierParams(info_gain_weight=0.0)
    )

    assert goal_no_params is not None
    assert goal_default_params is not None
    assert goal_no_params.target_row == goal_default_params.target_row
    assert goal_no_params.target_col == goal_default_params.target_col

