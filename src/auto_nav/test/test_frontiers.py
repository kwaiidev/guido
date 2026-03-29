from navigation.frontiers import extract_frontier_clusters
from navigation.frontiers import rank_frontier_goals
from navigation.frontiers import select_frontier_cluster
from navigation.frontiers import select_frontier_goal
from navigation.types import FrontierParams
from navigation.types import OccupancyGridSnapshot
from navigation.types import Pose2D


def _build_grid(width, height, default=-1):
    return [[default for _ in range(width)] for _ in range(height)]


def _snapshot(rows, resolution=1.0):
    return OccupancyGridSnapshot(
        width=len(rows[0]),
        height=len(rows),
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        data=tuple(value for row in rows for value in row),
    )


def test_extract_frontier_clusters_finds_8_connected_unknown_boundary():
    rows = _build_grid(8, 8, default=100)
    for row in range(2, 6):
        for col in range(2, 6):
            rows[row][col] = 0
    for col in range(2, 6):
        rows[1][col] = -1

    clusters = extract_frontier_clusters(_snapshot(rows), min_cluster_size=2)
    assert len(clusters) == 1
    assert clusters[0].size == 4
    assert round(clusters[0].centroid_x, 1) == 4.0
    assert round(clusters[0].centroid_y, 1) == 1.5


def test_extract_frontier_clusters_rejects_small_clusters():
    rows = _build_grid(6, 6, default=100)
    for row in range(2, 4):
        for col in range(2, 4):
            rows[row][col] = 0
    rows[1][2] = -1

    clusters = extract_frontier_clusters(_snapshot(rows), min_cluster_size=2)
    assert clusters == ()


def test_select_frontier_cluster_picks_nearest_non_blacklisted_centroid():
    rows = _build_grid(10, 8, default=100)
    for row in range(2, 6):
        for col in range(1, 8):
            rows[row][col] = 0
    for row in range(2, 6):
        rows[row][0] = -1
        rows[row][8] = -1

    selected = select_frontier_cluster(
        _snapshot(rows),
        Pose2D(3.5, 3.5, 0.0),
        blacklist_points=(Pose2D(0.5, 4.0, 0.0),),
        min_cluster_size=2,
    )
    assert selected is not None
    assert round(selected.centroid_x, 1) == 8.5


def test_select_frontier_goal_uses_reachable_free_space_pose():
    rows = _build_grid(10, 8, default=100)
    for row in range(2, 6):
        for col in range(1, 8):
            rows[row][col] = 0
    for row in range(2, 6):
        rows[row][8] = -1

    selected = select_frontier_goal(
        _snapshot(rows),
        Pose2D(3.5, 3.5, 0.0),
        min_cluster_size=2,
    )
    assert selected is not None
    assert selected.target_row in range(2, 6)
    assert selected.target_col == 7
    assert round(selected.target_pose.x, 1) == 7.5
    assert round(selected.reference_pose.x, 1) == 8.5


def test_select_frontier_goal_skips_disconnected_clusters():
    rows = _build_grid(10, 8, default=100)
    for row in range(2, 6):
        for col in range(1, 4):
            rows[row][col] = 0
        for col in range(6, 8):
            rows[row][col] = 0
    for row in range(2, 6):
        rows[row][4] = 100
        rows[row][5] = 100
        rows[row][8] = -1

    selected = select_frontier_goal(
        _snapshot(rows),
        Pose2D(2.5, 3.5, 0.0),
        min_cluster_size=2,
    )
    assert selected is None


def test_information_gain_scores_higher_for_larger_unknown_region():
    rows = _build_grid(14, 8, default=100)
    for row in range(2, 6):
        for col in range(1, 12):
            rows[row][col] = 0
    for row in range(2, 6):
        rows[row][0] = -1
        rows[row][12] = -1
    for row in range(1, 7):
        rows[row][13] = -1

    goals = rank_frontier_goals(
        _snapshot(rows),
        Pose2D(5.5, 3.5, 0.0),
        min_cluster_size=2,
        params=FrontierParams(info_gain_weight=0.3, info_gain_radius=4),
    )
    assert len(goals) == 2
    left_goal = next(goal for goal in goals if goal.cluster.centroid_x < 2.0)
    right_goal = next(goal for goal in goals if goal.cluster.centroid_x > 10.0)
    assert right_goal.information_gain > left_goal.information_gain


def test_rank_with_info_gain_weight_reorders_frontiers():
    rows = _build_grid(14, 8, default=100)
    for row in range(2, 6):
        for col in range(1, 12):
            rows[row][col] = 0
    for row in range(2, 6):
        rows[row][0] = -1
        rows[row][12] = -1
    for row in range(1, 7):
        rows[row][13] = -1

    robot_pose = Pose2D(3.5, 3.5, 0.0)
    goals_dist = rank_frontier_goals(
        _snapshot(rows),
        robot_pose,
        min_cluster_size=2,
        params=FrontierParams(info_gain_weight=0.0, info_gain_radius=4),
    )
    goals_ig = rank_frontier_goals(
        _snapshot(rows),
        robot_pose,
        min_cluster_size=2,
        params=FrontierParams(info_gain_weight=2.0, info_gain_radius=4),
    )
    assert goals_dist[0].cluster.centroid_x < 2.0
    assert goals_ig[0].cluster.centroid_x > 10.0


def test_frontier_params_defaults_match_existing_behavior():
    rows = _build_grid(12, 16, default=100)
    for row in range(2, 14):
        for col in range(1, 9):
            rows[row][col] = 0
    for row in range(2, 14):
        rows[row][9] = -1

    goal_no_params = select_frontier_goal(_snapshot(rows), Pose2D(4.5, 7.5, 0.0))
    goal_default_params = select_frontier_goal(
        _snapshot(rows),
        Pose2D(4.5, 7.5, 0.0),
        params=FrontierParams(),
    )
    assert goal_no_params is not None
    assert goal_default_params is not None
    assert (goal_no_params.target_row, goal_no_params.target_col) == (
        goal_default_params.target_row,
        goal_default_params.target_col,
    )
