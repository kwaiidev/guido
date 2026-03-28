from pathlib import Path

from navigation.commands import parse_nav_command
from navigation.health import HealthMonitor
from navigation.supervisor import NavigationSupervisor
from navigation.types import (
    ActionType,
    CommandContext,
    NavigationResult,
    NavigationStatus,
    OccupancyGridSnapshot,
    Pose2D,
    Waypoint,
)
from navigation.waypoints import WaypointStore


class FakeClock:
    def __init__(self, start=0.0):
        self.now = start

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def _healthy_monitor(clock):
    monitor = HealthMonitor(clock=clock)
    monitor.record_scan()
    monitor.record_odom()
    monitor.record_tf()
    return monitor


def _exploration_grid(first_frontier=True, second_frontier=True):
    width = 20
    height = 12
    rows = [[100 for _ in range(width)] for _ in range(height)]
    for row in range(height):
        if first_frontier:
            for col in range(5):
                rows[row][col] = 0
            rows[row][5] = -1
        if second_frontier:
            for col in range(10, 15):
                rows[row][col] = 0
            rows[row][15] = -1
    if second_frontier:
        for col in range(4, 15):
            rows[height - 1][col] = 0

    return OccupancyGridSnapshot(
        width=width,
        height=height,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        data=tuple(value for row in rows for value in row),
    )


def _no_frontier_grid():
    rows = [[0 for _ in range(8)] for _ in range(8)]
    return OccupancyGridSnapshot(
        width=8,
        height=8,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        data=tuple(value for row in rows for value in row),
    )


def _exploration_supervisor(tmp_path, clock):
    return NavigationSupervisor(
        waypoint_store=WaypointStore(tmp_path / "waypoints.yaml"),
        health_monitor=_healthy_monitor(clock),
        active_map_id="live_map",
        navigation_timeout=10.0,
        clock=clock,
        timestamp_factory=lambda: "2026-03-28T18:00:00+00:00",
        mode="exploration",
        map_output_dir=Path("/tmp/exploration-maps"),
        map_save_stem_factory=lambda: "explore_20260328_180000",
    )


def test_health_monitor_reports_stale_sources():
    clock = FakeClock()
    monitor = HealthMonitor(clock=clock)

    initial = monitor.status()
    assert not initial.healthy
    assert set(initial.stale_sources) == {"scan", "odom", "tf"}

    monitor.record_scan()
    monitor.record_odom()
    monitor.record_tf()
    assert monitor.status().healthy

    clock.advance(1.1)
    stale = monitor.status()
    assert not stale.healthy
    assert "tf" in stale.stale_sources


def test_supervisor_saves_waypoint_and_lists_it(tmp_path):
    clock = FakeClock()
    store = WaypointStore(tmp_path / "waypoints.yaml")
    monitor = HealthMonitor(clock=clock)
    supervisor = NavigationSupervisor(
        waypoint_store=store,
        health_monitor=monitor,
        active_map_id="demo-map",
        clock=clock,
        timestamp_factory=lambda: "2026-03-28T18:00:00+00:00",
    )

    save_response = supervisor.handle_command(
        parse_nav_command("save_waypoint couch"),
        context=CommandContext(current_pose=Pose2D(x=1.0, y=2.0, yaw=0.5)),
    )
    list_response = supervisor.handle_command(parse_nav_command("list_waypoints"))

    assert save_response.accepted
    assert "Saved waypoint 'couch'" in save_response.message
    assert list_response.accepted
    assert "couch" in list_response.message


def test_supervisor_rejects_unhealthy_navigation(tmp_path):
    clock = FakeClock()
    store = WaypointStore(tmp_path / "waypoints.yaml")
    store.save_waypoint(
        Waypoint(
            name="desk",
            map_id="demo-map",
            pose=Pose2D(x=1.0, y=2.0, yaw=0.0),
            created_at="2026-03-28T18:00:00+00:00",
        )
    )
    monitor = HealthMonitor(clock=clock)
    supervisor = NavigationSupervisor(
        waypoint_store=store,
        health_monitor=monitor,
        active_map_id="demo-map",
        clock=clock,
    )

    response = supervisor.handle_command(parse_nav_command("navigate_to desk"))

    assert not response.accepted
    assert "stale scan, odom, tf" in response.message


def test_supervisor_timeout_emits_stop_action(tmp_path):
    clock = FakeClock()
    store = WaypointStore(tmp_path / "waypoints.yaml")
    store.save_waypoint(
        Waypoint(
            name="desk",
            map_id="demo-map",
            pose=Pose2D(x=1.0, y=2.0, yaw=0.0),
            created_at="2026-03-28T18:00:00+00:00",
        )
    )
    supervisor = NavigationSupervisor(
        waypoint_store=store,
        health_monitor=_healthy_monitor(clock),
        active_map_id="demo-map",
        navigation_timeout=10.0,
        clock=clock,
    )

    navigate = supervisor.handle_command(parse_nav_command("navigate_to desk"))
    clock.advance(10.1)
    timeout = supervisor.tick()

    assert navigate.accepted
    assert timeout is not None
    assert not timeout.accepted
    assert timeout.actions[0].action_type == ActionType.STOP
    assert "timed out" in timeout.message


def test_supervisor_blocks_new_goal_while_cancel_is_in_progress(tmp_path):
    clock = FakeClock()
    store = WaypointStore(tmp_path / "waypoints.yaml")
    for name in ("desk", "door"):
        store.save_waypoint(
            Waypoint(
                name=name,
                map_id="demo-map",
                pose=Pose2D(x=1.0, y=2.0, yaw=0.0),
                created_at="2026-03-28T18:00:00+00:00",
            )
        )
    supervisor = NavigationSupervisor(
        waypoint_store=store,
        health_monitor=_healthy_monitor(clock),
        active_map_id="demo-map",
        clock=clock,
    )

    navigate = supervisor.handle_command(parse_nav_command("navigate_to desk"))
    cancel = supervisor.handle_command(parse_nav_command("cancel_navigation"))
    second_navigate = supervisor.handle_command(parse_nav_command("navigate_to door"))

    assert navigate.accepted
    assert cancel.accepted
    assert not second_navigate.accepted
    assert "finish canceling" in second_navigate.message


def test_supervisor_applies_navigation_results(tmp_path):
    clock = FakeClock()
    store = WaypointStore(tmp_path / "waypoints.yaml")
    monitor = HealthMonitor(clock=clock)
    supervisor = NavigationSupervisor(
        waypoint_store=store,
        health_monitor=monitor,
        active_map_id="demo-map",
        clock=clock,
    )

    response = supervisor.apply_navigation_result(
        NavigationResult(
            status=NavigationStatus.SUCCEEDED,
            waypoint_name="desk",
        )
    )

    assert response.accepted
    assert "Arrived at 'desk'" in response.message


def test_navigation_mode_rejects_start_exploration(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=WaypointStore(tmp_path / "waypoints.yaml"),
        health_monitor=_healthy_monitor(clock),
        active_map_id="demo-map",
        clock=clock,
    )

    response = supervisor.handle_command(parse_nav_command("start_exploration"))

    assert not response.accepted
    assert "only available in exploration mode" in response.message


def test_exploration_start_requires_health_map_cmd_vel_and_pose(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=WaypointStore(tmp_path / "waypoints.yaml"),
        health_monitor=HealthMonitor(clock=clock),
        active_map_id="live_map",
        clock=clock,
        mode="exploration",
    )

    response = supervisor.handle_command(
        parse_nav_command("start_exploration"),
        context=CommandContext(),
    )

    assert not response.accepted
    assert "/map" in response.message
    assert "/cmd_vel" in response.message
    assert "stale scan, odom, tf" in response.message


def test_exploration_rejects_waypoint_commands_in_exploration_mode(tmp_path):
    clock = FakeClock()
    supervisor = _exploration_supervisor(tmp_path, clock)

    response = supervisor.handle_command(parse_nav_command("list_waypoints"))

    assert not response.accepted
    assert "unavailable in exploration mode" in response.message


def test_exploration_success_selects_next_frontier_on_updated_map(tmp_path):
    clock = FakeClock()
    supervisor = _exploration_supervisor(tmp_path, clock)
    context = CommandContext(
        current_pose=Pose2D(x=1.0, y=6.0, yaw=0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    start = supervisor.handle_command(
        parse_nav_command("start_exploration"),
        context=context,
    )
    first_goal = supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_exploration_grid(first_frontier=True, second_frontier=True),
    )
    result = supervisor.apply_navigation_result(
        NavigationResult(
            status=NavigationStatus.SUCCEEDED,
            waypoint_name="frontier_1",
        )
    )
    second_goal = supervisor.plan_exploration(
        current_pose=Pose2D(x=8.0, y=11.0, yaw=0.0),
        occupancy_grid=_exploration_grid(first_frontier=False, second_frontier=True),
    )

    assert start.accepted
    assert first_goal is not None and first_goal.accepted
    assert result.accepted
    assert "Selecting the next frontier" in result.message
    assert second_goal is not None
    assert second_goal.accepted
    assert second_goal.actions[0].waypoint.name == "frontier_2"


def test_exploration_failure_blacklists_frontier_and_continues(tmp_path):
    clock = FakeClock()
    supervisor = _exploration_supervisor(tmp_path, clock)
    context = CommandContext(
        current_pose=Pose2D(x=1.0, y=6.0, yaw=0.0),
        map_available=True,
        cmd_vel_ready=True,
    )
    supervisor.handle_command(parse_nav_command("start_exploration"), context=context)
    first_goal = supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_exploration_grid(),
    )

    result = supervisor.apply_navigation_result(
        NavigationResult(
            status=NavigationStatus.FAILED,
            waypoint_name="frontier_1",
            message="Nav2 rejected the frontier.",
        )
    )
    second_goal = supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_exploration_grid(),
    )

    assert first_goal is not None and first_goal.accepted
    assert not result.accepted
    assert "Blacklisted the frontier" in result.message
    assert second_goal is not None
    assert second_goal.actions[0].waypoint.name == "frontier_2"


def test_exploration_timeout_blacklists_frontier_and_stops_before_replanning(tmp_path):
    clock = FakeClock()
    monitor = HealthMonitor(
        scan_timeout=20.0,
        odom_timeout=20.0,
        tf_timeout=20.0,
        clock=clock,
    )
    monitor.record_scan()
    monitor.record_odom()
    monitor.record_tf()
    supervisor = NavigationSupervisor(
        waypoint_store=WaypointStore(tmp_path / "waypoints.yaml"),
        health_monitor=monitor,
        active_map_id="live_map",
        navigation_timeout=10.0,
        clock=clock,
        timestamp_factory=lambda: "2026-03-28T18:00:00+00:00",
        mode="exploration",
        map_output_dir=Path("/tmp/exploration-maps"),
        map_save_stem_factory=lambda: "explore_20260328_180000",
    )
    context = CommandContext(
        current_pose=Pose2D(x=1.0, y=6.0, yaw=0.0),
        map_available=True,
        cmd_vel_ready=True,
    )
    supervisor.handle_command(parse_nav_command("start_exploration"), context=context)
    first_goal = supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_exploration_grid(),
    )

    clock.advance(10.1)
    timeout = supervisor.tick()
    second_goal = supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_exploration_grid(),
    )

    assert first_goal is not None and first_goal.accepted
    assert timeout is not None
    assert timeout.actions[0].action_type == ActionType.STOP
    assert "Blacklisted the frontier" in timeout.message
    assert second_goal is not None
    assert second_goal.actions[0].waypoint.name == "frontier_2"


def test_exploration_aborts_when_health_goes_stale(tmp_path):
    clock = FakeClock()
    supervisor = _exploration_supervisor(tmp_path, clock)
    context = CommandContext(
        current_pose=Pose2D(x=1.0, y=6.0, yaw=0.0),
        map_available=True,
        cmd_vel_ready=True,
    )
    supervisor.handle_command(parse_nav_command("start_exploration"), context=context)
    supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_exploration_grid(),
    )

    clock.advance(1.1)
    response = supervisor.tick()

    assert response is not None
    assert not response.accepted
    assert response.actions[0].action_type == ActionType.STOP
    assert "Exploration aborted" in response.message
    assert not supervisor.exploration_active


def test_cancel_navigation_ends_exploration_mode(tmp_path):
    clock = FakeClock()
    supervisor = _exploration_supervisor(tmp_path, clock)
    context = CommandContext(
        current_pose=Pose2D(x=1.0, y=6.0, yaw=0.0),
        map_available=True,
        cmd_vel_ready=True,
    )
    supervisor.handle_command(parse_nav_command("start_exploration"), context=context)
    supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_exploration_grid(),
    )

    response = supervisor.handle_command(parse_nav_command("cancel_navigation"))

    assert response.accepted
    assert response.actions[0].action_type == ActionType.CANCEL
    assert "ending exploration mode" in response.message
    assert not supervisor.exploration_active


def test_stop_ends_exploration_mode_without_map_save(tmp_path):
    clock = FakeClock()
    supervisor = _exploration_supervisor(tmp_path, clock)
    context = CommandContext(
        current_pose=Pose2D(x=1.0, y=6.0, yaw=0.0),
        map_available=True,
        cmd_vel_ready=True,
    )
    supervisor.handle_command(parse_nav_command("start_exploration"), context=context)
    supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_exploration_grid(),
    )

    response = supervisor.handle_command(parse_nav_command("stop"))

    assert response.accepted
    assert response.actions[0].action_type == ActionType.STOP
    assert "Stopping exploration goal" in response.message
    assert supervisor.pending_map_save is None
    assert not supervisor.exploration_active


def test_exploration_completion_saves_map_and_updates_active_map_id(tmp_path):
    clock = FakeClock()
    supervisor = _exploration_supervisor(tmp_path, clock)
    context = CommandContext(
        current_pose=Pose2D(x=1.0, y=6.0, yaw=0.0),
        map_available=True,
        cmd_vel_ready=True,
    )
    supervisor.handle_command(parse_nav_command("start_exploration"), context=context)

    first = supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_no_frontier_grid(),
    )
    second = supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_no_frontier_grid(),
    )
    third = supervisor.plan_exploration(
        current_pose=context.current_pose,
        occupancy_grid=_no_frontier_grid(),
    )

    pending = supervisor.begin_pending_map_save()
    save_message = supervisor.apply_map_save_result(True)

    assert first is not None and first.accepted
    assert second is not None and second.accepted
    assert third is not None and third.accepted
    assert third.actions[0].action_type == ActionType.STOP
    assert pending is not None
    assert pending.path == "/tmp/exploration-maps/explore_20260328_180000"
    assert supervisor.active_map_id == "explore_20260328_180000"
    assert "active map id set" in save_message
    assert not supervisor.exploration_active
    assert not supervisor.has_active_goal
