from pathlib import Path

from navigation.commands import parse_nav_command
from navigation.health import HealthMonitor
from navigation.supervisor import NavigationSupervisor
from navigation.types import ActionType
from navigation.types import CommandContext
from navigation.types import NavigationResult
from navigation.types import NavigationStatus
from navigation.types import OccupancyGridSnapshot
from navigation.types import Pose2D
from navigation.types import Waypoint
from navigation.waypoints import WaypointStore


class FakeClock:
    def __init__(self, value: float = 0.0):
        self._value = value

    def __call__(self) -> float:
        return self._value

    def advance(self, seconds: float) -> None:
        self._value += seconds


def _healthy_monitor(clock: FakeClock) -> HealthMonitor:
    monitor = HealthMonitor(clock=clock)
    monitor.record_scan()
    monitor.record_odom()
    monitor.record_tf()
    return monitor


def _waypoint_store(tmp_path) -> WaypointStore:
    return WaypointStore(tmp_path / 'waypoints.yaml')


def _exploration_grid() -> OccupancyGridSnapshot:
    rows = [[100 for _ in range(16)] for _ in range(16)]
    for row in range(2, 14):
        for col in range(1, 13):
            rows[row][col] = 0
    for row in range(2, 14):
        rows[row][0] = -1
        rows[row][13] = -1
    return OccupancyGridSnapshot(
        width=16,
        height=16,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        data=tuple(value for row in rows for value in row),
    )


def _no_frontier_grid() -> OccupancyGridSnapshot:
    rows = [[100 for _ in range(16)] for _ in range(16)]
    for row in range(2, 14):
        for col in range(1, 13):
            rows[row][col] = 0
    return OccupancyGridSnapshot(
        width=16,
        height=16,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        data=tuple(value for row in rows for value in row),
    )


def test_health_monitor_reports_stale_sources():
    clock = FakeClock()
    monitor = HealthMonitor(clock=clock)
    initial = monitor.status()
    assert not initial.healthy
    assert set(initial.stale_sources) == {'scan', 'odom', 'tf'}

    monitor.record_scan()
    monitor.record_odom()
    monitor.record_tf()
    assert monitor.status().healthy

    clock.advance(1.1)
    stale = monitor.status()
    assert not stale.healthy
    assert 'tf' in stale.stale_sources


def test_supervisor_saves_waypoint_and_lists_it(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=HealthMonitor(clock=clock),
        active_map_id='demo-map',
        clock=clock,
        timestamp_factory=lambda: '2026-03-28T18:00:00+00:00',
    )

    save_response = supervisor.handle_command(
        parse_nav_command('save_waypoint couch'),
        CommandContext(current_pose=Pose2D(1.0, 2.0, 0.0)),
    )
    assert save_response.accepted
    assert "Saved waypoint 'couch'" in save_response.message

    list_response = supervisor.handle_command(parse_nav_command('list_waypoints'))
    assert 'couch' in list_response.message


def test_supervisor_rejects_unhealthy_navigation(tmp_path):
    clock = FakeClock()
    store = _waypoint_store(tmp_path)
    store.save_waypoint(
        Waypoint(
            name='desk',
            map_id='demo-map',
            pose=Pose2D(1.0, 2.0, 0.0),
            created_at='2026-03-28T18:00:00+00:00',
        )
    )
    supervisor = NavigationSupervisor(
        waypoint_store=store,
        health_monitor=HealthMonitor(clock=clock),
        active_map_id='demo-map',
        clock=clock,
    )

    response = supervisor.handle_command(parse_nav_command('navigate_to desk'))
    assert not response.accepted
    assert 'stale scan, odom, tf' in response.message


def test_supervisor_timeout_emits_stop_action(tmp_path):
    clock = FakeClock()
    store = _waypoint_store(tmp_path)
    store.save_waypoint(
        Waypoint(
            name='desk',
            map_id='demo-map',
            pose=Pose2D(1.0, 2.0, 0.0),
            created_at='2026-03-28T18:00:00+00:00',
        )
    )
    supervisor = NavigationSupervisor(
        waypoint_store=store,
        health_monitor=_healthy_monitor(clock),
        active_map_id='demo-map',
        clock=clock,
        navigation_timeout=10.0,
    )

    navigate = supervisor.handle_command(parse_nav_command('navigate_to desk'))
    assert navigate.accepted

    clock.advance(10.1)
    timeout = supervisor.tick()
    assert timeout is not None
    assert not timeout.accepted
    assert timeout.actions[0].action_type == ActionType.STOP
    assert 'timed out' in timeout.message


def test_supervisor_blocks_new_goal_while_cancel_is_in_progress(tmp_path):
    clock = FakeClock()
    store = _waypoint_store(tmp_path)
    store.save_waypoint(
        Waypoint(
            name='desk',
            map_id='demo-map',
            pose=Pose2D(1.0, 2.0, 0.0),
            created_at='2026-03-28T18:00:00+00:00',
        )
    )
    store.save_waypoint(
        Waypoint(
            name='door',
            map_id='demo-map',
            pose=Pose2D(3.0, 2.0, 0.0),
            created_at='2026-03-28T18:00:00+00:00',
        )
    )
    supervisor = NavigationSupervisor(
        waypoint_store=store,
        health_monitor=_healthy_monitor(clock),
        active_map_id='demo-map',
        clock=clock,
    )

    assert supervisor.handle_command(parse_nav_command('navigate_to desk')).accepted
    cancel = supervisor.handle_command(parse_nav_command('cancel_navigation'))
    assert cancel.accepted
    second_navigate = supervisor.handle_command(parse_nav_command('navigate_to door'))
    assert not second_navigate.accepted
    assert 'finish canceling' in second_navigate.message


def test_supervisor_applies_navigation_results(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='demo-map',
        clock=clock,
    )
    supervisor._active_goal = supervisor._active_goal = type(
        'Goal',
        (),
        {
            'waypoint': Waypoint(
                name='desk',
                map_id='demo-map',
                pose=Pose2D(0.0, 0.0, 0.0),
                created_at='2026-03-28T18:00:00+00:00',
            ),
            'started_at': 0.0,
            'timeout': 10.0,
            'cancel_requested': False,
            'is_exploration': False,
            'frontier_reference_pose': None,
            'frontier_cells': tuple(),
        },
    )()

    response = supervisor.apply_navigation_result(
        NavigationResult(status=NavigationStatus.SUCCEEDED, waypoint_name='desk')
    )
    assert response.accepted
    assert "Arrived at 'desk'" in response.message


def test_navigation_mode_rejects_start_exploration(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='demo-map',
        clock=clock,
    )

    response = supervisor.handle_command(parse_nav_command('start_exploration'))
    assert not response.accepted
    assert 'only available in exploration mode' in response.message


def test_exploration_start_requires_health_map_cmd_vel_and_pose(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=HealthMonitor(clock=clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
    )

    response = supervisor.handle_command(
        parse_nav_command('start_exploration'),
        CommandContext(),
    )
    assert not response.accepted
    assert '/map' in response.message
    assert '/cmd_vel' in response.message
    assert 'stale scan, odom, tf' in response.message


def test_exploration_success_selects_next_frontier_on_updated_map(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
    )
    context = CommandContext(
        current_pose=Pose2D(4.5, 3.5, 0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    start = supervisor.handle_command(parse_nav_command('start_exploration'), context)
    assert start.accepted
    first_goal = supervisor.plan_exploration(context.current_pose, _exploration_grid())
    assert first_goal is not None
    assert first_goal.accepted
    assert 'frontier_1' in first_goal.message

    result = supervisor.apply_navigation_result(
        NavigationResult(status=NavigationStatus.SUCCEEDED, waypoint_name='frontier_1')
    )
    assert result.accepted
    assert 'Selecting the next frontier' in result.message

    second_goal = supervisor.plan_exploration(context.current_pose, _exploration_grid())
    assert second_goal is not None
    assert 'frontier_2' in second_goal.message


def test_exploration_failure_blacklists_frontier_and_continues(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
    )
    context = CommandContext(
        current_pose=Pose2D(4.5, 3.5, 0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    supervisor.handle_command(parse_nav_command('start_exploration'), context)
    first_goal = supervisor.plan_exploration(context.current_pose, _exploration_grid())
    assert first_goal is not None
    result = supervisor.apply_navigation_result(
        NavigationResult(
            status=NavigationStatus.FAILED,
            waypoint_name='frontier_1',
            message='Nav2 rejected the frontier.',
        )
    )
    assert not result.accepted
    assert 'Blacklisted the frontier' in result.message
    second_goal = supervisor.plan_exploration(context.current_pose, _exploration_grid())
    assert second_goal is not None
    assert 'frontier_2' in second_goal.message


def test_exploration_timeout_blacklists_frontier_and_stops_before_replanning(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=HealthMonitor(
            scan_timeout=100.0,
            odom_timeout=100.0,
            tf_timeout=100.0,
            clock=clock,
        ),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
        base_timeout=5.0,
        expected_speed=1.0,
        timeout_safety_factor=1.0,
    )
    supervisor._health_monitor.record_scan()
    supervisor._health_monitor.record_odom()
    supervisor._health_monitor.record_tf()
    context = CommandContext(
        current_pose=Pose2D(4.5, 3.5, 0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    supervisor.handle_command(parse_nav_command('start_exploration'), context)
    first_goal = supervisor.plan_exploration(context.current_pose, _exploration_grid())
    assert first_goal is not None

    clock.advance(6.0)
    timeout = supervisor.tick()
    assert timeout is not None
    assert not timeout.accepted
    assert timeout.actions[0].action_type == ActionType.STOP
    assert 'Blacklisted the frontier' in timeout.message

    second_goal = supervisor.plan_exploration(context.current_pose, _exploration_grid())
    assert second_goal is not None
    assert 'frontier_2' in second_goal.message


def test_exploration_aborts_when_health_goes_stale(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
    )
    context = CommandContext(
        current_pose=Pose2D(4.5, 3.5, 0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    supervisor.handle_command(parse_nav_command('start_exploration'), context)
    supervisor.plan_exploration(context.current_pose, _exploration_grid())

    clock.advance(2.1)
    response = supervisor.tick()
    assert response is not None
    assert response.actions[0].action_type == ActionType.STOP
    assert 'Exploration aborted' in response.message
    assert not supervisor.exploration_active


def test_cancel_navigation_ends_exploration_mode(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
    )
    context = CommandContext(
        current_pose=Pose2D(4.5, 3.5, 0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    supervisor.handle_command(parse_nav_command('start_exploration'), context)
    supervisor.plan_exploration(context.current_pose, _exploration_grid())
    response = supervisor.handle_command(parse_nav_command('cancel_navigation'))
    assert response.accepted
    assert response.actions[0].action_type == ActionType.CANCEL
    assert 'ending exploration mode' in response.message
    assert not supervisor.exploration_active


def test_stop_ends_exploration_mode_without_map_save(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
    )
    context = CommandContext(
        current_pose=Pose2D(4.5, 3.5, 0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    supervisor.handle_command(parse_nav_command('start_exploration'), context)
    supervisor.plan_exploration(context.current_pose, _exploration_grid())
    response = supervisor.handle_command(parse_nav_command('stop'))
    assert response.accepted
    assert response.actions[0].action_type == ActionType.STOP
    assert 'Stopping exploration goal' in response.message
    assert supervisor.pending_map_save is None
    assert not supervisor.exploration_active


def test_exploration_completion_saves_map_and_updates_active_map_id(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
        map_output_dir=Path('/tmp/exploration-maps'),
        map_save_stem_factory=lambda: 'explore_20260328_180000',
        empty_cycle_threshold=1,
        coverage_threshold=0.0,
    )
    context = CommandContext(
        current_pose=Pose2D(4.5, 3.5, 0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    supervisor.handle_command(parse_nav_command('start_exploration'), context)
    result = supervisor.plan_exploration(context.current_pose, _no_frontier_grid())
    assert result is not None
    assert result.accepted
    assert 'Exploration complete' in result.message
    pending = supervisor.begin_pending_map_save()
    assert pending is not None
    assert pending.path == '/tmp/exploration-maps/explore_20260328_180000'

    save_message = supervisor.apply_map_save_result(True)
    assert 'active map id set' in save_message
    assert supervisor.active_map_id == 'explore_20260328_180000'
    assert not supervisor.exploration_active
    assert not supervisor.has_active_goal


def test_adaptive_timeout_scales_with_distance(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
        base_timeout=5.0,
        expected_speed=1.0,
        timeout_safety_factor=2.0,
    )
    assert supervisor._compute_adaptive_timeout(1.0) == 5.0
    assert supervisor._compute_adaptive_timeout(6.0) == 12.0


def test_coverage_threshold_triggers_immediate_termination(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
        map_output_dir=Path('/tmp/exploration-maps'),
        map_save_stem_factory=lambda: 'explore_20260328_180000',
        coverage_threshold=0.8,
        empty_cycle_threshold=3,
    )
    context = CommandContext(
        current_pose=Pose2D(4.5, 3.5, 0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    supervisor.handle_command(parse_nav_command('start_exploration'), context)
    result = supervisor.plan_exploration(context.current_pose, _no_frontier_grid())
    assert result is not None
    assert result.accepted
    assert 'coverage' in result.message
    assert not supervisor.exploration_active


def test_empty_cycle_threshold_is_configurable(tmp_path):
    clock = FakeClock()
    supervisor = NavigationSupervisor(
        waypoint_store=_waypoint_store(tmp_path),
        health_monitor=_healthy_monitor(clock),
        active_map_id='live_map',
        clock=clock,
        mode='exploration',
        map_output_dir=Path('/tmp/exploration-maps'),
        map_save_stem_factory=lambda: 'explore_20260328_180000',
        empty_cycle_threshold=2,
    )
    context = CommandContext(
        current_pose=Pose2D(4.5, 3.5, 0.0),
        map_available=True,
        cmd_vel_ready=True,
    )

    supervisor.handle_command(parse_nav_command('start_exploration'), context)
    first = supervisor.plan_exploration(context.current_pose, _no_frontier_grid())
    assert first is not None
    assert supervisor.exploration_active
    second = supervisor.plan_exploration(context.current_pose, _no_frontier_grid())
    assert second is not None
    assert 'Exploration complete' in second.message
    assert not supervisor.exploration_active
