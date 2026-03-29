from pathlib import Path

from navigation.commands import parse_nav_command
from navigation.health import HealthMonitor
from navigation.supervisor import NavigationSupervisor
from navigation.types import ActionType
from navigation.types import CommandContext
from navigation.types import NavigationResult
from navigation.types import NavigationStatus
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
    supervisor._active_goal = type(
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
        },
    )()

    response = supervisor.apply_navigation_result(
        NavigationResult(status=NavigationStatus.SUCCEEDED, waypoint_name='desk')
    )
    assert response.accepted
    assert "Arrived at 'desk'" in response.message
