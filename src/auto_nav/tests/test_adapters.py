from navigation.adapters import CommandBridgeAdapter, NavigationBridgeAdapter
from navigation.health import HealthMonitor
from navigation.supervisor import NavigationSupervisor
from navigation.types import ActionType, Pose2D, Waypoint
from navigation.waypoints import WaypointStore


class FakeClock:
    def __init__(self, start=0.0):
        self.now = start

    def __call__(self):
        return self.now


class FakeNavigatorClient:
    def __init__(self):
        self.calls = []

    def start_navigation(self, request):
        self.calls.append(('navigate', request.waypoint_name, request.pose))

    def cancel_navigation(self, reason=''):
        self.calls.append(('cancel', reason))

    def stop_navigation(self, reason=''):
        self.calls.append(('stop', reason))


def test_command_bridge_turns_supervisor_actions_into_requests(tmp_path):
    clock = FakeClock()
    store = WaypointStore(tmp_path / 'waypoints.yaml')
    store.save_waypoint(
        Waypoint(
            name='desk',
            map_id='demo-map',
            pose=Pose2D(x=1.5, y=-0.2, yaw=0.3),
            created_at='2026-03-28T18:00:00+00:00',
        )
    )
    monitor = HealthMonitor(clock=clock)
    monitor.record_scan()
    monitor.record_odom()
    monitor.record_tf()
    supervisor = NavigationSupervisor(
        waypoint_store=store,
        health_monitor=monitor,
        active_map_id='demo-map',
        clock=clock,
    )
    adapter = CommandBridgeAdapter(supervisor)

    dispatch = adapter.handle_text_command('navigate_to desk')

    assert dispatch.accepted
    assert dispatch.requests[0].action_type == ActionType.NAVIGATE
    assert dispatch.requests[0].waypoint_name == 'desk'
    assert dispatch.requests[0].pose == Pose2D(x=1.5, y=-0.2, yaw=0.3)


def test_navigation_bridge_dispatches_and_maps_results():
    navigator = FakeNavigatorClient()
    adapter = NavigationBridgeAdapter(navigator)

    status_message = adapter.handle_request(
        request=type(
            'Request',
            (),
            {
                'action_type': ActionType.STOP,
                'waypoint_name': None,
                'pose': None,
                'reason': 'user requested stop',
            },
        )()
    )
    result = adapter.map_result('canceled', waypoint_name='desk')

    assert status_message == 'Forwarded stop request to Nav2.'
    assert navigator.calls == [('stop', 'user requested stop')]
    assert result.status.value == 'canceled'
    assert "desk" in result.message
