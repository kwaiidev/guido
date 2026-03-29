from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from agents.guido_mission_agent import backend
from navigation.types import Pose2D
from navigation.types import Waypoint
from navigation.waypoints import WaypointStore


@pytest.fixture(autouse=True)
def _isolated_waypoint_backend(monkeypatch, tmp_path):
    monkeypatch.setenv('GUIDO_WAYPOINT_FILE', str(tmp_path / 'waypoints.yaml'))
    backend.MISSION_STATE.update(
        {
            'status': 'idle',
            'active_mission': None,
            'last_message': 'No waypoint command has been queued yet.',
            'pending_ros_command': None,
        }
    )


def test_lookup_destination_uses_saved_waypoints():
    store = WaypointStore(Path(backend.get_robot_status()['waypoint_file']))
    store.save_waypoint(
        Waypoint(
            name='charging dock',
            map_id='lab-map',
            pose=Pose2D(1.0, 2.0, 0.1),
            created_at='2026-03-29T10:00:00+00:00',
        )
    )
    store.save_waypoint(
        Waypoint(
            name='room 2',
            map_id='lab-map',
            pose=Pose2D(3.0, 4.0, 0.2),
            created_at='2026-03-29T10:05:00+00:00',
        )
    )

    assert backend.list_destinations()['destinations'] == ['charging dock', 'room 2']
    assert backend.lookup_destination('go back to room 2')['name'] == 'room 2'
    assert backend.lookup_destination('the charging dock')['name'] == 'charging dock'


def test_remember_current_location_queues_save_command():
    result = backend.remember_current_location('charging dock')

    assert result['accepted'] is True
    assert result['ros_command'] == 'save_waypoint charging dock'
    assert backend.consume_pending_ros_command() == 'save_waypoint charging dock'
    assert backend.consume_pending_ros_command() is None


def test_remember_current_location_rejects_duplicate_saved_name():
    store = WaypointStore(Path(backend.get_robot_status()['waypoint_file']))
    store.save_waypoint(
        Waypoint(
            name='charging dock',
            map_id='lab-map',
            pose=Pose2D(1.0, 2.0, 0.1),
            created_at='2026-03-29T10:00:00+00:00',
        )
    )

    result = backend.remember_current_location('charging dock')

    assert result['accepted'] is False
    assert 'already exists' in result['message']


def test_queue_navigation_to_waypoint_uses_saved_name():
    store = WaypointStore(Path(backend.get_robot_status()['waypoint_file']))
    store.save_waypoint(
        Waypoint(
            name='room 2',
            map_id='lab-map',
            pose=Pose2D(3.0, 4.0, 0.2),
            created_at='2026-03-29T10:05:00+00:00',
        )
    )

    result = backend.queue_navigation_to_waypoint('go back to room 2')

    assert result['accepted'] is True
    assert result['ros_command'] == 'navigate_to room 2'
    assert backend.MISSION_STATE['status'] == 'queued'
