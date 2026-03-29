import pytest

from navigation.types import Pose2D
from navigation.types import Waypoint
from navigation.waypoints import DuplicateWaypointError
from navigation.waypoints import MapMismatchError
from navigation.waypoints import WaypointStore
from navigation.waypoints import coerce_waypoints_section


def test_waypoint_store_round_trip(tmp_path):
    store = WaypointStore(tmp_path / 'waypoints.yaml')
    waypoint = Waypoint(
        name='desk',
        map_id='office-map',
        pose=Pose2D(1.0, 2.0, 0.5),
        created_at='2026-03-28T18:00:00+00:00',
    )
    store.save_waypoint(waypoint)

    loaded = store.load_waypoint('desk')
    assert loaded == waypoint
    assert store.list_waypoint_names() == ['desk']


def test_waypoint_store_rejects_duplicates(tmp_path):
    store = WaypointStore(tmp_path / 'waypoints.yaml')
    waypoint = Waypoint(
        name='desk',
        map_id='office-map',
        pose=Pose2D(0.0, 0.0, 0.0),
        created_at='2026-03-28T18:00:00+00:00',
    )
    store.save_waypoint(waypoint)

    with pytest.raises(DuplicateWaypointError) as exc:
        store.save_waypoint(waypoint)
    assert 'already exists' in str(exc.value)


def test_waypoint_store_rejects_map_mismatch(tmp_path):
    store = WaypointStore(tmp_path / 'waypoints.yaml')
    store.save_waypoint(
        Waypoint(
            name='desk',
            map_id='office-map',
            pose=Pose2D(0.0, 0.0, 0.0),
            created_at='2026-03-28T18:00:00+00:00',
        )
    )

    with pytest.raises(MapMismatchError) as exc:
        store.load_waypoint('desk', expected_map_id='lab-map')
    assert "belongs to map 'office-map'" in str(exc.value)


def test_coerce_list_waypoints_keyboard_format(tmp_path):
    path = tmp_path / 'waypoints.yaml'
    path.write_text(
        'waypoints:\n'
        '  - name: wp_1\n'
        '    x: 1.0\n'
        '    y: 2.0\n'
        '    yaw: 0.5\n',
        encoding='utf-8',
    )
    store = WaypointStore(path)
    loaded = store.load_waypoint('wp_1', expected_map_id='live_map')
    assert loaded.name == 'wp_1'
    assert loaded.pose.x == 1.0
    assert loaded.pose.y == 2.0
    assert loaded.pose.yaw == 0.5
    assert loaded.map_id == 'live_map'


def test_coerce_waypoints_section_list():
    raw = [{'name': 'a', 'x': 0.0, 'y': 1.0, 'yaw': 0.0}]
    d = coerce_waypoints_section(raw)
    assert 'a' in d
    assert d['a']['map_id'] == 'live_map'
