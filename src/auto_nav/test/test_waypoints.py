import pytest

from navigation.types import Pose2D
from navigation.types import Waypoint
from navigation.waypoints import DuplicateWaypointError
from navigation.waypoints import MapMismatchError
from navigation.waypoints import WaypointStore


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
