from navigation.types import Pose2D, Waypoint
from navigation.waypoints import (
    DuplicateWaypointError,
    MapMismatchError,
    WaypointStore,
)


def test_waypoint_store_round_trip(tmp_path):
    store = WaypointStore(tmp_path / 'waypoints.yaml')
    waypoint = Waypoint(
        name='desk',
        map_id='office-map',
        pose=Pose2D(x=1.2, y=-0.4, yaw=0.8),
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
        pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        created_at='2026-03-28T18:00:00+00:00',
    )

    store.save_waypoint(waypoint)

    try:
        store.save_waypoint(waypoint)
    except DuplicateWaypointError as exc:
        assert "already exists" in str(exc)
    else:
        raise AssertionError('Expected duplicate waypoint save to fail')


def test_waypoint_store_rejects_map_mismatch(tmp_path):
    store = WaypointStore(tmp_path / 'waypoints.yaml')
    store.save_waypoint(
        Waypoint(
            name='desk',
            map_id='office-map',
            pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
            created_at='2026-03-28T18:00:00+00:00',
        )
    )

    try:
        store.load_waypoint('desk', expected_map_id='lab-map')
    except MapMismatchError as exc:
        assert "belongs to map 'office-map'" in str(exc)
    else:
        raise AssertionError('Expected mismatched map id to fail')
