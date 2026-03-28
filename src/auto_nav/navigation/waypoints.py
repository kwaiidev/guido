"""Waypoint persistence for autonomous navigation targets."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional

from .types import Waypoint

try:
    import yaml
except ImportError:  # pragma: no cover - exercised through fallback path in tests.
    yaml = None


class DuplicateWaypointError(ValueError):
    """Raised when a waypoint name already exists."""


class WaypointNotFoundError(KeyError):
    """Raised when a waypoint cannot be found."""


class MapMismatchError(ValueError):
    """Raised when a waypoint belongs to a different saved map."""


class WaypointStore:
    """Persist and query named waypoints from a YAML-backed store."""

    def __init__(self, path: Path):
        self._path = Path(path)

    @property
    def path(self) -> Path:
        return self._path

    def list_waypoints(self) -> List[Waypoint]:
        records = self._load_records()
        return [
            Waypoint.from_record(name, record)
            for name, record in sorted(records.items())
        ]

    def list_waypoint_names(self) -> List[str]:
        return [waypoint.name for waypoint in self.list_waypoints()]

    def load_waypoint(self, name: str, expected_map_id: Optional[str] = None) -> Waypoint:
        records = self._load_records()
        if name not in records:
            raise WaypointNotFoundError("Waypoint '{}' was not found.".format(name))

        waypoint = Waypoint.from_record(name, records[name])
        if expected_map_id and waypoint.map_id != expected_map_id:
            raise MapMismatchError(
                "Waypoint '{}' belongs to map '{}' instead of '{}'.".format(
                    name,
                    waypoint.map_id,
                    expected_map_id,
                )
            )

        return waypoint

    def save_waypoint(self, waypoint: Waypoint, overwrite: bool = False) -> Waypoint:
        data = self._load_data()
        waypoints = data.setdefault('waypoints', {})
        if not overwrite and waypoint.name in waypoints:
            raise DuplicateWaypointError(
                "Waypoint '{}' already exists.".format(waypoint.name)
            )

        waypoints[waypoint.name] = waypoint.as_record()
        self._write_data(data)
        return waypoint

    def _load_records(self) -> Dict[str, Dict[str, object]]:
        data = self._load_data()
        records = data.get('waypoints', {})
        if not isinstance(records, dict):
            raise ValueError('Waypoint store is malformed: waypoints must be a mapping.')
        return records

    def _load_data(self) -> Dict[str, object]:
        if not self._path.exists():
            return {'version': 1, 'waypoints': {}}

        text = self._path.read_text(encoding='utf-8').strip()
        if not text:
            return {'version': 1, 'waypoints': {}}

        if yaml is not None:
            data = yaml.safe_load(text)
        else:
            data = json.loads(text)

        if data is None:
            return {'version': 1, 'waypoints': {}}
        if not isinstance(data, dict):
            raise ValueError('Waypoint store must contain a mapping at the root.')
        data.setdefault('version', 1)
        data.setdefault('waypoints', {})
        return data

    def _write_data(self, data: Dict[str, object]) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        serialized = _serialize_mapping(data)
        self._path.write_text(serialized, encoding='utf-8')


def _serialize_mapping(data: Dict[str, object]) -> str:
    if yaml is not None:
        return yaml.safe_dump(data, sort_keys=True)
    return json.dumps(data, indent=2, sort_keys=True) + '\n'
