"""Waypoint persistence for autonomous navigation targets."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict
from typing import List
from typing import Optional

try:
    import yaml
except ImportError:  # pragma: no cover - depends on system packages
    yaml = None

from navigation.types import Waypoint


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
        return [Waypoint.from_record(record) for _, record in sorted(records.items())]

    def list_waypoint_names(self) -> List[str]:
        return [waypoint.name for waypoint in self.list_waypoints()]

    def load_waypoint(self, name: str, expected_map_id: Optional[str] = None) -> Waypoint:
        records = self._load_records()
        if name not in records:
            raise WaypointNotFoundError(name)
        waypoint = Waypoint.from_record(records[name])
        if expected_map_id is not None and waypoint.map_id != expected_map_id:
            raise MapMismatchError(
                f"Waypoint '{name}' belongs to map '{waypoint.map_id}', not '{expected_map_id}'."
            )
        return waypoint

    def save_waypoint(self, waypoint: Waypoint, overwrite: bool = False) -> Waypoint:
        data = self._load_data()
        records = data.setdefault('waypoints', {})
        if not isinstance(records, dict):
            raise ValueError("Waypoint store 'waypoints' section must be a mapping.")
        if waypoint.name in records and not overwrite:
            raise DuplicateWaypointError(f"Waypoint '{waypoint.name}' already exists.")
        records[waypoint.name] = waypoint.as_record()
        self._write_data(data)
        return waypoint

    def _load_records(self) -> Dict[str, Dict[str, object]]:
        data = self._load_data()
        records = data.get('waypoints', {})
        if records is None:
            return {}
        if not isinstance(records, dict):
            raise ValueError("Waypoint store 'waypoints' section must be a mapping.")
        return records

    def _load_data(self) -> Dict[str, object]:
        if not self._path.exists():
            return {'waypoints': {}}
        raw_text = self._path.read_text(encoding='utf-8')
        if not raw_text.strip():
            return {'waypoints': {}}
        if yaml is not None:
            loaded = yaml.safe_load(raw_text)
        else:
            loaded = json.loads(raw_text)
        if loaded is None:
            return {'waypoints': {}}
        if not isinstance(loaded, dict):
            raise ValueError('Waypoint store must deserialize to a JSON object.')
        return loaded

    def _write_data(self, data: Dict[str, object]) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._path.write_text(_serialize_mapping(data), encoding='utf-8')


def _serialize_mapping(data):
    if yaml is not None:
        return yaml.safe_dump(data, sort_keys=True)
    return json.dumps(data, indent=2, sort_keys=True)
