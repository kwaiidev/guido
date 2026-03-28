"""Shared types for autonomous navigation logic."""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class Pose2D:
    """A lightweight 2D pose in the map frame."""

    x: float
    y: float
    yaw: float

    def as_dict(self) -> Dict[str, float]:
        return {'x': self.x, 'y': self.y, 'yaw': self.yaw}

    def planar_distance_to(self, other: 'Pose2D') -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return (dx * dx + dy * dy) ** 0.5


@dataclass(frozen=True)
class Waypoint:
    """A named navigation target persisted in the active map frame."""

    name: str
    map_id: str
    pose: Pose2D
    created_at: str

    def as_record(self) -> Dict[str, object]:
        return {
            'map_id': self.map_id,
            'x': self.pose.x,
            'y': self.pose.y,
            'yaw': self.pose.yaw,
            'created_at': self.created_at,
        }

    @classmethod
    def from_record(cls, name: str, record: Dict[str, object]) -> 'Waypoint':
        return cls(
            name=name,
            map_id=str(record['map_id']),
            pose=Pose2D(
                x=float(record['x']),
                y=float(record['y']),
                yaw=float(record['yaw']),
            ),
            created_at=str(record['created_at']),
        )


class CommandType(str, Enum):
    SAVE_WAYPOINT = 'save_waypoint'
    NAVIGATE_TO = 'navigate_to'
    LIST_WAYPOINTS = 'list_waypoints'
    START_EXPLORATION = 'start_exploration'
    CANCEL_NAVIGATION = 'cancel_navigation'
    STOP = 'stop'
    HELP = 'help'


@dataclass(frozen=True)
class NavCommand:
    command_type: CommandType
    argument: Optional[str] = None


class ActionType(str, Enum):
    NAVIGATE = 'navigate'
    CANCEL = 'cancel'
    STOP = 'stop'


class OperatingMode(str, Enum):
    NAVIGATION = 'navigation'
    EXPLORATION = 'exploration'


@dataclass(frozen=True)
class SupervisorAction:
    action_type: ActionType
    waypoint: Optional[Waypoint] = None
    reason: Optional[str] = None


@dataclass(frozen=True)
class CommandResponse:
    accepted: bool
    message: str
    actions: Tuple[SupervisorAction, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class HealthStatus:
    healthy: bool
    stale_sources: Tuple[str, ...]
    details: str


@dataclass(frozen=True)
class CommandContext:
    current_pose: Optional[Pose2D] = None
    map_available: bool = False
    cmd_vel_ready: bool = False


@dataclass(frozen=True)
class OccupancyGridSnapshot:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Tuple[int, ...]

    def index(self, row: int, col: int) -> int:
        return row * self.width + col

    def value(self, row: int, col: int) -> int:
        return self.data[self.index(row, col)]

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def cell_center(self, row: int, col: int) -> Tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return (x, y)

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        col = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)
        if not self.in_bounds(row, col):
            return None
        return (row, col)


@dataclass(frozen=True)
class PendingMapSave:
    map_id: str
    path: str


@dataclass(frozen=True)
class NavigationRequest:
    action_type: ActionType
    waypoint_name: Optional[str] = None
    map_id: Optional[str] = None
    pose: Optional[Pose2D] = None
    reason: Optional[str] = None


class NavigationStatus(str, Enum):
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    CANCELED = 'canceled'
    TIMED_OUT = 'timed_out'


@dataclass(frozen=True)
class NavigationResult:
    status: NavigationStatus
    waypoint_name: Optional[str] = None
    message: str = ''


@dataclass(frozen=True)
class AdapterDispatch:
    accepted: bool
    status_messages: Tuple[str, ...] = field(default_factory=tuple)
    requests: Tuple[NavigationRequest, ...] = field(default_factory=tuple)
