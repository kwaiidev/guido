from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from datetime import timezone
import time
from typing import Callable
from typing import Optional

from navigation.commands import HELP_TEXT
from navigation.health import HealthMonitor
from navigation.types import ActionType
from navigation.types import CommandContext
from navigation.types import CommandResponse
from navigation.types import CommandType
from navigation.types import NavigationResult
from navigation.types import NavigationStatus
from navigation.types import Pose2D
from navigation.types import SupervisorAction
from navigation.types import Waypoint
from navigation.waypoints import DuplicateWaypointError
from navigation.waypoints import MapMismatchError
from navigation.waypoints import WaypointNotFoundError
from navigation.waypoints import WaypointStore


@dataclass
class _ActiveGoal:
    waypoint: Waypoint
    started_at: float
    timeout: float
    cancel_requested: bool = False


class NavigationSupervisor:
    def __init__(
        self,
        waypoint_store: WaypointStore,
        health_monitor: HealthMonitor,
        active_map_id: str,
        navigation_timeout: float = 10.0,
        clock: Optional[Callable[[], float]] = None,
        timestamp_factory: Optional[Callable[[], str]] = None,
    ):
        self._waypoint_store = waypoint_store
        self._health_monitor = health_monitor
        self._active_map_id = active_map_id
        self._navigation_timeout = navigation_timeout
        self._clock = clock or time.monotonic
        self._timestamp_factory = timestamp_factory or _utc_now

        self._active_goal: Optional[_ActiveGoal] = None

    @property
    def active_map_id(self):
        return self._active_map_id

    @property
    def active_goal_name(self):
        return self._active_goal.waypoint.name if self._active_goal is not None else None

    @property
    def has_active_goal(self):
        return self._active_goal is not None

    def set_active_map_id(self, map_id: str) -> None:
        self._active_map_id = map_id

    def handle_command(self, command, context: Optional[CommandContext] = None) -> CommandResponse:
        context = context or CommandContext()

        if command.command_type == CommandType.SAVE_WAYPOINT:
            return self._save_waypoint(command.argument, context.current_pose)
        if command.command_type == CommandType.NAVIGATE_TO:
            return self._navigate_to(command.argument)
        if command.command_type == CommandType.LIST_WAYPOINTS:
            return self._list_waypoints()
        if command.command_type == CommandType.CANCEL_NAVIGATION:
            return self._cancel_navigation()
        if command.command_type == CommandType.STOP:
            return self._stop_navigation()
        if command.command_type == CommandType.HELP:
            return CommandResponse(accepted=True, message=HELP_TEXT)

        return CommandResponse(
            accepted=False,
            message=f'Unsupported command. {HELP_TEXT}',
        )

    def tick(self, now: Optional[float] = None) -> Optional[CommandResponse]:
        current_time = self._clock() if now is None else now

        if self._active_goal is None:
            return None

        goal_timeout = self._active_goal.timeout
        if current_time - self._active_goal.started_at <= goal_timeout:
            return None

        active_goal = self._active_goal
        self._active_goal = None
        return CommandResponse(
            accepted=False,
            message="Navigation to '{}' timed out after {:.1f}s.".format(
                active_goal.waypoint.name,
                goal_timeout,
            ),
            actions=(
                SupervisorAction(action_type=ActionType.STOP, reason='navigation timeout'),
            ),
        )

    def apply_navigation_result(self, result: NavigationResult) -> CommandResponse:
        if self._active_goal is None:
            return CommandResponse(
                accepted=False,
                message='Ignoring navigation result because no goal is active.',
            )

        self._active_goal = None
        waypoint_name = result.waypoint_name
        if result.status == NavigationStatus.SUCCEEDED:
            return CommandResponse(accepted=True, message=f"Arrived at '{waypoint_name}'.")
        if result.status == NavigationStatus.CANCELED:
            return CommandResponse(
                accepted=True,
                message=f"Navigation to '{waypoint_name}' canceled.",
            )
        if result.status == NavigationStatus.TIMED_OUT:
            return CommandResponse(
                accepted=False,
                message=f"Navigation to '{waypoint_name}' timed out.",
            )
        return CommandResponse(
            accepted=False,
            message=f"Navigation to '{waypoint_name}' failed.",
        )

    def _save_waypoint(
        self,
        waypoint_name: Optional[str],
        current_pose: Optional[Pose2D],
    ) -> CommandResponse:
        if current_pose is None:
            return CommandResponse(
                accepted=False,
                message='Cannot save waypoint without a current localized pose.',
            )

        waypoint = Waypoint(
            name=waypoint_name or '',
            map_id=self._active_map_id,
            pose=current_pose,
            created_at=self._timestamp_factory(),
        )
        try:
            self._waypoint_store.save_waypoint(waypoint)
        except DuplicateWaypointError as exc:
            return CommandResponse(accepted=False, message=str(exc))

        return CommandResponse(accepted=True, message="Saved waypoint '{}'.".format(waypoint.name))

    def _navigate_to(self, waypoint_name: Optional[str]) -> CommandResponse:
        if self._active_goal is not None and self._active_goal.cancel_requested:
            return CommandResponse(
                accepted=False,
                message=(
                    "Waiting for '{}' to finish canceling before starting another goal."
                ).format(self._active_goal.waypoint.name),
            )
        if self._active_goal is not None:
            return CommandResponse(
                accepted=False,
                message=(
                    "Already navigating to '{}'. Cancel or stop first."
                ).format(self._active_goal.waypoint.name),
            )

        health_status = self._health_monitor.status()
        if not health_status.healthy:
            return CommandResponse(accepted=False, message=health_status.details)

        try:
            waypoint = self._waypoint_store.load_waypoint(
                waypoint_name or '',
                expected_map_id=self._active_map_id,
            )
        except (WaypointNotFoundError, MapMismatchError) as exc:
            return CommandResponse(accepted=False, message=str(exc))

        self._active_goal = _ActiveGoal(
            waypoint=waypoint,
            started_at=self._clock(),
            timeout=self._navigation_timeout,
        )
        return CommandResponse(
            accepted=True,
            message="Navigating to '{}'.".format(waypoint.name),
            actions=(
                SupervisorAction(action_type=ActionType.NAVIGATE, waypoint=waypoint),
            ),
        )

    def _list_waypoints(self) -> CommandResponse:
        names = self._waypoint_store.list_waypoint_names()
        if not names:
            return CommandResponse(accepted=True, message='No saved waypoints.')
        return CommandResponse(
            accepted=True,
            message='Saved waypoints: {}.'.format(', '.join(names)),
        )

    def _cancel_navigation(self) -> CommandResponse:
        if self._active_goal is None:
            return CommandResponse(
                accepted=False,
                message='No active navigation goal to cancel.',
            )
        if self._active_goal.cancel_requested:
            return CommandResponse(
                accepted=False,
                message=(
                    "Cancellation for '{}' is already in progress."
                ).format(self._active_goal.waypoint.name),
            )

        self._active_goal.cancel_requested = True
        return CommandResponse(
            accepted=True,
            message="Cancelling navigation to '{}'.".format(self._active_goal.waypoint.name),
            actions=(
                SupervisorAction(
                    action_type=ActionType.CANCEL,
                    reason='user canceled navigation',
                ),
            ),
        )

    def _stop_navigation(self) -> CommandResponse:
        if self._active_goal is not None:
            goal_name = self._active_goal.waypoint.name
            self._active_goal = None
            message = "Stopping navigation to '{}'.".format(goal_name)
        else:
            message = 'Issuing stop command.'
        return CommandResponse(
            accepted=True,
            message=message,
            actions=(
                SupervisorAction(action_type=ActionType.STOP, reason='user requested stop'),
            ),
        )


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()
