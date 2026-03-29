from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from datetime import timezone
from pathlib import Path
import time
from typing import Callable
from typing import Optional

from navigation.commands import help_text_for_mode
from navigation.frontiers import compute_coverage
from navigation.frontiers import extract_frontier_clusters
from navigation.frontiers import select_frontier_goal
from navigation.health import HealthMonitor
from navigation.types import ActionType
from navigation.types import CommandContext
from navigation.types import CommandResponse
from navigation.types import CommandType
from navigation.types import FrontierParams
from navigation.types import NavigationResult
from navigation.types import NavigationStatus
from navigation.types import OperatingMode
from navigation.types import PendingMapSave
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
    is_exploration: bool = False
    frontier_reference_pose: Optional[Pose2D] = None
    frontier_cells: tuple[tuple[int, int], ...] = tuple()


class NavigationSupervisor:
    def __init__(
        self,
        waypoint_store: WaypointStore,
        health_monitor: HealthMonitor,
        active_map_id: str,
        navigation_timeout: float = 10.0,
        clock: Optional[Callable[[], float]] = None,
        timestamp_factory: Optional[Callable[[], str]] = None,
        mode: str = 'navigation',
        map_output_dir: Optional[Path] = None,
        map_save_stem_factory: Optional[Callable[[], str]] = None,
        frontier_params: Optional[FrontierParams] = None,
        base_timeout: float = 15.0,
        expected_speed: float = 0.12,
        timeout_safety_factor: float = 2.5,
        empty_cycle_threshold: int = 3,
        coverage_threshold: float = 0.0,
    ):
        self._waypoint_store = waypoint_store
        self._health_monitor = health_monitor
        self._active_map_id = active_map_id
        self._navigation_timeout = navigation_timeout
        self._clock = clock or time.monotonic
        self._timestamp_factory = timestamp_factory or _utc_now
        self._mode = OperatingMode(mode)
        self._map_output_dir = Path(map_output_dir) if map_output_dir else Path('.guido') / 'maps'
        self._map_save_stem_factory = map_save_stem_factory or _default_map_save_stem
        self._frontier_params = frontier_params or FrontierParams()
        self._base_timeout = base_timeout
        self._expected_speed = expected_speed
        self._timeout_safety_factor = timeout_safety_factor
        self._empty_cycle_threshold = empty_cycle_threshold
        self._coverage_threshold = coverage_threshold

        self._active_goal: Optional[_ActiveGoal] = None
        self._exploration_active = False
        self._exploration_blacklist: list[Pose2D] = []
        self._exploration_coverage = 0.0
        self._empty_frontier_cycles = 0
        self._pending_map_save: Optional[PendingMapSave] = None
        self._frontier_counter = 0

    @property
    def mode(self):
        return self._mode

    @property
    def active_map_id(self):
        return self._active_map_id

    @property
    def active_goal_name(self):
        return self._active_goal.waypoint.name if self._active_goal is not None else None

    @property
    def has_active_goal(self):
        return self._active_goal is not None

    @property
    def exploration_active(self):
        return self._exploration_active

    @property
    def exploration_coverage(self):
        return self._exploration_coverage

    @property
    def pending_map_save(self):
        return self._pending_map_save

    @property
    def exploration_blacklist(self):
        return tuple(self._exploration_blacklist)

    @property
    def active_exploration_goal_pose(self):
        if self._active_goal is None or not self._active_goal.is_exploration:
            return None
        return self._active_goal.waypoint.pose

    @property
    def active_exploration_frontier_reference(self):
        if self._active_goal is None or not self._active_goal.is_exploration:
            return None
        return self._active_goal.frontier_reference_pose

    @property
    def active_exploration_frontier_cells(self):
        if self._active_goal is None or not self._active_goal.is_exploration:
            return tuple()
        return self._active_goal.frontier_cells

    def set_active_map_id(self, map_id: str) -> None:
        self._active_map_id = map_id

    def handle_command(self, command, context: Optional[CommandContext] = None) -> CommandResponse:
        context = context or CommandContext()

        if (
            self._mode == OperatingMode.NAVIGATION
            and command.command_type == CommandType.START_EXPLORATION
        ):
            return CommandResponse(
                accepted=False,
                message="Command 'start_exploration' is only available in exploration mode.",
            )

        if self._mode == OperatingMode.EXPLORATION and command.command_type in {
            CommandType.SAVE_WAYPOINT,
            CommandType.NAVIGATE_TO,
            CommandType.LIST_WAYPOINTS,
        }:
            return self._unsupported_in_exploration(command.command_type)

        if command.command_type == CommandType.SAVE_WAYPOINT:
            return self._save_waypoint(command.argument, context.current_pose)
        if command.command_type == CommandType.NAVIGATE_TO:
            return self._navigate_to(command.argument)
        if command.command_type == CommandType.LIST_WAYPOINTS:
            return self._list_waypoints()
        if command.command_type == CommandType.START_EXPLORATION:
            return self._start_exploration(context)
        if command.command_type == CommandType.CANCEL_NAVIGATION:
            if self._mode == OperatingMode.EXPLORATION:
                return self._cancel_exploration()
            return self._cancel_navigation()
        if command.command_type == CommandType.STOP:
            if self._mode == OperatingMode.EXPLORATION:
                return self._stop_exploration()
            return self._stop_navigation()
        if command.command_type == CommandType.HELP:
            return CommandResponse(accepted=True, message=help_text_for_mode(self._mode))

        return CommandResponse(
            accepted=False,
            message=f'Unsupported command. {help_text_for_mode(self._mode)}',
        )

    def tick(self, now: Optional[float] = None) -> Optional[CommandResponse]:
        current_time = self._clock() if now is None else now
        health_status = self._health_monitor.status(current_time)

        if self._exploration_active and not health_status.healthy:
            self._active_goal = None
            self._exploration_active = False
            return CommandResponse(
                accepted=False,
                message=f'Exploration aborted: {health_status.details}.',
                actions=(
                    SupervisorAction(
                        action_type=ActionType.STOP,
                        reason='exploration aborted due to stale health',
                    ),
                ),
            )

        if self._active_goal is None:
            return None

        goal_timeout = self._active_goal.timeout
        if current_time - self._active_goal.started_at <= goal_timeout:
            return None

        active_goal = self._active_goal
        self._active_goal = None
        if active_goal.is_exploration:
            self._blacklist_frontier(
                active_goal.frontier_reference_pose or active_goal.waypoint.pose
            )
            return CommandResponse(
                accepted=False,
                message=(
                    "Exploration goal '{}' timed out after {:.1f}s. "
                    'Blacklisted the frontier and stopping before replanning.'
                ).format(active_goal.waypoint.name, goal_timeout),
                actions=(
                    SupervisorAction(
                        action_type=ActionType.STOP,
                        reason='exploration goal timeout',
                    ),
                ),
            )

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

    def plan_exploration(self, current_pose, occupancy_grid) -> Optional[CommandResponse]:
        if not self._exploration_active or self._pending_map_save is not None:
            return None
        if self._active_goal is not None:
            return None
        if current_pose is None or occupancy_grid is None:
            return None

        self._exploration_coverage = compute_coverage(occupancy_grid)
        frontier_goal = select_frontier_goal(
            occupancy_grid,
            current_pose,
            blacklist_points=self._exploration_blacklist,
            params=self._frontier_params,
        )
        if frontier_goal is None:
            clusters = extract_frontier_clusters(
                occupancy_grid,
                min_cluster_size=self._frontier_params.min_cluster_size,
            )
            self._empty_frontier_cycles += 1
            coverage_met = (
                self._coverage_threshold > 0.0
                and self._exploration_coverage >= self._coverage_threshold
            )
            cycle_limit_met = self._empty_frontier_cycles >= self._empty_cycle_threshold

            if not coverage_met and not cycle_limit_met:
                return CommandResponse(
                    accepted=True,
                    message=(
                        'No valid frontier found ({}/{}). Coverage {:.1f}%. '
                        'Waiting for more map updates.'
                    ).format(
                        0,
                        len(clusters),
                        self._exploration_coverage * 100.0,
                    ),
                )

            reason = (
                'coverage {:.1f}% reached threshold'.format(self._exploration_coverage * 100.0)
                if coverage_met
                else '{} consecutive empty frontier cycles'.format(self._empty_frontier_cycles)
            )
            pending_save = self._build_pending_map_save()
            self._pending_map_save = pending_save
            self._exploration_active = False
            return CommandResponse(
                accepted=True,
                message=(
                    "Exploration complete ({}). Stopping to save the map to '{}'."
                ).format(reason, pending_save.path),
                actions=(
                    SupervisorAction(
                        action_type=ActionType.STOP,
                        reason='exploration complete',
                    ),
                ),
            )

        self._empty_frontier_cycles = 0
        self._frontier_counter += 1
        frontier_name = f'frontier_{self._frontier_counter}'
        waypoint = Waypoint(
            name=frontier_name,
            map_id=self._active_map_id,
            pose=frontier_goal.target_pose,
            created_at=self._timestamp_factory(),
        )
        distance = current_pose.planar_distance_to(frontier_goal.target_pose)
        self._active_goal = _ActiveGoal(
            waypoint=waypoint,
            started_at=self._clock(),
            timeout=self._compute_adaptive_timeout(distance),
            is_exploration=True,
            frontier_reference_pose=frontier_goal.reference_pose,
            frontier_cells=frontier_goal.cluster.cells,
        )
        return CommandResponse(
            accepted=True,
            message=(
                "Navigating to frontier '{}' at reachable pose ({:.2f}, {:.2f}) "
                'near frontier centroid ({:.2f}, {:.2f}).'
            ).format(
                frontier_name,
                frontier_goal.target_pose.x,
                frontier_goal.target_pose.y,
                frontier_goal.reference_pose.x,
                frontier_goal.reference_pose.y,
            ),
            actions=(
                SupervisorAction(action_type=ActionType.NAVIGATE, waypoint=waypoint),
            ),
        )

    def apply_navigation_result(self, result: NavigationResult) -> CommandResponse:
        if self._active_goal is None:
            return CommandResponse(
                accepted=False,
                message='Ignoring navigation result because no goal is active.',
            )

        active_goal = self._active_goal
        self._active_goal = None
        if active_goal.is_exploration:
            return self._apply_exploration_result(active_goal, result)

        waypoint_name = active_goal.waypoint.name
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

    def begin_pending_map_save(self) -> Optional[PendingMapSave]:
        return self._pending_map_save

    def apply_map_save_result(self, success: bool, detail: str = '') -> str:
        pending = self._pending_map_save
        if pending is None:
            return 'Ignoring map save result because no exploration map save is active.'

        self._pending_map_save = None
        if success:
            self._active_map_id = pending.map_id
            self._active_goal = None
            self._exploration_active = False
            return "Exploration map saved to '{}' and active map id set to '{}'.".format(
                pending.path,
                pending.map_id,
            )

        return "Exploration completed, but saving the map to '{}' failed: {}".format(
            pending.path,
            detail or 'unknown error',
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

    def _start_exploration(self, context: CommandContext) -> CommandResponse:
        if self._pending_map_save is not None:
            return CommandResponse(
                accepted=False,
                message='Cannot start exploration while a map save is pending.',
            )
        if self._exploration_active:
            return CommandResponse(accepted=False, message='Exploration is already active.')
        if self._active_goal is not None:
            return CommandResponse(
                accepted=False,
                message='A navigation goal is already active.',
            )

        blockers = []
        if not context.map_available:
            blockers.append("required topic '/map' is not available yet")
        if not context.cmd_vel_ready:
            blockers.append("required subscriber for '/cmd_vel' is not available yet")
        if context.current_pose is None:
            blockers.append('current pose TF in the map frame is unavailable')

        health_status = self._health_monitor.status()
        if not health_status.healthy:
            blockers.append(health_status.details)

        if blockers:
            return CommandResponse(
                accepted=False,
                message='Cannot start exploration: {}.'.format('; '.join(blockers)),
            )

        self._exploration_active = True
        self._exploration_blacklist = []
        self._exploration_coverage = 0.0
        self._empty_frontier_cycles = 0
        self._pending_map_save = None
        return CommandResponse(
            accepted=True,
            message='Exploration started. Selecting the first frontier.',
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

    def _cancel_exploration(self) -> CommandResponse:
        if not self._exploration_active and self._active_goal is None:
            return CommandResponse(
                accepted=False,
                message='Exploration mode ended with no active frontier goal.',
            )
        if self._active_goal is not None and self._active_goal.cancel_requested:
            return CommandResponse(
                accepted=False,
                message=(
                    "Cancellation for exploration goal '{}' is already in progress."
                ).format(self._active_goal.waypoint.name),
            )

        goal_name = self._active_goal.waypoint.name if self._active_goal is not None else None
        self._exploration_active = False
        if self._active_goal is not None:
            self._active_goal.cancel_requested = True
            return CommandResponse(
                accepted=True,
                message=(
                    "Cancelling exploration goal '{}' and ending exploration mode."
                ).format(goal_name),
                actions=(
                    SupervisorAction(
                        action_type=ActionType.CANCEL,
                        reason='user canceled exploration',
                    ),
                ),
            )

        return CommandResponse(
            accepted=True,
            message='Exploration mode ended with no active frontier goal.',
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

    def _stop_exploration(self) -> CommandResponse:
        if self._active_goal is not None:
            goal_name = self._active_goal.waypoint.name
            self._active_goal = None
            message = "Stopping exploration goal '{}' and exiting exploration mode.".format(
                goal_name
            )
        else:
            message = 'Stopping exploration and exiting exploration mode.'
        self._exploration_active = False
        self._pending_map_save = None
        return CommandResponse(
            accepted=True,
            message=message,
            actions=(
                SupervisorAction(
                    action_type=ActionType.STOP,
                    reason='user requested exploration stop',
                ),
            ),
        )

    def _apply_exploration_result(
        self,
        active_goal: _ActiveGoal,
        result: NavigationResult,
    ) -> CommandResponse:
        waypoint_name = active_goal.waypoint.name
        if result.status == NavigationStatus.SUCCEEDED:
            return CommandResponse(
                accepted=True,
                message="Reached exploration frontier '{}'. Selecting the next frontier.".format(
                    waypoint_name
                ),
            )

        if result.status == NavigationStatus.CANCELED:
            if self._exploration_active:
                return CommandResponse(
                    accepted=True,
                    message="Exploration frontier '{}' was canceled.".format(waypoint_name),
                )
            return CommandResponse(
                accepted=True,
                message=(
                    "Exploration goal '{}' canceled while exiting exploration mode."
                ).format(waypoint_name),
            )

        self._blacklist_frontier(active_goal.frontier_reference_pose or active_goal.waypoint.pose)
        detail = result.message.strip()
        if detail:
            detail = detail + ' '
        return CommandResponse(
            accepted=False,
            message=(
                "{}Blacklisted the frontier and continuing exploration."
            ).format(detail),
        )

    def _unsupported_in_exploration(self, command_type: CommandType) -> CommandResponse:
        return CommandResponse(
            accepted=False,
            message=(
                "Command '{}' is unavailable in exploration mode. {}"
            ).format(command_type.value, help_text_for_mode(OperatingMode.EXPLORATION)),
        )

    def _blacklist_frontier(self, pose: Pose2D) -> None:
        if pose is not None:
            self._exploration_blacklist.append(pose)

    def _compute_adaptive_timeout(self, distance: float) -> float:
        """Return a timeout proportional to the goal distance."""

        travel_estimate = max(0.0, distance) / max(self._expected_speed, 1e-6)
        return max(self._base_timeout, travel_estimate * self._timeout_safety_factor)

    def _build_pending_map_save(self) -> PendingMapSave:
        map_id = self._map_save_stem_factory()
        path = str(self._map_output_dir / map_id)
        return PendingMapSave(map_id=map_id, path=path)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _default_map_save_stem() -> str:
    return 'explore_{}'.format(datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
