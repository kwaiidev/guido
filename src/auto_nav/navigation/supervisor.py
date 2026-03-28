import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional

from .commands import HELP_TEXT, help_text_for_mode
from .frontiers import select_frontier_goal
from .health import HealthMonitor
from .types import (
    ActionType,
    CommandContext,
    CommandResponse,
    CommandType,
    NavigationResult,
    NavigationStatus,
    OccupancyGridSnapshot,
    OperatingMode,
    PendingMapSave,
    Pose2D,
    SupervisorAction,
    Waypoint,
)
from .waypoints import (
    DuplicateWaypointError,
    MapMismatchError,
    WaypointNotFoundError,
    WaypointStore,
)


@dataclass
class _ActiveGoal:
    waypoint: Waypoint
    started_at: float
    cancel_requested: bool = False
    is_exploration: bool = False
    frontier_reference_pose: Optional[Pose2D] = None
    frontier_cells: tuple[tuple[int, int], ...] = ()


class NavigationSupervisor:
    def __init__(
        self,
        waypoint_store: WaypointStore,
        health_monitor: HealthMonitor,
        active_map_id: str,
        navigation_timeout: float = 10.0,
        clock: Optional[Callable[[], float]] = None,
        timestamp_factory: Optional[Callable[[], str]] = None,
        mode: str = OperatingMode.NAVIGATION.value,
        map_output_dir: Optional[Path] = None,
        map_save_stem_factory: Optional[Callable[[], str]] = None,
    ):
        self._waypoint_store = waypoint_store
        self._health_monitor = health_monitor
        self._active_map_id = active_map_id
        self._navigation_timeout = navigation_timeout
        self._clock = clock or time.monotonic
        self._timestamp_factory = timestamp_factory or _utc_now
        self._mode = OperatingMode(mode)
        self._map_output_dir = (
            Path(map_output_dir).expanduser()
            if map_output_dir is not None
            else Path.home() / ".guido" / "maps"
        )
        self._map_save_stem_factory = map_save_stem_factory or _default_map_save_stem
        self._active_goal: Optional[_ActiveGoal] = None
        self._exploration_active = False
        self._exploration_blacklist: list[Pose2D] = []
        self._empty_frontier_cycles = 0
        self._frontier_goal_sequence = 0
        self._pending_map_save: Optional[PendingMapSave] = None
        self._map_save_in_progress: Optional[PendingMapSave] = None

    @property
    def mode(self) -> OperatingMode:
        return self._mode

    @property
    def active_map_id(self) -> str:
        return self._active_map_id

    @property
    def active_goal_name(self) -> Optional[str]:
        if self._active_goal is None:
            return None
        return self._active_goal.waypoint.name

    @property
    def has_active_goal(self) -> bool:
        return self._active_goal is not None

    @property
    def exploration_active(self) -> bool:
        return self._exploration_active

    @property
    def pending_map_save(self) -> Optional[PendingMapSave]:
        return self._pending_map_save

    @property
    def exploration_blacklist(self) -> tuple[Pose2D, ...]:
        return tuple(self._exploration_blacklist)

    @property
    def active_exploration_goal_pose(self) -> Optional[Pose2D]:
        if self._active_goal is None or not self._active_goal.is_exploration:
            return None
        return self._active_goal.waypoint.pose

    @property
    def active_exploration_frontier_reference(self) -> Optional[Pose2D]:
        if self._active_goal is None or not self._active_goal.is_exploration:
            return None
        return self._active_goal.frontier_reference_pose

    @property
    def active_exploration_frontier_cells(self) -> tuple[tuple[int, int], ...]:
        if self._active_goal is None or not self._active_goal.is_exploration:
            return ()
        return self._active_goal.frontier_cells

    def set_active_map_id(self, map_id: str) -> None:
        self._active_map_id = map_id

    def handle_command(
        self,
        command,
        context: Optional[CommandContext] = None,
    ) -> CommandResponse:
        context = context or CommandContext()

        if command.command_type == CommandType.SAVE_WAYPOINT:
            if self._mode == OperatingMode.EXPLORATION:
                return self._unsupported_in_exploration(command.command_type)
            return self._save_waypoint(command.argument, context.current_pose)
        if command.command_type == CommandType.NAVIGATE_TO:
            if self._mode == OperatingMode.EXPLORATION:
                return self._unsupported_in_exploration(command.command_type)
            return self._navigate_to(command.argument)
        if command.command_type == CommandType.LIST_WAYPOINTS:
            if self._mode == OperatingMode.EXPLORATION:
                return self._unsupported_in_exploration(command.command_type)
            return self._list_waypoints()
        if command.command_type == CommandType.START_EXPLORATION:
            if self._mode != OperatingMode.EXPLORATION:
                return CommandResponse(
                    accepted=False,
                    message=(
                        "Command 'start_exploration' is only available in exploration mode."
                    ),
                )
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
            return CommandResponse(
                accepted=True,
                message=help_text_for_mode(self._mode),
            )
        return CommandResponse(
            accepted=False, message="Unsupported command. {}".format(HELP_TEXT)
        )

    def tick(self, now: Optional[float] = None) -> Optional[CommandResponse]:
        current_time = self._clock() if now is None else now

        if self._exploration_active:
            health_status = self._health_monitor.status(now=current_time)
            if not health_status.healthy:
                self._exploration_active = False
                self._empty_frontier_cycles = 0
                self._pending_map_save = None
                self._map_save_in_progress = None
                self._active_goal = None
                return CommandResponse(
                    accepted=False,
                    message="Exploration aborted: {}.".format(health_status.details),
                    actions=(
                        SupervisorAction(
                            action_type=ActionType.STOP,
                            reason="exploration aborted due to stale health",
                        ),
                    ),
                )

        if self._active_goal is None:
            return None

        if current_time - self._active_goal.started_at <= self._navigation_timeout:
            return None

        active_goal = self._active_goal
        self._active_goal = None

        if active_goal.is_exploration and self._exploration_active:
            self._blacklist_frontier(
                active_goal.frontier_reference_pose or active_goal.waypoint.pose
            )
            return CommandResponse(
                accepted=False,
                message=(
                    "Exploration goal '{}' timed out after {:.1f}s. "
                    "Blacklisted the frontier and stopping before replanning."
                ).format(active_goal.waypoint.name, self._navigation_timeout),
                actions=(
                    SupervisorAction(
                        action_type=ActionType.STOP,
                        reason="exploration goal timeout",
                    ),
                ),
            )

        return CommandResponse(
            accepted=False,
            message=(
                "Navigation to '{}' timed out after {:.1f}s.".format(
                    active_goal.waypoint.name,
                    self._navigation_timeout,
                )
            ),
            actions=(
                SupervisorAction(
                    action_type=ActionType.STOP,
                    reason="navigation timeout",
                ),
            ),
        )

    def plan_exploration(
        self,
        current_pose: Optional[Pose2D],
        occupancy_grid: Optional[OccupancyGridSnapshot],
    ) -> Optional[CommandResponse]:
        if self._mode != OperatingMode.EXPLORATION:
            return None
        if not self._exploration_active or self._active_goal is not None:
            return None
        if self._pending_map_save is not None or self._map_save_in_progress is not None:
            return None
        if current_pose is None or occupancy_grid is None:
            return None

        frontier_goal = select_frontier_goal(
            occupancy_grid,
            robot_pose=current_pose,
            blacklist_points=tuple(self._exploration_blacklist),
        )
        if frontier_goal is None:
            self._empty_frontier_cycles += 1
            if self._empty_frontier_cycles < 3:
                return CommandResponse(
                    accepted=True,
                    message=(
                        "No valid frontier found ({}/3). Waiting for more map updates."
                    ).format(self._empty_frontier_cycles),
                )

            self._exploration_active = False
            pending_save = self._build_pending_map_save()
            self._pending_map_save = pending_save
            return CommandResponse(
                accepted=True,
                message=(
                    "No valid frontier found for 3 consecutive cycles. "
                    "Exploration complete; stopping to save the map to '{}'."
                ).format(pending_save.path),
                actions=(
                    SupervisorAction(
                        action_type=ActionType.STOP,
                        reason="exploration complete",
                    ),
                ),
            )

        self._empty_frontier_cycles = 0
        self._frontier_goal_sequence += 1
        target_pose = frontier_goal.target_pose
        waypoint = Waypoint(
            name="frontier_{}".format(self._frontier_goal_sequence),
            map_id=self._active_map_id,
            pose=target_pose,
            created_at=self._timestamp_factory(),
        )
        self._active_goal = _ActiveGoal(
            waypoint=waypoint,
            started_at=self._clock(),
            is_exploration=True,
            frontier_reference_pose=frontier_goal.reference_pose,
            frontier_cells=frontier_goal.cluster.cells,
        )
        return CommandResponse(
            accepted=True,
            message=(
                "Navigating to frontier '{}' at reachable pose ({:.2f}, {:.2f}) "
                "near frontier centroid ({:.2f}, {:.2f})."
            ).format(
                waypoint.name,
                target_pose.x,
                target_pose.y,
                frontier_goal.reference_pose.x,
                frontier_goal.reference_pose.y,
            ),
            actions=(
                SupervisorAction(
                    action_type=ActionType.NAVIGATE,
                    waypoint=waypoint,
                ),
            ),
        )

    def apply_navigation_result(self, result: NavigationResult) -> CommandResponse:
        active_goal = None
        if self._active_goal and (
            result.waypoint_name is None
            or result.waypoint_name == self._active_goal.waypoint.name
        ):
            active_goal = self._active_goal
            self._active_goal = None

        if active_goal and active_goal.is_exploration:
            return self._apply_exploration_result(active_goal, result)

        waypoint_name = result.waypoint_name or "requested waypoint"
        if result.status == NavigationStatus.SUCCEEDED:
            message = result.message or "Arrived at '{}'.".format(waypoint_name)
            return CommandResponse(accepted=True, message=message)
        if result.status == NavigationStatus.CANCELED:
            message = result.message or "Navigation to '{}' canceled.".format(
                waypoint_name
            )
            return CommandResponse(accepted=False, message=message)
        if result.status == NavigationStatus.TIMED_OUT:
            message = result.message or "Navigation to '{}' timed out.".format(
                waypoint_name
            )
            return CommandResponse(accepted=False, message=message)

        message = result.message or "Navigation to '{}' failed.".format(waypoint_name)
        return CommandResponse(accepted=False, message=message)

    def begin_pending_map_save(self) -> Optional[PendingMapSave]:
        if self._pending_map_save is None or self._map_save_in_progress is not None:
            return None

        self._map_save_in_progress = self._pending_map_save
        self._pending_map_save = None
        return self._map_save_in_progress

    def apply_map_save_result(self, success: bool, detail: str = "") -> str:
        pending = self._map_save_in_progress
        self._map_save_in_progress = None
        if pending is None:
            return "Ignoring map save result because no exploration map save is active."

        if success:
            self._active_map_id = pending.map_id
            return (
                "Exploration map saved to '{}' and active map id set to '{}'."
            ).format(pending.path, pending.map_id)

        return ("Exploration completed, but saving the map to '{}' failed: {}").format(
            pending.path, detail or "unknown error"
        )

    def _save_waypoint(
        self,
        waypoint_name: Optional[str],
        current_pose: Optional[Pose2D],
    ) -> CommandResponse:
        if current_pose is None:
            return CommandResponse(
                accepted=False,
                message="Cannot save waypoint without a current localized pose.",
            )

        waypoint = Waypoint(
            name=waypoint_name or "",
            map_id=self._active_map_id,
            pose=current_pose,
            created_at=self._timestamp_factory(),
        )

        try:
            self._waypoint_store.save_waypoint(waypoint)
        except DuplicateWaypointError as exc:
            return CommandResponse(accepted=False, message=str(exc))

        return CommandResponse(
            accepted=True,
            message="Saved waypoint '{}'.".format(waypoint.name),
        )

    def _navigate_to(self, waypoint_name: Optional[str]) -> CommandResponse:
        if self._active_goal is not None:
            if self._active_goal.cancel_requested:
                return CommandResponse(
                    accepted=False,
                    message=(
                        "Waiting for '{}' to finish canceling before starting another goal.".format(
                            self._active_goal.waypoint.name
                        )
                    ),
                )
            return CommandResponse(
                accepted=False,
                message=(
                    "Already navigating to '{}'. Cancel or stop first.".format(
                        self._active_goal.waypoint.name
                    )
                ),
            )

        health_status = self._health_monitor.status()
        if not health_status.healthy:
            return CommandResponse(accepted=False, message=health_status.details)

        try:
            waypoint = self._waypoint_store.load_waypoint(
                waypoint_name or "",
                expected_map_id=self._active_map_id,
            )
        except (WaypointNotFoundError, MapMismatchError) as exc:
            return CommandResponse(accepted=False, message=str(exc))

        self._active_goal = _ActiveGoal(
            waypoint=waypoint,
            started_at=self._clock(),
        )
        return CommandResponse(
            accepted=True,
            message="Navigating to '{}'.".format(waypoint.name),
            actions=(
                SupervisorAction(
                    action_type=ActionType.NAVIGATE,
                    waypoint=waypoint,
                ),
            ),
        )

    def _start_exploration(self, context: CommandContext) -> CommandResponse:
        if self._pending_map_save is not None or self._map_save_in_progress is not None:
            return CommandResponse(
                accepted=False,
                message="Cannot start exploration while a map save is pending.",
            )
        if self._exploration_active:
            return CommandResponse(
                accepted=False,
                message="Exploration is already active.",
            )
        if self._active_goal is not None:
            return CommandResponse(
                accepted=False,
                message="A navigation goal is already active.",
            )

        blockers = []
        health_status = self._health_monitor.status()
        if not health_status.healthy:
            blockers.append(health_status.details)
        if not context.map_available:
            blockers.append("required topic '/map' is not available yet")
        if not context.cmd_vel_ready:
            blockers.append("required subscriber for '/cmd_vel' is not available yet")
        if context.current_pose is None:
            blockers.append("current pose TF in the map frame is unavailable")

        if blockers:
            return CommandResponse(
                accepted=False,
                message="Cannot start exploration: {}.".format("; ".join(blockers)),
            )

        self._exploration_active = True
        self._exploration_blacklist = []
        self._empty_frontier_cycles = 0
        self._frontier_goal_sequence = 0
        return CommandResponse(
            accepted=True,
            message="Exploration started. Selecting the first frontier.",
        )

    def _list_waypoints(self) -> CommandResponse:
        names = self._waypoint_store.list_waypoint_names()
        if not names:
            return CommandResponse(accepted=True, message="No saved waypoints.")

        return CommandResponse(
            accepted=True,
            message="Saved waypoints: {}.".format(", ".join(names)),
        )

    def _cancel_navigation(self) -> CommandResponse:
        if self._active_goal is None:
            return CommandResponse(
                accepted=False,
                message="No active navigation goal to cancel.",
            )

        if self._active_goal.cancel_requested:
            return CommandResponse(
                accepted=False,
                message=(
                    "Cancellation for '{}' is already in progress.".format(
                        self._active_goal.waypoint.name
                    )
                ),
            )

        goal_name = self._active_goal.waypoint.name
        self._active_goal.cancel_requested = True
        return CommandResponse(
            accepted=True,
            message="Cancelling navigation to '{}'.".format(goal_name),
            actions=(
                SupervisorAction(
                    action_type=ActionType.CANCEL,
                    reason="user canceled navigation",
                ),
            ),
        )

    def _cancel_exploration(self) -> CommandResponse:
        self._exploration_active = False
        self._empty_frontier_cycles = 0
        if self._active_goal is None:
            return CommandResponse(
                accepted=True,
                message="Exploration mode ended with no active frontier goal.",
            )
        if self._active_goal.cancel_requested:
            return CommandResponse(
                accepted=False,
                message=(
                    "Cancellation for exploration goal '{}' is already in progress.".format(
                        self._active_goal.waypoint.name
                    )
                ),
            )

        goal_name = self._active_goal.waypoint.name
        self._active_goal.cancel_requested = True
        return CommandResponse(
            accepted=True,
            message=(
                "Cancelling exploration goal '{}' and ending exploration mode."
            ).format(goal_name),
            actions=(
                SupervisorAction(
                    action_type=ActionType.CANCEL,
                    reason="user canceled exploration",
                ),
            ),
        )

    def _stop_navigation(self) -> CommandResponse:
        goal_name = self._active_goal.waypoint.name if self._active_goal else None
        if self._active_goal is not None:
            self._active_goal.cancel_requested = True
        if goal_name:
            message = "Stopping navigation to '{}'.".format(goal_name)
        else:
            message = "Issuing stop command."

        return CommandResponse(
            accepted=True,
            message=message,
            actions=(
                SupervisorAction(
                    action_type=ActionType.STOP,
                    reason="user requested stop",
                ),
            ),
        )

    def _stop_exploration(self) -> CommandResponse:
        self._exploration_active = False
        self._empty_frontier_cycles = 0
        goal_name = self._active_goal.waypoint.name if self._active_goal else None
        if self._active_goal is not None:
            self._active_goal.cancel_requested = True
        if goal_name:
            message = (
                "Stopping exploration goal '{}' and exiting exploration mode.".format(
                    goal_name
                )
            )
        else:
            message = "Stopping exploration and exiting exploration mode."

        return CommandResponse(
            accepted=True,
            message=message,
            actions=(
                SupervisorAction(
                    action_type=ActionType.STOP,
                    reason="user requested exploration stop",
                ),
            ),
        )

    def _apply_exploration_result(
        self,
        active_goal: _ActiveGoal,
        result: NavigationResult,
    ) -> CommandResponse:
        waypoint_name = result.waypoint_name or active_goal.waypoint.name

        if result.status == NavigationStatus.SUCCEEDED:
            return CommandResponse(
                accepted=True,
                message=(
                    result.message
                    or "Reached exploration frontier '{}'. Selecting the next frontier.".format(
                        waypoint_name
                    )
                ),
            )

        if result.status == NavigationStatus.CANCELED:
            if self._exploration_active:
                return CommandResponse(
                    accepted=False,
                    message=(
                        result.message
                        or "Exploration frontier '{}' was canceled.".format(
                            waypoint_name
                        )
                    ),
                )
            return CommandResponse(
                accepted=False,
                message=(
                    result.message
                    or "Exploration goal '{}' canceled while exiting exploration mode.".format(
                        waypoint_name
                    )
                ),
            )

        if self._exploration_active:
            self._blacklist_frontier(
                active_goal.frontier_reference_pose or active_goal.waypoint.pose
            )
            return CommandResponse(
                accepted=False,
                message=(
                    "{} Blacklisted the frontier and continuing exploration."
                ).format(
                    result.message
                    or "Exploration goal '{}' failed.".format(waypoint_name)
                ),
            )

        return CommandResponse(
            accepted=False,
            message=(
                result.message or "Exploration goal '{}' failed.".format(waypoint_name)
            ),
        )

    def _unsupported_in_exploration(self, command_type: CommandType) -> CommandResponse:
        return CommandResponse(
            accepted=False,
            message=("Command '{}' is unavailable in exploration mode. {}").format(
                command_type.value, help_text_for_mode(OperatingMode.EXPLORATION)
            ),
        )

    def _blacklist_frontier(self, pose: Pose2D) -> None:
        self._exploration_blacklist.append(Pose2D(x=pose.x, y=pose.y, yaw=0.0))

    def _build_pending_map_save(self) -> PendingMapSave:
        map_id = self._map_save_stem_factory()
        path = str((self._map_output_dir / map_id).expanduser())
        return PendingMapSave(map_id=map_id, path=path)


def _utc_now() -> str:
    return datetime.now(tz=timezone.utc).isoformat()


def _default_map_save_stem() -> str:
    return "explore_{}".format(datetime.now(tz=timezone.utc).strftime("%Y%m%d_%H%M%S"))
