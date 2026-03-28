from typing import Optional, Protocol

from .commands import parse_nav_command
from .supervisor import NavigationSupervisor
from .types import (
    ActionType,
    AdapterDispatch,
    CommandContext,
    CommandResponse,
    OccupancyGridSnapshot,
    NavigationRequest,
    NavigationResult,
    NavigationStatus,
    SupervisorAction,
)


class NavigatorClient(Protocol):
    def start_navigation(self, request: NavigationRequest) -> None: ...

    def cancel_navigation(self, reason: str = "") -> None: ...

    def stop_navigation(self, reason: str = "") -> None: ...


class CommandBridgeAdapter:
    def __init__(self, supervisor: NavigationSupervisor):
        self._supervisor = supervisor

    def handle_text_command(
        self,
        raw_text: str,
        context: Optional[CommandContext] = None,
    ) -> AdapterDispatch:
        try:
            command = parse_nav_command(raw_text)
        except ValueError as exc:
            return AdapterDispatch(
                accepted=False,
                status_messages=(str(exc),),
            )

        response = self._supervisor.handle_command(command, context=context)
        return self._to_dispatch(response)

    def handle_navigation_result(self, result: NavigationResult) -> AdapterDispatch:
        return self._to_dispatch(self._supervisor.apply_navigation_result(result))

    def plan_exploration(
        self,
        current_pose,
        occupancy_grid: Optional[OccupancyGridSnapshot],
    ) -> Optional[AdapterDispatch]:
        response = self._supervisor.plan_exploration(
            current_pose=current_pose,
            occupancy_grid=occupancy_grid,
        )
        if response is None:
            return None
        return self._to_dispatch(response)

    def tick(self, now: Optional[float] = None) -> Optional[AdapterDispatch]:
        response = self._supervisor.tick(now=now)
        if response is None:
            return None
        return self._to_dispatch(response)

    def _to_dispatch(self, response: CommandResponse) -> AdapterDispatch:
        requests = tuple(_action_to_request(action) for action in response.actions)
        messages = (response.message,) if response.message else ()
        return AdapterDispatch(
            accepted=response.accepted,
            status_messages=messages,
            requests=requests,
        )


class NavigationBridgeAdapter:
    def __init__(self, navigator: NavigatorClient):
        self._navigator = navigator

    def handle_request(self, request: NavigationRequest) -> str:
        if request.action_type == ActionType.NAVIGATE:
            self._navigator.start_navigation(request)
            return "Sent navigation goal '{}' to Nav2.".format(
                request.waypoint_name or "unknown"
            )

        if request.action_type == ActionType.CANCEL:
            self._navigator.cancel_navigation(reason=request.reason or "")
            return "Forwarded navigation cancel request to Nav2."

        self._navigator.stop_navigation(reason=request.reason or "")
        return "Forwarded stop request to Nav2."

    @staticmethod
    def map_result(
        status: str,
        waypoint_name: str = None,
        detail: str = "",
    ) -> NavigationResult:
        normalized_status = status.strip().lower()
        if normalized_status == NavigationStatus.SUCCEEDED.value:
            message = detail or "Arrived at '{}'.".format(
                waypoint_name or "destination"
            )
            return NavigationResult(
                status=NavigationStatus.SUCCEEDED,
                waypoint_name=waypoint_name,
                message=message,
            )
        if normalized_status == NavigationStatus.CANCELED.value:
            message = detail or "Navigation to '{}' canceled.".format(
                waypoint_name or "destination"
            )
            return NavigationResult(
                status=NavigationStatus.CANCELED,
                waypoint_name=waypoint_name,
                message=message,
            )
        if normalized_status == NavigationStatus.TIMED_OUT.value:
            message = detail or "Navigation to '{}' timed out.".format(
                waypoint_name or "destination"
            )
            return NavigationResult(
                status=NavigationStatus.TIMED_OUT,
                waypoint_name=waypoint_name,
                message=message,
            )

        message = detail or "Navigation to '{}' failed.".format(
            waypoint_name or "destination"
        )
        return NavigationResult(
            status=NavigationStatus.FAILED,
            waypoint_name=waypoint_name,
            message=message,
        )


def _action_to_request(action: SupervisorAction) -> NavigationRequest:
    if action.action_type == ActionType.NAVIGATE:
        if action.waypoint is None:
            raise ValueError("Navigate actions require a waypoint.")
        return NavigationRequest(
            action_type=action.action_type,
            waypoint_name=action.waypoint.name,
            map_id=action.waypoint.map_id,
            pose=action.waypoint.pose,
            reason=action.reason,
        )

    return NavigationRequest(
        action_type=action.action_type,
        waypoint_name=action.waypoint.name if action.waypoint else None,
        map_id=action.waypoint.map_id if action.waypoint else None,
        pose=action.waypoint.pose if action.waypoint else None,
        reason=action.reason,
    )
