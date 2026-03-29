from __future__ import annotations

from typing import Optional
from typing import Protocol

from navigation.commands import parse_nav_command
from navigation.supervisor import NavigationSupervisor
from navigation.types import ActionType
from navigation.types import AdapterDispatch
from navigation.types import CommandContext
from navigation.types import NavigationRequest
from navigation.types import NavigationResult
from navigation.types import NavigationStatus
from navigation.types import SupervisorAction


class NavigatorClient(Protocol):
    def start_navigation(self, request: NavigationRequest):
        ...

    def cancel_navigation(self, reason: str):
        ...

    def stop_navigation(self, reason: str):
        ...


class CommandBridgeAdapter:
    def __init__(self, supervisor: NavigationSupervisor):
        self._supervisor = supervisor

    def handle_text_command(
        self,
        raw_text: str,
        context: Optional[CommandContext] = None,
    ) -> AdapterDispatch:
        command = parse_nav_command(raw_text)
        response = self._supervisor.handle_command(command, context)
        return self._to_dispatch(response)

    def handle_navigation_result(self, result: NavigationResult) -> AdapterDispatch:
        return self._to_dispatch(self._supervisor.apply_navigation_result(result))

    def plan_exploration(self, current_pose, occupancy_grid) -> AdapterDispatch:
        return self._to_dispatch(self._supervisor.plan_exploration(current_pose, occupancy_grid))

    def tick(self, now: Optional[float] = None) -> AdapterDispatch:
        return self._to_dispatch(self._supervisor.tick(now))

    def _to_dispatch(self, response) -> AdapterDispatch:
        if response is None:
            return AdapterDispatch(accepted=False)
        requests = tuple(
            _action_to_request(action)
            for action in response.actions
        )
        status_messages = tuple(message for message in (response.message,) if message)
        return AdapterDispatch(
            accepted=response.accepted,
            status_messages=status_messages,
            requests=requests,
        )


class NavigationBridgeAdapter:
    def __init__(self, navigator: NavigatorClient):
        self._navigator = navigator

    def handle_request(self, request: NavigationRequest) -> str:
        if request.action_type == ActionType.NAVIGATE:
            self._navigator.start_navigation(request)
            return "Sent navigation goal '{}' to Nav2.".format(
                request.waypoint_name or 'unknown'
            )
        if request.action_type == ActionType.CANCEL:
            self._navigator.cancel_navigation(request.reason or '')
            return 'Forwarded navigation cancel request to Nav2.'
        if request.action_type == ActionType.STOP:
            self._navigator.stop_navigation(request.reason or 'stop requested')
            return 'Forwarded stop request to Nav2.'
        raise ValueError(f'Unsupported navigation action: {request.action_type!r}')

    @staticmethod
    def map_result(status, waypoint_name, detail) -> NavigationResult:
        if isinstance(status, NavigationStatus):
            nav_status = status
        else:
            nav_status = NavigationStatus(str(status))

        target_name = waypoint_name or 'destination'
        if nav_status == NavigationStatus.SUCCEEDED:
            message = detail or "Arrived at '{}'.".format(target_name)
        elif nav_status == NavigationStatus.CANCELED:
            message = detail or "Navigation to '{}' canceled.".format(target_name)
        elif nav_status == NavigationStatus.TIMED_OUT:
            message = detail or "Navigation to '{}' timed out.".format(target_name)
        else:
            message = detail or "Navigation to '{}' failed.".format(target_name)

        return NavigationResult(
            status=nav_status,
            waypoint_name=waypoint_name,
            message=message,
        )


def _action_to_request(action: SupervisorAction) -> NavigationRequest:
    if action.action_type == ActionType.NAVIGATE:
        if action.waypoint is None:
            raise ValueError('Navigate actions require a waypoint.')
        return NavigationRequest(
            action_type=ActionType.NAVIGATE,
            waypoint_name=action.waypoint.name,
            map_id=action.waypoint.map_id,
            pose=action.waypoint.pose,
            reason=action.reason,
        )
    return NavigationRequest(action_type=action.action_type, reason=action.reason)
