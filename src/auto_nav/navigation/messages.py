import json
from typing import Any, Dict

from .types import (
    ActionType,
    NavigationRequest,
    NavigationResult,
    NavigationStatus,
    Pose2D,
)


def encode_navigation_request(request: NavigationRequest) -> str:
    payload = {
        "action_type": request.action_type.value,
        "waypoint_name": request.waypoint_name,
        "map_id": request.map_id,
        "reason": request.reason,
        "pose": request.pose.as_dict() if request.pose else None,
    }
    return json.dumps(payload, sort_keys=True)


def decode_navigation_request(payload: str) -> NavigationRequest:
    data = _load_json(payload)
    pose_data = data.get("pose")
    pose = None
    if pose_data is not None:
        pose = Pose2D(
            x=float(pose_data["x"]),
            y=float(pose_data["y"]),
            yaw=float(pose_data["yaw"]),
        )

    return NavigationRequest(
        action_type=ActionType(data["action_type"]),
        waypoint_name=data.get("waypoint_name"),
        map_id=data.get("map_id"),
        pose=pose,
        reason=data.get("reason"),
    )


def encode_navigation_result(result: NavigationResult) -> str:
    payload = {
        "status": result.status.value,
        "waypoint_name": result.waypoint_name,
        "message": result.message,
    }
    return json.dumps(payload, sort_keys=True)


def decode_navigation_result(payload: str) -> NavigationResult:
    data = _load_json(payload)
    return NavigationResult(
        status=NavigationStatus(data["status"]),
        waypoint_name=data.get("waypoint_name"),
        message=data.get("message", ""),
    )


def _load_json(payload: str) -> Dict[str, Any]:
    data = json.loads(payload)
    if not isinstance(data, dict):
        raise ValueError("Payload must be a JSON object.")
    return data
