from __future__ import annotations

import difflib
import os
from pathlib import Path
import re
import sys
from typing import Any
from uuid import uuid4


REPO_ROOT = Path(__file__).resolve().parents[2]
AUTO_NAV_ROOT = REPO_ROOT / 'src' / 'auto_nav'
if str(AUTO_NAV_ROOT) not in sys.path:
    sys.path.insert(0, str(AUTO_NAV_ROOT))

from navigation.waypoints import WaypointStore


MISSION_STATE: dict[str, Any] = {
    'status': 'idle',
    'active_mission': None,
    'last_message': 'No waypoint command has been queued yet.',
    'pending_ros_command': None,
}

_LEADING_ARTICLES = ('the ', 'a ', 'an ', 'my ', 'our ')
_FILLER_PHRASES = (
    'go back to ',
    'take me to ',
    'bring me to ',
    'drive me to ',
    'head to ',
    'navigate to ',
    'go to ',
)


def list_destinations() -> dict[str, Any]:
    waypoints = _load_saved_waypoints()
    return {
        'destinations': [waypoint['name'] for waypoint in waypoints],
        'count': len(waypoints),
        'waypoint_file': str(_waypoint_file_path()),
    }


def lookup_destination(name: str) -> dict[str, Any]:
    query = _clean_lookup_phrase(name)
    waypoints = _load_saved_waypoints()
    available_destinations = [waypoint['name'] for waypoint in waypoints]

    if not query:
        return {
            'found': False,
            'query': name,
            'available_destinations': available_destinations,
            'reason': 'empty_query',
        }

    best_match = None
    best_score = 0.0
    for waypoint in waypoints:
        score = _waypoint_match_score(query, waypoint['normalized_name'])
        if score > best_score:
            best_score = score
            best_match = waypoint

    if best_match is None or best_score < 0.72:
        return {
            'found': False,
            'query': name,
            'available_destinations': available_destinations,
            'reason': 'no_saved_match',
        }

    pose = best_match['pose']
    return {
        'found': True,
        'name': best_match['name'],
        'goal_id': best_match['name'],
        'mission_type': 'waypoint_goal',
        'map_id': best_match['map_id'],
        'created_at': best_match['created_at'],
        'goal_pose': {
            'x': pose['x'],
            'y': pose['y'],
            'theta': pose['yaw'],
        },
        'match_score': round(best_score, 3),
    }


def remember_current_location(name: str) -> dict[str, Any]:
    waypoint_name = _clean_waypoint_name(name)
    if not waypoint_name:
        return {'accepted': False, 'message': 'A waypoint name is required.'}

    existing = lookup_destination(waypoint_name)
    if existing.get('found') and _normalize_lookup_text(existing['name']) == _normalize_lookup_text(
        waypoint_name
    ):
        return {
            'accepted': False,
            'message': f"Waypoint '{existing['name']}' already exists.",
            'waypoint_name': existing['name'],
        }

    command = f'save_waypoint {waypoint_name}'
    mission = {
        'mission_id': f"mission-{uuid4().hex[:12]}",
        'mission_type': 'save_waypoint',
        'goal_id': waypoint_name,
    }
    return _queue_ros_command(
        command,
        status='queued_save',
        mission=mission,
        message=f"Queued save for the current pose as '{waypoint_name}'.",
    )


def queue_navigation_to_waypoint(name: str) -> dict[str, Any]:
    match = lookup_destination(name)
    if not match.get('found'):
        return {
            'accepted': False,
            'message': f"No saved waypoint matches '{name}'.",
            'available_destinations': match.get('available_destinations', []),
        }

    destination_name = match['name']
    mission = {
        'mission_id': f"mission-{uuid4().hex[:12]}",
        'mission_type': 'waypoint_goal',
        'goal_id': destination_name,
        'goal_pose': match.get('goal_pose'),
    }
    return _queue_ros_command(
        f'navigate_to {destination_name}',
        status='queued',
        mission=mission,
        message=f"Queued navigation to '{destination_name}'.",
    )


def queue_stop_command() -> dict[str, Any]:
    return _queue_ros_command(
        'stop',
        status='stopped',
        mission=None,
        message='Queued stop command.',
    )


def queue_cancel_navigation() -> dict[str, Any]:
    return _queue_ros_command(
        'cancel_navigation',
        status='cancel_requested',
        mission=None,
        message='Queued navigation cancel command.',
    )


def get_robot_status() -> dict[str, Any]:
    waypoint_summary = list_destinations()
    return {
        'status': MISSION_STATE['status'],
        'active_mission': MISSION_STATE['active_mission'],
        'last_message': MISSION_STATE['last_message'],
        'pending_ros_command': MISSION_STATE['pending_ros_command'],
        'saved_destinations': waypoint_summary['destinations'],
        'saved_destination_count': waypoint_summary['count'],
        'waypoint_file': waypoint_summary['waypoint_file'],
        'backend': 'auto_nav_waypoint_store',
    }


def consume_pending_ros_command() -> str | None:
    command = MISSION_STATE.get('pending_ros_command')
    MISSION_STATE['pending_ros_command'] = None
    return command


def _queue_ros_command(
    command: str,
    *,
    status: str,
    mission: dict[str, Any] | None,
    message: str,
) -> dict[str, Any]:
    MISSION_STATE['status'] = status
    MISSION_STATE['active_mission'] = mission
    MISSION_STATE['last_message'] = message
    MISSION_STATE['pending_ros_command'] = command
    return {
        'accepted': True,
        'message': message,
        'mission': mission,
        'ros_command': command,
        'backend': 'auto_nav_waypoint_store',
    }


def _load_saved_waypoints() -> list[dict[str, Any]]:
    store = WaypointStore(_waypoint_file_path())
    return [
        {
            'name': waypoint.name,
            'normalized_name': _normalize_lookup_text(waypoint.name),
            'map_id': waypoint.map_id,
            'pose': waypoint.pose.as_dict(),
            'created_at': waypoint.created_at,
        }
        for waypoint in store.list_waypoints()
    ]


def _waypoint_file_path() -> Path:
    configured = os.getenv('GUIDO_WAYPOINT_FILE') or os.getenv('AUTO_NAV_WAYPOINT_FILE')
    if configured:
        return Path(configured).expanduser()
    return REPO_ROOT / '.guido' / 'waypoints.yaml'


def _clean_waypoint_name(name: str) -> str:
    cleaned = ' '.join((name or '').strip().split())
    cleaned = cleaned.strip("\"'`.,!?")
    return cleaned


def _clean_lookup_phrase(name: str) -> str:
    cleaned = _clean_waypoint_name(name).casefold()
    for filler in _FILLER_PHRASES:
        if cleaned.startswith(filler):
            cleaned = cleaned[len(filler) :].strip()
            break
    return _normalize_lookup_text(cleaned)


def _normalize_lookup_text(text: str) -> str:
    normalized = (text or '').casefold()
    normalized = normalized.replace('_', ' ').replace('-', ' ')
    normalized = re.sub(r'[^a-z0-9\s]', ' ', normalized)
    normalized = re.sub(r'\s+', ' ', normalized).strip()
    for article in _LEADING_ARTICLES:
        if normalized.startswith(article):
            normalized = normalized[len(article) :].strip()
            break
    return normalized


def _waypoint_match_score(query: str, candidate: str) -> float:
    if not query or not candidate:
        return 0.0
    if query == candidate:
        return 1.0
    if candidate.startswith(query) or query.startswith(candidate):
        return 0.92

    query_tokens = set(query.split())
    candidate_tokens = set(candidate.split())
    if query_tokens and query_tokens == candidate_tokens:
        return 0.9
    if query_tokens and query_tokens.issubset(candidate_tokens):
        overlap = len(query_tokens) / max(len(candidate_tokens), 1)
        return 0.82 + (0.08 * overlap)

    return difflib.SequenceMatcher(a=query, b=candidate).ratio()
