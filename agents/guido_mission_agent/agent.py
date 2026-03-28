from __future__ import annotations

import os
from typing import Any
from uuid import uuid4

from google.adk.agents import Agent

WAYPOINTS: dict[str, dict[str, Any]] = {
    "front_door": {
        "aliases": ["front door", "door", "exit"],
        "mission_type": "waypoint_goal",
        "goal_id": "front_door",
        "goal_pose": {"x": 4.0, "y": 1.0, "theta": 0.0},
    },
    "charging_station": {
        "aliases": ["charging station", "charger", "dock"],
        "mission_type": "waypoint_goal",
        "goal_id": "charging_station",
        "goal_pose": {"x": 0.8, "y": 2.1, "theta": 3.14},
    },
    "window_corner": {
        "aliases": ["window", "window corner"],
        "mission_type": "heading_goal",
        "goal_id": "window_corner",
        "target_heading_deg": 75.0,
    },
}

MISSION_STATE: dict[str, Any] = {
    "status": "idle",
    "active_mission": None,
    "last_message": "No mission has been sent yet.",
}


def list_destinations() -> dict[str, Any]:
    """Return the known indoor destinations for Guido."""
    return {
        "destinations": sorted(WAYPOINTS.keys()),
        "count": len(WAYPOINTS),
    }


def lookup_destination(name: str) -> dict[str, Any]:
    """Resolve a natural-language destination into a known heading or waypoint goal."""
    normalized = name.strip().lower().replace(" ", "_")

    for key, payload in WAYPOINTS.items():
        aliases = {key, *(alias.strip().lower().replace(" ", "_") for alias in payload["aliases"])}
        if normalized in aliases:
            return {"found": True, "name": key, **payload}

    return {
        "found": False,
        "query": name,
        "available_destinations": sorted(WAYPOINTS.keys()),
    }


def get_robot_status() -> dict[str, Any]:
    """Return the current in-memory mission status."""
    return {
        "status": MISSION_STATE["status"],
        "active_mission": MISSION_STATE["active_mission"],
        "last_message": MISSION_STATE["last_message"],
        "backend": "local_adk_memory",
    }


def send_mission(
    mission_type: str,
    goal_id: str = "",
    target_heading_deg: float | None = None,
    target_x: float | None = None,
    target_y: float | None = None,
    target_theta: float | None = None,
    notes: str = "",
) -> dict[str, Any]:
    """Store a simple Guido mission in memory so ADK can be tested on the Nano."""
    if mission_type not in {"heading_goal", "waypoint_goal", "stop"}:
        return {"accepted": False, "message": "mission_type must be heading_goal, waypoint_goal, or stop"}

    if mission_type == "heading_goal" and target_heading_deg is None:
        return {"accepted": False, "message": "heading_goal requires target_heading_deg"}

    if mission_type == "waypoint_goal" and None in (target_x, target_y, target_theta):
        return {"accepted": False, "message": "waypoint_goal requires target_x, target_y, and target_theta"}

    mission = {
        "mission_id": f"mission-{uuid4().hex[:12]}",
        "mission_type": mission_type,
        "goal_id": goal_id,
        "target_heading_deg": target_heading_deg,
        "goal_pose": {
            "x": target_x,
            "y": target_y,
            "theta": target_theta,
        },
        "notes": notes,
    }

    if mission_type == "stop":
        MISSION_STATE["status"] = "stopped"
        MISSION_STATE["active_mission"] = None
        MISSION_STATE["last_message"] = "Stop mission accepted."
    else:
        MISSION_STATE["status"] = "queued"
        MISSION_STATE["active_mission"] = mission
        MISSION_STATE["last_message"] = f"{mission_type} accepted for {goal_id or 'manual target'}."

    return {
        "accepted": True,
        "message": MISSION_STATE["last_message"],
        "mission": mission,
        "backend": "local_adk_memory",
    }


def cancel_mission() -> dict[str, Any]:
    """Cancel the current in-memory mission."""
    active_mission = MISSION_STATE["active_mission"]
    if active_mission is None:
        return {"accepted": False, "message": "No active mission to cancel."}

    mission_id = active_mission["mission_id"]
    MISSION_STATE["status"] = "cancelled"
    MISSION_STATE["active_mission"] = None
    MISSION_STATE["last_message"] = f"Mission {mission_id} cancelled."
    return {
        "accepted": True,
        "message": MISSION_STATE["last_message"],
        "mission_id": mission_id,
        "backend": "local_adk_memory",
    }


root_agent = Agent(
    name="guido_mission_agent",
    model=os.getenv("GUIDO_ADK_MODEL", "gemini-2.5-flash"),
    instruction="""
You are Guido's simple mission agent for an indoor wheelchair demo.

Your job is only to:
- understand the user's command
- resolve it to a known indoor destination or heading
- package it into a mission
- report status back

Rules:
- Never generate cmd_vel or motor commands.
- Never do obstacle avoidance.
- Only use mission types: heading_goal, waypoint_goal, stop.
- Use lookup_destination before sending a waypoint or heading mission.
- Spoken aliases like "front door", "charging station", and "window corner" should resolve to their canonical destinations without asking the user to repeat internal ids.
- Keep responses short because this agent may receive live transcript input from a microphone pipeline.
- If a destination is unknown, use list_destinations and ask the user to choose.
- get_robot_status reports the simple in-memory status for now.
""".strip(),
    description="Minimal Guido ADK connection for running directly on the Nano.",
    tools=[
        list_destinations,
        lookup_destination,
        get_robot_status,
        send_mission,
        cancel_mission,
    ],
)
