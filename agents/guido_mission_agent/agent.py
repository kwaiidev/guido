from __future__ import annotations

import os
from typing import Any
from uuid import uuid4

from google.adk.agents import Agent
from google.adk.agents import SequentialAgent

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
MODEL_NAME = os.getenv("GUIDO_ADK_MODEL", "gemini-2.5-flash")


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


intent_agent = Agent(
    name="intent_agent",
    model=MODEL_NAME,
    description="Classifies the user's latest wheelchair request.",
    instruction="""
You are Guido's Intent Agent.

Read the user's latest command and classify it into one of:
- navigate
- stop
- status
- unknown

Treat phrases like "take me", "go to", "turn toward", "head to", or "navigate to"
as navigate.
Treat phrases like "stop", "cancel", or "halt" as stop.
Treat phrases like "what's my status" or "where are you" as status.

Output exactly these four lines:
intent=<navigate|stop|status|unknown>
target_phrase=<destination phrase or none>
needs_destination=<yes|no>
needs_status=<yes|no>
""".strip(),
    output_key="intent_summary",
)

destination_agent = Agent(
    name="destination_agent",
    model=MODEL_NAME,
    description="Resolves named indoor destinations into known goals.",
    instruction="""
You are Guido's Destination Agent.

Read the current mission intent from:
{intent_summary}

Rules:
- If intent is not navigate, do not resolve a destination.
- If intent is navigate, call lookup_destination using the target_phrase.
- Spoken aliases like "front door", "charging station", and "window corner"
  should resolve to the canonical destination without asking for the internal id.
- If lookup_destination fails, call list_destinations.

Output exactly these lines:
destination_status=<resolved|unknown|not_needed>
destination_name=<canonical name or none>
mission_type=<heading_goal|waypoint_goal|none>
goal_id=<goal id or none>
target_x=<number or none>
target_y=<number or none>
target_theta=<number or none>
target_heading_deg=<number or none>
available_destinations=<comma separated names or none>
""".strip(),
    tools=[lookup_destination, list_destinations],
    output_key="resolved_destination",
    include_contents="none",
)

mission_planner_agent = Agent(
    name="mission_planner_agent",
    model=MODEL_NAME,
    description="Packages a resolved request into a Guido mission.",
    instruction="""
You are Guido's Mission Planner Agent.

Use these prior agent results:
Intent:
{intent_summary}

Destination:
{resolved_destination}

Rules:
- Never generate cmd_vel or low-level motor commands.
- Only use mission types: heading_goal, waypoint_goal, stop.
- If intent is stop, call send_mission with mission_type='stop'.
- If intent is status, do not send a mission.
- If intent is navigate and destination_status is resolved, call send_mission with
  the resolved mission type and target fields.
- If the destination is unknown, do not send a mission.

Output exactly these lines:
mission_status=<accepted|status_only|destination_unknown|unsupported>
mission_message=<short summary>
""".strip(),
    tools=[send_mission, cancel_mission],
    output_key="mission_dispatch",
    include_contents="none",
)

status_agent = Agent(
    name="status_agent",
    model=MODEL_NAME,
    description="Returns the final user-facing wheelchair mission response.",
    instruction="""
You are Guido's Status Agent.

Summarize the final result for the user using these prior agent results:
Intent:
{intent_summary}

Destination:
{resolved_destination}

Mission:
{mission_dispatch}

Rules:
- If mission_status is accepted or status_only, call get_robot_status.
- If destination_status is unknown, respond with a short destination error and
  include the available destinations.
- Keep the final answer short and plain.
- Do not mention internal agent names or state keys.
""".strip(),
    tools=[get_robot_status],
    include_contents="none",
)

root_agent = SequentialAgent(
    name="guido_mission_agent",
    description="Minimal Guido ADK mission workforce for command understanding, destination resolution, mission planning, and status reporting.",
    sub_agents=[
        intent_agent,
        destination_agent,
        mission_planner_agent,
        status_agent,
    ],
)
