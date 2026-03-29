from __future__ import annotations

import os

from google.adk.agents import Agent
from google.adk.agents import SequentialAgent

from .backend import MISSION_STATE
from .backend import consume_pending_ros_command
from .backend import get_robot_status
from .backend import list_destinations
from .backend import lookup_destination
from .backend import queue_cancel_navigation
from .backend import queue_navigation_to_waypoint
from .backend import queue_stop_command
from .backend import remember_current_location


MODEL_NAME = os.getenv('GUIDO_ADK_MODEL', 'gemini-2.5-flash')


intent_agent = Agent(
    name='intent_agent',
    model=MODEL_NAME,
    description='Classifies the user request for waypoint memory or navigation.',
    instruction="""
You are Guido's Intent Agent.

Read the user's latest request and classify it into one of:
- remember
- navigate
- list
- stop
- cancel
- status
- unknown

Rules:
- "remember this as charging dock", "save this as room 2", and "mark this spot as desk"
  are remember.
- "go to room 2", "go back to room 2", "take me to charging dock", and
  "navigate to the desk" are navigate.
- "what places do you know", "list saved places", and "where can you go"
  are list.
- "stop" and "halt" are stop.
- "cancel navigation" and "never mind" are cancel.
- "what's your status" is status.

Output exactly these lines:
intent=<remember|navigate|list|stop|cancel|status|unknown>
target_phrase=<bare waypoint name or none>
needs_lookup=<yes|no>
""".strip(),
    output_key='intent_summary',
)

destination_agent = Agent(
    name='destination_agent',
    model=MODEL_NAME,
    description='Resolves a spoken destination against saved auto_nav waypoints.',
    instruction="""
You are Guido's Destination Agent.

Read the current intent:
{intent_summary}

Rules:
- If intent is navigate, call lookup_destination using the target_phrase.
- For remember, list, stop, cancel, status, and unknown, do not resolve a destination.
- If lookup_destination fails, call list_destinations.

Output exactly these lines:
destination_status=<resolved|unknown|not_needed>
destination_name=<saved waypoint name or none>
available_destinations=<comma separated names or none>
""".strip(),
    tools=[lookup_destination, list_destinations],
    output_key='resolved_destination',
    include_contents='none',
)

mission_planner_agent = Agent(
    name='mission_planner_agent',
    model=MODEL_NAME,
    description='Queues safe auto_nav commands from a resolved intent.',
    instruction="""
You are Guido's Mission Planner Agent.

Use these prior agent results:
Intent:
{intent_summary}

Destination:
{resolved_destination}

Rules:
- Never generate raw motor commands or cmd_vel.
- If intent is remember, call remember_current_location using the target_phrase.
- If intent is navigate and destination_status is resolved, call
  queue_navigation_to_waypoint using the destination_name.
- If intent is list or status, do not queue a ROS command.
- If intent is stop, call queue_stop_command.
- If intent is cancel, call queue_cancel_navigation.
- If the destination is unknown, do not queue a ROS command.

Output exactly these lines:
mission_status=<queued|status_only|destination_unknown|unsupported>
mission_message=<short summary>
""".strip(),
    tools=[
        remember_current_location,
        queue_navigation_to_waypoint,
        queue_stop_command,
        queue_cancel_navigation,
    ],
    output_key='mission_dispatch',
    include_contents='none',
)

status_agent = Agent(
    name='status_agent',
    model=MODEL_NAME,
    description='Returns the final user-facing response for waypoint memory and navigation.',
    instruction="""
You are Guido's Status Agent.

Summarize the final result for the user using:
Intent:
{intent_summary}

Destination:
{resolved_destination}

Mission:
{mission_dispatch}

Rules:
- If mission_status is queued or status_only, call get_robot_status.
- If intent is list, call list_destinations and answer with the saved waypoint names.
- If destination_status is unknown, answer with a short destination error and include
  the available saved waypoint names.
- Keep the final answer short and plain.
- Mention when a command was queued into ROS, not that motion already happened.
""".strip(),
    tools=[get_robot_status, list_destinations],
    include_contents='none',
)

root_agent = SequentialAgent(
    name='guido_mission_agent',
    description=(
        'Guido ADK workflow that remembers named places from the live auto_nav waypoint '
        'store and queues safe ROS navigation commands.'
    ),
    sub_agents=[
        intent_agent,
        destination_agent,
        mission_planner_agent,
        status_agent,
    ],
)


__all__ = [
    'MISSION_STATE',
    'consume_pending_ros_command',
    'root_agent',
]
