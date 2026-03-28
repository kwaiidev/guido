from typing import Optional

from .types import CommandType, NavCommand, OperatingMode


HELP_TEXT = (
    "Supported commands: "
    "save_waypoint <name>, navigate_to <name>, list_waypoints, "
    "start_exploration, cancel_navigation, stop, help"
)

EXPLORATION_HELP_TEXT = (
    "Supported commands: start_exploration, cancel_navigation, stop, help"
)

_ARGUMENT_COMMANDS = {
    CommandType.SAVE_WAYPOINT,
    CommandType.NAVIGATE_TO,
}

_NO_ARGUMENT_COMMANDS = {
    CommandType.LIST_WAYPOINTS,
    CommandType.START_EXPLORATION,
    CommandType.CANCEL_NAVIGATION,
    CommandType.STOP,
    CommandType.HELP,
}


def help_text_for_mode(mode: OperatingMode) -> str:
    if mode == OperatingMode.EXPLORATION:
        return EXPLORATION_HELP_TEXT
    return HELP_TEXT


def parse_nav_command(raw_text: Optional[str]) -> NavCommand:
    if raw_text is None:
        raise ValueError("Command is required.")

    text = " ".join(raw_text.strip().split())
    if not text:
        raise ValueError("Command is empty.")

    parts = text.split(" ", 1)
    command_name = parts[0].lower()
    argument = parts[1].strip() if len(parts) > 1 else None

    try:
        command_type = CommandType(command_name)
    except ValueError as exc:
        raise ValueError(
            "Unsupported command '{}'. {}".format(command_name, HELP_TEXT)
        ) from exc

    if command_type in _ARGUMENT_COMMANDS and not argument:
        raise ValueError(
            "Command '{}' requires a waypoint name.".format(command_type.value)
        )

    if command_type in _NO_ARGUMENT_COMMANDS and argument:
        raise ValueError(
            "Command '{}' does not take an argument.".format(command_type.value)
        )

    return NavCommand(command_type=command_type, argument=argument)
