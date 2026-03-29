from navigation.types import CommandType
from navigation.types import NavCommand
from navigation.types import OperatingMode


HELP_TEXT = (
    'Supported commands: save_waypoint <name>, navigate_to <name>, '
    'list_waypoints, start_exploration, cancel_navigation, stop, help'
)
EXPLORATION_HELP_TEXT = (
    'Supported commands: start_exploration, cancel_navigation, stop, help'
)

_ARGUMENT_COMMANDS = {
    CommandType.SAVE_WAYPOINT.value: CommandType.SAVE_WAYPOINT,
    CommandType.NAVIGATE_TO.value: CommandType.NAVIGATE_TO,
}
_NO_ARGUMENT_COMMANDS = {
    CommandType.LIST_WAYPOINTS.value: CommandType.LIST_WAYPOINTS,
    CommandType.START_EXPLORATION.value: CommandType.START_EXPLORATION,
    CommandType.CANCEL_NAVIGATION.value: CommandType.CANCEL_NAVIGATION,
    CommandType.STOP.value: CommandType.STOP,
    CommandType.HELP.value: CommandType.HELP,
}


def help_text_for_mode(mode) -> str:
    if OperatingMode(mode) == OperatingMode.EXPLORATION:
        return EXPLORATION_HELP_TEXT
    return HELP_TEXT


def parse_nav_command(raw_text: str) -> NavCommand:
    if raw_text is None:
        raise ValueError('Command is required.')

    normalized = ' '.join(raw_text.strip().split())
    if not normalized:
        raise ValueError('Command is empty.')

    parts = normalized.lower().split(' ', 1)
    command_name = parts[0]
    argument = parts[1].strip() if len(parts) > 1 else None

    if command_name in _ARGUMENT_COMMANDS:
        if not argument:
            raise ValueError(f"Command '{command_name}' requires a waypoint name.")
        return NavCommand(command_type=_ARGUMENT_COMMANDS[command_name], argument=argument)

    if command_name in _NO_ARGUMENT_COMMANDS:
        if argument:
            raise ValueError(f"Command '{command_name}' does not take an argument.")
        return NavCommand(command_type=_NO_ARGUMENT_COMMANDS[command_name])

    raise ValueError(f"Unsupported command '{command_name}'. {HELP_TEXT}")
