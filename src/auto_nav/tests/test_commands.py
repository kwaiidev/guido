from navigation.commands import HELP_TEXT, parse_nav_command
from navigation.types import CommandType


def test_parse_nav_command_with_waypoint_argument():
    command = parse_nav_command('navigate_to kitchen')

    assert command.command_type == CommandType.NAVIGATE_TO
    assert command.argument == 'kitchen'


def test_parse_nav_command_rejects_missing_argument():
    try:
        parse_nav_command('save_waypoint')
    except ValueError as exc:
        assert 'requires a waypoint name' in str(exc)
    else:
        raise AssertionError('Expected parse_nav_command to reject missing waypoint name')


def test_parse_nav_command_rejects_unknown_command():
    try:
        parse_nav_command('dance now')
    except ValueError as exc:
        assert HELP_TEXT in str(exc)
    else:
        raise AssertionError('Expected parse_nav_command to reject an unknown command')


def test_parse_nav_command_accepts_start_exploration_without_argument():
    command = parse_nav_command('start_exploration')

    assert command.command_type == CommandType.START_EXPLORATION
    assert command.argument is None


def test_parse_nav_command_rejects_argument_for_start_exploration():
    try:
        parse_nav_command('start_exploration now')
    except ValueError as exc:
        assert "does not take an argument" in str(exc)
    else:
        raise AssertionError(
            'Expected parse_nav_command to reject arguments for start_exploration'
        )
