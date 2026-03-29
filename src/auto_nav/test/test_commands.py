import pytest

from navigation.commands import HELP_TEXT
from navigation.commands import parse_nav_command
from navigation.types import CommandType


def test_parse_nav_command_with_waypoint_argument():
    command = parse_nav_command('navigate_to kitchen')
    assert command.command_type == CommandType.NAVIGATE_TO
    assert command.argument == 'kitchen'


def test_parse_nav_command_rejects_missing_argument():
    with pytest.raises(ValueError) as exc:
        parse_nav_command('save_waypoint')
    assert 'requires a waypoint name' in str(exc.value)


def test_parse_nav_command_rejects_unknown_command():
    with pytest.raises(ValueError) as exc:
        parse_nav_command('dance now')
    assert HELP_TEXT in str(exc.value)


def test_parse_nav_command_accepts_start_exploration_without_argument():
    command = parse_nav_command('start_exploration')
    assert command.command_type == CommandType.START_EXPLORATION
    assert command.argument is None


def test_parse_nav_command_rejects_argument_for_start_exploration():
    with pytest.raises(ValueError) as exc:
        parse_nav_command('start_exploration now')
    assert 'does not take an argument' in str(exc.value)
