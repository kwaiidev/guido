from dataclasses import dataclass
import importlib.util
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'start_frontier_mapping.py'


def _load_launcher_module():
    spec = importlib.util.spec_from_file_location('start_frontier_mapping', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    sys.modules.setdefault(spec.name, module)
    spec.loader.exec_module(module)
    return module


launcher = _load_launcher_module()


@dataclass
class _FakeResult:
    returncode: int
    stdout: str = ''
    stderr: str = ''


class _FakeProcess:
    def __init__(self, poll_results):
        self._poll_results = list(poll_results)
        self._index = 0

    def poll(self):
        if self._index >= len(self._poll_results):
            return self._poll_results[-1]
        value = self._poll_results[self._index]
        self._index += 1
        return value


class _FakeProbe:
    def __init__(self, snapshots):
        self._snapshots = list(snapshots)
        self._index = 0
        self.commands = []

    def spin_once(self, _timeout_sec: float) -> None:
        if self._index < len(self._snapshots) - 1:
            self._index += 1

    def snapshot(self):
        return self._snapshots[self._index]

    def publish_command(self, command: str) -> None:
        self.commands.append(command)

    def close(self) -> None:
        return None


def _ready_snapshot(**overrides):
    payload = {
        'command_topic_ready': True,
        'map_saver_ready': True,
        'navigation_action_ready': True,
        'scan_received': True,
        'odom_received': True,
        'map_received': True,
        'tf_ready': True,
    }
    payload.update(overrides)
    return launcher.ReadinessSnapshot(**payload)


def test_load_ros_parameter_block_supports_wildcard_root(tmp_path):
    config = tmp_path / 'ldlidar.yaml'
    config.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    comm:\n"
        "      serial_port: /dev/ldlidar\n",
        encoding='utf-8',
    )

    parameters = launcher.load_ros_parameter_block(config)
    assert parameters['comm']['serial_port'] == '/dev/ldlidar'


def test_load_device_paths_reads_repo_style_configs(tmp_path):
    serial_config = tmp_path / 'src' / 'guido_base' / 'config'
    lidar_config = tmp_path / 'src' / 'guido_bringup' / 'config'
    serial_config.mkdir(parents=True)
    lidar_config.mkdir(parents=True)

    (serial_config / 'serial_bridge.yaml').write_text(
        "guido_serial_bridge:\n"
        "  ros__parameters:\n"
        "    serial_port: /dev/ttyUSB0\n",
        encoding='utf-8',
    )
    (lidar_config / 'ldlidar.yaml').write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    comm:\n"
        "      serial_port: /dev/ldlidar\n",
        encoding='utf-8',
    )

    device_paths = launcher.load_device_paths(tmp_path)
    assert device_paths.serial_port == '/dev/ttyUSB0'
    assert device_paths.lidar_port == '/dev/ldlidar'


def test_check_ros_package_reports_failure_with_fix_hint():
    result = launcher.check_ros_package(
        'nav2_bringup',
        '/usr/bin/ros2',
        runner=lambda *_args, **_kwargs: _FakeResult(
            returncode=1,
            stderr='package not found',
        ),
    )

    assert not result.ok
    assert 'package not found' in result.detail
    assert 'source /opt/ros/jazzy/setup.bash' in result.fix_hint


def test_check_device_path_reports_missing_device():
    result = launcher.check_device_path(
        '/definitely/not/here',
        label='Arduino serial device',
        fix_hint='reconnect hardware',
    )

    assert not result.ok
    assert 'Missing device path' in result.detail
    assert result.fix_hint == 'reconnect hardware'


def test_wait_for_phase_returns_when_probe_becomes_ready():
    probe = _FakeProbe(
        [
            _ready_snapshot(command_topic_ready=False, map_saver_ready=False),
            _ready_snapshot(command_topic_ready=True, map_saver_ready=True),
        ]
    )
    process = _FakeProcess([None, None, None])
    timestamps = iter([0.0, 0.2, 0.4])

    snapshot = launcher.wait_for_phase(
        probe,
        process,
        timeout=1.0,
        phase_name='launch interface readiness',
        predicate=lambda current: current.launch_ready(),
        missing_checks=lambda current: current.missing_launch_checks(),
        time_fn=lambda: next(timestamps),
    )

    assert snapshot.launch_ready()


def test_wait_for_phase_reports_missing_navigation_action():
    probe = _FakeProbe(
        [
            _ready_snapshot(navigation_action_ready=False),
            _ready_snapshot(navigation_action_ready=False),
        ]
    )
    process = _FakeProcess([None, None, None])
    timestamps = iter([0.0, 0.2, 0.4, 1.2])

    with pytest.raises(launcher.LaunchError) as exc:
        launcher.wait_for_phase(
            probe,
            process,
            timeout=1.0,
            phase_name='launch interface readiness',
            predicate=lambda current: current.launch_ready(),
            missing_checks=lambda current: current.missing_launch_checks(),
            time_fn=lambda: next(timestamps),
        )

    assert 'Nav2 navigate_to_pose action' in str(exc.value)


def test_wait_for_phase_reports_missing_lidar_messages():
    probe = _FakeProbe(
        [
            _ready_snapshot(scan_received=False),
            _ready_snapshot(scan_received=False),
        ]
    )
    process = _FakeProcess([None, None, None])
    timestamps = iter([0.0, 0.2, 0.5, 1.3])

    with pytest.raises(launcher.LaunchError) as exc:
        launcher.wait_for_phase(
            probe,
            process,
            timeout=1.0,
            phase_name='sensor and map health',
            predicate=lambda current: current.health_ready(),
            missing_checks=lambda current: current.missing_health_checks(),
            time_fn=lambda: next(timestamps),
        )

    assert '/ldlidar_node/scan messages' in str(exc.value)


def test_wait_for_phase_raises_when_launch_process_dies_early():
    probe = _FakeProbe([_ready_snapshot(command_topic_ready=False)])
    process = _FakeProcess([None, 23])
    timestamps = iter([0.0, 0.4, 0.8, 0.9])

    with pytest.raises(launcher.LaunchError) as exc:
        launcher.wait_for_phase(
            probe,
            process,
            timeout=1.0,
            phase_name='launch interface readiness',
            predicate=lambda current: current.launch_ready(),
            missing_checks=lambda current: current.missing_launch_checks(),
            time_fn=lambda: next(timestamps),
        )

    assert 'code 23' in str(exc.value)
