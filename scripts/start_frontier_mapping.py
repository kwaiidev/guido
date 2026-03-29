#!/usr/bin/env python3
"""Single-command frontier exploration bringup for the Guido wheelchair."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Any
from typing import Callable
from typing import Protocol

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
GUIDO_RUNTIME_DIR = REPO_ROOT / '.guido'
DEFAULT_LOG_FILE = GUIDO_RUNTIME_DIR / 'logs' / 'frontier_mapping.log'
REQUIRED_ROS_PACKAGES = ('auto_nav', 'guido_bringup', 'nav2_bringup')
PACKAGE_FIX_HINT = (
    'Source ROS 2 and the workspace, then rebuild if needed: '
    'source /opt/ros/jazzy/setup.bash && source install/local_setup.bash'
)
SUBMODULE_FIX_HINT = 'git submodule update --init --recursive src/ldrobot-lidar-ros2'
LIDAR_FIX_HINT = (
    'Reconnect the LD19 and verify udev rules: ./scripts/setup_lidar.sh, then check /dev/ldlidar'
)
SERIAL_FIX_HINT = (
    'Reconnect the Arduino/motor controller and update src/guido_base/config/serial_bridge.yaml '
    'if the serial path changed'
)
ROS_FIX_HINT = (
    'Source ROS 2 and the workspace before launching: '
    'source /opt/ros/jazzy/setup.bash && source install/local_setup.bash'
)


class LaunchError(RuntimeError):
    """Raised when the runtime launcher cannot safely continue."""


@dataclass(frozen=True)
class CheckResult:
    name: str
    ok: bool
    detail: str
    fix_hint: str | None = None


@dataclass(frozen=True)
class DevicePaths:
    serial_port: str
    lidar_port: str


@dataclass(frozen=True)
class ReadinessSnapshot:
    command_topic_ready: bool
    map_saver_ready: bool
    navigation_action_ready: bool
    scan_received: bool
    odom_received: bool
    map_received: bool
    tf_ready: bool

    def launch_ready(self) -> bool:
        return (
            self.command_topic_ready
            and self.map_saver_ready
            and self.navigation_action_ready
        )

    def health_ready(self) -> bool:
        return (
            self.scan_received
            and self.odom_received
            and self.map_received
            and self.tf_ready
        )

    def fully_ready(self) -> bool:
        return self.launch_ready() and self.health_ready()

    def missing_launch_checks(self) -> tuple[str, ...]:
        missing = []
        if not self.command_topic_ready:
            missing.append('/auto_nav/command subscriber')
        if not self.map_saver_ready:
            missing.append('/map_saver/save_map service')
        if not self.navigation_action_ready:
            missing.append('Nav2 navigate_to_pose action')
        return tuple(missing)

    def missing_health_checks(self) -> tuple[str, ...]:
        missing = []
        if not self.scan_received:
            missing.append('/ldlidar_node/scan messages')
        if not self.odom_received:
            missing.append('/odom messages')
        if not self.map_received:
            missing.append('/map messages')
        if not self.tf_ready:
            missing.append('map -> base_footprint/base_link TF')
        return tuple(missing)


class ReadinessProbe(Protocol):
    def spin_once(self, timeout_sec: float) -> None:
        ...

    def snapshot(self) -> ReadinessSnapshot:
        ...

    def publish_command(self, command: str) -> None:
        ...

    def close(self) -> None:
        ...


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--no-auto-start',
        action='store_true',
        help='Launch and verify the stack, but do not publish start_exploration.',
    )
    parser.add_argument(
        '--launch-timeout',
        type=float,
        default=20.0,
        help='Seconds to wait for command/service/action interfaces to come online.',
    )
    parser.add_argument(
        '--health-timeout',
        type=float,
        default=30.0,
        help='Seconds to wait for scan, odom, map, and TF health after launch interfaces are ready.',
    )
    parser.add_argument(
        '--log-file',
        type=Path,
        default=DEFAULT_LOG_FILE,
        help='Path to the combined launch log file.',
    )
    parser.add_argument(
        '--dry-run-checks',
        action='store_true',
        help='Run preflight checks only and exit without launching ROS.',
    )
    return parser.parse_args(argv)


def load_ros_parameter_block(config_path: Path) -> dict[str, Any]:
    try:
        payload = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    except FileNotFoundError as exc:
        raise LaunchError(f'Missing config file: {config_path}') from exc
    except yaml.YAMLError as exc:
        raise LaunchError(f'Unable to parse YAML config: {config_path}') from exc

    if not isinstance(payload, dict):
        raise LaunchError(f'Unexpected config structure in {config_path}')

    for value in payload.values():
        if not isinstance(value, dict):
            continue
        parameters = value.get('ros__parameters')
        if isinstance(parameters, dict):
            return parameters

    raise LaunchError(f'No ros__parameters block found in {config_path}')


def load_device_paths(repo_root: Path) -> DevicePaths:
    serial_config = repo_root / 'src' / 'guido_base' / 'config' / 'serial_bridge.yaml'
    lidar_config = repo_root / 'src' / 'guido_bringup' / 'config' / 'ldlidar.yaml'

    serial_parameters = load_ros_parameter_block(serial_config)
    lidar_parameters = load_ros_parameter_block(lidar_config)

    serial_port = str(serial_parameters.get('serial_port', '')).strip()
    lidar_port = str(lidar_parameters.get('comm', {}).get('serial_port', '')).strip()

    if not serial_port:
        raise LaunchError(f'No serial_port configured in {serial_config}')
    if not lidar_port:
        raise LaunchError(f'No comm.serial_port configured in {lidar_config}')

    return DevicePaths(serial_port=serial_port, lidar_port=lidar_port)


def run_command(
    command: list[str],
    *,
    cwd: Path | None = None,
    timeout: float = 10.0,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=str(cwd) if cwd is not None else None,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def check_ros_package(
    package_name: str,
    ros2_executable: str,
    *,
    runner: Callable[..., Any] = run_command,
) -> CheckResult:
    result = runner([ros2_executable, 'pkg', 'prefix', package_name], timeout=10.0)
    if result.returncode == 0:
        prefix = (result.stdout or '').strip() or 'package resolved'
        return CheckResult(
            name=f'ROS package {package_name}',
            ok=True,
            detail=prefix,
        )

    stderr = (result.stderr or '').strip() or (result.stdout or '').strip()
    return CheckResult(
        name=f'ROS package {package_name}',
        ok=False,
        detail=stderr or f'Package {package_name} is not discoverable.',
        fix_hint=PACKAGE_FIX_HINT,
    )


def check_device_path(path_text: str, *, label: str, fix_hint: str) -> CheckResult:
    path = Path(path_text)
    if path.exists():
        return CheckResult(name=label, ok=True, detail=str(path))
    return CheckResult(
        name=label,
        ok=False,
        detail=f'Missing device path: {path}',
        fix_hint=fix_hint,
    )


def check_directory_writable(path: Path) -> CheckResult:
    try:
        path.mkdir(parents=True, exist_ok=True)
        probe = path / '.write_test'
        probe.write_text('ok\n', encoding='utf-8')
        probe.unlink()
    except OSError as exc:
        return CheckResult(
            name=f'Writable directory {path}',
            ok=False,
            detail=str(exc),
        )

    return CheckResult(name=f'Writable directory {path}', ok=True, detail=str(path))


def check_lidar_submodule_restored(submodule_path: Path) -> CheckResult:
    package_xml = next(submodule_path.rglob('package.xml'), None) if submodule_path.exists() else None
    if package_xml is not None:
        return CheckResult(
            name='LDLiDAR driver submodule',
            ok=True,
            detail=str(package_xml),
        )

    return CheckResult(
        name='LDLiDAR driver submodule',
        ok=False,
        detail=f'No package.xml found under {submodule_path}',
        fix_hint=SUBMODULE_FIX_HINT,
    )


def run_preflight_checks(
    *,
    repo_root: Path,
    log_file: Path,
    which: Callable[[str], str | None] = shutil.which,
    runner: Callable[..., Any] = run_command,
) -> tuple[DevicePaths | None, list[CheckResult]]:
    results: list[CheckResult] = []
    runtime_root = repo_root / '.guido'
    ros2_executable = which('ros2')
    if ros2_executable is None:
        results.append(
            CheckResult(
                name='ros2 CLI',
                ok=False,
                detail='ros2 is not on PATH.',
                fix_hint=ROS_FIX_HINT,
            )
        )
    else:
        results.append(
            CheckResult(name='ros2 CLI', ok=True, detail=ros2_executable)
        )
        for package_name in REQUIRED_ROS_PACKAGES:
            results.append(
                check_ros_package(package_name, ros2_executable, runner=runner)
            )

    results.append(
        check_lidar_submodule_restored(repo_root / 'src' / 'ldrobot-lidar-ros2')
    )

    device_paths: DevicePaths | None = None
    try:
        device_paths = load_device_paths(repo_root)
    except LaunchError as exc:
        results.append(CheckResult(name='Config parsing', ok=False, detail=str(exc)))
    else:
        results.append(
            CheckResult(
                name='Config parsing',
                ok=True,
                detail=(
                    f'serial={device_paths.serial_port}, '
                    f'lidar={device_paths.lidar_port}'
                ),
            )
        )
        results.append(
            check_device_path(
                device_paths.serial_port,
                label='Arduino serial device',
                fix_hint=SERIAL_FIX_HINT,
            )
        )
        results.append(
            check_device_path(
                device_paths.lidar_port,
                label='LiDAR serial device',
                fix_hint=LIDAR_FIX_HINT,
            )
        )

    runtime_paths = (
        runtime_root,
        runtime_root / 'maps',
        runtime_root / 'logs',
        log_file.expanduser().resolve().parent,
    )
    seen_paths: set[Path] = set()
    for path in runtime_paths:
        resolved = path.resolve()
        if resolved in seen_paths:
            continue
        seen_paths.add(resolved)
        results.append(check_directory_writable(resolved))

    return device_paths, results


class RealRosReadinessProbe:
    """Lazy ROS 2 graph probe used only when the environment is sourced."""

    def __init__(
        self,
        *,
        command_topic: str = '/auto_nav/command',
        scan_topic: str = '/ldlidar_node/scan',
        odom_topic: str = '/odom',
        map_topic: str = '/map',
        map_frame: str = 'map',
        base_frames: tuple[str, ...] = ('base_footprint', 'base_link'),
        map_save_service: str = '/map_saver/save_map',
        navigation_action: str = 'navigate_to_pose',
    ):
        try:
            import rclpy
            from nav2_msgs.action import NavigateToPose
            from nav2_msgs.srv import SaveMap
            from nav_msgs.msg import OccupancyGrid
            from nav_msgs.msg import Odometry
            from rclpy.action import ActionClient
            from rclpy.duration import Duration
            from rclpy.time import Time
            from sensor_msgs.msg import LaserScan
            from std_msgs.msg import String
            from tf2_ros import Buffer
            from tf2_ros import TransformListener
        except ImportError as exc:
            raise LaunchError(
                'Unable to import ROS 2 Python libraries. '
                f'{ROS_FIX_HINT}'
            ) from exc

        self._rclpy = rclpy
        self._time_type = Time
        self._duration_type = Duration
        self._string_type = String
        self._base_frames = base_frames
        self._map_frame = map_frame

        rclpy.init(args=None)
        self._node = rclpy.create_node('frontier_mapping_launcher')

        self._scan_received = False
        self._odom_received = False
        self._map_received = False

        self._node.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self._node.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._node.create_subscription(OccupancyGrid, map_topic, self._on_map, 10)

        self._command_publisher = self._node.create_publisher(String, command_topic, 10)
        self._map_saver_client = self._node.create_client(SaveMap, map_save_service)
        self._navigation_action_client = ActionClient(
            self._node,
            NavigateToPose,
            navigation_action,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self._node,
            spin_thread=False,
        )

    def _on_scan(self, _: Any) -> None:
        self._scan_received = True

    def _on_odom(self, _: Any) -> None:
        self._odom_received = True

    def _on_map(self, _: Any) -> None:
        self._map_received = True

    def spin_once(self, timeout_sec: float) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=max(timeout_sec, 0.0))

    def snapshot(self) -> ReadinessSnapshot:
        return ReadinessSnapshot(
            command_topic_ready=self._command_interface_ready(),
            map_saver_ready=self._map_saver_client.service_is_ready(),
            navigation_action_ready=self._navigation_action_client.server_is_ready(),
            scan_received=self._scan_received,
            odom_received=self._odom_received,
            map_received=self._map_received,
            tf_ready=self._tf_ready(),
        )

    def publish_command(self, command: str) -> None:
        message = self._string_type()
        message.data = command
        self._command_publisher.publish(message)
        self.spin_once(0.05)

    def close(self) -> None:
        try:
            self._node.destroy_node()
        finally:
            if self._rclpy.ok():
                self._rclpy.shutdown()

    def _command_interface_ready(self) -> bool:
        counter = getattr(self._command_publisher, 'get_subscription_count', None)
        if callable(counter):
            return counter() > 0
        return True

    def _tf_ready(self) -> bool:
        for base_frame in self._base_frames:
            try:
                if self._tf_buffer.can_transform(
                    self._map_frame,
                    base_frame,
                    self._time_type(),
                    timeout=self._duration_type(seconds=0.0),
                ):
                    return True
            except Exception:
                continue
        return False


class LaunchOutputPump:
    def __init__(self, process: subprocess.Popen[str], log_file: Path):
        self._process = process
        self._log_file = log_file
        self.imu_warning_seen = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        self._thread.join(timeout=timeout)

    def _run(self) -> None:
        if self._process.stdout is None:
            return

        with self._log_file.open('a', encoding='utf-8') as handle:
            handle.write(
                '\n=== frontier mapping launch started at '
                f'{time.strftime("%Y-%m-%d %H:%M:%S")} ===\n'
            )
            handle.flush()
            for line in self._process.stdout:
                if 'MPU6050 connection failed' in line:
                    self.imu_warning_seen = True
                sys.stdout.write(line)
                sys.stdout.flush()
                handle.write(line)
                handle.flush()


def launch_exploration_stack(repo_root: Path) -> subprocess.Popen[str]:
    return subprocess.Popen(
        ['ros2', 'launch', 'auto_nav', 'exploration.launch.py'],
        cwd=str(repo_root),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )


def wait_for_phase(
    probe: ReadinessProbe,
    process: Any,
    *,
    timeout: float,
    phase_name: str,
    predicate: Callable[[ReadinessSnapshot], bool],
    missing_checks: Callable[[ReadinessSnapshot], tuple[str, ...]],
    time_fn: Callable[[], float] = time.monotonic,
) -> ReadinessSnapshot:
    deadline = time_fn() + timeout
    snapshot = probe.snapshot()

    while time_fn() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise LaunchError(
                f'ros2 launch exited during {phase_name} with code {return_code}.'
            )

        probe.spin_once(min(0.2, max(0.0, deadline - time_fn())))
        snapshot = probe.snapshot()
        if predicate(snapshot):
            return snapshot

    missing = ', '.join(missing_checks(snapshot)) or 'unknown readiness check'
    raise LaunchError(f'Timed out waiting for {phase_name}: {missing}.')


def format_check_results(results: list[CheckResult]) -> str:
    lines = []
    for result in results:
        status = 'OK' if result.ok else 'FAIL'
        line = f'[{status}] {result.name}: {result.detail}'
        if result.fix_hint:
            line += f' | fix: {result.fix_hint}'
        lines.append(line)
    return '\n'.join(lines)


def format_readiness_summary(snapshot: ReadinessSnapshot) -> str:
    statuses = [
        ('/auto_nav/command subscriber', snapshot.command_topic_ready),
        ('/map_saver/save_map service', snapshot.map_saver_ready),
        ('Nav2 navigate_to_pose action', snapshot.navigation_action_ready),
        ('/ldlidar_node/scan messages', snapshot.scan_received),
        ('/odom messages', snapshot.odom_received),
        ('/map messages', snapshot.map_received),
        ('map -> base_footprint/base_link TF', snapshot.tf_ready),
    ]
    lines = ['Readiness summary:']
    for label, healthy in statuses:
        lines.append(f"  [{'OK' if healthy else 'WAIT'}] {label}")
    lines.append(
        "  [WARN] MPU6050 is not ROS-visible in this repo, so IMU health is not a launch gate."
    )
    return '\n'.join(lines)


def safe_publish_stop(probe: ReadinessProbe | None) -> None:
    if probe is None:
        return
    try:
        probe.publish_command('stop')
    except Exception:
        pass


def terminate_process_group(
    process: subprocess.Popen[str] | None,
    *,
    interrupt_timeout: float = 5.0,
    terminate_timeout: float = 3.0,
    kill_timeout: float = 2.0,
) -> bool:
    if process is None:
        return True
    if process.poll() is not None:
        return True

    for sig, timeout in (
        (signal.SIGINT, interrupt_timeout),
        (signal.SIGTERM, terminate_timeout),
        (signal.SIGKILL, kill_timeout),
    ):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return True

        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            continue

    return process.poll() is not None


def install_signal_handlers(stop_event: threading.Event) -> dict[int, Any]:
    previous = {}

    def _handler(signum: int, _frame: Any) -> None:
        stop_event.set()
        print(f'\nReceived signal {signum}; stopping frontier mapping...', flush=True)

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, _handler)
    return previous


def restore_signal_handlers(previous_handlers: dict[int, Any]) -> None:
    for signum, handler in previous_handlers.items():
        signal.signal(signum, handler)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_file = args.log_file.expanduser().resolve()

    _, preflight_results = run_preflight_checks(repo_root=REPO_ROOT, log_file=log_file)
    print(format_check_results(preflight_results), flush=True)

    if any(not result.ok for result in preflight_results):
        print(
            '\nPreflight failed. Fix the reported items and rerun '
            '`python3 scripts/start_frontier_mapping.py`.',
            file=sys.stderr,
        )
        return 2

    if args.dry_run_checks:
        print('\nDry-run checks passed. No ROS processes were started.')
        return 0

    stop_event = threading.Event()
    previous_handlers = install_signal_handlers(stop_event)
    probe: ReadinessProbe | None = None
    launch_process: subprocess.Popen[str] | None = None
    output_pump: LaunchOutputPump | None = None

    try:
        probe = RealRosReadinessProbe()
        launch_process = launch_exploration_stack(REPO_ROOT)
        output_pump = LaunchOutputPump(launch_process, log_file)
        output_pump.start()

        print('\nWaiting for ROS launch interfaces...', flush=True)
        wait_for_phase(
            probe,
            launch_process,
            timeout=args.launch_timeout,
            phase_name='launch interface readiness',
            predicate=lambda snapshot: snapshot.launch_ready(),
            missing_checks=lambda snapshot: snapshot.missing_launch_checks(),
        )

        print('Waiting for live sensor/map health...', flush=True)
        snapshot = wait_for_phase(
            probe,
            launch_process,
            timeout=args.health_timeout,
            phase_name='sensor and map health',
            predicate=lambda snapshot: snapshot.health_ready(),
            missing_checks=lambda snapshot: snapshot.missing_health_checks(),
        )
        print(format_readiness_summary(snapshot), flush=True)
        if output_pump.imu_warning_seen:
            print(
                '  [WARN] Launch output reported "MPU6050 connection failed". '
                'Exploration is not blocked by this repo version, but IMU data is not trusted.',
                flush=True,
            )

        if args.no_auto_start:
            print(
                "\nLauncher is ready. Auto-start is disabled, so exploration has not begun yet.",
                flush=True,
            )
        else:
            probe.publish_command('start_exploration')
            print(
                '\nPublished /auto_nav/command -> start_exploration',
                flush=True,
            )

        while not stop_event.is_set():
            return_code = launch_process.poll()
            if return_code is not None:
                raise LaunchError(
                    f'ros2 launch auto_nav exploration.launch.py exited unexpectedly with code {return_code}.'
                )
            time.sleep(0.2)

        print('Stopping exploration stack...', flush=True)
        safe_publish_stop(probe)
        if not terminate_process_group(launch_process):
            raise LaunchError('Unable to terminate the exploration launch process cleanly.')
        return 0
    except LaunchError as exc:
        print(f'\nLauncher error: {exc}', file=sys.stderr, flush=True)
        safe_publish_stop(probe)
        terminate_process_group(launch_process)
        return 1
    finally:
        restore_signal_handlers(previous_handlers)
        if output_pump is not None:
            output_pump.join(timeout=1.0)
        if probe is not None:
            probe.close()
        if launch_process is not None and launch_process.poll() is None:
            terminate_process_group(launch_process)


if __name__ == '__main__':
    raise SystemExit(main())
