import time
from typing import Callable
from typing import Optional

from navigation.types import HealthStatus


class HealthMonitor:
    def __init__(
        self,
        scan_timeout: float = 0.5,
        odom_timeout: float = 2.0,
        tf_timeout: float = 1.0,
        clock: Optional[Callable[[], float]] = None,
    ):
        self._scan_timeout = scan_timeout
        self._odom_timeout = odom_timeout
        self._tf_timeout = tf_timeout
        self._clock = clock or time.monotonic
        self._last_seen = {
            'scan': None,
            'odom': None,
            'tf': None,
        }

    def record_scan(self, timestamp: Optional[float] = None) -> None:
        self._record('scan', timestamp)

    def record_odom(self, timestamp: Optional[float] = None) -> None:
        self._record('odom', timestamp)

    def record_tf(self, timestamp: Optional[float] = None) -> None:
        self._record('tf', timestamp)

    def status(self, now: Optional[float] = None) -> HealthStatus:
        current_time = self._clock() if now is None else now
        stale_sources = []
        timeouts = {
            'scan': self._scan_timeout,
            'odom': self._odom_timeout,
            'tf': self._tf_timeout,
        }
        for source_name, timeout in timeouts.items():
            seen_at = self._last_seen[source_name]
            if seen_at is None or current_time - seen_at > timeout:
                stale_sources.append(source_name)

        if stale_sources:
            return HealthStatus(
                healthy=False,
                stale_sources=tuple(stale_sources),
                details='stale ' + ', '.join(stale_sources),
            )

        return HealthStatus(healthy=True, stale_sources=tuple(), details='healthy')

    def _record(self, source_name: str, timestamp: Optional[float]) -> None:
        self._last_seen[source_name] = self._clock() if timestamp is None else timestamp
