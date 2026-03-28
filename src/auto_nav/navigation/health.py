import time
from typing import Callable, Dict, Optional

from .types import HealthStatus


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
        self._last_seen: Dict[str, float] = {}

    def record_scan(self, timestamp: Optional[float] = None) -> None:
        self._record("scan", timestamp)

    def record_odom(self, timestamp: Optional[float] = None) -> None:
        self._record("odom", timestamp)

    def record_tf(self, timestamp: Optional[float] = None) -> None:
        self._record("tf", timestamp)

    def status(self, now: Optional[float] = None) -> HealthStatus:
        current_time = self._clock() if now is None else now
        stale_sources = []

        for source_name, timeout in (
            ("scan", self._scan_timeout),
            ("odom", self._odom_timeout),
            ("tf", self._tf_timeout),
        ):
            last_seen = self._last_seen.get(source_name)
            if last_seen is None or current_time - last_seen > timeout:
                stale_sources.append(source_name)

        if stale_sources:
            details = "Navigation health check failed: stale {}.".format(
                ", ".join(stale_sources)
            )
        else:
            details = "Navigation health check passed."

        return HealthStatus(
            healthy=not stale_sources,
            stale_sources=tuple(stale_sources),
            details=details,
        )

    def _record(self, source_name: str, timestamp: Optional[float]) -> None:
        self._last_seen[source_name] = self._clock() if timestamp is None else timestamp
