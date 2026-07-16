"""Diagnostics state that can be tested independently from rclpy."""


class WarningThrottle:
    def __init__(self) -> None:
        self.last_emit = {}

    def should_emit(self, key: str, now: float, interval_sec: float) -> bool:
        if (now - self.last_emit.get(key, 0.0)) < interval_sec:
            return False
        self.last_emit[key] = now
        return True
