from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class AvoidanceConfig:
    stop_threshold_m: float = 0.6
    clear_threshold_m: float = 0.9
    forward_throttle: float = 0.25
    reverse_throttle: float = -0.2
    pivot_rate: float = 0.3
    reverse_seconds: float = 0.7
    pivot_seconds: float = 0.9
    stale_seconds: float = 0.75


@dataclass(frozen=True)
class AvoidanceCommand:
    state: str
    reason: str
    throttle: float = 0.0
    turn: float = 0.0


class DumbObstacleAvoider:
    def __init__(self, config: AvoidanceConfig | None = None) -> None:
        self.config = config or AvoidanceConfig()
        self.enabled = False
        self.state = "manual"
        self.reason = "manual control"
        self._state_until = 0.0
        self._pivot_direction = 1.0

    def set_enabled(self, enabled: bool, now: Optional[float] = None) -> AvoidanceCommand:
        now = time.time() if now is None else now
        self.enabled = enabled
        if enabled:
            self.state = "active"
            self.reason = "avoidance enabled"
            self._state_until = now
        else:
            self.state = "manual"
            self.reason = "manual control"
            self._state_until = now
        return AvoidanceCommand(self.state, self.reason)

    def stop(self) -> AvoidanceCommand:
        self.enabled = False
        self.state = "manual"
        self.reason = "stopped"
        return AvoidanceCommand("manual", "stopped")

    def command(self, center_m: Optional[float], now: Optional[float] = None) -> AvoidanceCommand:
        now = time.time() if now is None else now
        if not self.enabled:
            return AvoidanceCommand("manual", self.reason)

        if center_m is None:
            self.state = "stopped"
            self.reason = "no valid depth reading"
            return AvoidanceCommand(self.state, self.reason)

        if self.state == "reversing":
            if now < self._state_until:
                return AvoidanceCommand("reversing", "obstacle detected", self.config.reverse_throttle, 0.0)
            self.state = "pivoting"
            self.reason = "turning away"
            self._state_until = now + self.config.pivot_seconds
            self._pivot_direction *= -1.0

        if self.state == "pivoting":
            if now < self._state_until:
                return AvoidanceCommand("pivoting", self.reason, 0.0, self._pivot_direction * self.config.pivot_rate)
            self.state = "active"
            self.reason = "checking path"

        if center_m < self.config.stop_threshold_m:
            self.state = "reversing"
            self.reason = f"obstacle at {center_m:.2f} m"
            self._state_until = now + self.config.reverse_seconds
            return AvoidanceCommand("reversing", self.reason, self.config.reverse_throttle, 0.0)

        if center_m < self.config.clear_threshold_m:
            self.state = "blocked"
            self.reason = f"path not clear: {center_m:.2f} m"
            return AvoidanceCommand("blocked", self.reason)

        self.state = "active"
        self.reason = f"path clear: {center_m:.2f} m"
        return AvoidanceCommand("active", self.reason, self.config.forward_throttle, 0.0)

    def status(self) -> dict:
        return {
            "enabled": self.enabled,
            "state": self.state,
            "reason": self.reason,
        }
