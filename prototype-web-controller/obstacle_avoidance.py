"""Corridor-aware dumb obstacle avoidance.

The avoider consumes a :class:`camera_service.CorridorReading` (three depth
bands covering the rover's footprint at the look-ahead distance) and emits an
:class:`AvoidanceCommand` describing the desired throttle/turn for the drive
loop. It is intentionally simple — there is no map, no path planning, no
heading estimate — but it is markedly more robust than a single center-pixel
threshold:

* Any band can veto forward motion, so the rover doesn't clip its corners on
  obstacles the camera sees but the center pixel misses.
* Fully lost depth is treated as open space for this prototype, so the rover
  keeps moving unless at least one trusted band reports an obstacle.
* When blocked, the avoider actively pivots toward the side with the most
  clearance instead of stalling, and keeps retrying while avoidance is enabled.
* Stop distance scales with cruise throttle and the configured top speed so
  the rover always has room to point-turn in the gap it stops in.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from camera_service import BandReading, CorridorReading


@dataclass(frozen=True)
class AvoidanceConfig:
    """Tunable thresholds and timings for :class:`CorridorAvoider`.

    Distance values are in meters; durations are in seconds. Defaults assume a
    ~0.7 m wide rover moving at conservative throttles. Tune on hardware.
    """

    cruise_throttle: float = 0.25
    slow_throttle: float = 0.12
    reverse_throttle: float = -0.18
    pivot_rate: float = 0.35
    recovery_pivot_rate: float = 0.2

    stop_distance_m: float = 0.7
    caution_distance_m: float = 1.2
    clear_distance_m: float = 1.5
    reverse_distance_m: float = 0.55

    min_valid_ratio: float = 0.35
    min_valid_pixels: int = 60

    reverse_seconds: float = 0.7
    pivot_step_s: float = 2.0
    no_depth_grace_s: float = 1.0
    no_depth_search_s: float = 3.0
    max_pivot_attempts: int = 4
    pivot_clearance_delta_m: float = 0.20

    stale_seconds: float = 0.3
    reaction_factor: float = 0.4
    recovery_verify_s: float = 0.3


@dataclass(frozen=True)
class AvoidanceCommand:
    state: str
    reason: str
    throttle: float = 0.0
    turn: float = 0.0
    pivot_side: Optional[str] = None
    attempts: int = 0


def _band_dist(
    band: BandReading,
    min_valid_ratio: float,
    min_valid_pixels: int = 0,
) -> Optional[float]:
    """Trusted distance for a band, or ``None`` if too few valid pixels.

    Both gates apply: the ratio rejects bands where the corridor is mostly
    invalid (a sane sensor with mostly-noise input), and the absolute count
    rejects bands where a high ratio is satisfied by a tiny pixel cluster
    (e.g. one bright edge on a glass wall).
    """
    if band.min_m is None or band.valid_ratio < min_valid_ratio:
        return None
    if band.valid_pixels < min_valid_pixels:
        return None
    return float(band.min_m)


class CorridorAvoider:
    def __init__(
        self,
        config: Optional[AvoidanceConfig] = None,
        max_linear_velocity: float = 1.0,
    ) -> None:
        self.config = config or AvoidanceConfig()
        self.max_linear_velocity = float(max_linear_velocity)
        self.enabled = False
        self.state = "manual"
        self.reason = "manual control"
        self.attempts = 0
        self.pivot_side: Optional[str] = None
        self._state_until = 0.0
        self._last_depth_at: Optional[float] = None
        self._last_clear_side: Optional[str] = None
        self._verify_until = 0.0

    # -- public API -----------------------------------------------------------

    def effective_stop_distance(self) -> float:
        cfg = self.config
        return cfg.stop_distance_m + cfg.reaction_factor * cfg.cruise_throttle * max(
            self.max_linear_velocity, 0.0
        )

    def set_enabled(
        self, enabled: bool, now: Optional[float] = None
    ) -> AvoidanceCommand:
        now = time.time() if now is None else now
        self.enabled = enabled
        if enabled:
            self.state = "cruising"
            self.reason = "avoidance enabled"
            self._state_until = now
            self._verify_until = 0.0
            self.attempts = 0
            self.pivot_side = None
            self._last_depth_at = now
        else:
            self.state = "manual"
            self.reason = "manual control"
            self.attempts = 0
            self.pivot_side = None
        return self._snapshot()

    def stop(self) -> AvoidanceCommand:
        self.enabled = False
        self.state = "manual"
        self.reason = "stopped"
        self.attempts = 0
        self.pivot_side = None
        return AvoidanceCommand("manual", "stopped")

    def status(self) -> dict:
        return {
            "enabled": self.enabled,
            "state": self.state,
            "reason": self.reason,
            "attempts": self.attempts,
            "pivot_side": self.pivot_side,
        }

    def command(
        self,
        reading: Optional[CorridorReading],
        now: Optional[float] = None,
    ) -> AvoidanceCommand:
        now = time.time() if now is None else now
        if not self.enabled:
            return self._snapshot()
        if self.state == "stuck":
            # Stay stuck until operator toggles avoidance off and on again.
            return self._snapshot()

        cfg = self.config
        stale = (
            reading is None
            or (now - reading.timestamp) > cfg.stale_seconds
        )

        if not stale and reading is not None:
            left = _band_dist(reading.left, cfg.min_valid_ratio, cfg.min_valid_pixels)
            center = _band_dist(reading.center, cfg.min_valid_ratio, cfg.min_valid_pixels)
            right = _band_dist(reading.right, cfg.min_valid_ratio, cfg.min_valid_pixels)
        else:
            left = center = right = None

        all_blind = stale or (left is None and center is None and right is None)
        if all_blind:
            return self._assume_clear_no_depth()

        self._last_depth_at = now
        if self.state not in ("searching", "reversing"):
            self._update_clear_side_cache(left, right)

        valid_dists = [d for d in (left, center, right) if d is not None]
        worst = min(valid_dists) if valid_dists else None

        # Emergency: any band shows a near obstacle.
        if worst is not None and worst < cfg.reverse_distance_m:
            return self._begin_reverse(now, worst)

        # In-progress timed states finish out their windows before re-deciding.
        if self.state == "reversing":
            if now < self._state_until:
                return self._snapshot(cfg.reverse_throttle, 0.0)
            return self._begin_search(now, reading)

        if self.state == "searching":
            stop_thr = self.effective_stop_distance()
            chosen_side = self.pivot_side
            chosen_dist = (
                left if chosen_side == "left"
                else right if chosen_side == "right"
                else None
            )
            opened = (
                center is not None
                and center >= cfg.clear_distance_m
                and worst is not None
                and worst >= stop_thr
            ) or (
                chosen_dist is not None
                and chosen_dist >= cfg.clear_distance_m
                and (center is None or center >= stop_thr)
            )
            if opened:
                self.state = "cruising"
                self.reason = (
                    f"path opened: center {center:.2f} m"
                    if center is not None
                    else "path opened toward chosen side"
                )
                self.attempts = 0
                self.pivot_side = None
                return self._snapshot(cfg.cruise_throttle, 0.0)
            if now < self._state_until:
                return self._snapshot(0.0, self._pivot_turn_value())
            # Take a small reverse step before the next pivot — gives us room
            # to swing without clipping the obstacle we just rotated past.
            return self._begin_reverse(
                now, worst if worst is not None else cfg.reverse_distance_m
            )

        # Forward-flow: cruising or slowing.
        stop_thr = self.effective_stop_distance()
        if center is not None and center < stop_thr:
            return self._begin_search(now, reading)
        if worst is not None and worst < stop_thr:
            return self._begin_search(now, reading)

        # Center band missing but sides ok — be cautious, don't trust forward.
        if center is None:
            self.state = "slowing"
            self.reason = "center depth missing — slowing"
            return self._snapshot(cfg.slow_throttle, 0.0)

        # Hysteresis: enter slowing at caution_distance, only return to
        # cruising once everything clears clear_distance_m.
        if worst is not None and worst < cfg.caution_distance_m:
            self.state = "slowing"
            self.reason = f"caution: worst band {worst:.2f} m"
            return self._snapshot(cfg.slow_throttle, 0.0)

        if self.state == "slowing":
            if (
                worst is None
                or worst < cfg.clear_distance_m
                or now < self._verify_until
            ):
                return self._snapshot(cfg.slow_throttle, 0.0)
            self.state = "cruising"
            self.reason = self._clear_reason(left, center, right, worst)
            return self._snapshot(cfg.cruise_throttle, 0.0)

        self.state = "cruising"
        self.reason = self._clear_reason(left, center, right, worst)
        return self._snapshot(cfg.cruise_throttle, 0.0)

    # -- internals ------------------------------------------------------------

    def _snapshot(self, throttle: float = 0.0, turn: float = 0.0) -> AvoidanceCommand:
        return AvoidanceCommand(
            state=self.state,
            reason=self.reason,
            throttle=throttle,
            turn=turn,
            pivot_side=self.pivot_side,
            attempts=self.attempts,
        )

    def _clear_reason(
        self,
        left: Optional[float],
        center: Optional[float],
        right: Optional[float],
        worst: Optional[float],
    ) -> str:
        blind_sides = []
        if left is None:
            blind_sides.append("left")
        if right is None:
            blind_sides.append("right")
        if center is not None and len(blind_sides) == 1:
            visible_side = right if blind_sides[0] == "left" else left
            if visible_side is not None:
                return f"{blind_sides[0]} depth missing — proceeding"
        if worst is not None:
            return f"clear: worst band {worst:.2f} m"
        return "clear"

    def _update_clear_side_cache(
        self, left: Optional[float], right: Optional[float]
    ) -> None:
        if left is None and right is None:
            return
        if left is None:
            self._last_clear_side = "right"
            return
        if right is None:
            self._last_clear_side = "left"
            return
        delta = left - right
        if abs(delta) < self.config.pivot_clearance_delta_m:
            return
        self._last_clear_side = "left" if delta > 0.0 else "right"

    def _choose_pivot_side(self, reading: CorridorReading) -> str:
        cfg = self.config
        left = _band_dist(reading.left, cfg.min_valid_ratio, cfg.min_valid_pixels)
        right = _band_dist(reading.right, cfg.min_valid_ratio, cfg.min_valid_pixels)
        if left is None and right is None:
            return "left"
        if left is None:
            return "right"
        if right is None:
            return "left"
        delta = left - right
        if delta >= cfg.pivot_clearance_delta_m:
            return "left"
        if delta <= -cfg.pivot_clearance_delta_m:
            return "right"
        return "left"

    def _pivot_turn_value(self) -> float:
        # Convention matches arcade(): positive turn yaws right. So to pivot
        # toward the left side we use a negative turn value.
        rate = self.config.pivot_rate
        return -rate if self.pivot_side == "left" else rate

    def _begin_reverse(self, now: float, worst: float) -> AvoidanceCommand:
        cfg = self.config
        self.state = "reversing"
        self.reason = f"obstacle at {worst:.2f} m, backing off"
        self._state_until = now + cfg.reverse_seconds
        return self._snapshot(cfg.reverse_throttle, 0.0)

    def _begin_search(
        self, now: float, reading: Optional[CorridorReading]
    ) -> AvoidanceCommand:
        cfg = self.config
        self.attempts += 1
        if reading is not None:
            self.pivot_side = self._choose_pivot_side(reading)
        else:
            self.pivot_side = self.pivot_side or "left"
        self.state = "searching"
        self.reason = f"scanning {self.pivot_side} for opening"
        self._state_until = now + cfg.pivot_step_s
        return self._snapshot(0.0, self._pivot_turn_value())

    def _assume_clear_no_depth(self) -> AvoidanceCommand:
        cfg = self.config
        self.state = "cruising"
        self.reason = "no valid depth - assuming clear"
        self.attempts = 0
        self.pivot_side = None
        return self._snapshot(cfg.cruise_throttle, 0.0)
