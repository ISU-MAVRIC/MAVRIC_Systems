"""
skid_steer_drive.py
===================

Arcade-style differential (skid-steer) drive math and hardware interface
for a 6-wheel robot powered by NEO550 motors through SparkMAX controllers.

Layers
------
- DriveConfig: physical + tuning parameters
- SkidSteerKinematics: pure math, no hardware (unit-testable)
- SkidSteerDrive: hardware-facing class that commands motors

Typical use
-----------

    from SparkCANLib import SparkCAN
    from skid_steer_drive import DriveConfig, SkidSteerDrive

    bus = SparkCAN.SparkBus(channel="can0", bustype="socketcan", bitrate=1000000)

    config = DriveConfig(
        max_linear_velocity=3.0,        # m/s, used as top-speed scale
        left_motor_ids=[11, 12, 13],
        right_motor_ids=[21, 22, 23],
    )

    drive = SkidSteerDrive(bus, config)
    drive.arcade(throttle=0.5, turn=0.2)
    drive.point_turn(0.4)
    drive.stop()
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Literal, Optional, Tuple


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# NEO550 free speed is ~11,000 RPM. We cap well below that by default to leave
# headroom for the closed-loop controller and to avoid thermal issues.
DEFAULT_MAX_MOTOR_RPM = 8000.0

# 10 inch wheels in meters.
DEFAULT_WHEEL_DIAMETER_M = 0.254

# Standard 20:1 planetary reduction between motor and wheel.
DEFAULT_GEAR_RATIO = 20.0


SaturationMode = Literal["scale", "clip"]


# ---------------------------------------------------------------------------
# DriveConfig
# ---------------------------------------------------------------------------

@dataclass
class DriveConfig:
    """
    Configuration for a 6-wheel skid-steer drivetrain.

    All three motors on a given side receive the same velocity command, so
    motor IDs are provided as two lists of three. Track width is intentionally
    optional: pure arcade mixing does not require it. Provide it only if you
    later add odometry or angular-velocity helpers.

    Parameters
    ----------
    max_linear_velocity : float
        Top speed of the robot in m/s. A throttle input of 1.0 maps to this
        speed. This is the only required speed parameter.
    left_motor_ids : list[int]
        CAN IDs for the three left-side motors.
    right_motor_ids : list[int]
        CAN IDs for the three right-side motors.
    wheel_diameter : float, default 0.254
        Wheel diameter in meters (default is 10 in).
    gear_ratio : float, default 20.0
        Motor-to-wheel reduction. A value of 20 means the motor spins 20 times
        for each wheel rotation.
    max_motor_rpm : float, default 8000.0
        Hard ceiling applied after the kinematics stage. Commands that would
        exceed this are either clipped or scaled (see ``saturation_mode``).
    deadband : float, default 0.0
        Inputs with absolute value below this threshold are treated as zero.
        Applied to both ``throttle`` and ``turn`` independently. Must be in
        [0, 1).
    saturation_mode : {"scale", "clip"}, default "scale"
        How to handle arcade mixes that exceed the [-1, 1] range.
        "scale" preserves the throttle/turn ratio (recommended); "clip"
        independently clamps each side.
    left_inverted : bool, default False
        Convenience flag applied to the final left-side RPM sign. Use this
        only if the SparkController ``dir`` property is not already set.
    right_inverted : bool, default True
        Same, for the right side. Defaults to True because on most chassis
        the right-side motors are mirrored relative to the left.
    track_width : float, optional
        Distance between left and right wheel contact points in meters. Not
        used by arcade mixing; present for future odometry use.
    """

    max_linear_velocity: float
    left_motor_ids: List[int]
    right_motor_ids: List[int]

    wheel_diameter: float = DEFAULT_WHEEL_DIAMETER_M
    gear_ratio: float = DEFAULT_GEAR_RATIO
    max_motor_rpm: float = DEFAULT_MAX_MOTOR_RPM
    deadband: float = 0.0
    saturation_mode: SaturationMode = "scale"

    left_inverted: bool = False
    right_inverted: bool = True

    track_width: Optional[float] = None

    def __post_init__(self) -> None:
        if self.max_linear_velocity <= 0:
            raise ValueError("max_linear_velocity must be > 0")
        if self.wheel_diameter <= 0:
            raise ValueError("wheel_diameter must be > 0")
        if self.gear_ratio <= 0:
            raise ValueError("gear_ratio must be > 0")
        if self.max_motor_rpm <= 0:
            raise ValueError("max_motor_rpm must be > 0")
        if not (0.0 <= self.deadband < 1.0):
            raise ValueError("deadband must be in [0, 1)")
        if self.saturation_mode not in ("scale", "clip"):
            raise ValueError("saturation_mode must be 'scale' or 'clip'")
        if len(self.left_motor_ids) != 3:
            raise ValueError("left_motor_ids must contain exactly 3 IDs")
        if len(self.right_motor_ids) != 3:
            raise ValueError("right_motor_ids must contain exactly 3 IDs")

        all_ids = self.left_motor_ids + self.right_motor_ids
        if len(set(all_ids)) != len(all_ids):
            raise ValueError("motor IDs must be unique across both sides")

        if self.track_width is not None and self.track_width <= 0:
            raise ValueError("track_width must be > 0 when provided")

    # -- derived quantities ---------------------------------------------------

    @property
    def wheel_circumference(self) -> float:
        """Circumference of a single wheel in meters."""
        return math.pi * self.wheel_diameter

    @property
    def max_wheel_rpm_from_max_velocity(self) -> float:
        """
        Wheel RPM corresponding to ``max_linear_velocity``.

        This is what one wheel spins at when the robot is moving forward at
        top speed in a straight line.
        """
        # revs per second = v / circumference, times 60 for per minute
        return (self.max_linear_velocity / self.wheel_circumference) * 60.0

    @property
    def max_motor_rpm_from_max_velocity(self) -> float:
        """Motor RPM corresponding to ``max_linear_velocity``."""
        return self.max_wheel_rpm_from_max_velocity * self.gear_ratio


# ---------------------------------------------------------------------------
# Kinematics (pure math, no hardware)
# ---------------------------------------------------------------------------

class SkidSteerKinematics:
    """
    Arcade-style skid-steer kinematics.

    Converts normalized ``(throttle, turn)`` inputs into per-side motor RPM
    commands. This class has no hardware dependencies and is safe to unit
    test directly.
    """

    def __init__(self, config: DriveConfig) -> None:
        self.config = config

    # -- public API -----------------------------------------------------------

    def compute(self, throttle: float, turn: float) -> Tuple[float, float]:
        """
        Compute per-side motor RPM for an arcade-style input.

        Parameters
        ----------
        throttle : float
            Forward/back command in [-1.0, 1.0]. Positive is forward.
        turn : float
            Turn command in [-1.0, 1.0]. Positive turns the robot to one
            side (by convention here: positive turn slows the right side,
            so the robot yaws right when moving forward).

        Returns
        -------
        (left_motor_rpm, right_motor_rpm) : tuple of float
            Motor-shaft RPM (not wheel RPM) for each side, ready to pass to
            ``SparkController.velocity_output``. Inversion flags from the
            config are already applied.
        """
        throttle = self._apply_deadband(self._clamp(throttle, -1.0, 1.0))
        turn = self._apply_deadband(self._clamp(turn, -1.0, 1.0))

        # Arcade mix: standard tank formula.
        left_norm = throttle + turn
        right_norm = throttle - turn

        # Handle the case where left_norm or right_norm exceeds [-1, 1].
        left_norm, right_norm = self._saturate(left_norm, right_norm)

        # Map normalized command to motor RPM. The scale is the motor RPM
        # corresponding to max_linear_velocity — i.e. 1.0 -> top-speed RPM.
        scale = self.config.max_motor_rpm_from_max_velocity
        scale = min(scale, self.config.max_motor_rpm)  # respect the hard cap

        left_rpm = left_norm * scale
        right_rpm = right_norm * scale

        # Final hard clamp against max_motor_rpm as a safety net.
        left_rpm = self._clamp(
            left_rpm, -self.config.max_motor_rpm, self.config.max_motor_rpm
        )
        right_rpm = self._clamp(
            right_rpm, -self.config.max_motor_rpm, self.config.max_motor_rpm
        )

        # Apply side inversions.
        if self.config.left_inverted:
            left_rpm = -left_rpm
        if self.config.right_inverted:
            right_rpm = -right_rpm

        return left_rpm, right_rpm

    # -- helpers --------------------------------------------------------------

    def _apply_deadband(self, value: float) -> float:
        if abs(value) < self.config.deadband:
            return 0.0
        return value

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _saturate(self, left: float, right: float) -> Tuple[float, float]:
        """
        Ensure both sides fit within [-1, 1].

        In "scale" mode, if the larger magnitude exceeds 1, divide both sides
        by that magnitude so the throttle/turn ratio is preserved. In "clip"
        mode, independently clamp each side.
        """
        if self.config.saturation_mode == "clip":
            return self._clamp(left, -1.0, 1.0), self._clamp(right, -1.0, 1.0)

        # "scale" mode
        largest = max(abs(left), abs(right))
        if largest > 1.0:
            left /= largest
            right /= largest
        return left, right


# ---------------------------------------------------------------------------
# Hardware-facing drive class
# ---------------------------------------------------------------------------

class SkidSteerDrive:
    """
    Hardware-facing arcade drive for a 6-motor skid-steer chassis.

    This class wraps a ``SparkBus`` and six ``SparkController`` instances
    (three per side) and provides a small, drive-team-friendly API.

    Parameters
    ----------
    bus : SparkBus
        The CAN bus wrapper from SparkCANLib. Must expose
        ``init_controller(id)``.
    config : DriveConfig
        Physical and tuning parameters.

    Attributes
    ----------
    left_motors : list[SparkController]
        Initialized controllers for the three left-side motors.
    right_motors : list[SparkController]
        Initialized controllers for the three right-side motors.
    kinematics : SkidSteerKinematics
        The pure-math object used for every command.
    """

    def __init__(self, bus, config: DriveConfig) -> None:
        self.bus = bus
        self.config = config
        self.kinematics = SkidSteerKinematics(config)

        self.left_motors = [bus.init_controller(i) for i in config.left_motor_ids]
        self.right_motors = [bus.init_controller(i) for i in config.right_motor_ids]

    # -- primary command API --------------------------------------------------

    def arcade(self, throttle: float, turn: float) -> Tuple[float, float]:
        """
        Drive the robot arcade-style.

        Parameters
        ----------
        throttle : float
            Forward/back in [-1.0, 1.0].
        turn : float
            Turn in [-1.0, 1.0].

        Returns
        -------
        (left_rpm, right_rpm) : tuple of float
            The commanded motor RPM sent to each side. Useful for logging.
        """
        left_rpm, right_rpm = self.kinematics.compute(throttle, turn)
        self._send_side(self.left_motors, left_rpm)
        self._send_side(self.right_motors, right_rpm)
        return left_rpm, right_rpm

    def point_turn(self, rate: float) -> Tuple[float, float]:
        """
        Spin the robot in place around its own center.

        Equivalent to ``arcade(throttle=0, turn=rate)``.

        Parameters
        ----------
        rate : float
            Turn rate in [-1.0, 1.0]. Positive rotates the robot in the same
            direction that a positive ``turn`` does in :meth:`arcade`.
        """
        return self.arcade(throttle=0.0, turn=rate)

    def pivot_turn(
        self, stationary_side: Literal["left", "right"], rate: float
    ) -> Tuple[float, float]:
        """
        Pivot around one stationary side.

        Drives the non-stationary side at ``rate`` and holds the other side
        at zero, producing a turn that pivots around the stationary wheels.

        Parameters
        ----------
        stationary_side : {"left", "right"}
            The side that should stay at zero RPM and act as the pivot.
        rate : float
            Speed of the driven side in [-1.0, 1.0]. Positive drives forward,
            negative drives backward.
        """
        if stationary_side not in ("left", "right"):
            raise ValueError("stationary_side must be 'left' or 'right'")

        # arcade: left = throttle + turn, right = throttle - turn
        # right stationary -> throttle = turn = rate/2 gives left = rate, right = 0
        # left stationary  -> throttle = rate/2, turn = -rate/2 gives left = 0, right = rate
        half = rate / 2.0
        if stationary_side == "right":
            return self.arcade(throttle=half, turn=half)
        else:
            return self.arcade(throttle=half, turn=-half)

    def stop(self) -> None:
        """
        Immediately stop all motors.

        Uses ``percent_output(0)`` rather than ``velocity_output(0)`` for a
        hard, open-loop stop that works even if the velocity PID is
        misconfigured.
        """
        for motor in self.left_motors + self.right_motors:
            motor.percent_output(0.0)

    # -- telemetry ------------------------------------------------------------

    def get_wheel_velocities(self) -> Dict[int, float]:
        """
        Read current motor velocity (RPM) from every controller.

        Returns
        -------
        dict[int, float]
            Mapping of CAN ID to reported motor RPM.
        """
        out: Dict[int, float] = {}
        for motor in self.left_motors + self.right_motors:
            out[motor.id] = motor.velocity
        return out

    def get_wheel_positions(self) -> Dict[int, float]:
        """
        Read current motor position from every controller.

        Returns
        -------
        dict[int, float]
            Mapping of CAN ID to reported motor position (units as defined by
            the SparkController's ``positionProps.countConversion``).
        """
        out: Dict[int, float] = {}
        for motor in self.left_motors + self.right_motors:
            out[motor.id] = motor.position
        return out

    def get_average_side_rpm(self) -> Tuple[float, float]:
        """
        Report the mean commanded-axis RPM per side, read from the hardware.

        Averages the three motors on each side. Useful for coarse closed-loop
        checks and for future odometry.

        Returns
        -------
        (left_rpm, right_rpm) : tuple of float
            Mean motor RPM on the left and right sides. Inversion flags from
            the config are undone so the result is "chassis-frame" positive
            forward on both sides.
        """
        left_raw = sum(m.velocity for m in self.left_motors) / len(self.left_motors)
        right_raw = sum(m.velocity for m in self.right_motors) / len(self.right_motors)

        if self.config.left_inverted:
            left_raw = -left_raw
        if self.config.right_inverted:
            right_raw = -right_raw

        return left_raw, right_raw

    # -- internals ------------------------------------------------------------

    @staticmethod
    def _send_side(motors, rpm: float) -> None:
        for motor in motors:
            motor.velocity_output(rpm)
