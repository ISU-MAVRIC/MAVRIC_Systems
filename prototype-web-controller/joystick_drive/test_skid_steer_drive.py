"""
Tests for skid_steer_drive.

Uses a mock SparkController and SparkBus so nothing actually touches CAN.
Focuses on the kinematics math and the dispatching behavior of
SkidSteerDrive.
"""

import math
import unittest
from dataclasses import dataclass, field
from typing import List

from skid_steer_drive import (
    DEFAULT_GEAR_RATIO,
    DEFAULT_WHEEL_DIAMETER_M,
    DriveConfig,
    SkidSteerDrive,
    SkidSteerKinematics,
)


# ---------------------------------------------------------------------------
# Mocks
# ---------------------------------------------------------------------------

class MockController:
    def __init__(self, controller_id: int):
        self.id = controller_id
        self.percent_commands: List[float] = []
        self.velocity_commands: List[float] = []
        self.position_commands: List[float] = []
        self._velocity = 0.0
        self._position = 0.0

    def percent_output(self, value):
        self.percent_commands.append(value)

    def velocity_output(self, value):
        self.velocity_commands.append(value)
        # Pretend the motor instantly achieves commanded velocity for tests.
        self._velocity = value

    def position_output(self, value):
        self.position_commands.append(value)

    @property
    def velocity(self):
        return self._velocity

    @property
    def position(self):
        return self._position


class MockBus:
    def __init__(self):
        self.controllers = {}

    def init_controller(self, controller_id: int) -> MockController:
        c = MockController(controller_id)
        self.controllers[controller_id] = c
        return c


# ---------------------------------------------------------------------------
# DriveConfig validation
# ---------------------------------------------------------------------------

class TestDriveConfig(unittest.TestCase):
    def _base_kwargs(self, **overrides):
        kwargs = dict(
            max_linear_velocity=3.0,
            left_motor_ids=[1, 2, 3],
            right_motor_ids=[4, 5, 6],
        )
        kwargs.update(overrides)
        return kwargs

    def test_valid_config_builds(self):
        cfg = DriveConfig(**self._base_kwargs())
        self.assertEqual(cfg.gear_ratio, DEFAULT_GEAR_RATIO)
        self.assertAlmostEqual(
            cfg.wheel_circumference, math.pi * DEFAULT_WHEEL_DIAMETER_M
        )

    def test_rejects_nonpositive_max_velocity(self):
        with self.assertRaises(ValueError):
            DriveConfig(**self._base_kwargs(max_linear_velocity=0))
        with self.assertRaises(ValueError):
            DriveConfig(**self._base_kwargs(max_linear_velocity=-1))

    def test_rejects_wrong_id_count(self):
        with self.assertRaises(ValueError):
            DriveConfig(**self._base_kwargs(left_motor_ids=[1, 2]))
        with self.assertRaises(ValueError):
            DriveConfig(**self._base_kwargs(right_motor_ids=[1, 2, 3, 4]))

    def test_rejects_duplicate_ids(self):
        with self.assertRaises(ValueError):
            DriveConfig(**self._base_kwargs(right_motor_ids=[1, 5, 6]))

    def test_rejects_out_of_range_deadband(self):
        with self.assertRaises(ValueError):
            DriveConfig(**self._base_kwargs(deadband=-0.1))
        with self.assertRaises(ValueError):
            DriveConfig(**self._base_kwargs(deadband=1.0))

    def test_rejects_bad_saturation_mode(self):
        with self.assertRaises(ValueError):
            DriveConfig(**self._base_kwargs(saturation_mode="nope"))

    def test_derived_rpm_values(self):
        # Pick numbers where the math is easy:
        # 10 in wheel, 20:1 gear, 1 m/s top speed.
        cfg = DriveConfig(**self._base_kwargs(max_linear_velocity=1.0))
        expected_wheel_rpm = 60.0 / (math.pi * DEFAULT_WHEEL_DIAMETER_M)
        self.assertAlmostEqual(
            cfg.max_wheel_rpm_from_max_velocity, expected_wheel_rpm
        )
        self.assertAlmostEqual(
            cfg.max_motor_rpm_from_max_velocity,
            expected_wheel_rpm * DEFAULT_GEAR_RATIO,
        )


# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------

class TestSkidSteerKinematics(unittest.TestCase):
    def _make(self, **overrides):
        kwargs = dict(
            max_linear_velocity=1.0,
            left_motor_ids=[1, 2, 3],
            right_motor_ids=[4, 5, 6],
            left_inverted=False,
            right_inverted=False,  # turn off for easier sign reasoning
        )
        kwargs.update(overrides)
        return SkidSteerKinematics(DriveConfig(**kwargs))

    def test_zero_input_is_zero(self):
        k = self._make()
        self.assertEqual(k.compute(0, 0), (0.0, 0.0))

    def test_straight_forward_full_throttle(self):
        k = self._make()
        left, right = k.compute(1.0, 0.0)
        self.assertAlmostEqual(left, right)
        self.assertGreater(left, 0)
        # Should equal motor RPM at max_linear_velocity.
        self.assertAlmostEqual(
            left, k.config.max_motor_rpm_from_max_velocity
        )

    def test_straight_reverse_full_throttle(self):
        k = self._make()
        left, right = k.compute(-1.0, 0.0)
        self.assertAlmostEqual(left, right)
        self.assertLess(left, 0)

    def test_point_turn_equal_opposite(self):
        k = self._make()
        left, right = k.compute(0.0, 1.0)
        self.assertAlmostEqual(left, -right)
        self.assertGreater(left, 0)

    def test_pivot_right_stationary(self):
        # throttle = turn => right side stops, left side drives.
        k = self._make()
        left, right = k.compute(0.5, 0.5)
        self.assertGreater(left, 0)
        self.assertAlmostEqual(right, 0.0)

    def test_pivot_left_stationary(self):
        k = self._make()
        left, right = k.compute(0.5, -0.5)
        self.assertAlmostEqual(left, 0.0)
        self.assertGreater(right, 0)

    def test_saturation_scale_preserves_ratio(self):
        k = self._make(saturation_mode="scale")
        # throttle=1, turn=1 -> raw left=2, right=0 -> scaled to 1, 0
        left, right = k.compute(1.0, 1.0)
        scale = k.config.max_motor_rpm_from_max_velocity
        self.assertAlmostEqual(left, scale)
        self.assertAlmostEqual(right, 0.0)

    def test_saturation_clip_mode(self):
        k = self._make(saturation_mode="clip")
        # throttle=1, turn=1 -> raw left=2 (clipped to 1), right=0
        left, right = k.compute(1.0, 1.0)
        scale = k.config.max_motor_rpm_from_max_velocity
        self.assertAlmostEqual(left, scale)
        self.assertAlmostEqual(right, 0.0)

    def test_saturation_clip_differs_from_scale_on_partial(self):
        # throttle=0.8, turn=0.8 -> raw left=1.6, right=0
        # scale: left=1.0, right=0.0
        # clip:  left=1.0, right=0.0
        # On this input they agree; pick one where they disagree:
        # throttle=0.6, turn=0.6 -> raw left=1.2, right=0
        # scale: left=1.0, right=0.0
        # clip:  left=1.0, right=0.0
        # Actually for any case where ONE side saturates and the other is 0,
        # both modes give the same result. Use a case where both sides are
        # nonzero and the larger saturates:
        # throttle=0.9, turn=0.3 -> raw left=1.2, right=0.6
        # scale: left=1.0, right=0.5
        # clip:  left=1.0, right=0.6
        k_scale = self._make(saturation_mode="scale")
        k_clip = self._make(saturation_mode="clip")
        ls, rs = k_scale.compute(0.9, 0.3)
        lc, rc = k_clip.compute(0.9, 0.3)
        self.assertAlmostEqual(ls, lc)  # both hit max on the left
        self.assertNotAlmostEqual(rs, rc)  # right side differs
        self.assertLess(rs, rc)  # scale mode slows the right more

    def test_deadband_zeros_small_inputs(self):
        k = self._make(deadband=0.1)
        left, right = k.compute(0.05, 0.05)
        self.assertEqual((left, right), (0.0, 0.0))

    def test_deadband_passes_through_large_inputs(self):
        k = self._make(deadband=0.1)
        left, right = k.compute(0.5, 0.0)
        self.assertGreater(left, 0)

    def test_input_clamping(self):
        k = self._make()
        # Values outside [-1, 1] should be clamped before mixing.
        left_over, right_over = k.compute(5.0, 0.0)
        left_max, right_max = k.compute(1.0, 0.0)
        self.assertAlmostEqual(left_over, left_max)
        self.assertAlmostEqual(right_over, right_max)

    def test_max_motor_rpm_hard_cap(self):
        # Set max_linear_velocity so that the derived top RPM is huge,
        # then apply a low max_motor_rpm and confirm it wins.
        k = self._make(
            max_linear_velocity=100.0,  # absurdly high -> huge derived RPM
            max_motor_rpm=1000.0,
        )
        left, right = k.compute(1.0, 0.0)
        self.assertLessEqual(abs(left), 1000.0 + 1e-9)
        self.assertLessEqual(abs(right), 1000.0 + 1e-9)

    def test_left_inversion_flips_sign(self):
        k = self._make(left_inverted=True)
        left, right = k.compute(1.0, 0.0)
        self.assertLess(left, 0)
        self.assertGreater(right, 0)

    def test_right_inversion_flips_sign(self):
        k = self._make(right_inverted=True)
        left, right = k.compute(1.0, 0.0)
        self.assertGreater(left, 0)
        self.assertLess(right, 0)


# ---------------------------------------------------------------------------
# Hardware-facing drive class
# ---------------------------------------------------------------------------

class TestSkidSteerDrive(unittest.TestCase):
    def _make(self, **overrides):
        kwargs = dict(
            max_linear_velocity=1.0,
            left_motor_ids=[1, 2, 3],
            right_motor_ids=[4, 5, 6],
            left_inverted=False,
            right_inverted=False,
        )
        kwargs.update(overrides)
        bus = MockBus()
        drive = SkidSteerDrive(bus, DriveConfig(**kwargs))
        return bus, drive

    def test_init_creates_six_controllers(self):
        bus, drive = self._make()
        self.assertEqual(len(bus.controllers), 6)
        self.assertEqual([m.id for m in drive.left_motors], [1, 2, 3])
        self.assertEqual([m.id for m in drive.right_motors], [4, 5, 6])

    def test_arcade_fans_out_to_three_motors_per_side(self):
        bus, drive = self._make()
        left_rpm, right_rpm = drive.arcade(throttle=0.5, turn=0.0)

        for motor in drive.left_motors:
            self.assertEqual(motor.velocity_commands, [left_rpm])
        for motor in drive.right_motors:
            self.assertEqual(motor.velocity_commands, [right_rpm])

    def test_stop_sends_zero_percent_to_all(self):
        bus, drive = self._make()
        drive.arcade(1.0, 0.0)
        drive.stop()
        for motor in drive.left_motors + drive.right_motors:
            self.assertEqual(motor.percent_commands, [0.0])

    def test_point_turn_zero_throttle(self):
        bus, drive = self._make()
        left_rpm, right_rpm = drive.point_turn(0.5)
        self.assertAlmostEqual(left_rpm, -right_rpm)
        self.assertGreater(left_rpm, 0)

    def test_pivot_turn_right_stationary(self):
        bus, drive = self._make()
        left_rpm, right_rpm = drive.pivot_turn("right", 1.0)
        self.assertAlmostEqual(right_rpm, 0.0)
        self.assertGreater(left_rpm, 0)

    def test_pivot_turn_left_stationary(self):
        bus, drive = self._make()
        left_rpm, right_rpm = drive.pivot_turn("left", 1.0)
        self.assertAlmostEqual(left_rpm, 0.0)
        self.assertGreater(right_rpm, 0)

    def test_pivot_turn_reverse(self):
        bus, drive = self._make()
        left_rpm, right_rpm = drive.pivot_turn("right", -0.5)
        self.assertAlmostEqual(right_rpm, 0.0)
        self.assertLess(left_rpm, 0)

    def test_pivot_turn_invalid_side(self):
        bus, drive = self._make()
        with self.assertRaises(ValueError):
            drive.pivot_turn("middle", 0.5)

    def test_get_wheel_velocities_returns_all_six(self):
        bus, drive = self._make()
        drive.arcade(0.5, 0.0)
        velocities = drive.get_wheel_velocities()
        self.assertEqual(set(velocities.keys()), {1, 2, 3, 4, 5, 6})

    def test_get_average_side_rpm_undoes_inversion(self):
        # With right_inverted=True, commanding forward throttle sends
        # *negative* RPM to the physical right motors. get_average_side_rpm
        # should report it as positive again in the chassis frame.
        bus, drive = self._make(right_inverted=True)
        drive.arcade(0.5, 0.0)
        left_avg, right_avg = drive.get_average_side_rpm()
        self.assertGreater(left_avg, 0)
        self.assertGreater(right_avg, 0)
        self.assertAlmostEqual(left_avg, right_avg)


if __name__ == "__main__":
    unittest.main()
