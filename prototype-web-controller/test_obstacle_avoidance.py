import unittest

from camera_service import BandReading, CorridorReading
from obstacle_avoidance import AvoidanceConfig, CorridorAvoider


def make_band(distance, valid_ratio=1.0):
    return BandReading(min_m=distance, mean_m=distance, valid_ratio=valid_ratio)


def make_reading(left, center, right, ts=10.0, valid_ratio=1.0):
    return CorridorReading(
        left=make_band(left, valid_ratio),
        center=make_band(center, valid_ratio),
        right=make_band(right, valid_ratio),
        timestamp=ts,
    )


def make_blind_reading(ts=10.0):
    blind = BandReading(min_m=None, mean_m=None, valid_ratio=0.0)
    return CorridorReading(blind, blind, blind, ts)


def make_avoider(**overrides):
    cfg_kwargs = dict(
        cruise_throttle=0.25,
        slow_throttle=0.12,
        reverse_throttle=-0.18,
        pivot_rate=0.35,
        recovery_pivot_rate=0.2,
        stop_distance_m=1.0,
        caution_distance_m=1.6,
        clear_distance_m=2.0,
        reverse_distance_m=0.55,
        min_valid_ratio=0.35,
        reverse_seconds=0.7,
        pivot_step_s=0.6,
        no_depth_grace_s=1.0,
        no_depth_search_s=3.0,
        max_pivot_attempts=4,
        stale_seconds=0.75,
        reaction_factor=0.4,
    )
    cfg_kwargs.update(overrides)
    config = AvoidanceConfig(**cfg_kwargs)
    return CorridorAvoider(config=config, max_linear_velocity=1.0)


class TestCorridorAvoider(unittest.TestCase):
    def test_disabled_returns_manual_command(self):
        avoider = make_avoider()
        command = avoider.command(make_reading(2.5, 2.5, 2.5, ts=10.0), now=10.1)
        self.assertEqual(command.state, "manual")
        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.turn, 0.0)

    def test_clear_path_drives_forward(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(make_reading(2.5, 2.5, 2.5, ts=10.0), now=10.1)
        self.assertEqual(command.state, "cruising")
        self.assertEqual(command.throttle, 0.25)
        self.assertEqual(command.turn, 0.0)

    def test_marginal_distance_enters_slowing(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(make_reading(2.5, 1.4, 2.5, ts=10.0), now=10.1)
        self.assertEqual(command.state, "slowing")
        self.assertEqual(command.throttle, 0.12)
        self.assertEqual(command.turn, 0.0)

    def test_blocked_center_chooses_more_open_side(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(make_reading(2.5, 0.8, 1.7, ts=10.0), now=10.1)
        self.assertEqual(command.state, "searching")
        self.assertEqual(command.pivot_side, "left")
        self.assertEqual(command.attempts, 1)
        self.assertLess(command.turn, 0.0)
        self.assertEqual(command.throttle, 0.0)

    def test_blocked_center_pivots_right_when_right_clearer(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(make_reading(1.4, 0.8, 2.5, ts=10.0), now=10.1)
        self.assertEqual(command.pivot_side, "right")
        self.assertGreater(command.turn, 0.0)

    def test_emergency_distance_reverses(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(make_reading(0.4, 0.4, 0.4, ts=10.0), now=10.1)
        self.assertEqual(command.state, "reversing")
        self.assertEqual(command.throttle, -0.18)
        self.assertEqual(command.turn, 0.0)

    def test_no_depth_within_grace_holds_position(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(make_blind_reading(ts=10.1), now=10.5)
        self.assertEqual(command.state, "recovering")
        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.turn, 0.0)
        self.assertEqual(command.attempts, 0)

    def test_no_depth_after_grace_rotates_slowly(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(make_blind_reading(ts=11.5), now=11.5)
        self.assertEqual(command.state, "recovering")
        self.assertEqual(command.throttle, 0.0)
        self.assertAlmostEqual(abs(command.turn), 0.2)

    def test_no_depth_after_search_timeout_escalates_to_searching(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(make_blind_reading(ts=14.0), now=14.0)
        self.assertEqual(command.state, "searching")
        self.assertEqual(command.attempts, 1)
        self.assertAlmostEqual(abs(command.turn), 0.35)

    def test_hysteresis_does_not_flap_between_cruising_and_slowing(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)

        first = avoider.command(make_reading(2.5, 1.4, 2.5, ts=10.0), now=10.1)
        self.assertEqual(first.state, "slowing")

        # Worst rises above caution but not yet above clear -> stay slowing.
        held = avoider.command(make_reading(2.5, 1.7, 2.5, ts=10.2), now=10.3)
        self.assertEqual(held.state, "slowing")

        # Worst clears the higher hysteresis threshold -> back to cruising.
        cleared = avoider.command(make_reading(2.5, 2.1, 2.5, ts=10.4), now=10.5)
        self.assertEqual(cleared.state, "cruising")

    def test_center_blind_with_clear_sides_slows(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        reading = CorridorReading(
            left=make_band(2.5),
            center=BandReading(min_m=None, mean_m=None, valid_ratio=0.1),
            right=make_band(2.5),
            timestamp=10.0,
        )
        command = avoider.command(reading, now=10.1)
        self.assertEqual(command.state, "slowing")
        self.assertEqual(command.throttle, 0.12)

    def test_pivot_ladder_escalates_to_stuck(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)

        # Distance worth pivoting around but not so close that we always reverse.
        blocked = lambda ts: make_reading(0.7, 0.7, 0.7, ts=ts)

        now = 10.0
        attempts_seen = []
        for _ in range(8):  # plenty of time for 4 search/reverse cycles
            now += 0.05
            cmd = avoider.command(blocked(now), now=now)
            attempts_seen.append((cmd.state, cmd.attempts))
            if cmd.state == "stuck":
                break
            # Advance through the in-progress search/reverse window.
            now += 1.5
            cmd = avoider.command(blocked(now), now=now)
            attempts_seen.append((cmd.state, cmd.attempts))
            if cmd.state == "stuck":
                break

        self.assertEqual(avoider.state, "stuck")
        self.assertEqual(avoider.attempts, 4)

    def test_re_enable_resets_attempts(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        # Force several pivots to accumulate attempts.
        now = 10.0
        for _ in range(3):
            now += 0.05
            avoider.command(make_reading(0.7, 0.7, 0.7, ts=now), now=now)
            now += 1.5
            avoider.command(make_reading(0.7, 0.7, 0.7, ts=now), now=now)
        self.assertGreater(avoider.attempts, 0)

        avoider.set_enabled(False, now=now)
        avoider.set_enabled(True, now=now)
        self.assertEqual(avoider.attempts, 0)
        self.assertEqual(avoider.state, "cruising")

    def test_stop_returns_to_manual(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.stop()
        self.assertFalse(avoider.enabled)
        self.assertEqual(command.state, "manual")
        self.assertEqual(avoider.command(make_reading(2.5, 2.5, 2.5, ts=10.0), now=10.1).state, "manual")

    def test_speed_aware_stop_distance_scales_with_max_velocity(self):
        avoider = make_avoider()
        baseline = avoider.effective_stop_distance()
        avoider.max_linear_velocity = 3.0
        scaled = avoider.effective_stop_distance()
        self.assertGreater(scaled, baseline)
        # Doubling max_velocity should add reaction_factor * cruise * (3 - 1) of margin.
        expected_extra = 0.4 * 0.25 * 2.0
        self.assertAlmostEqual(scaled - baseline, expected_extra, places=4)

    def test_status_dict_includes_extended_fields(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        avoider.command(make_reading(2.5, 0.8, 1.5, ts=10.0), now=10.1)
        status = avoider.status()
        self.assertIn("attempts", status)
        self.assertIn("pivot_side", status)
        self.assertEqual(status["state"], "searching")
        self.assertEqual(status["enabled"], True)


if __name__ == "__main__":
    unittest.main()
