import unittest

from camera_service import BandReading, CorridorReading
from obstacle_avoidance import AvoidanceConfig, CorridorAvoider


def make_band(distance, valid_ratio=1.0, valid_pixels=1000):
    return BandReading(
        min_m=distance,
        mean_m=distance,
        valid_ratio=valid_ratio,
        valid_pixels=valid_pixels,
    )


def make_reading(left, center, right, ts=10.0, valid_ratio=1.0, valid_pixels=1000):
    return CorridorReading(
        left=make_band(left, valid_ratio, valid_pixels),
        center=make_band(center, valid_ratio, valid_pixels),
        right=make_band(right, valid_ratio, valid_pixels),
        timestamp=ts,
    )


def make_blind_reading(ts=10.0):
    blind = BandReading(min_m=None, mean_m=None, valid_ratio=0.0, valid_pixels=0)
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
        min_valid_pixels=60,
        reverse_seconds=0.7,
        pivot_step_s=2.0,
        no_depth_grace_s=1.0,
        no_depth_search_s=3.0,
        max_pivot_attempts=4,
        stale_seconds=0.3,
        reaction_factor=0.4,
        recovery_verify_s=0.3,
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

    def test_no_depth_scanning_continues_past_old_attempt_limit(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)

        for step in range(6):
            now = 14.0 + (step * 2.5)
            command = avoider.command(make_blind_reading(ts=now), now=now)

        self.assertEqual(command.state, "searching")
        self.assertGreater(command.attempts, 4)
        self.assertNotEqual(command.state, "stuck")
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

    def test_one_blind_side_with_clear_center_cruises_forward(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        reading = CorridorReading(
            left=BandReading(min_m=None, mean_m=None, valid_ratio=0.1),
            center=make_band(2.5),
            right=make_band(2.5),
            timestamp=10.0,
        )
        command = avoider.command(reading, now=10.1)
        self.assertEqual(command.state, "cruising")
        self.assertEqual(command.throttle, 0.25)
        self.assertEqual(command.turn, 0.0)
        self.assertIn("left depth missing", command.reason)

    def test_low_validity_bands_are_blind_not_clear(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)

        command = avoider.command(
            make_reading(2.5, 2.5, 2.5, ts=10.1, valid_ratio=0.2),
            now=10.5,
        )

        self.assertEqual(command.state, "recovering")
        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.turn, 0.0)

    def test_pivot_ladder_continues_past_old_attempt_limit(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)

        # Distance worth pivoting around but not so close that we always reverse.
        blocked = lambda ts: make_reading(0.7, 0.7, 0.7, ts=ts)

        now = 10.0
        states_seen = []
        for _ in range(10):
            now += 0.05
            cmd = avoider.command(blocked(now), now=now)
            states_seen.append(cmd.state)
            # Advance through the in-progress search/reverse window.
            now += 2.5
            cmd = avoider.command(blocked(now), now=now)
            states_seen.append(cmd.state)

        self.assertGreater(avoider.attempts, 4)
        self.assertNotEqual(avoider.state, "stuck")
        self.assertIn(avoider.state, ("reversing", "searching"))
        self.assertIn("searching", states_seen)
        self.assertIn("reversing", states_seen)

    def test_re_enable_resets_attempts(self):
        avoider = make_avoider()
        avoider.set_enabled(True, now=10.0)
        # Force several pivots to accumulate attempts.
        now = 10.0
        for _ in range(3):
            now += 0.05
            avoider.command(make_reading(0.7, 0.7, 0.7, ts=now), now=now)
            now += 2.5
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

    def test_high_ratio_but_few_pixels_treated_as_blind(self):
        # A glass edge can satisfy min_valid_ratio with a tiny pixel cluster.
        # The absolute-pixel-count gate must reject that band.
        avoider = make_avoider(min_valid_pixels=60)
        avoider.set_enabled(True, now=10.0)
        # All three bands satisfy valid_ratio=1.0 but only 5 valid pixels each.
        reading = make_reading(2.5, 2.5, 2.5, ts=10.0, valid_pixels=5)
        command = avoider.command(reading, now=10.1)
        # Reading is fresh (age 0.1 < stale 0.3) but the per-band gate makes
        # all bands blind -> _handle_no_depth -> recovering hold.
        self.assertEqual(command.state, "recovering")
        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.turn, 0.0)

    def test_stale_reading_above_threshold_triggers_no_depth_handling(self):
        # With stale_seconds=0.3, a reading whose timestamp is 0.4 s old must
        # be treated as if no depth was received this tick.
        avoider = make_avoider(stale_seconds=0.3)
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(make_reading(2.5, 2.5, 2.5, ts=10.1), now=10.5)
        self.assertEqual(command.state, "recovering")
        self.assertEqual(command.throttle, 0.0)

    def test_recovery_routes_through_slowing_then_promotes_to_cruising(self):
        # Cover the lens briefly, then uncover with a fully clear corridor.
        # The avoider must hold slowing for at least recovery_verify_s before
        # promoting back to cruising, even though everything is clear.
        avoider = make_avoider(recovery_verify_s=0.3)
        avoider.set_enabled(True, now=10.0)

        # Step 1: blind reading drives the avoider into recovering.
        cmd_blind = avoider.command(make_blind_reading(ts=10.4), now=10.5)
        self.assertEqual(cmd_blind.state, "recovering")
        self.assertEqual(cmd_blind.throttle, 0.0)

        # Step 2: depth recovers, all bands clear. Must enter slowing, not
        # snap to cruise.
        cmd_recover = avoider.command(make_reading(2.5, 2.5, 2.5, ts=10.6), now=10.65)
        self.assertEqual(cmd_recover.state, "slowing")
        self.assertEqual(cmd_recover.throttle, 0.12)

        # Step 3: still inside the verify window, still slowing.
        cmd_hold = avoider.command(make_reading(2.5, 2.5, 2.5, ts=10.8), now=10.85)
        self.assertEqual(cmd_hold.state, "slowing")

        # Step 4: past the verify window with a clear corridor -> cruising.
        cmd_clear = avoider.command(make_reading(2.5, 2.5, 2.5, ts=11.0), now=11.05)
        self.assertEqual(cmd_clear.state, "cruising")
        self.assertEqual(cmd_clear.throttle, 0.25)


if __name__ == "__main__":
    unittest.main()
