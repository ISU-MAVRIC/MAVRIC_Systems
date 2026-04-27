import unittest

from obstacle_avoidance import AvoidanceConfig, DumbObstacleAvoider


class TestDumbObstacleAvoider(unittest.TestCase):
    def make_avoider(self):
        return DumbObstacleAvoider(
            AvoidanceConfig(
                stop_threshold_m=0.6,
                clear_threshold_m=0.9,
                forward_throttle=0.25,
                reverse_throttle=-0.2,
                pivot_rate=0.3,
                reverse_seconds=0.7,
                pivot_seconds=0.9,
            )
        )

    def test_clear_path_drives_forward(self):
        avoider = self.make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(1.2, now=10.1)

        self.assertEqual(command.state, "active")
        self.assertEqual(command.throttle, 0.25)
        self.assertEqual(command.turn, 0.0)

    def test_close_obstacle_reverses_then_pivots(self):
        avoider = self.make_avoider()
        avoider.set_enabled(True, now=10.0)

        reverse = avoider.command(0.4, now=10.1)
        pivot = avoider.command(1.2, now=11.0)

        self.assertEqual(reverse.state, "reversing")
        self.assertEqual(reverse.throttle, -0.2)
        self.assertEqual(pivot.state, "pivoting")
        self.assertEqual(pivot.throttle, 0.0)
        self.assertNotEqual(pivot.turn, 0.0)

    def test_unclear_path_stops_without_reversing(self):
        avoider = self.make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(0.75, now=10.1)

        self.assertEqual(command.state, "blocked")
        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.turn, 0.0)

    def test_missing_depth_fails_safe(self):
        avoider = self.make_avoider()
        avoider.set_enabled(True, now=10.0)
        command = avoider.command(None, now=10.1)

        self.assertEqual(command.state, "stopped")
        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.turn, 0.0)

    def test_stop_disables_avoidance(self):
        avoider = self.make_avoider()
        avoider.set_enabled(True, now=10.0)
        avoider.stop()

        self.assertFalse(avoider.enabled)
        self.assertEqual(avoider.command(1.2, now=10.1).state, "manual")


if __name__ == "__main__":
    unittest.main()
