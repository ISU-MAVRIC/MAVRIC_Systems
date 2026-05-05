import itertools
import time
import unittest

import camera_service
from camera_service import (
    CameraGeometry,
    RealSenseCameraService,
    configure_indoor_depth_sensor,
    compute_corridor_from_depth,
)


class FakeIntrinsics:
    def __init__(self, width=9, height=7, fx=10.0, fy=10.0):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.ppx = (width - 1) / 2.0
        self.ppy = (height - 1) / 2.0


class FakeOptions:
    emitter_enabled = object()
    laser_power = object()


class FakeRs:
    option = FakeOptions()


class FakeDepthSensor:
    def __init__(self, supported):
        self.supported = set(supported)
        self.values = {}

    def supports(self, option):
        return option in self.supported

    def set_option(self, option, value):
        self.values[option] = value

    def get_option(self, option):
        return self.values[option]


class TestRealSenseCameraService(unittest.TestCase):
    def test_default_status_is_simulated_and_available(self):
        service = RealSenseCameraService()
        status = service.status()

        self.assertTrue(status.available)
        self.assertTrue(status.simulated)

    def test_simulated_mode_runs_without_realsense_dependencies(self):
        original_rs = camera_service.rs
        original_cv2 = camera_service.cv2
        original_np = camera_service.np
        camera_service.rs = None
        camera_service.cv2 = None
        camera_service.np = None
        service = RealSenseCameraService(fps=5)
        try:
            service.start()
            time.sleep(0.15)
            status = service.status()
            distance = service.distance()
            corridor = service.corridor()
            chunk = next(itertools.islice(service.mjpeg_frames(), 1))
        finally:
            service.stop()
            camera_service.rs = original_rs
            camera_service.cv2 = original_cv2
            camera_service.np = original_np

        self.assertTrue(status.available)
        self.assertTrue(status.simulated)
        self.assertIsNotNone(distance.center_m)
        self.assertIsNotNone(corridor.center.min_m)
        self.assertIn(b"Content-Type: image/jpeg", chunk)

    def test_indoor_depth_sensor_options_are_enabled_when_supported(self):
        original_rs = camera_service.rs
        camera_service.rs = FakeRs()
        sensor = FakeDepthSensor(
            [FakeOptions.emitter_enabled, FakeOptions.laser_power]
        )
        try:
            status = configure_indoor_depth_sensor(sensor, laser_power=275.0)
        finally:
            camera_service.rs = original_rs

        self.assertEqual(sensor.values[FakeOptions.emitter_enabled], 1.0)
        self.assertEqual(sensor.values[FakeOptions.laser_power], 275.0)
        self.assertIn("emitter=1", status)
        self.assertIn("laser=275", status)

    def test_indoor_depth_sensor_options_ignore_unsupported_controls(self):
        original_rs = camera_service.rs
        camera_service.rs = FakeRs()
        sensor = FakeDepthSensor([])
        try:
            status = configure_indoor_depth_sensor(sensor)
        finally:
            camera_service.rs = original_rs

        self.assertEqual(sensor.values, {})
        self.assertEqual(status, "emitter=unsupported, laser=unsupported")


@unittest.skipIf(camera_service.np is None, "numpy is required for this test")
class TestComputeCorridorFromDepth(unittest.TestCase):
    def setUp(self):
        self.np = camera_service.np

    def test_no_intrinsics_splits_full_image_into_thirds(self):
        depth = self.np.full((30, 30), 2.0, dtype="float32")
        depth[:, :10] = 0.5
        depth[:, 20:] = 3.0

        reading = compute_corridor_from_depth(
            depth, intrinsics=None, geometry=CameraGeometry(), timestamp=1.0
        )

        self.assertAlmostEqual(reading.left.min_m, 0.5, places=2)
        self.assertAlmostEqual(reading.center.min_m, 2.0, places=2)
        self.assertAlmostEqual(reading.right.min_m, 3.0, places=2)
        self.assertEqual(reading.left.valid_ratio, 1.0)
        self.assertEqual(reading.center.valid_ratio, 1.0)
        self.assertEqual(reading.right.valid_ratio, 1.0)
        self.assertEqual(reading.timestamp, 1.0)

    def test_all_invalid_pixels_yield_blind_bands(self):
        depth = self.np.zeros((20, 20), dtype="float32")
        reading = compute_corridor_from_depth(
            depth, intrinsics=None, geometry=CameraGeometry()
        )
        self.assertIsNone(reading.center.min_m)
        self.assertIsNone(reading.left.min_m)
        self.assertIsNone(reading.right.min_m)
        self.assertEqual(reading.center.valid_ratio, 0.0)

    def test_partial_validity_reports_correct_ratio(self):
        depth = self.np.zeros((10, 30), dtype="float32")
        # Right band (x in [20, 30)): half of pixels are valid at 1.5 m.
        depth[: depth.shape[0] // 2, 20:] = 1.5
        reading = compute_corridor_from_depth(
            depth, intrinsics=None, geometry=CameraGeometry()
        )
        self.assertIsNone(reading.left.min_m)
        self.assertIsNone(reading.center.min_m)
        self.assertAlmostEqual(reading.right.min_m, 1.5, places=2)
        self.assertAlmostEqual(reading.right.valid_ratio, 0.5, places=2)

    def test_intrinsics_sampler_detects_obstacle_nearer_than_lookahead(self):
        depth = self.np.full((7, 9), 2.0, dtype="float32")
        depth[2:5, 4] = 0.45
        geometry = CameraGeometry(
            rover_width_m=0.7,
            width_margin_m=0.0,
            min_depth_m=0.2,
            max_depth_m=2.0,
            lookahead_m=1.0,
            camera_forward_offset_m=0.0,
            camera_pitch_deg=0.0,
        )

        reading = compute_corridor_from_depth(
            depth, intrinsics=FakeIntrinsics(), geometry=geometry
        )

        self.assertAlmostEqual(reading.center.min_m, 0.45, places=2)

    def test_intrinsics_sampler_assigns_bands_by_lateral_position(self):
        depth = self.np.full((7, 9), 2.0, dtype="float32")
        depth[:, 2] = 0.8
        depth[:, 4] = 1.2
        depth[:, 6] = 1.6
        geometry = CameraGeometry(
            rover_width_m=2.4,
            width_margin_m=0.0,
            min_depth_m=0.2,
            max_depth_m=2.0,
            camera_forward_offset_m=0.0,
        )

        reading = compute_corridor_from_depth(
            depth, intrinsics=FakeIntrinsics(fx=3.0, fy=10.0), geometry=geometry
        )

        self.assertAlmostEqual(reading.left.min_m, 0.8, places=2)
        self.assertAlmostEqual(reading.center.min_m, 1.2, places=2)
        self.assertAlmostEqual(reading.right.min_m, 1.6, places=2)

    def test_intrinsics_sampler_rejects_outside_rover_width(self):
        depth = self.np.full((7, 9), 2.0, dtype="float32")
        depth[3, 0] = 0.4
        geometry = CameraGeometry(
            rover_width_m=0.4,
            width_margin_m=0.0,
            min_depth_m=0.2,
            max_depth_m=2.0,
            camera_forward_offset_m=0.0,
        )

        reading = compute_corridor_from_depth(
            depth, intrinsics=FakeIntrinsics(), geometry=geometry
        )

        self.assertGreater(reading.left.min_m, 1.0)
        self.assertGreater(reading.center.min_m, 1.0)
        self.assertGreater(reading.right.min_m, 1.0)

    def test_intrinsics_sampler_counts_invalid_holes_in_valid_ratio(self):
        depth = self.np.full((7, 9), 2.0, dtype="float32")
        depth[:3, 4] = 0.0
        geometry = CameraGeometry(
            rover_width_m=0.7,
            width_margin_m=0.0,
            min_depth_m=0.2,
            max_depth_m=2.0,
            camera_forward_offset_m=0.0,
        )

        reading = compute_corridor_from_depth(
            depth, intrinsics=FakeIntrinsics(), geometry=geometry
        )

        self.assertIsNotNone(reading.center.min_m)
        self.assertLess(reading.center.valid_ratio, 1.0)

    def test_pitch_changes_vertical_corridor_mask(self):
        depth = self.np.full((7, 9), 2.0, dtype="float32")
        level_geometry = CameraGeometry(
            rover_width_m=0.7,
            width_margin_m=0.0,
            min_depth_m=0.2,
            max_depth_m=2.0,
            camera_forward_offset_m=0.0,
            camera_pitch_deg=0.0,
            ceiling_clip_m=0.2,
            floor_clip_m=0.05,
        )
        pitched_geometry = CameraGeometry(
            rover_width_m=0.7,
            width_margin_m=0.0,
            min_depth_m=0.2,
            max_depth_m=2.0,
            camera_forward_offset_m=0.0,
            camera_pitch_deg=-20.0,
            ceiling_clip_m=0.2,
            floor_clip_m=0.05,
        )

        level = compute_corridor_from_depth(
            depth, intrinsics=FakeIntrinsics(), geometry=level_geometry
        )
        pitched = compute_corridor_from_depth(
            depth, intrinsics=FakeIntrinsics(), geometry=pitched_geometry
        )

        self.assertIsNotNone(level.center.min_m)
        self.assertIsNone(pitched.center.min_m)


if __name__ == "__main__":
    unittest.main()
