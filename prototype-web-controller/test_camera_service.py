import itertools
import logging
import time
import unittest

import camera_service
from camera_service import (
    CameraGeometry,
    RealSenseCameraService,
    configure_navigation_filters,
    configure_indoor_depth_sensor,
    compute_corridor_from_depth,
    load_camera_geometry_from_env,
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
    visual_preset = object()
    enable_auto_exposure = object()
    emitter_enabled = object()
    laser_power = object()
    holes_fill = object()


class FakeRs:
    option = FakeOptions()


class FakeDepthSensor:
    def __init__(self, supported, option_ranges=None):
        self.supported = set(supported)
        self.option_ranges = option_ranges or {}
        self.values = {}

    def supports(self, option):
        return option in self.supported

    def set_option(self, option, value):
        self.values[option] = value

    def get_option(self, option):
        return self.values[option]

    def get_option_range(self, option):
        return self.option_ranges[option]


class FakeOptionRange:
    def __init__(self, minimum, maximum):
        self.min = minimum
        self.max = maximum


class FakeProcessingBlock:
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
        self.assertFalse(status.avoidance_usable)

    def test_default_depth_profile_is_d435_navigation_profile(self):
        service = RealSenseCameraService()

        self.assertEqual(service.width, 848)
        self.assertEqual(service.height, 480)
        self.assertEqual(service.fps, 30)

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
        self.assertFalse(status.avoidance_usable)
        self.assertIsNotNone(distance.center_m)
        self.assertIsNotNone(corridor.center.min_m)
        self.assertIn(b"Content-Type: image/jpeg", chunk)

    def test_indoor_depth_sensor_options_are_enabled_when_supported(self):
        original_rs = camera_service.rs
        camera_service.rs = FakeRs()
        sensor = FakeDepthSensor(
            [
                FakeOptions.visual_preset,
                FakeOptions.enable_auto_exposure,
                FakeOptions.emitter_enabled,
                FakeOptions.laser_power,
            ],
            {FakeOptions.laser_power: FakeOptionRange(0.0, 360.0)},
        )
        try:
            status = configure_indoor_depth_sensor(sensor, laser_power=275.0)
        finally:
            camera_service.rs = original_rs

        self.assertEqual(sensor.values[FakeOptions.visual_preset], 3.0)
        self.assertEqual(sensor.values[FakeOptions.enable_auto_exposure], 1.0)
        self.assertEqual(sensor.values[FakeOptions.emitter_enabled], 1.0)
        self.assertEqual(sensor.values[FakeOptions.laser_power], 275.0)
        self.assertIn("visual_preset=3", status)
        self.assertIn("auto_exposure=1", status)
        self.assertIn("emitter=1", status)
        self.assertIn("laser=275", status)

    def test_indoor_depth_sensor_laser_power_is_clamped_to_supported_range(self):
        original_rs = camera_service.rs
        camera_service.rs = FakeRs()
        sensor = FakeDepthSensor(
            [FakeOptions.laser_power],
            {FakeOptions.laser_power: FakeOptionRange(0.0, 240.0)},
        )
        try:
            status = configure_indoor_depth_sensor(sensor, laser_power=300.0)
        finally:
            camera_service.rs = original_rs

        self.assertEqual(sensor.values[FakeOptions.laser_power], 240.0)
        self.assertIn("laser=240", status)

    def test_indoor_depth_sensor_options_ignore_unsupported_controls(self):
        original_rs = camera_service.rs
        camera_service.rs = FakeRs()
        sensor = FakeDepthSensor([])
        try:
            status = configure_indoor_depth_sensor(sensor)
        finally:
            camera_service.rs = original_rs

        self.assertEqual(sensor.values, {})
        self.assertEqual(
            status,
            (
                "visual_preset=unsupported, auto_exposure=unsupported, "
                "emitter=unsupported, laser=unsupported"
            ),
        )

    def test_navigation_filters_disable_temporal_persistence(self):
        original_rs = camera_service.rs
        camera_service.rs = FakeRs()
        temporal = FakeProcessingBlock([FakeOptions.holes_fill])
        try:
            status = configure_navigation_filters(temporal)
        finally:
            camera_service.rs = original_rs

        self.assertEqual(temporal.values[FakeOptions.holes_fill], 0.0)
        self.assertEqual(status, "temporal_persistence=0")

    def test_geometry_env_requires_all_measured_values(self):
        geometry = load_camera_geometry_from_env({})

        self.assertFalse(geometry.measured)

    def test_geometry_env_loads_measured_values(self):
        geometry = load_camera_geometry_from_env(
            {
                "MAVRIC_ROVER_WIDTH_M": "0.82",
                "MAVRIC_CAMERA_HEIGHT_M": "0.46",
                "MAVRIC_CAMERA_FORWARD_OFFSET_M": "0.18",
                "MAVRIC_CAMERA_PITCH_DEG": "-8",
                "MAVRIC_LOOKAHEAD_M": "1.4",
                "MAVRIC_WIDTH_MARGIN_M": "0.12",
            }
        )

        self.assertTrue(geometry.measured)
        self.assertAlmostEqual(geometry.rover_width_m, 0.82)
        self.assertAlmostEqual(geometry.camera_height_m, 0.46)
        self.assertAlmostEqual(geometry.camera_forward_offset_m, 0.18)
        self.assertAlmostEqual(geometry.camera_pitch_deg, -8.0)
        self.assertAlmostEqual(geometry.lookahead_m, 1.4)
        self.assertAlmostEqual(geometry.width_margin_m, 0.12)

    def test_geometry_warns_when_bumper_blind_zone_is_large(self):
        # camera_forward_offset=0.0 + min_depth=0.20 -> 0.20 m bumper-blind.
        with self.assertLogs("web-controller.camera", level="WARNING") as logs:
            load_camera_geometry_from_env(
                {
                    "MAVRIC_ROVER_WIDTH_M": "0.7",
                    "MAVRIC_CAMERA_HEIGHT_M": "0.46",
                    "MAVRIC_CAMERA_FORWARD_OFFSET_M": "0.0",
                    "MAVRIC_CAMERA_PITCH_DEG": "0",
                    "MAVRIC_LOOKAHEAD_M": "1.0",
                }
            )
        self.assertTrue(
            any("Bumper-blind region" in line for line in logs.output),
            f"expected bumper-blind warning, got {logs.output!r}",
        )

    def test_default_min_depth_matches_d435_reliable_floor(self):
        geometry = CameraGeometry()
        self.assertAlmostEqual(geometry.min_depth_m, 0.20)


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

    def test_band_metrics_populate_valid_pixels(self):
        depth = self.np.full((10, 30), 1.5, dtype="float32")
        depth[:, :10] = 0.0  # left band entirely invalid
        reading = compute_corridor_from_depth(
            depth, intrinsics=None, geometry=CameraGeometry()
        )
        self.assertEqual(reading.left.valid_pixels, 0)
        self.assertGreater(reading.center.valid_pixels, 0)
        self.assertGreater(reading.right.valid_pixels, 0)

    def test_corridor_as_dict_includes_depth_health(self):
        depth = self.np.full((10, 30), 1.5, dtype="float32")
        depth[:, :10] = 0.0  # left band invalid -> worst-band valid_ratio = 0
        reading = compute_corridor_from_depth(
            depth, intrinsics=None, geometry=CameraGeometry()
        )
        as_dict = reading.as_dict()
        self.assertIn("depth_health", as_dict)
        self.assertAlmostEqual(as_dict["depth_health"], 0.0)
        self.assertIn("valid_pixels", as_dict["center"])
        self.assertGreater(as_dict["center"]["valid_pixels"], 0)

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
