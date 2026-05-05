import itertools
import time
import unittest

import camera_service
from camera_service import (
    CameraGeometry,
    RealSenseCameraService,
    compute_corridor_from_depth,
)


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


if __name__ == "__main__":
    unittest.main()
