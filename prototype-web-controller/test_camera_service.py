import itertools
import time
import unittest

import camera_service
from camera_service import RealSenseCameraService


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
            chunk = next(itertools.islice(service.mjpeg_frames(), 1))
        finally:
            service.stop()
            camera_service.rs = original_rs
            camera_service.cv2 = original_cv2
            camera_service.np = original_np

        self.assertTrue(status.available)
        self.assertTrue(status.simulated)
        self.assertIsNotNone(distance.center_m)
        self.assertIn(b"Content-Type: image/jpeg", chunk)


if __name__ == "__main__":
    unittest.main()
