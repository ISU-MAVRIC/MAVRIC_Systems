from __future__ import annotations

import base64
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Iterator, Optional

try:
    import cv2
except Exception:  # pragma: no cover - depends on optional runtime package
    cv2 = None

try:
    import numpy as np
except Exception:  # pragma: no cover - depends on optional runtime package
    np = None

try:
    import pyrealsense2 as rs
except Exception:  # pragma: no cover - RealSense is optional in development
    rs = None


_log = logging.getLogger("web-controller.camera")

_PLACEHOLDER_JPEG = base64.b64decode(
    b"/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAP//////////////////////////////////////////////////////////////////////////////////////"
    b"////////////////////////////////////////////2wBDAf//////////////////////////////////////////////////////////////////////////////////////"
    b"////////////////////////////////////////////wAARCAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAX/xAAUEAEAAAAAAAAAAAAAAAAAAAAA"
    b"/9oADAMBAAIQAxAAAAH/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAEFAqf/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oACAEDAQE/AV//xAAUEQEAAAAAAAAA"
    b"AAAAAAAAAAAA/9oACAECAQE/AV//xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAY/Al//xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAE/IV//2gAMAwEA"
    b"AhEDEQA/AP/EABQRAQAAAAAAAAAAAAAAAAAAABD/2gAIAQMBAT8QH//EABQRAQAAAAAAAAAAAAAAAAAAABD/2gAIAQIBAT8QH//EABQQAQAAAAAAAAAAAAAAAAAA"
    b"ABD/2gAIAQEAAT8QH//Z"
)


@dataclass(frozen=True)
class CameraStatus:
    available: bool
    simulated: bool
    message: str

    def as_dict(self) -> dict:
        return {
            "available": self.available,
            "simulated": self.simulated,
            "message": self.message,
        }


@dataclass(frozen=True)
class DistanceReading:
    center_m: Optional[float]
    min_m: Optional[float]
    timestamp: float

    def as_dict(self) -> dict:
        return {
            "center_m": self.center_m,
            "min_m": self.min_m,
            "timestamp": self.timestamp,
        }


class RealSenseCameraService:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._frame_jpeg = _PLACEHOLDER_JPEG
        self._distance = DistanceReading(center_m=1.5, min_m=1.2, timestamp=time.time())
        self._status = CameraStatus(
            available=True,
            simulated=True,
            message="Simulated camera stream; no RealSense frame has been read.",
        )

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def status(self) -> CameraStatus:
        with self._lock:
            return self._status

    def distance(self) -> DistanceReading:
        with self._lock:
            return self._distance

    def mjpeg_frames(self) -> Iterator[bytes]:
        while not self._stop.is_set():
            with self._lock:
                frame = self._frame_jpeg
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(1.0 / min(self.fps, 15))

    def _run(self) -> None:
        if rs is None or cv2 is None or np is None:
            self._run_simulated("RealSense Python/OpenCV dependencies are unavailable.")
            return

        try:
            if len(rs.context().query_devices()) == 0:
                self._run_simulated("No RealSense device connected.")
                return
        except Exception as exc:
            self._run_simulated(f"Unable to query RealSense devices: {exc}")
            return

        pipeline = rs.pipeline()
        align = rs.align(rs.stream.color)
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        try:
            pipeline.start(config)
        except Exception as exc:
            _log.warning("RealSense unavailable, using simulated camera: %s", exc)
            self._run_simulated(f"RealSense unavailable: {exc}")
            return

        self._set_status(CameraStatus(True, False, "RealSense D435 stream active."))
        try:
            while not self._stop.is_set():
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                aligned = align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                center_m = self._center_distance(depth_frame)
                min_m = self._min_distance(depth_frame)
                ok, encoded = cv2.imencode(".jpg", color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    continue

                with self._lock:
                    self._frame_jpeg = encoded.tobytes()
                    self._distance = DistanceReading(center_m, min_m, time.time())
        except Exception as exc:
            _log.exception("RealSense stream failed; falling back to simulation: %s", exc)
            self._run_simulated(f"RealSense stream failed: {exc}")
        finally:
            pipeline.stop()

    def _run_simulated(self, message: str) -> None:
        self._set_status(CameraStatus(True, True, message))
        while not self._stop.is_set():
            now = time.time()
            center = 1.1 + 0.5 * math.sin(now / 4.0)
            self._set_simulated_frame(center)
            with self._lock:
                self._distance = DistanceReading(round(center, 2), round(max(0.25, center - 0.2), 2), now)
            time.sleep(1.0 / min(self.fps, 10))

    def _set_simulated_frame(self, center_m: float) -> None:
        if cv2 is None or np is None:
            with self._lock:
                self._frame_jpeg = _PLACEHOLDER_JPEG
            return

        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[:] = (35, 35, 35)
        color = (80, 180, 80) if center_m >= 0.6 else (70, 70, 210)
        cv2.rectangle(image, (0, 0), (self.width - 1, self.height - 1), color, 8)
        cv2.drawMarker(
            image,
            (self.width // 2, self.height // 2),
            (240, 240, 240),
            cv2.MARKER_CROSS,
            42,
            2,
        )
        cv2.putText(
            image,
            "SIMULATED REALSENSE",
            (28, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (235, 235, 235),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"Center: {center_m:.2f} m",
            (28, 86),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (200, 230, 255),
            2,
            cv2.LINE_AA,
        )
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        with self._lock:
            self._frame_jpeg = encoded.tobytes() if ok else _PLACEHOLDER_JPEG

    def _set_status(self, status: CameraStatus) -> None:
        with self._lock:
            self._status = status

    @staticmethod
    def _center_distance(depth_frame) -> Optional[float]:
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        samples = []
        for dx in (-8, 0, 8):
            for dy in (-8, 0, 8):
                distance = depth_frame.get_distance(width // 2 + dx, height // 2 + dy)
                if 0.0 < distance < 10.0:
                    samples.append(distance)
        if not samples:
            return None
        return round(float(sum(samples) / len(samples)), 3)

    @staticmethod
    def _min_distance(depth_frame) -> Optional[float]:
        if np is None:
            return None
        depth = np.asanyarray(depth_frame.get_data()).astype("float32")
        depth *= float(depth_frame.get_units())
        valid = depth[(depth > 0.0) & (depth < 10.0)]
        if valid.size == 0:
            return None
        return round(float(np.percentile(valid, 5)), 3)
