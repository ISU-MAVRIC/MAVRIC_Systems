from __future__ import annotations

import base64
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Iterator, Optional, Sequence, Tuple

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
class CameraGeometry:
    """Physical geometry of the rover and camera mount.

    Coordinate convention follows pyrealsense2: +x right, +y down, +z forward.
    The camera is assumed to sit on the rover centerline (no horizontal offset)
    and to look forward with negligible roll. ``camera_pitch_deg`` and
    ``camera_forward_offset_m`` are accepted as configuration but are not yet
    applied to the corridor projection — they exist so callers can record the
    actual geometry and we can wire them in once measured on hardware.
    """

    rover_width_m: float = 0.7
    camera_height_m: float = 0.5
    camera_forward_offset_m: float = 0.1
    camera_pitch_deg: float = 0.0
    lookahead_m: float = 1.0
    floor_clip_m: float = 0.05
    ceiling_clip_m: float = 0.4


@dataclass(frozen=True)
class BandReading:
    """Aggregated depth metrics for one corridor band."""

    min_m: Optional[float]
    mean_m: Optional[float]
    valid_ratio: float

    def as_dict(self) -> dict:
        return {
            "min_m": self.min_m,
            "mean_m": self.mean_m,
            "valid_ratio": round(self.valid_ratio, 3),
        }


_EMPTY_BAND = BandReading(min_m=None, mean_m=None, valid_ratio=0.0)


@dataclass(frozen=True)
class CorridorReading:
    """Three-band depth view of the rover's forward corridor."""

    left: BandReading
    center: BandReading
    right: BandReading
    timestamp: float

    def as_dict(self) -> dict:
        return {
            "left": self.left.as_dict(),
            "center": self.center.as_dict(),
            "right": self.right.as_dict(),
            "timestamp": self.timestamp,
        }


@dataclass(frozen=True)
class DistanceReading:
    """Backward-compatible thin summary derived from the corridor reading."""

    center_m: Optional[float]
    min_m: Optional[float]
    timestamp: float

    def as_dict(self) -> dict:
        return {
            "center_m": self.center_m,
            "min_m": self.min_m,
            "timestamp": self.timestamp,
        }


def _min_of(values: Iterable[Optional[float]]) -> Optional[float]:
    valid = [v for v in values if v is not None]
    if not valid:
        return None
    return round(min(valid), 3)


def _project_point(intrinsics, point: Sequence[float]) -> Optional[Tuple[float, float]]:
    if rs is None or intrinsics is None:
        return None
    try:
        px, py = rs.rs2_project_point_to_pixel(intrinsics, list(point))
    except Exception:
        return None
    return float(px), float(py)


def _band_metrics(depth_band) -> BandReading:
    """Reduce a 2D depth slice (meters) to a BandReading."""
    if np is None or depth_band.size == 0:
        return _EMPTY_BAND
    valid = depth_band[(depth_band > 0.0) & (depth_band < 10.0)]
    valid_ratio = float(valid.size) / float(depth_band.size)
    if valid.size == 0:
        return BandReading(min_m=None, mean_m=None, valid_ratio=valid_ratio)
    min_m = float(np.percentile(valid, 10))
    mean_m = float(valid.mean())
    return BandReading(
        min_m=round(min_m, 3),
        mean_m=round(mean_m, 3),
        valid_ratio=valid_ratio,
    )


def compute_corridor_from_depth(
    depth_m,
    intrinsics,
    geometry: CameraGeometry,
    *,
    timestamp: Optional[float] = None,
) -> CorridorReading:
    """Compute a CorridorReading from a numpy depth array (meters).

    Falls back to splitting the full image into thirds when intrinsics are not
    available, so the same helper works for both the live RealSense pipeline
    and synthetic test data.
    """
    if timestamp is None:
        timestamp = time.time()

    if np is None or depth_m.size == 0:
        return CorridorReading(_EMPTY_BAND, _EMPTY_BAND, _EMPTY_BAND, timestamp)

    height, width = depth_m.shape[:2]

    look = max(geometry.lookahead_m, 0.05)
    half_w = geometry.rover_width_m / 2.0
    left_proj = _project_point(intrinsics, (-half_w, 0.0, look))
    right_proj = _project_point(intrinsics, (half_w, 0.0, look))
    top_proj = _project_point(intrinsics, (0.0, -geometry.ceiling_clip_m, look))
    bottom_y_world = max(geometry.camera_height_m - geometry.floor_clip_m, 0.01)
    bottom_proj = _project_point(intrinsics, (0.0, bottom_y_world, look))

    if left_proj is None or right_proj is None:
        x_left, x_right = 0, width
    else:
        x_left = int(max(0, math.floor(min(left_proj[0], right_proj[0]))))
        x_right = int(min(width, math.ceil(max(left_proj[0], right_proj[0]))))

    if top_proj is None or bottom_proj is None:
        y_top, y_bottom = 0, height
    else:
        y_top = int(max(0, math.floor(min(top_proj[1], bottom_proj[1]))))
        y_bottom = int(min(height, math.ceil(max(top_proj[1], bottom_proj[1]))))

    if x_right - x_left < 3 or y_bottom - y_top < 1:
        return CorridorReading(_EMPTY_BAND, _EMPTY_BAND, _EMPTY_BAND, timestamp)

    band_w = x_right - x_left
    third = max(1, band_w // 3)
    left_slice = depth_m[y_top:y_bottom, x_left : x_left + third]
    center_slice = depth_m[y_top:y_bottom, x_left + third : x_left + 2 * third]
    right_slice = depth_m[y_top:y_bottom, x_left + 2 * third : x_right]

    return CorridorReading(
        left=_band_metrics(left_slice),
        center=_band_metrics(center_slice),
        right=_band_metrics(right_slice),
        timestamp=timestamp,
    )


def _distance_from_corridor(reading: CorridorReading) -> DistanceReading:
    overall_min = _min_of(
        (reading.left.min_m, reading.center.min_m, reading.right.min_m)
    )
    return DistanceReading(reading.center.min_m, overall_min, reading.timestamp)


class RealSenseCameraService:
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        geometry: Optional[CameraGeometry] = None,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.geometry = geometry or CameraGeometry()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._frame_jpeg = _PLACEHOLDER_JPEG
        seed_band = BandReading(min_m=1.5, mean_m=1.5, valid_ratio=1.0)
        now = time.time()
        self._corridor = CorridorReading(seed_band, seed_band, seed_band, now)
        self._distance = _distance_from_corridor(self._corridor)
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

    def corridor(self) -> CorridorReading:
        with self._lock:
            return self._corridor

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
            profile = pipeline.start(config)
        except Exception as exc:
            _log.warning("RealSense unavailable, using simulated camera: %s", exc)
            self._run_simulated(f"RealSense unavailable: {exc}")
            return

        # Standard D400-series filter chain. Order matters: decimate first so
        # downstream filters touch fewer pixels; spatial smooths within a frame;
        # temporal smooths across frames; hole-fill is the last resort for
        # pixels that are still invalid.
        decimate = rs.decimation_filter(2)
        spatial = rs.spatial_filter()
        temporal = rs.temporal_filter()
        hole_fill = rs.hole_filling_filter(1)

        try:
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = float(depth_sensor.get_depth_scale())
        except Exception:
            depth_scale = 0.001  # D435 default: 1 mm per unit

        self._set_status(CameraStatus(True, False, "RealSense D435 stream active."))
        try:
            while not self._stop.is_set():
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                aligned = align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                filtered = depth_frame
                for stage in (decimate, spatial, temporal, hole_fill):
                    filtered = stage.process(filtered)

                try:
                    depth_video = filtered.as_video_frame()
                    intrinsics = (
                        depth_video.get_profile()
                        .as_video_stream_profile()
                        .get_intrinsics()
                    )
                    depth_raw = np.asanyarray(depth_video.get_data()).astype("float32")
                except Exception:
                    intrinsics = None
                    depth_raw = np.asanyarray(filtered.get_data()).astype("float32")

                depth_m = depth_raw * depth_scale
                reading = compute_corridor_from_depth(
                    depth_m, intrinsics, self.geometry, timestamp=time.time()
                )

                color_image = np.asanyarray(color_frame.get_data())
                self._draw_corridor_overlay(color_image, reading)
                ok, encoded = cv2.imencode(
                    ".jpg", color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                )
                if not ok:
                    continue

                with self._lock:
                    self._frame_jpeg = encoded.tobytes()
                    self._corridor = reading
                    self._distance = _distance_from_corridor(reading)
        except Exception as exc:
            _log.exception("RealSense stream failed; falling back to simulation: %s", exc)
            self._run_simulated(f"RealSense stream failed: {exc}")
        finally:
            pipeline.stop()

    def _run_simulated(self, message: str) -> None:
        self._set_status(CameraStatus(True, True, message))
        while not self._stop.is_set():
            now = time.time()
            base = 1.6 + 0.6 * math.sin(now / 4.0)
            left = max(0.25, base + 0.4 * math.sin(now / 3.1 + 1.0))
            right = max(0.25, base + 0.4 * math.sin(now / 2.7 - 1.4))
            center = max(0.25, base)
            self._set_simulated_frame(center)
            band = lambda v: BandReading(round(v, 2), round(v, 2), 1.0)
            reading = CorridorReading(
                left=band(left),
                center=band(center),
                right=band(right),
                timestamp=now,
            )
            with self._lock:
                self._corridor = reading
                self._distance = _distance_from_corridor(reading)
            time.sleep(1.0 / min(self.fps, 10))

    def _set_simulated_frame(self, center_m: float) -> None:
        if cv2 is None or np is None:
            with self._lock:
                self._frame_jpeg = _PLACEHOLDER_JPEG
            return

        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[:] = (35, 35, 35)
        color = (80, 180, 80) if center_m >= 1.0 else (70, 70, 210)
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

    def _draw_corridor_overlay(self, image, reading: CorridorReading) -> None:
        if cv2 is None or np is None:
            return
        h, w = image.shape[:2]
        third = w // 3
        for idx, band in enumerate(
            (reading.left, reading.center, reading.right)
        ):
            x0 = idx * third
            x1 = (idx + 1) * third if idx < 2 else w
            d = band.min_m
            if d is None:
                color = (60, 60, 200)  # red-ish for invalid
                label = "--"
            elif d < 1.0:
                color = (40, 40, 220)
                label = f"{d:.2f}m"
            elif d < 1.6:
                color = (60, 200, 220)
                label = f"{d:.2f}m"
            else:
                color = (80, 200, 80)
                label = f"{d:.2f}m"
            cv2.rectangle(image, (x0, h - 4), (x1, h), color, -1)
            cv2.putText(
                image,
                label,
                (x0 + 6, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (240, 240, 240),
                1,
                cv2.LINE_AA,
            )

    def _set_status(self, status: CameraStatus) -> None:
        with self._lock:
            self._status = status
