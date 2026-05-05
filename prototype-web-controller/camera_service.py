from __future__ import annotations

import base64
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Iterator, Optional

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
    The camera is assumed to sit on the rover centerline with negligible roll.
    The corridor sampler rotates camera points by ``camera_pitch_deg`` into the
    rover frame, then evaluates the swept rover footprint from near to far.
    """

    rover_width_m: float = 0.7
    camera_height_m: float = 0.5
    camera_forward_offset_m: float = 0.1
    camera_pitch_deg: float = 0.0
    lookahead_m: float = 1.0
    min_depth_m: float = 0.15
    max_depth_m: float = 2.0
    width_margin_m: float = 0.08
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


def _intrinsics_values(intrinsics) -> Optional[tuple[float, float, float, float]]:
    if intrinsics is None:
        return None
    try:
        fx = float(intrinsics.fx)
        fy = float(intrinsics.fy)
        ppx = float(intrinsics.ppx)
        ppy = float(intrinsics.ppy)
    except Exception:
        return None
    if fx <= 0.0 or fy <= 0.0:
        return None
    return fx, fy, ppx, ppy


def _fallback_corridor_from_image_thirds(depth_m, timestamp: float) -> CorridorReading:
    height, width = depth_m.shape[:2]
    if width < 3 or height < 1:
        return CorridorReading(_EMPTY_BAND, _EMPTY_BAND, _EMPTY_BAND, timestamp)

    third = max(1, width // 3)
    left_slice = depth_m[:, :third]
    center_slice = depth_m[:, third : third * 2]
    right_slice = depth_m[:, third * 2 :]
    return CorridorReading(
        left=_band_metrics(left_slice),
        center=_band_metrics(center_slice),
        right=_band_metrics(right_slice),
        timestamp=timestamp,
    )


def _band_metrics_from_masked_depths(depth_m, mask) -> BandReading:
    if np is None:
        return _EMPTY_BAND
    candidate_count = int(mask.sum())
    if candidate_count == 0:
        return _EMPTY_BAND
    valid_depths = depth_m[mask]
    valid = valid_depths[(valid_depths > 0.0) & (valid_depths < 10.0)]
    valid_ratio = float(valid.size) / float(candidate_count)
    if valid.size == 0:
        return BandReading(min_m=None, mean_m=None, valid_ratio=valid_ratio)
    return BandReading(
        min_m=round(float(np.percentile(valid, 10)), 3),
        mean_m=round(float(valid.mean()), 3),
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

    With intrinsics, every valid depth pixel is deprojected into camera-space,
    rotated into the rover frame, and tested against the swept rover footprint.
    Without intrinsics, this falls back to splitting the full image into thirds
    so synthetic tests and simulated data keep working.
    """
    if timestamp is None:
        timestamp = time.time()

    if np is None or depth_m.size == 0:
        return CorridorReading(_EMPTY_BAND, _EMPTY_BAND, _EMPTY_BAND, timestamp)

    values = _intrinsics_values(intrinsics)
    if values is None:
        return _fallback_corridor_from_image_thirds(depth_m, timestamp)

    height, width = depth_m.shape[:2]
    if width < 3 or height < 1:
        return CorridorReading(_EMPTY_BAND, _EMPTY_BAND, _EMPTY_BAND, timestamp)

    fx, fy, ppx, ppy = values
    ys, xs = np.indices((height, width), dtype="float32")
    z_cam = depth_m.astype("float32", copy=False)

    pitch = math.radians(geometry.camera_pitch_deg)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)

    half_width = max(geometry.rover_width_m / 2.0 + geometry.width_margin_m, 0.01)
    min_depth = max(geometry.min_depth_m, 0.01)
    max_depth = max(geometry.max_depth_m, geometry.lookahead_m, min_depth)
    ground_y = float(geometry.camera_height_m)
    y_min = -float(geometry.ceiling_clip_m)
    y_max = max(ground_y - float(geometry.floor_clip_m), y_min)

    valid_depth = (z_cam > 0.0) & (z_cam < 10.0)
    sample_depth = max(min(geometry.lookahead_m, max_depth), min_depth)
    sample_z_cam = max(sample_depth - geometry.camera_forward_offset_m, 0.01)
    z_for_mask = np.where(valid_depth, z_cam, sample_z_cam).astype("float32")
    x_for_mask = (xs - ppx) / fx * z_for_mask
    y_for_mask = (ys - ppy) / fy * z_for_mask
    y_rover = (cos_p * y_for_mask) - (sin_p * z_for_mask)
    z_rover = (
        (sin_p * y_for_mask)
        + (cos_p * z_for_mask)
        + geometry.camera_forward_offset_m
    )

    corridor = (
        (z_rover >= min_depth)
        & (z_rover <= max_depth)
        & (np.abs(x_for_mask) <= half_width)
        & (y_rover >= y_min)
        & (y_rover <= y_max)
    )
    if int(corridor.sum()) == 0:
        return CorridorReading(_EMPTY_BAND, _EMPTY_BAND, _EMPTY_BAND, timestamp)

    center_half_width = half_width / 3.0
    left_mask = corridor & (x_for_mask < -center_half_width)
    center_mask = (
        corridor
        & (x_for_mask >= -center_half_width)
        & (x_for_mask <= center_half_width)
    )
    right_mask = corridor & (x_for_mask > center_half_width)

    return CorridorReading(
        left=_band_metrics_from_masked_depths(depth_m, left_mask),
        center=_band_metrics_from_masked_depths(depth_m, center_mask),
        right=_band_metrics_from_masked_depths(depth_m, right_mask),
        timestamp=timestamp,
    )


def _set_sensor_option(depth_sensor, option, value: float, label: str) -> str:
    if depth_sensor is None or option is None:
        return f"{label}=unsupported"
    try:
        if not depth_sensor.supports(option):
            return f"{label}=unsupported"
        depth_sensor.set_option(option, value)
        try:
            actual = depth_sensor.get_option(option)
        except Exception:
            actual = value
        return f"{label}={actual:g}"
    except Exception as exc:
        _log.warning("Unable to set RealSense %s to %s: %s", label, value, exc)
        return f"{label}=error"


def configure_indoor_depth_sensor(depth_sensor, laser_power: float = 300.0) -> str:
    """Enable D400 active IR settings when the attached device supports them."""
    option = getattr(rs, "option", None) if rs is not None else None
    emitter = getattr(option, "emitter_enabled", None) if option is not None else None
    laser = getattr(option, "laser_power", None) if option is not None else None
    settings = [
        _set_sensor_option(depth_sensor, emitter, 1.0, "emitter"),
        _set_sensor_option(depth_sensor, laser, float(laser_power), "laser"),
    ]
    return ", ".join(settings)


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
            depth_sensor = None
            depth_scale = 0.001  # D435 default: 1 mm per unit

        sensor_status = configure_indoor_depth_sensor(depth_sensor)
        self._set_status(
            CameraStatus(
                True,
                False,
                f"RealSense D435 stream active ({sensor_status}).",
            )
        )
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
