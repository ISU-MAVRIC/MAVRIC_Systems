import sys
import os
import threading
import time
import logging

from SparkCANLib.SparkCAN import SparkBus
from camera_service import RealSenseCameraService
from joystick_drive.skid_steer_drive import DriveConfig, SkidSteerDrive
from obstacle_avoidance import DumbObstacleAvoider

from flask import Flask, Response, render_template
from flask_socketio import SocketIO, emit

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
_log = logging.getLogger("web-controller")

app = Flask(__name__)
app.config["SECRET_KEY"] = "mavric-skid-steer"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ── Drive state ───────────────────────────────────────────────────────────────

DEFAULT_MAX_VELOCITY = 1.0
DEFAULT_MAX_RPM = 5000

_bus = None
_drive = None
_drive_lock = threading.Lock()
_camera = RealSenseCameraService()
_avoider = DumbObstacleAvoider()


def _init_drive(max_velocity: float, max_rpm: float) -> None:
    global _bus, _drive
    with _drive_lock:
        if _drive is not None:
            _drive.stop()
        _bus = SparkBus()
        config = DriveConfig(
            max_linear_velocity=max_velocity,
            left_motor_ids=[1, 2, 3],
            right_motor_ids=[4, 5, 6],
            max_motor_rpm=max_rpm,
        )
        _drive = SkidSteerDrive(_bus, config)
        _log.info(f"Drive init: max_velocity={max_velocity} max_rpm={max_rpm} simulated={_bus.simulated}")


_init_drive(DEFAULT_MAX_VELOCITY, DEFAULT_MAX_RPM)
_camera.start()

# ── Safety watchdog ───────────────────────────────────────────────────────────

_last_heartbeat = time.time()
WATCHDOG_TIMEOUT = 0.5  # seconds — stop motors if no client message for this long


def _watchdog() -> None:
    while True:
        time.sleep(0.1)
        with _drive_lock:
            if _drive is not None and (time.time() - _last_heartbeat) > WATCHDOG_TIMEOUT:
                _avoider.stop()
                _drive.stop()


threading.Thread(target=_watchdog, daemon=True).start()

# ── Telemetry thread ──────────────────────────────────────────────────────────


def _telemetry() -> None:
    while True:
        time.sleep(0.1)
        with _drive_lock:
            if _drive is None:
                continue
            try:
                left, right = _drive.get_average_side_rpm()
                socketio.emit("rpm_update", {"left": round(left, 1), "right": round(right, 1)})
            except Exception:
                pass
        socketio.emit("camera_status", _camera.status().as_dict())
        socketio.emit("distance_update", _camera.distance().as_dict())
        socketio.emit("avoidance_status", _avoider.status())


threading.Thread(target=_telemetry, daemon=True).start()

# ── Obstacle avoidance thread ────────────────────────────────────────────────


def _obstacle_avoidance_loop() -> None:
    while True:
        time.sleep(0.05)
        if not _avoider.enabled:
            continue

        distance = _camera.distance()
        age = time.time() - distance.timestamp
        center_m = distance.center_m if age <= _avoider.config.stale_seconds else None
        command = _avoider.command(center_m)

        with _drive_lock:
            if _drive is None:
                continue
            if command.throttle == 0.0 and command.turn == 0.0:
                _drive.stop()
            else:
                _drive.arcade(command.throttle, command.turn)


threading.Thread(target=_obstacle_avoidance_loop, daemon=True).start()

# ── Routes ────────────────────────────────────────────────────────────────────


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/camera/stream")
def camera_stream():
    return Response(
        _camera.mjpeg_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ── SocketIO handlers ─────────────────────────────────────────────────────────


def _refresh_heartbeat() -> None:
    global _last_heartbeat
    _last_heartbeat = time.time()


@socketio.on("heartbeat")
def handle_heartbeat(_data):
    _refresh_heartbeat()


@socketio.on("drive")
def handle_drive(data):
    _refresh_heartbeat()
    if _avoider.enabled:
        emit("avoidance_status", _avoider.status())
        return
    with _drive_lock:
        if _drive is None:
            return
        _drive.arcade(float(data["throttle"]), float(data["turn"]))


@socketio.on("pivot")
def handle_pivot(data):
    _refresh_heartbeat()
    if _avoider.enabled:
        emit("avoidance_status", _avoider.status())
        return
    with _drive_lock:
        if _drive is None:
            return
        _drive.pivot_turn(data["side"], float(data["rate"]))


@socketio.on("stop")
def handle_stop(_data):
    _avoider.stop()
    with _drive_lock:
        if _drive is not None:
            _drive.stop()
    emit("avoidance_status", _avoider.status(), broadcast=True)


@socketio.on("disconnect")
def handle_disconnect():
    _log.info("Client disconnected — stopping motors")
    _avoider.stop()
    with _drive_lock:
        if _drive is not None:
            _drive.stop()


@socketio.on("update_config")
def handle_update_config(data):
    try:
        mv = float(data["max_velocity"])
        mr = float(data["max_rpm"])
        _init_drive(mv, mr)
        emit("config_applied", {"max_velocity": mv, "max_rpm": mr})
    except (ValueError, KeyError) as exc:
        emit("config_error", {"error": str(exc)})


@socketio.on("set_avoidance")
def handle_set_avoidance(data):
    enabled = bool(data.get("enabled"))
    if enabled and not _camera.status().available:
        _avoider.stop()
        emit(
            "avoidance_status",
            {"enabled": False, "state": "stopped", "reason": "camera unavailable"},
            broadcast=True,
        )
        return

    command = _avoider.set_enabled(enabled)
    if not enabled or (command.throttle == 0.0 and command.turn == 0.0):
        with _drive_lock:
            if _drive is not None:
                _drive.stop()
    emit("avoidance_status", _avoider.status(), broadcast=True)


if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=6060, debug=False)
