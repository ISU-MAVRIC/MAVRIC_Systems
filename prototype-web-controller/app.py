import asyncio
import contextlib
import logging
import threading
import time
from pathlib import Path
from typing import Any

from SparkCANLib.SparkCAN import SparkBus
from joystick_drive.skid_steer_drive import DriveConfig, SkidSteerDrive

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
_log = logging.getLogger("web-controller")

BASE_DIR = Path(__file__).resolve().parent


# -- Drive state --------------------------------------------------------------

DEFAULT_MAX_VELOCITY = 1.0
DEFAULT_MAX_RPM = 5000
WATCHDOG_TIMEOUT = 0.5  # seconds; stop motors if no client message arrives.

_bus = None
_drive = None
_drive_lock = threading.Lock()
_last_heartbeat = time.time()


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
        _log.info(
            "Drive init: max_velocity=%s max_rpm=%s simulated=%s",
            max_velocity,
            max_rpm,
            getattr(_bus, "simulated", False),
        )


def _refresh_heartbeat() -> None:
    global _last_heartbeat
    _last_heartbeat = time.time()


def handle_client_message(data: dict[str, Any]) -> dict[str, Any] | None:
    """Handle one JSON message from the browser and return an optional response."""
    message_type = data.get("type")

    if message_type == "heartbeat":
        _refresh_heartbeat()
        return None

    if message_type == "drive":
        _refresh_heartbeat()
        with _drive_lock:
            if _drive is not None:
                _drive.arcade(float(data["throttle"]), float(data["turn"]))
        return None

    if message_type == "pivot":
        _refresh_heartbeat()
        with _drive_lock:
            if _drive is not None:
                _drive.pivot_turn(data["side"], float(data["rate"]))
        return None

    if message_type == "stop":
        with _drive_lock:
            if _drive is not None:
                _drive.stop()
        return None

    if message_type == "update_config":
        try:
            max_velocity = float(data["max_velocity"])
            max_rpm = float(data["max_rpm"])
            _init_drive(max_velocity, max_rpm)
            return {
                "type": "config_applied",
                "max_velocity": max_velocity,
                "max_rpm": max_rpm,
            }
        except (ValueError, KeyError) as exc:
            return {"type": "config_error", "error": str(exc)}

    return {"type": "config_error", "error": f"Unknown message type: {message_type}"}


async def _watchdog() -> None:
    while True:
        await asyncio.sleep(0.1)
        with _drive_lock:
            if _drive is not None and (time.time() - _last_heartbeat) > WATCHDOG_TIMEOUT:
                _drive.stop()


class ConnectionManager:
    def __init__(self) -> None:
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self._clients.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(websocket)
        _log.info("Client disconnected; stopping motors")
        with _drive_lock:
            if _drive is not None:
                _drive.stop()

    async def broadcast(self, message: dict[str, Any]) -> None:
        async with self._lock:
            clients = list(self._clients)

        stale_clients: list[WebSocket] = []
        for websocket in clients:
            try:
                await websocket.send_json(message)
            except Exception:
                stale_clients.append(websocket)

        if stale_clients:
            async with self._lock:
                for websocket in stale_clients:
                    self._clients.discard(websocket)


manager = ConnectionManager()


async def _telemetry() -> None:
    while True:
        await asyncio.sleep(0.1)
        with _drive_lock:
            if _drive is None:
                continue
            try:
                left, right = _drive.get_average_side_rpm()
            except Exception:
                continue

        await manager.broadcast(
            {"type": "rpm_update", "left": round(left, 1), "right": round(right, 1)}
        )


@contextlib.asynccontextmanager
async def lifespan(_app: FastAPI):
    _init_drive(DEFAULT_MAX_VELOCITY, DEFAULT_MAX_RPM)
    tasks = [
        asyncio.create_task(_watchdog()),
        asyncio.create_task(_telemetry()),
    ]
    try:
        yield
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        with _drive_lock:
            if _drive is not None:
                _drive.stop()


app = FastAPI(lifespan=lifespan)
templates = Jinja2Templates(directory=str(BASE_DIR / "templates"))
app.mount("/static", StaticFiles(directory=str(BASE_DIR / "static")), name="static")


@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse(request, "index.html")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            response = handle_client_message(data)
            if response is not None:
                await websocket.send_json(response)
    except WebSocketDisconnect:
        pass
    finally:
        await manager.disconnect(websocket)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("app:app", host="0.0.0.0", port=6060, reload=False)
