import importlib
import unittest
from unittest.mock import patch

try:
    from fastapi.testclient import TestClient
except ModuleNotFoundError as exc:
    TestClient = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

controller_app = importlib.import_module("app")


class FakeBus:
    simulated = True


class FakeDrive:
    instances = []

    def __init__(self, _bus=None, config=None):
        self.config = config
        self.arcade_calls = []
        self.pivot_calls = []
        self.stop_calls = 0
        FakeDrive.instances.append(self)

    def arcade(self, throttle, turn):
        self.arcade_calls.append((throttle, turn))

    def pivot_turn(self, side, rate):
        self.pivot_calls.append((side, rate))

    def stop(self):
        self.stop_calls += 1

    def get_average_side_rpm(self):
        return 123.44, 56.78


class FakeStatus:
    available = True
    simulated = True
    message = "fake camera"

    def as_dict(self):
        return {
            "available": self.available,
            "simulated": self.simulated,
            "message": self.message,
        }


class FakeDistance:
    center_m = 1.2
    min_m = 1.0
    timestamp = 1.0

    def as_dict(self):
        return {
            "center_m": self.center_m,
            "min_m": self.min_m,
            "timestamp": self.timestamp,
        }


class FakeCamera:
    def __init__(self):
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def status(self):
        return FakeStatus()

    def distance(self):
        return FakeDistance()

    def mjpeg_frames(self):
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\nfake\r\n"


def _receive_type(websocket, message_type):
    for _ in range(10):
        message = websocket.receive_json()
        if message.get("type") == message_type:
            return message
    raise AssertionError(f"Did not receive {message_type}")


class TestClientMessageHandling(unittest.TestCase):
    def setUp(self):
        FakeDrive.instances.clear()
        controller_app._drive = FakeDrive()
        controller_app._last_heartbeat = 0
        controller_app._avoider.stop()

    def tearDown(self):
        controller_app._drive = None
        controller_app._avoider.stop()

    def test_heartbeat_refreshes_timestamp(self):
        controller_app.handle_client_message({"type": "heartbeat"})
        self.assertGreater(controller_app._last_heartbeat, 0)

    def test_drive_dispatches_arcade_command(self):
        controller_app.handle_client_message(
            {"type": "drive", "throttle": "0.4", "turn": "-0.2"}
        )
        self.assertEqual(controller_app._drive.arcade_calls, [(0.4, -0.2)])

    def test_pivot_dispatches_pivot_command(self):
        controller_app.handle_client_message(
            {"type": "pivot", "side": "left", "rate": "0.5"}
        )
        self.assertEqual(controller_app._drive.pivot_calls, [("left", 0.5)])

    def test_stop_stops_drive_and_disables_avoidance(self):
        controller_app._avoider.set_enabled(True)
        response = controller_app.handle_client_message({"type": "stop"})

        self.assertEqual(controller_app._drive.stop_calls, 1)
        self.assertFalse(controller_app._avoider.enabled)
        self.assertEqual(response["type"], "avoidance_status")

    def test_update_config_returns_success_response(self):
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ):
            response = controller_app.handle_client_message(
                {"type": "update_config", "max_velocity": "1.5", "max_rpm": "4500"}
            )

        self.assertEqual(
            response,
            {"type": "config_applied", "max_velocity": 1.5, "max_rpm": 4500.0},
        )
        self.assertEqual(controller_app._drive.config.max_linear_velocity, 1.5)
        self.assertEqual(controller_app._drive.config.max_motor_rpm, 4500.0)

    def test_update_config_returns_error_response(self):
        response = controller_app.handle_client_message(
            {"type": "update_config", "max_velocity": "bad", "max_rpm": "4500"}
        )
        self.assertEqual(response["type"], "config_error")

    def test_set_avoidance_enables_mode(self):
        response = controller_app.handle_client_message({"type": "set_avoidance", "enabled": True})

        self.assertEqual(response["type"], "avoidance_status")
        self.assertTrue(response["enabled"])

    def test_manual_commands_ignored_while_avoidance_enabled(self):
        controller_app.handle_client_message({"type": "set_avoidance", "enabled": True})
        response = controller_app.handle_client_message(
            {"type": "drive", "throttle": 0.8, "turn": 0.2}
        )

        self.assertEqual(response["type"], "avoidance_status")
        self.assertEqual(controller_app._drive.arcade_calls, [])


@unittest.skipIf(TestClient is None, f"FastAPI test dependencies unavailable: {IMPORT_ERROR}")
class TestFastApiWebSocket(unittest.TestCase):
    def setUp(self):
        FakeDrive.instances.clear()

    def tearDown(self):
        controller_app._drive = None
        controller_app._avoider.stop()

    def test_index_static_assets_and_camera_stream_are_served_locally(self):
        fake_camera = FakeCamera()
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ), patch.object(controller_app, "_camera", fake_camera):
            with TestClient(controller_app.app) as client:
                index_response = client.get("/")
                static_response = client.get("/static/controller.js")
                with client.stream("GET", "/camera/stream") as stream_response:
                    first_chunk = next(stream_response.iter_bytes())

        self.assertEqual(index_response.status_code, 200)
        self.assertIn("Skid-Steer Controller", index_response.text)
        self.assertNotIn("https://", index_response.text)
        self.assertEqual(static_response.status_code, 200)
        self.assertIn("new WebSocket", static_response.text)
        self.assertNotIn("io(", static_response.text)
        self.assertNotIn("https://", static_response.text)
        self.assertIn(b"Content-Type: image/jpeg", first_chunk)

    def test_websocket_accepts_drive_stop_and_avoidance_commands(self):
        fake_camera = FakeCamera()
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ), patch.object(controller_app, "_camera", fake_camera):
            with TestClient(controller_app.app) as client:
                with client.websocket_connect("/ws") as websocket:
                    websocket.send_json(
                        {"type": "drive", "throttle": 0.25, "turn": -0.75}
                    )
                    websocket.send_json({"type": "set_avoidance", "enabled": True})
                    avoidance = _receive_type(websocket, "avoidance_status")
                    websocket.send_json({"type": "drive", "throttle": 1.0, "turn": 1.0})
                    ignored = _receive_type(websocket, "avoidance_status")
                    websocket.send_json({"type": "stop"})
                    stopped = _receive_type(websocket, "avoidance_status")
                    websocket.send_json({"type": "unknown"})
                    error = _receive_type(websocket, "config_error")

                    drive = controller_app._drive
                    self.assertEqual(drive.arcade_calls, [(0.25, -0.75)])
                    self.assertGreaterEqual(drive.stop_calls, 1)

        self.assertTrue(avoidance["enabled"])
        self.assertTrue(ignored["enabled"])
        self.assertFalse(stopped["enabled"])
        self.assertIn("Unknown message type", error["error"])

    def test_websocket_returns_config_responses(self):
        fake_camera = FakeCamera()
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ), patch.object(controller_app, "_camera", fake_camera):
            with TestClient(controller_app.app) as client:
                with client.websocket_connect("/ws") as websocket:
                    websocket.send_json(
                        {
                            "type": "update_config",
                            "max_velocity": 2.0,
                            "max_rpm": 6000,
                        }
                    )
                    applied = _receive_type(websocket, "config_applied")

                    websocket.send_json(
                        {
                            "type": "update_config",
                            "max_velocity": "nope",
                            "max_rpm": 6000,
                        }
                    )
                    error = _receive_type(websocket, "config_error")

        self.assertEqual(applied["max_velocity"], 2.0)
        self.assertEqual(applied["max_rpm"], 6000.0)
        self.assertIn("could not convert", error["error"])

    def test_websocket_broadcasts_telemetry_camera_distance_and_avoidance(self):
        fake_camera = FakeCamera()
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ), patch.object(controller_app, "_camera", fake_camera):
            with TestClient(controller_app.app) as client:
                with client.websocket_connect("/ws") as websocket:
                    rpm = _receive_type(websocket, "rpm_update")
                    camera = _receive_type(websocket, "camera_status")
                    distance = _receive_type(websocket, "distance_update")
                    avoidance = _receive_type(websocket, "avoidance_status")

        self.assertEqual(rpm, {"type": "rpm_update", "left": 123.4, "right": 56.8})
        self.assertEqual(camera["message"], "fake camera")
        self.assertEqual(distance["center_m"], 1.2)
        self.assertIn("enabled", avoidance)

    def test_websocket_disconnect_stops_drive_and_avoidance(self):
        fake_camera = FakeCamera()
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ), patch.object(controller_app, "_camera", fake_camera):
            with TestClient(controller_app.app) as client:
                with client.websocket_connect("/ws"):
                    controller_app._avoider.set_enabled(True)
                    drive = controller_app._drive
                    self.assertEqual(drive.stop_calls, 0)

                self.assertGreaterEqual(drive.stop_calls, 1)
                self.assertFalse(controller_app._avoider.enabled)


if __name__ == "__main__":
    unittest.main()
