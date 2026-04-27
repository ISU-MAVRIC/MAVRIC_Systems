import importlib
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

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

    def tearDown(self):
        controller_app._drive = None

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

    def test_stop_stops_drive(self):
        controller_app.handle_client_message({"type": "stop"})
        self.assertEqual(controller_app._drive.stop_calls, 1)

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


class TestFastApiWebSocket(unittest.TestCase):
    def setUp(self):
        FakeDrive.instances.clear()

    def tearDown(self):
        controller_app._drive = None

    def test_index_and_static_assets_are_served(self):
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ):
            with TestClient(controller_app.app) as client:
                index_response = client.get("/")
                static_response = client.get("/static/controller.js")

        self.assertEqual(index_response.status_code, 200)
        self.assertIn("Skid-Steer Controller", index_response.text)
        self.assertEqual(static_response.status_code, 200)
        self.assertIn("new WebSocket", static_response.text)

    def test_websocket_accepts_drive_and_stop_commands(self):
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ):
            with TestClient(controller_app.app) as client:
                with client.websocket_connect("/ws") as websocket:
                    websocket.send_json(
                        {"type": "drive", "throttle": 0.25, "turn": -0.75}
                    )
                    websocket.send_json({"type": "stop"})
                    websocket.send_json({"type": "unknown"})
                    _receive_type(websocket, "config_error")

                    drive = controller_app._drive
                    self.assertEqual(drive.arcade_calls, [(0.25, -0.75)])
                    self.assertEqual(drive.stop_calls, 1)

    def test_websocket_returns_config_responses(self):
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ):
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

    def test_websocket_broadcasts_telemetry(self):
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ):
            with TestClient(controller_app.app) as client:
                with client.websocket_connect("/ws") as websocket:
                    telemetry = _receive_type(websocket, "rpm_update")

        self.assertEqual(telemetry, {"type": "rpm_update", "left": 123.4, "right": 56.8})

    def test_websocket_disconnect_stops_drive(self):
        with patch.object(controller_app, "SparkBus", FakeBus), patch.object(
            controller_app, "SkidSteerDrive", FakeDrive
        ):
            with TestClient(controller_app.app) as client:
                with client.websocket_connect("/ws"):
                    drive = controller_app._drive
                    self.assertEqual(drive.stop_calls, 0)

                self.assertGreaterEqual(drive.stop_calls, 1)


if __name__ == "__main__":
    unittest.main()
