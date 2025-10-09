#!/usr/bin/env python3
"""
Web-based Teleoperation Interface for MAVRIC Rover
Provides a browser-based GUI for controlling the rover with keyboard support.
"""

import logging
import threading
import time

from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit

from config import *
from SparkCANLib import SparkCAN, SparkController as COntroller

app = Flask(__name__)
app.config["SECRET_KEY"] = "mavric_secret_key"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

logger = logging.getLogger("mavric.web_teleop")
if not logger.handlers:
    logging.basicConfig(level=logging.INFO)

# Initialize CAN bus and controllers
kit = None
logger.info("Initializing CAN bus...")
try:
    bus = SparkCAN.SparkBus()

    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=16)
    logger.info("✓ CAN bus initialized (or running in simulation mode)")
except Exception as e:
    logger.warning("⚠ CAN bus initialization warning: %s", e)
    logger.warning("Continuing in simulation mode...")
    bus = SparkCAN.SparkBus()

# Initialize drive motors (all 4 wheels)
drive_motors = {
    'FLD': bus.init_controller(FLD_ID),
    'FRD': bus.init_controller(FRD_ID),
    'BLD': bus.init_controller(BLD_ID),
    'BRD': bus.init_controller(BRD_ID)
}

# Initialize steer motors (all 4 wheels)
steer_motors = {
    'FLS': bus.init_controller(FLS_ID),
    'FRS': bus.init_controller(FRS_ID),
    'BLS': bus.init_controller(BLS_ID),
    'BRS': bus.init_controller(BRS_ID)
}

# Initialize arm motors
arm_motors = {
    'SHOULDER_PITCH': bus.init_controller(SHOULDER_PITCH_ID),
    'SHOULDER_ROT': bus.init_controller(SHOULDER_ROT_ID),
    'ELBOW_PITCH': bus.init_controller(ELBOW_PITCH_ID),
    'WRIST_PITCH': bus.init_controller(WRIST_PITCH_ID),
    'WRIST_ROT': bus.init_controller(WRIST_ROT_ID)
}

# Track currently pressed keys; guard shared state with this lock
pressed_keys = set()
control_lock = threading.Lock()


class ControlState:
    """Cache of the last commands sent to each subsystem.

    Prevents flooding the CAN bus with duplicate messages while keeping
    the most recent command values easily accessible to worker threads.
    """

    def __init__(self):
        self.drive_speed = None
        self.rotation_speed = None
        self.steer_positions = None  # Tuple FL, FR, BL, BR
        self.arm_outputs = {}

    def reset(self):
        self.drive_speed = None
        self.rotation_speed = None
        self.steer_positions = None
        for key in self.arm_outputs:
            self.arm_outputs[key] = None


control_state = ControlState()
control_state.arm_outputs = {name: None for name in arm_motors}


def _reset_control_state():
    """Clear cached command state so the next command always sends."""
    control_state.reset()


def reset_all():
    """Reset all motors and servos to default state"""
    set_drive_speeds(0, force=True)
    set_rotation_speed(0, force=True)
    set_steer_pos_immediate(DEFAULT_STEER_POS, force=True)
    for motor in arm_motors.values():
        motor.percent_output(0)
    if kit is not None:
        kit.continuous_servo[CLAW_CHANNEL].throttle = 0
    _reset_control_state()
    drive_manager.reset()


def set_drive_speeds(speed, *, force=False):
    """Set all drive motor speeds (accounts for motor direction)"""
    if not force and control_state.drive_speed == speed:
        return

    control_state.drive_speed = speed
    drive_motors['FLD'].percent_output(speed)
    drive_motors['FRD'].percent_output(-speed)
    drive_motors['BLD'].percent_output(speed)
    drive_motors['BRD'].percent_output(-speed)


# ---------------------------------------------------------------------------
# Steering helpers
# ---------------------------------------------------------------------------


def _apply_steer_positions(fls, frs, bls, brs, *, force=False):
    """Apply steering outputs if they differ from the last command.

    Returns True when a new command is sent, False otherwise.
    """
    positions = (fls, frs, bls, brs)
    if not force and control_state.steer_positions == positions:
        return False

    control_state.steer_positions = positions
    steer_motors['FLS'].position_output(fls)
    steer_motors['FRS'].position_output(frs)
    steer_motors['BLS'].position_output(bls)
    steer_motors['BRS'].position_output(brs)
    return True


def set_steer_pos_immediate(pos, *, force=False):
    """Set steering position immediately without waiting"""
    return _apply_steer_positions(pos, pos, -pos, -pos, force=force)


def set_rotation_pos(*, force=False):
    """Command wheels into rotation pose."""
    return _apply_steer_positions(
        STEER_ROTATION_POS,
        -STEER_ROTATION_POS,
        -STEER_ROTATION_POS,
        STEER_ROTATION_POS,
        force=force,
    )


def _current_steer_positions():
    """Return the current encoder readouts for each steering motor."""
    return (
        steer_motors['FLS'].position,
        steer_motors['FRS'].position,
        steer_motors['BLS'].position,
        steer_motors['BRS'].position,
    )


def _movement_targets(pos):
    """Steering targets for conventional driving."""
    return (pos, pos, -pos, -pos)


def _rotation_targets():
    """Steering targets for the rotation pose."""
    return (
        STEER_ROTATION_POS,
        -STEER_ROTATION_POS,
        -STEER_ROTATION_POS,
        STEER_ROTATION_POS,
    )


def _positions_close(targets, margin=POS_MARGIN_ERROR):
    """Determine if actual steering positions are within tolerance."""
    return all(abs(actual - target) <= margin for actual, target in zip(_current_steer_positions(), targets))


class DriveManager:
    """Background worker that enforces steering/drive ordering rules.

    The main Socket.IO handlers enqueue movement or rotation requests here.
    The worker thread serializes those requests and blocks as needed so the
    public constraints are always honored (stop before rotating, wait for
    steering alignment before driving, etc.).
    """

    ROTATION_TIMEOUT = 2.0
    STEER_TIMEOUT = 2.0

    def __init__(self):
        self._condition = threading.Condition()
        self._command_id = 0
        self._mode = "idle"  # idle, movement, rotation
        self._target_speed = 0
        self._target_steer = DEFAULT_STEER_POS
        self._target_rotation_speed = 0
        self._shutdown = False
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def shutdown(self):
        with self._condition:
            self._shutdown = True
            self._condition.notify_all()
        self._worker.join(timeout=1.0)

    def request_movement(self, speed: float, steer_pos: float):
        with self._condition:
            self._command_id += 1
            self._mode = "movement"
            self._target_speed = speed
            self._target_steer = steer_pos
            self._condition.notify_all()

    def request_rotation(self, rotation_speed: float):
        with self._condition:
            self._command_id += 1
            self._mode = "rotation"
            self._target_rotation_speed = rotation_speed
            self._condition.notify_all()

    def reset(self):
        with self._condition:
            self._command_id += 1
            self._mode = "idle"
            self._target_speed = 0
            self._target_rotation_speed = 0
            self._target_steer = DEFAULT_STEER_POS
            self._condition.notify_all()

    def _run(self):
        while True:
            with self._condition:
                if not self._shutdown:
                    self._condition.wait(timeout=0.02)
                command_id = self._command_id
                mode = self._mode
                speed = self._target_speed
                steer_pos = self._target_steer
                rotation_speed = self._target_rotation_speed
                shutdown = self._shutdown

            if shutdown:
                break

            if mode == "rotation":
                self._handle_rotation(command_id, rotation_speed)
            elif mode == "movement":
                self._handle_movement(command_id, speed, steer_pos)
            else:
                self._handle_idle(command_id)

    def _is_stale(self, command_id: int) -> bool:
        with self._condition:
            return command_id != self._command_id

    def _handle_rotation(self, command_id: int, rotation_speed: float):
        """Wait for wheels to reach the rotation pose before spinning."""
        # Ensure drive motors are stopped before any rotation attempt
        set_drive_speeds(0, force=True)
        set_rotation_speed(0, force=True)
        set_rotation_pos()

        deadline = time.time() + self.ROTATION_TIMEOUT
        while time.time() < deadline:
            if self._is_stale(command_id):
                return
            if _positions_close(_rotation_targets()):
                break
            time.sleep(0.01)
        else:
            logger.warning("Rotation positioning timeout; keeping wheels stopped")
            return

        if self._is_stale(command_id):
            return

        set_rotation_speed(rotation_speed)

    def _handle_movement(self, command_id: int, speed: float, steer_pos: float):
        """Ensure steering matches the requested angle before driving."""
        # Always stop rotation when movement requested
        # set_rotation_speed(0)

        blocking = speed != 0
        if blocking:
            set_drive_speeds(0, force=True)
        if not self._ensure_steer_alignment(command_id, steer_pos, blocking):
            return

        if self._is_stale(command_id):
            return

        set_drive_speeds(speed)

    def _handle_idle(self, command_id: int):
        """Return drivetrain to a neutral state."""
        set_rotation_speed(0)
        set_drive_speeds(0)
        self._ensure_steer_alignment(command_id, DEFAULT_STEER_POS, blocking=False)

    def _ensure_steer_alignment(self, command_id: int, steer_pos: float, blocking: bool) -> bool:
        """Wait until the steering sensors read the desired pose."""
        targets = _movement_targets(steer_pos)
        command_sent = set_steer_pos_immediate(steer_pos)

        if not blocking:
            return True

        if not command_sent and _positions_close(targets):
            return True

        deadline = time.time() + self.STEER_TIMEOUT
        while time.time() < deadline:
            if self._is_stale(command_id):
                return False
            if _positions_close(targets):
                return True
            time.sleep(0.01)

        logger.warning("Steering alignment timeout (target=%s)", steer_pos)
        return False


# Single global manager instance coordinates all drivetrain updates.
drive_manager = DriveManager()


def set_rotation_speed(speed, *, force=False):
    """Set rotation speed for all drive motors"""
    if not force and control_state.rotation_speed == speed:
        return

    control_state.rotation_speed = speed
    for motor in drive_motors.values():
        motor.percent_output(speed)


def state_movement():
    """Forward drive inputs to the manager while handling key modifiers."""
    speed = 0
    steer_pos = DEFAULT_STEER_POS

    # Determine drive speed with priority order
    w_pressed = "w" in pressed_keys
    s_pressed = "s" in pressed_keys
    shift_pressed = "shift" in pressed_keys
    capslock_pressed = "capslock" in pressed_keys
    
    if w_pressed and s_pressed:
        speed = 0
    elif w_pressed:
        if shift_pressed:
            speed = NORMAL_DRIVE_SPEED
        elif capslock_pressed:
            speed = MAX_DRIVE_SPEED
        else:
            speed = MIN_DRIVE_SPEED
    elif s_pressed:
        speed = -MIN_DRIVE_SPEED

    # Adjust speed when turning (reduce speed for better control)
    a_pressed = "a" in pressed_keys
    d_pressed = "d" in pressed_keys
    
    # if (w_pressed or s_pressed) and (a_pressed or d_pressed):
    #     # Turning - reduce to minimum speed for better control
    #     speed = MIN_DRIVE_SPEED if speed > 0 else -MIN_DRIVE_SPEED

    # Determine steering position
    if a_pressed and d_pressed:
        steer_pos = DEFAULT_STEER_POS
    elif a_pressed:
        steer_pos = STEER_LEFT_POS
    elif d_pressed:
        steer_pos = STEER_RIGHT_POS

    drive_manager.request_movement(speed, steer_pos)


def state_rotation():
    """Forward rotation inputs to the manager and compute desired speed."""
    q_pressed = "q" in pressed_keys
    e_pressed = "e" in pressed_keys

    # Determine desired rotation speed
    if q_pressed and e_pressed:
        desired_speed = 0
    elif q_pressed:
        desired_speed = -STEER_ROTATION_SPEED
    elif e_pressed:
        desired_speed = STEER_ROTATION_SPEED
    else:
        desired_speed = 0

    drive_manager.request_rotation(desired_speed)


def arm_controls():
    """Handle arm controls (always active, non-blocking)"""
    # Define control mappings: (positive_key, negative_key, speed_value)
    arm_control_map = {
        'SHOULDER_ROT': ('x', 'z', SHOULDER_ROT_SPEED),
        'SHOULDER_PITCH': ('y', 'h', SHOULDER_PITCH_SPEED),
        'ELBOW_PITCH': ('j', 'u', ELBOW_PITCH_SPEED),
        'WRIST_PITCH': ('i', 'k', WRIST_PITCH_SPEED),
        'WRIST_ROT': ('v', 'c', WRIST_ROT_SPEED),
    }
    
    # Process each arm motor
    for motor_name, (pos_key, neg_key, speed) in arm_control_map.items():
        pos_pressed = pos_key in pressed_keys
        neg_pressed = neg_key in pressed_keys
        
        if pos_pressed and neg_pressed:
            # Both keys pressed - stop
            output = 0
        elif pos_pressed:
            output = speed
        elif neg_pressed:
            output = -speed
        else:
            output = 0
        
        if control_state.arm_outputs.get(motor_name) == output:
            continue

        control_state.arm_outputs[motor_name] = output
        arm_motors[motor_name].percent_output(output)
    
    # Handle claw (servo, not a motor controller)
    if "[" in pressed_keys and "]" in pressed_keys:
        claw = 0
    elif "[" in pressed_keys:
        claw = CLAW_SPEED
    elif "]" in pressed_keys:
        claw = -CLAW_SPEED
    else:
        claw = 0
    
    if kit is not None:
        kit.continuous_servo[CLAW_CHANNEL].throttle = claw


def update_controls():
    """Main control update function (assumes caller holds control_lock)"""
    logger.debug("update_controls() pressed_keys: %s", pressed_keys)
    # Arm controls are always active
    arm_controls()

    # Movement and rotation are mutually exclusive
    if "q" in pressed_keys or "e" in pressed_keys:
        logger.debug("  -> Calling state_rotation()")
        state_rotation()
    else:
        logger.debug("  -> Calling state_movement()")
        state_movement()


@app.route("/")
def index():
    """Serve the main web interface"""
    return render_template("index.html")


@app.route("/api/status")
def status():
    """Get current control status"""
    with control_lock:
        return jsonify({"pressed_keys": list(pressed_keys), "status": "running"})


# ---------------------------------------------------------------------------
# Socket.IO event handlers
# ---------------------------------------------------------------------------


@socketio.on("connect")
def handle_connect():
    """Handle client connection"""
    logger.info("Client connected")
    emit("status", {"message": "Connected to MAVRIC Rover"})


@socketio.on("disconnect")
def handle_disconnect():
    """Handle client disconnection - stop all motors"""
    logger.info("Client disconnected")
    with control_lock:
        pressed_keys.clear()
        update_controls()


@socketio.on("key_down")
def handle_key_down(data):
    """Handle key press event from web client"""
    logger.debug("Received key_down event: %s", data)
    key = data.get("key", "").lower()
    if key:
        logger.debug("Processing key down: %s", key)
        with control_lock:
            pressed_keys.add(key)
            update_controls()
        emit("key_state", {"pressed_keys": list(pressed_keys)}, broadcast=True)


@socketio.on("key_up")
def handle_key_up(data):
    """Handle key release event from web client"""
    logger.debug("Received key_up event: %s", data)
    key = data.get("key", "").lower()
    if key:
        logger.debug("Processing key up: %s", key)
        with control_lock:
            pressed_keys.discard(key)
            update_controls()
        emit("key_state", {"pressed_keys": list(pressed_keys)}, broadcast=True)


@socketio.on("emergency_stop")
def handle_emergency_stop():
    """Emergency stop - clear all controls"""
    logger.warning("Emergency stop triggered")
    with control_lock:
        pressed_keys.clear()
        update_controls()
    emit("key_state", {"pressed_keys": []}, broadcast=True)


def main(port=5000):
    """Start the web server"""
    print("=" * 60)
    print("Starting MAVRIC Web Teleop Interface...")
    print("=" * 60)
    print("")
    print("Access the interface at:")
    print(f"  - Local:    http://localhost:{port}")
    print(f"  - Network:  http://0.0.0.0:{port}")
    print("")
    print("Press Ctrl+C to stop the server")
    print("=" * 60)
    print("")

    reset_all()

    try:
        socketio.run(
            app, host="0.0.0.0", port=port, debug=False, allow_unsafe_werkzeug=True
        )
    except PermissionError:
        print(f"\n❌ Permission denied on port {port}. Try a different port:")
        print(f"   python3 web_teleop.py --port 8080")
    except OSError as e:
        if "Address already in use" in str(e):
            print(f"\n❌ Port {port} is already in use!")
            print("\nOptions:")
            print("  1. Kill the existing process:")
            print(f"     lsof -ti :{port} | xargs kill")
            print("  2. Use a different port:")
            print("     python3 web_teleop.py --port 8080")

            # Try alternative port
            alt_port = port + 1
            print(f"\n🔄 Attempting to use port {alt_port} instead...")
            try:
                socketio.run(
                    app,
                    host="0.0.0.0",
                    port=alt_port,
                    debug=False,
                    allow_unsafe_werkzeug=True,
                )
            except Exception as e2:
                print(f"❌ Failed on port {alt_port} too: {e2}")
        else:
            print(f"\n❌ Network error: {e}")
    except KeyboardInterrupt:
        print("\n\n🛑 Server stopped by user")
    except Exception as e:
        print(f"\n❌ Error starting server: {e}")
        import traceback

        traceback.print_exc()
    finally:
        drive_manager.shutdown()


if __name__ == "__main__":
    import sys

    port = 8001

    # Parse command line arguments
    if len(sys.argv) > 1:
        if "--port" in sys.argv:
            try:
                port_idx = sys.argv.index("--port")
                port = int(sys.argv[port_idx + 1])
            except (IndexError, ValueError):
                print("Usage: python3 web_teleop.py [--port PORT]")
                print("Example: python3 web_teleop.py --port 8080")
                sys.exit(1)

    main(port)
