#!/usr/bin/env python3
"""
Web-based Teleoperation Interface for MAVRIC Rover
Provides a browser-based GUI for controlling the rover with keyboard support.
"""

from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
from config import *
from SparkCANLib import SparkCAN, SparkController as COntroller
from adafruit_servokit import ServoKit
import time
import threading

app = Flask(__name__)
app.config["SECRET_KEY"] = "mavric_secret_key"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Initialize CAN bus and controllers
print("Initializing CAN bus...")
try:
    kit = ServoKit(channels=16)
    bus = SparkCAN.SparkBus()
    print("✓ CAN bus initialized (or running in simulation mode)")
except Exception as e:
    print(f"⚠ CAN bus initialization warning: {e}")
    print("  Continuing in simulation mode...")
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

# Track currently pressed keys and steering state
pressed_keys = set()
control_lock = threading.Lock()
steering_thread = None
steering_target = None
steering_lock = threading.Lock()


def reset_all():
    """Reset all motors and servos to default state"""
    set_drive_speeds(0)
    set_steer_pos_immediate(DEFAULT_STEER_POS)
    for motor in arm_motors.values():
        motor.percent_output(0)
    kit.continuous_servo[CLAW_CHANNEL].throttle = 0


def set_drive_speeds(speed):
    """Set all drive motor speeds (accounts for motor direction)"""
    print(f"    set_drive_speeds({speed})")
    drive_motors['FLD'].percent_output(speed)
    drive_motors['FRD'].percent_output(-speed)
    drive_motors['BLD'].percent_output(speed)
    drive_motors['BRD'].percent_output(-speed)


def set_steer_pos_immediate(pos):
    """Set steering position immediately without waiting"""
    steer_motors['FLS'].position_output(pos)
    steer_motors['FRS'].position_output(pos)
    steer_motors['BLS'].position_output(-pos)
    steer_motors['BRS'].position_output(-pos)


def _steering_worker(target_pos, check_position=False):
    """Background worker to handle steering transitions (non-blocking)"""
    set_steer_pos_immediate(target_pos)
    
    if check_position:
        # Optional: wait for position to be reached in background
        start_time = time.time()
        timeout = 2.0  # 2 second timeout
        
        while time.time() - start_time < timeout:
            with steering_lock:
                # Check if we've been interrupted by a new steering command
                if steering_target != target_pos:
                    return
            
            # Check if position reached
            if abs(steer_motors['FRS'].position - target_pos) <= POS_MARGIN_ERROR:
                break
            time.sleep(0.01)


def set_steer_pos(pos, wait=False):
    """Set steering position (non-blocking by default)"""
    global steering_thread, steering_target
    
    with steering_lock:
        steering_target = pos
    
    # Send command immediately
    set_steer_pos_immediate(pos)
    
    # Optionally spawn background thread for position monitoring
    if wait:
        if steering_thread and steering_thread.is_alive():
            pass  # Let existing thread finish or be interrupted
        steering_thread = threading.Thread(target=_steering_worker, args=(pos, True), daemon=True)
        steering_thread.start()


def set_rotation_pos():
    """Position wheels for rotation mode (non-blocking)"""
    global steering_target
    
    target_pos = STEER_ROTATION_POS
    with steering_lock:
        steering_target = -target_pos
    
    steer_motors['FLS'].position_output(STEER_ROTATION_POS)
    steer_motors['FRS'].position_output(-STEER_ROTATION_POS)
    steer_motors['BLS'].position_output(-STEER_ROTATION_POS)
    steer_motors['BRS'].position_output(STEER_ROTATION_POS)

    steering_thread = threading.Thread(target=_steering_worker, args=(target_pos,), daemon=True)
    steering_thread.start()


def set_rotation_speed(speed):
    """Set rotation speed for all drive motors"""
    for motor in drive_motors.values():
        motor.percent_output(speed)


def is_in_rotation_mode():
    """Check if wheels are currently in rotation position"""
    try:
        return (abs(STEER_ROTATION_POS - (-steer_motors['FRS'].position)) < POS_MARGIN_ERROR and 
                abs(STEER_ROTATION_POS - (-steer_motors['BLS'].position)) < POS_MARGIN_ERROR)
    except:
        return False


def state_movement():
    """Handle movement state controls (W, A, S, D) - non-blocking"""
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

    # If wheels are in rotation mode and we want to move, reset steering first
    if is_in_rotation_mode() and speed != 0:
        set_steer_pos(steer_pos, wait=True)  # BLOCKING - wait for wheels to reach position
    else:
        set_steer_pos(steer_pos, wait=False)
    
    set_drive_speeds(speed)


def state_rotation():
    """Handle rotation state controls (Q, E) - non-blocking"""
    set_rotation_pos()  # Non-blocking now
    
    q_pressed = "q" in pressed_keys
    e_pressed = "e" in pressed_keys

    if q_pressed and e_pressed:
        speed = 0
    elif q_pressed:
        speed = -STEER_ROTATION_SPEED
    elif e_pressed:
        speed = STEER_ROTATION_SPEED
    else:
        speed = 0
    
    set_rotation_speed(speed)


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
    
    kit.continuous_servo[CLAW_CHANNEL].throttle = claw


def update_controls():
    """Main control update function (assumes caller holds control_lock)"""
    print(f"update_controls() called - pressed_keys: {pressed_keys}")
    # Arm controls are always active
    arm_controls()

    # Movement and rotation are mutually exclusive
    if "q" in pressed_keys or "e" in pressed_keys:
        print("  -> Calling state_rotation()")
        state_rotation()
    else:
        # reset_steer_pos()
        print("  -> Calling state_movement()")
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


@socketio.on("connect")
def handle_connect():
    """Handle client connection"""
    print("Client connected")
    emit("status", {"message": "Connected to MAVRIC Rover"})


@socketio.on("disconnect")
def handle_disconnect():
    """Handle client disconnection - stop all motors"""
    print("Client disconnected")
    with control_lock:
        pressed_keys.clear()
        update_controls()


@socketio.on("key_down")
def handle_key_down(data):
    """Handle key press event from web client"""
    print(f"Received key_down event: {data}")
    key = data.get("key", "").lower()
    if key:
        print(f"Processing key down: {key}")
        with control_lock:
            pressed_keys.add(key)
            update_controls()
        emit("key_state", {"pressed_keys": list(pressed_keys)}, broadcast=True)


@socketio.on("key_up")
def handle_key_up(data):
    """Handle key release event from web client"""
    print(f"Received key_up event: {data}")
    key = data.get("key", "").lower()
    if key:
        print(f"Processing key up: {key}")
        with control_lock:
            pressed_keys.discard(key)
            update_controls()
        emit("key_state", {"pressed_keys": list(pressed_keys)}, broadcast=True)


@socketio.on("emergency_stop")
def handle_emergency_stop():
    """Emergency stop - clear all controls"""
    print("Emergency stop triggered")
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
