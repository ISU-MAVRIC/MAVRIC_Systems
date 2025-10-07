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
app.config['SECRET_KEY'] = 'mavric_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Initialize CAN bus and controllers
print("Initializing CAN bus...")
try:
    bus = SparkCAN.SparkBus()
    kit = ServoKit(channels=16)
    print("✓ CAN bus initialized (or running in simulation mode)")
except Exception as e:
    print(f"⚠ CAN bus initialization warning: {e}")
    print("  Continuing in simulation mode...")
    bus = SparkCAN.SparkBus()

FLD = bus.init_controller(FLD_ID)
FRD = bus.init_controller(FRD_ID)
BLD = bus.init_controller(BLD_ID)
BRD = bus.init_controller(BRD_ID)

FLS = bus.init_controller(FLS_ID)
FRS = bus.init_controller(FRS_ID)
BLS = bus.init_controller(BLS_ID)
BRS = bus.init_controller(BRS_ID)

SHOULDER_PITCH = bus.init_controller(SHOULDER_PITCH_ID)
SHOULDER_ROT = bus.init_controller(SHOULDER_ROT_ID)
ELBOW_PITCH = bus.init_controller(ELBOW_PITCH_ID)
WRIST_PITCH = bus.init_controller(WRIST_PITCH_ID)
WRIST_ROT = bus.init_controller(WRIST_ROT_ID)

# Track currently pressed keys
pressed_keys = set()
control_lock = threading.Lock()


def set_drive_speeds(speed):
    FLD.percent_output(speed)
    FRD.percent_output(-1 * speed)
    BLD.percent_output(speed)
    BRD.percent_output(-1 * speed)

def set_steer_pos(pos):
    FLS.position_output(pos)
    FRS.position_output(pos)
    BLS.position_output(-1 * pos)
    BRS.position_output(-1 * pos)

def reset_steer_pos():
    FLS.position_output(DEFAULT_STEER_POS)
    FRS.position_output(DEFAULT_STEER_POS)
    BLS.position_output(-1 * DEFAULT_STEER_POS)
    BRS.position_output(-1 * DEFAULT_STEER_POS)

def set_rotation_pos():
    # Add check for wheel positions
    FLS.position_output(STEER_ROTATION_POS)
    FRS.position_output(-1 * STEER_ROTATION_POS)
    BLS.position_output(-1 * STEER_ROTATION_POS)
    BRS.position_output(STEER_ROTATION_POS)
    target = -1 * STEER_ROTATION_POS
    while abs(FRS.position - target) > 0.8:
        time.sleep(0.01)

def set_rotation_speed(speed):
    FLD.percent_output(speed)
    BLD.percent_output(speed)
    FRD.percent_output(speed)
    BRD.percent_output(speed)


def state_movement():
    """Handle movement state controls (W, A, S, D)"""
    speed = 0
    steer_pos = DEFAULT_STEER_POS

    # Determine drive speed
    if 'w' in pressed_keys and 'shift' in pressed_keys:
        speed = MAX_DRIVE_SPEED
    elif 'w' in pressed_keys and 'ctrl' in pressed_keys:
        speed = MIN_DRIVE_SPEED
    elif 'w' in pressed_keys and 's' in pressed_keys:
        speed = 0
    elif 'w' in pressed_keys:
        speed = NORMAL_DRIVE_SPEED
    elif 's' in pressed_keys:
        speed = -1 * MIN_DRIVE_SPEED

    # Adjust speed when turning
    if ('w' in pressed_keys and 'a' in pressed_keys) or ('w' in pressed_keys and 'd' in pressed_keys):
        speed = MIN_DRIVE_SPEED
    elif ('s' in pressed_keys and 'a' in pressed_keys) or ('s' in pressed_keys and 'd' in pressed_keys):
        speed = -1 * MIN_DRIVE_SPEED

    # Determine steering position
    if 'a' in pressed_keys and 'd' in pressed_keys:
        steer_pos = DEFAULT_STEER_POS
    elif 'a' in pressed_keys:
        steer_pos = STEER_LEFT_POS
    elif 'd' in pressed_keys:
        steer_pos = STEER_RIGHT_POS
    
    set_steer_pos(steer_pos)
    set_drive_speeds(speed)


def state_rotation():
    """Handle rotation state controls (Q, E)"""
    set_rotation_pos()

    if 'q' in pressed_keys and 'e' in pressed_keys:
        set_drive_speeds(0)
    elif 'q' in pressed_keys:
        set_rotation_speed(-1 * STEER_ROTATION_SPEED)
    elif 'e' in pressed_keys:
        set_rotation_speed(STEER_ROTATION_SPEED)
    else:
        set_drive_speeds(0)


def arm_controls():
    """Handle arm controls (always active)"""
    shoulder_rot = 0
    shoulder_pitch = 0
    elbow_pitch = 0
    wrist_pitch = 0
    wrist_rot = 0
    claw = 0

    # Shoulder rotation (Z/X)
    if 'z' in pressed_keys and 'x' in pressed_keys:
        shoulder_rot = 0
    elif 'z' in pressed_keys:
        shoulder_rot = -1 * SHOULDER_ROT_SPEED
    elif 'x' in pressed_keys:
        shoulder_rot = SHOULDER_ROT_SPEED

    # Shoulder pitch (Y/H)
    if 'y' in pressed_keys and 'h' in pressed_keys:
        shoulder_pitch = 0
    elif 'y' in pressed_keys:
        shoulder_pitch = SHOULDER_PITCH_SPEED
    elif 'h' in pressed_keys:
        shoulder_pitch = -1 * SHOULDER_PITCH_SPEED

    # Elbow pitch (U/J)
    if 'u' in pressed_keys and 'j' in pressed_keys:
        elbow_pitch = 0
    elif 'u' in pressed_keys:
        elbow_pitch = -1 * ELBOW_PITCH_SPEED
    elif 'j' in pressed_keys:
        elbow_pitch = ELBOW_PITCH_SPEED

    # Wrist pitch (I/K)
    if 'i' in pressed_keys and 'k' in pressed_keys:
        wrist_pitch = 0
    elif 'i' in pressed_keys:
        wrist_pitch = WRIST_PITCH_SPEED
    elif 'k' in pressed_keys:
        wrist_pitch = -1 * WRIST_PITCH_SPEED

    # Wrist rotation (C/V)
    if 'c' in pressed_keys and 'v' in pressed_keys:
        wrist_rot = 0
    elif 'c' in pressed_keys:
        wrist_rot = -1 * WRIST_ROT_SPEED
    elif 'v' in pressed_keys:
        wrist_rot = WRIST_ROT_SPEED

    # Claw ([/])
    if '[' in pressed_keys and ']' in pressed_keys:
        claw = 0
    elif '[' in pressed_keys:
        claw = CLAW_SPEED
    elif ']' in pressed_keys:
        claw = -1 * CLAW_SPEED
    
    SHOULDER_PITCH.percent_output(shoulder_pitch)
    SHOULDER_ROT.percent_output(shoulder_rot)
    ELBOW_PITCH.percent_output(elbow_pitch)
    WRIST_PITCH.percent_output(wrist_pitch)
    WRIST_ROT.percent_output(wrist_rot)
    kit.continuous_servo[CLAW_CHANNEL].throttle = claw


def update_controls():
    """Main control update function"""
    with control_lock:
        # Arm controls are always active
        arm_controls()
        
        # Movement and rotation are mutually exclusive
        if 'q' in pressed_keys or 'e' in pressed_keys:
            state_rotation()
        else:
            reset_steer_pos()
            state_movement()


@app.route('/')
def index():
    """Serve the main web interface"""
    return render_template('index.html')


@app.route('/api/status')
def status():
    """Get current control status"""
    with control_lock:
        return jsonify({
            'pressed_keys': list(pressed_keys),
            'status': 'running'
        })


@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print('Client connected')
    emit('status', {'message': 'Connected to MAVRIC Rover'})


@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection - stop all motors"""
    print('Client disconnected')
    with control_lock:
        pressed_keys.clear()
        update_controls()


@socketio.on('key_down')
def handle_key_down(data):
    """Handle key press event from web client"""
    key = data.get('key', '').lower()
    if key:
        with control_lock:
            pressed_keys.add(key)
            update_controls()
        emit('key_state', {'pressed_keys': list(pressed_keys)}, broadcast=True)


@socketio.on('key_up')
def handle_key_up(data):
    """Handle key release event from web client"""
    key = data.get('key', '').lower()
    if key:
        with control_lock:
            pressed_keys.discard(key)
            update_controls()
        emit('key_state', {'pressed_keys': list(pressed_keys)}, broadcast=True)


@socketio.on('emergency_stop')
def handle_emergency_stop():
    """Emergency stop - clear all controls"""
    print('Emergency stop triggered')
    with control_lock:
        pressed_keys.clear()
        update_controls()
    emit('key_state', {'pressed_keys': []}, broadcast=True)


def main(port=5000):
    """Start the web server"""
    print("="*60)
    print("Starting MAVRIC Web Teleop Interface...")
    print("="*60)
    print("")
    print("Access the interface at:")
    print(f"  - Local:    http://localhost:{port}")
    print(f"  - Network:  http://0.0.0.0:{port}")
    print("")
    print("Press Ctrl+C to stop the server")
    print("="*60)
    print("")
    
    try:
        socketio.run(app, host='0.0.0.0', port=port, debug=False, allow_unsafe_werkzeug=True)
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
                socketio.run(app, host='0.0.0.0', port=alt_port, debug=False, allow_unsafe_werkzeug=True)
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
    port = 5000
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        if '--port' in sys.argv:
            try:
                port_idx = sys.argv.index('--port')
                port = int(sys.argv[port_idx + 1])
            except (IndexError, ValueError):
                print("Usage: python3 web_teleop.py [--port PORT]")
                print("Example: python3 web_teleop.py --port 8080")
                sys.exit(1)
    
    main(port)
