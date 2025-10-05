# ...existing code...
from config import *
from spark_can.spark_can import SparkBus
from spark_can.spark_controller import Controller
# from adafruit_servokit import ServoKit
import time
import curses
import sys
# ...existing code...

# kit = ServoKit(channels=16)
bus = SparkBus()

FLD = Controller(bus, FLD_ID)
FRD = Controller(bus, FRD_ID)
BLD = Controller(bus, BLD_ID)
BRD = Controller(bus, BRD_ID)

FLS = Controller(bus, FLS_ID)
FRS = Controller(bus, FRS_ID)
BLS = Controller(bus, BLD_ID)
BRS = Controller(bus, BRD_ID)

SHOULDER_PITCH = Controller(bus, SHOULDER_PITCH_ID)
SHOULDER_ROT = Controller(bus, SHOULDER_ROT_ID)
ELBOW_PITCH = Controller(bus, ELBOW_PITCH_ID)
WRIST_PITCH = Controller(bus, WRIST_PITCH_ID)
WRIST_ROT = Controller(bus, WRIST_ROT_ID)

pressed_keys = set()
_pressed_times = {}  # key -> last-seen timestamp
KEY_RELEASE_TIMEOUT = 0.35  # seconds; adjust to taste
POLL_DELAY = 0.02  # main loop sleep (seconds)

def set_drive_speeds(speed):
    FLD.set_speed(speed)
    BLD.set_speed(-1 * speed)
    FRD.set_speed(speed)
    BRD.set_speed(-1 * speed)

def set_steer_pos(pos):
    FLS.set_position(pos)
    BLS.set_position(-1 * pos)
    FRS.set_position(pos)
    BRS.set_position(-1 * pos)

def reset_steer_pos():
    FLS.set_position(DEFAULT_STEER_POS)
    BLS.set_position(-1 * DEFAULT_STEER_POS)
    FRS.set_position(DEFAULT_STEER_POS)
    BRS.set_position(-1 * DEFAULT_STEER_POS)

def set_rotation_pos():
    # Add check for wheel positions
    FLS.set_position(STEER_ROTATION_POS)
    BLS.set_position(-1 * STEER_ROTATION_POS)
    FRS.set_position(STEER_ROTATION_POS)
    BRS.set_position(-1 * STEER_ROTATION_POS)

def set_rotation_speed(speed):
    FLS.set_speed(speed)
    BLS.set_speed(speed)
    FRS.set_speed(speed)
    BRS.set_speed(speed)

def state_movement():
    speed = 0
    steer_pos = DEFAULT_STEER_POS

    # Set defualt wheel positions first
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

    if ('w' in pressed_keys and 'a' in pressed_keys) or ('w' in pressed_keys and 'd' in pressed_keys):
        speed = NORMAL_DRIVE_SPEED / 1.5
    elif ('s' in pressed_keys and 'a' in pressed_keys) or ('s' in pressed_keys and 'd' in pressed_keys):
        speed = -1 * MIN_DRIVE_SPEED

    if ('a' in pressed_keys and 'd' in pressed_keys):
        steer_pos = DEFAULT_STEER_POS
    elif 'a' in pressed_keys:
        steer_pos = STEER_LEFT_POS
    elif 'd' in pressed_keys:
        steer_pos = STEER_RIGHT_POS

    set_steer_pos(steer_pos)
    set_drive_speeds(speed)

def state_rotation():
    set_rotation_pos()

    if 'q' in pressed_keys and 'e' in pressed_keys:
        set_drive_speeds(0)
    elif 'q' in pressed_keys:
        set_rotation_speed(STEER_ROTATION_SPEED)
    elif 'e' in pressed_keys:
        set_rotation_speed(-1 * STEER_ROTATION_SPEED)

def arm_controls():
    shoulder_rot = 0
    shoulder_pitch = 0
    elbow_pitch = 0
    wrist_pitch = 0
    wrist_rot = 0
    claw = 0

    if 'z' in pressed_keys and 'x' in pressed_keys:
        shoulder_rot = 0
    elif 'z' in pressed_keys:
        shoulder_rot = -1 * SHOULDER_ROT_SPEED
    elif 'x' in pressed_keys:
        shoulder_rot = SHOULDER_ROT_SPEED

    if 'y' in pressed_keys and 'h' in pressed_keys:
        shoulder_pitch = 0
    elif 'y' in pressed_keys:
        shoulder_pitch = SHOULDER_PITCH_SPEED
    elif 'h' in pressed_keys:
        shoulder_pitch = -1 * SHOULDER_PITCH_SPEED

    if 'u' in pressed_keys and 'j' in pressed_keys:
        elbow_pitch = 0
    elif 'u' in pressed_keys:
        elbow_pitch = ELBOW_PITCH_SPEED
    elif 'j' in pressed_keys:
        elbow_pitch = -1 * ELBOW_PITCH_SPEED

    if 'i' in pressed_keys and 'k' in pressed_keys:
        wrist_pitch = 0
    elif 'i' in pressed_keys:
        wrist_pitch = WRIST_PITCH_SPEED
    elif 'k' in pressed_keys:
        wrist_pitch = -1 * WRIST_PITCH_SPEED

    if 'c' in pressed_keys and 'v' in pressed_keys:
        wrist_rot = 0
    elif 'c' in pressed_keys:
        wrist_rot = WRIST_ROT_SPEED
    elif 'v' in pressed_keys:
        wrist_rot = -1 * WRIST_ROT_SPEED

    if '[' in pressed_keys and ']' in pressed_keys:
        claw = 0
    elif '[' in pressed_keys:
        claw = CLAW_SPEED
    elif ']' in pressed_keys:
        claw = -1 * CLAW_SPEED

    SHOULDER_PITCH.set_speed(shoulder_pitch)
    SHOULDER_ROT.set_speed(shoulder_rot)
    ELBOW_PITCH.set_speed(elbow_pitch)
    WRIST_PITCH.set_speed(wrist_pitch)
    WRIST_ROT.set_speed(wrist_rot)
    # kit.servo[CLAW_CHANNEL].throttle = claw

def update_controls():
    arm_controls()
    # rotation mode if q/e present
    if 'q' in pressed_keys or 'e' in pressed_keys:
        state_rotation()
    elif 'q' not in pressed_keys and 'e' not in pressed_keys:
        reset_steer_pos()
    else:
        state_movement()

def _key_name_from_code(ch):
    # handle ctrl-letter (1..26), uppercase -> shift, printable ASCII
    if ch == -1:
        return None
    if 1 <= ch <= 26:
        # Ctrl-A .. Ctrl-Z
        letter = chr(ch + 96)
        return ('ctrl', letter)
    if 65 <= ch <= 90:
        # 'A'..'Z' -> lowercase letter and shift
        letter = chr(ch + 32)
        return ('shift', letter)
    if 32 <= ch <= 126:
        return (None, chr(ch))
    # map some curses special keys if needed
    if ch == curses.KEY_LEFT:
        return (None, 'left')
    if ch == curses.KEY_RIGHT:
        return (None, 'right')
    if ch == curses.KEY_UP:
        return (None, 'up')
    if ch == curses.KEY_DOWN:
        return (None, 'down')
    return None

def _register_key(name):
    now = time.time()
    _pressed_times[name] = now
    pressed_keys.add(name)

def _cleanup_stale_keys():
    now = time.time()
    stale = [k for k, t in _pressed_times.items() if now - t > KEY_RELEASE_TIMEOUT]
    for k in stale:
        _pressed_times.pop(k, None)
        pressed_keys.discard(k)

def _process_input(stdscr):
    # read all available inputs
    while True:
        ch = stdscr.getch()
        if ch == -1:
            break
        mapped = _key_name_from_code(ch)
        if not mapped:
            continue
        mod, name = mapped if isinstance(mapped, tuple) else (None, mapped)
        if mod == 'ctrl':
            _register_key('ctrl')
            _register_key(name)
        elif mod == 'shift':
            _register_key('shift')
            _register_key(name)
        else:
            _register_key(name)

def _main_loop(stdscr):
    # curses setup
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.noecho()
    curses.cbreak()
    stdscr.clear()
    stdscr.addstr(0, 0, "Headless teleop: Press Ctrl-C to quit.")
    stdscr.refresh()

    try:
        while True:
            _process_input(stdscr)
            _cleanup_stale_keys()
            update_controls()
            time.sleep(POLL_DELAY)
    except KeyboardInterrupt:
        pass
    finally:
        # best-effort reset: stop all motors
        set_drive_speeds(0)
        reset_steer_pos()
        SHOULDER_PITCH.set_speed(0)
        SHOULDER_ROT.set_speed(0)
        ELBOW_PITCH.set_speed(0)
        WRIST_PITCH.set_speed(0)
        WRIST_ROT.set_speed(0)
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()

def main():
    # run curses wrapper which initializes terminal safely
    try:
        curses.wrapper(_main_loop)
    except Exception:
        # ensure we don't crash silently when run non-interactively
        print("Failed to initialize terminal input. Run in an interactive SSH terminal.", file=sys.stderr)

if __name__ == "__main__":
    main()
# ...existing code...