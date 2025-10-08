import curses
import time
from SparkCANLib import SparkCAN, SparkController as Controller
from adafruit_servokit import ServoKit
from config import *

# Initialize hardware
kit = ServoKit(channels=16)
bus = SparkCAN.SparkBus()

# Initialize drive motors (all 4 wheels)
drive_motors = {
    "FLD": bus.init_controller(FLD_ID),
    "FRD": bus.init_controller(FRD_ID),
    "BLD": bus.init_controller(BLD_ID),
    "BRD": bus.init_controller(BRD_ID),
}

# Initialize steer motors (all 4 wheels)
steer_motors = {
    "FLS": bus.init_controller(FLS_ID),
    "FRS": bus.init_controller(FRS_ID),
    "BLS": bus.init_controller(BLS_ID),
    "BRS": bus.init_controller(BRS_ID),
}

# Initialize arm motors
arm_motors = {
    "SHOULDER_PITCH": bus.init_controller(SHOULDER_PITCH_ID),
    "SHOULDER_ROT": bus.init_controller(SHOULDER_ROT_ID),
    "ELBOW_PITCH": bus.init_controller(ELBOW_PITCH_ID),
    "WRIST_PITCH": bus.init_controller(WRIST_PITCH_ID),
    "WRIST_ROT": bus.init_controller(WRIST_ROT_ID),
}


def set_drive_speeds(speed):
    """Set all drive motor speeds (accounts for motor direction)"""

    if (
        abs(STEER_ROTATION_POS - (-steer_motors["FRS"].position)) < POS_MARGIN_ERROR
        and abs(STEER_ROTATION_POS - (-steer_motors["BLS"].position))
        < POS_MARGIN_ERROR
    ):
        reset_steer_pos()
    drive_motors["FLD"].percent_output(speed)
    drive_motors["FRD"].percent_output(-speed)
    drive_motors["BLD"].percent_output(speed)
    drive_motors["BRD"].percent_output(-speed)


def set_steer_pos(pos):
    """Set all steering positions (accounts for motor direction)"""
    steer_motors["FLS"].position_output(pos)
    steer_motors["FRS"].position_output(pos)
    steer_motors["BLS"].position_output(-pos)
    steer_motors["BRS"].position_output(-pos)

    # Wait for wheels fully steer if previously in rotation mode
    # while (
    #     abs(pos - (steer_motors["FLS"].position)) > POS_MARGIN_ERROR
    #     and 
    #     abs(pos - (-1 * steer_motors["BLS"].position)) > POS_MARGIN_ERROR
    # ):
    #     time.sleep(0.01)


def reset_steer_pos():
    """Reset steering to default position"""
    set_steer_pos(DEFAULT_STEER_POS)

    # while abs(-steer_motors["FRS"].position - DEFAULT_STEER_POS) > POS_MARGIN_ERROR:
    #     time.sleep(0.01)


def set_rotation_pos():
    """Position wheels for rotation mode"""
    steer_motors["FLS"].position_output(STEER_ROTATION_POS)
    steer_motors["FRS"].position_output(-STEER_ROTATION_POS)
    steer_motors["BLS"].position_output(-STEER_ROTATION_POS)
    steer_motors["BRS"].position_output(STEER_ROTATION_POS)

    # Wait for wheels to reach position
    target = -STEER_ROTATION_POS
    while abs(steer_motors["FRS"].position - target) > POS_MARGIN_ERROR:
        time.sleep(0.01)


def set_rotation_speed(speed):
    """Set rotation speed for all drive motors"""
    for motor in drive_motors.values():
        motor.percent_output(speed)


def reset_all():
    """Reset all motors and servos to default state"""
    set_drive_speeds(0)
    reset_steer_pos()
    for motor in arm_motors.values():
        motor.percent_output(0)
    kit.continuous_servo[CLAW_CHANNEL].throttle = 0
    return "All controls reset"


def main(stdscr):
    """Main control loop using curses for input"""
    reset_all()
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()

    # Display controls header
    stdscr.addstr(
        0,
        0,
        "Controls: w/s drive | a/d steer | q/e rotate | "
        "z/x shoulder rot | y/h shoulder pitch | u/j elbow | "
        "i/k wrist pitch | c/v wrist rot | [/] claw | space reset | Q quit",
    )
    stdscr.refresh()

    # Key mapping for more efficient processing
    # Format: key -> (function, args, message)
    key_actions = {
        ord("w"): (set_drive_speeds, (MIN_DRIVE_SPEED,), "Drive forward"),
        ord("s"): (set_drive_speeds, (-MIN_DRIVE_SPEED,), "Drive backward"),
        ord("a"): (set_steer_pos, (STEER_LEFT_POS,), "Steer left"),
        ord("d"): (set_steer_pos, (STEER_RIGHT_POS,), "Steer right"),
        ord("z"): (
            lambda: arm_motors["SHOULDER_ROT"].percent_output(-SHOULDER_ROT_SPEED),
            (),
            "Shoulder rotate left",
        ),
        ord("x"): (
            lambda: arm_motors["SHOULDER_ROT"].percent_output(SHOULDER_ROT_SPEED),
            (),
            "Shoulder rotate right",
        ),
        ord("y"): (
            lambda: arm_motors["SHOULDER_PITCH"].percent_output(SHOULDER_PITCH_SPEED),
            (),
            "Shoulder pitch up",
        ),
        ord("h"): (
            lambda: arm_motors["SHOULDER_PITCH"].percent_output(-SHOULDER_PITCH_SPEED),
            (),
            "Shoulder pitch down",
        ),
        ord("u"): (
            lambda: arm_motors["ELBOW_PITCH"].percent_output(-ELBOW_PITCH_SPEED),
            (),
            "Elbow up",
        ),
        ord("j"): (
            lambda: arm_motors["ELBOW_PITCH"].percent_output(ELBOW_PITCH_SPEED),
            (),
            "Elbow down",
        ),
        ord("i"): (
            lambda: arm_motors["WRIST_PITCH"].percent_output(WRIST_PITCH_SPEED),
            (),
            "Wrist pitch up",
        ),
        ord("k"): (
            lambda: arm_motors["WRIST_PITCH"].percent_output(-WRIST_PITCH_SPEED),
            (),
            "Wrist pitch down",
        ),
        ord("c"): (
            lambda: arm_motors["WRIST_ROT"].percent_output(-WRIST_ROT_SPEED),
            (),
            "Wrist rotate right",
        ),
        ord("v"): (
            lambda: arm_motors["WRIST_ROT"].percent_output(WRIST_ROT_SPEED),
            (),
            "Wrist rotate left",
        ),
        ord("["): (
            lambda: setattr(kit.continuous_servo[CLAW_CHANNEL], "throttle", CLAW_SPEED),
            (),
            "Claw open",
        ),
        ord("]"): (
            lambda: setattr(
                kit.continuous_servo[CLAW_CHANNEL], "throttle", -CLAW_SPEED
            ),
            (),
            "Claw close",
        ),
        ord(" "): (reset_all, (), None),  # Message returned by function
    }

    # Special rotation handlers (need to set position first)
    def rotate_left():
        set_rotation_pos()
        set_rotation_speed(-STEER_ROTATION_SPEED)
        return "Rotate left"

    def rotate_right():
        set_rotation_pos()
        set_rotation_speed(STEER_ROTATION_SPEED)
        return "Rotate right"

    key_actions[ord("q")] = (rotate_left, (), None)
    key_actions[ord("e")] = (rotate_right, (), None)

    last_msg = ""

    while True:
        key = stdscr.getch()

        if key == -1:
            # No key pressed - sleep to reduce CPU usage
            time.sleep(0.05)
            continue

        # Handle quit
        if key == ord("Q"):
            reset_all()
            break

        # Process key action
        if key in key_actions:
            func, args, msg = key_actions[key]
            result = func(*args)
            # Use returned message if available, otherwise use predefined message
            msg = result if result is not None else msg
        else:
            msg = f"Unknown key: {key}"

        # Only update display if message changed
        if msg != last_msg:
            stdscr.addstr(2, 0, f"Last action: {msg}      ")
            stdscr.refresh()
            last_msg = msg

        time.sleep(0.05)


if __name__ == "__main__":
    curses.wrapper(main)
