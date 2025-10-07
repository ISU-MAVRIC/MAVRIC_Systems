import curses
import time
from SparkCANLib import SparkCAN, SparkController as Controller
from adafruit_servokit import ServoKit
from  config import *

# kit = ServoKit(channels=16)
bus = SparkCAN.SparkBus()

FLD = bus.init_controller(FLD_ID)
FRD = bus.init_controller(FRD_ID)
BLD = bus.init_controller(BLD_ID)
BRD = bus.init_controller(BRD_ID)

FLS = bus.init_controller(FLS_ID)
FRS = bus.init_controller(FRS_ID)
BLS = bus.init_controller(BLD_ID)
BRS = bus.init_controller(BRD_ID)

SHOULDER_PITCH = bus.init_controller(bus, SHOULDER_PITCH_ID)
SHOULDER_ROT = bus.init_controller(bus, SHOULDER_ROT_ID)
ELBOW_PITCH = bus.init_controller(bus, ELBOW_PITCH_ID)
WRIST_PITCH = bus.init_controller(bus, WRIST_PITCH_ID)
WRIST_ROT = bus.init_controller(bus, WRIST_ROT_ID)

def set_drive_speeds(speed):
    FLD.percent_output(speed)
    BLD.percent_output(-1 * speed)
    FRD.percent_output(speed)
    BRD.percent_output(-1 * speed)

def set_steer_pos(pos):
    FLS.position_output(pos)
    BLS.position_output(-1 * pos)
    FRS.position_output(pos)
    BRS.position_output(-1 * pos)

def reset_steer_pos():
    FLS.position_output(DEFAULT_STEER_POS)
    BLS.position_output(-1 * DEFAULT_STEER_POS)
    FRS.position_output(DEFAULT_STEER_POS)
    BRS.position_output(-1 * DEFAULT_STEER_POS)

def set_rotation_pos():
    # Add check for wheel positions
    FLS.position_output(STEER_ROTATION_POS)
    BLS.position_output(-1 * STEER_ROTATION_POS)
    FRS.position_output(STEER_ROTATION_POS)
    BRS.position_output(-1 * STEER_ROTATION_POS)

def set_rotation_speed(speed):
    FLS.percent_output(speed)
    BLS.percent_output(speed)
    FRS.percent_output(speed)
    BRS.percent_output(speed)

def reset_all():
    set_drive_speeds(0)
    reset_steer_pos()
    SHOULDER_PITCH.percent_output(0)
    SHOULDER_ROT.percent_output(0)
    ELBOW_PITCH.percent_output(0)
    WRIST_PITCH.percent_output(0)
    WRIST_ROT.percent_output(0)
    # kit.continuous_servo[CLAW_CHANNEL].throttle = 0
    return "All controls reset"

def main(stdscr):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()

    stdscr.addstr(0, 0,
        "Controls: w/s drive | a/d steer | q/e rotate | "
        "z/x shoulder rot | y/h shoulder pitch | u/j elbow | "
        "i/k wrist pitch | c/v wrist rot | [/] claw | space reset | q quit"
    )
    stdscr.refresh()

    while True:
        key = stdscr.getch()
        if key == -1:
            time.sleep(0.05)
            continue

        if key == ord("w"):
            set_drive_speeds(MIN_DRIVE_SPEED)
            msg = "Drive forward"
        elif key == ord("s"):
            set_drive_speeds(-1 * MIN_DRIVE_SPEED)
            msg = "Drive backward"
        elif key == ord("a"):
            set_steer_pos(STEER_LEFT_POS)
            msg = "Steer left"
        elif key == ord("d"):
            set_steer_pos(STEER_RIGHT_POS)
            msg = "Steer right"
        elif key == ord("q"):
            set_rotation_pos()
            set_rotation_speed(STEER_ROTATION_SPEED)
            msg = "Rotate left"
        elif key == ord("e"):
            set_rotation_pos()
            set_rotation_speed(-STEER_ROTATION_SPEED)
            msg = "Rotate right"
        elif key == ord("z"):
            SHOULDER_ROT.percent_output(-SHOULDER_ROT_SPEED)
            msg = "Shoulder rotate left"
        elif key == ord("x"):
            SHOULDER_ROT.percent_output(SHOULDER_ROT_SPEED)
            msg = "Shoulder rotate right"
        elif key == ord("y"):
            SHOULDER_PITCH.percent_output(SHOULDER_PITCH_SPEED)
            msg = "Shoulder pitch up"
        elif key == ord("h"):
            SHOULDER_PITCH.percent_output(-SHOULDER_PITCH_SPEED)
            msg = "Shoulder pitch down"
        elif key == ord("u"):
            ELBOW_PITCH.percent_output(ELBOW_PITCH_SPEED)
            msg = "Elbow up"
        elif key == ord("j"):
            ELBOW_PITCH.percent_output(-1 * ELBOW_PITCH_SPEED)
            msg = "Elbow down"
        elif key == ord("i"):
            WRIST_PITCH.percent_output(WRIST_PITCH_SPEED)
            msg = "Wrist pitch up"
        elif key == ord("k"):
            WRIST_PITCH.percent_output(-1 * WRIST_PITCH_SPEED)
            msg = "Wrist pitch down"
        elif key == ord("c"):
            WRIST_ROT.percent_output(WRIST_ROT_SPEED)
            msg = "Wrist rotate right"
        elif key == ord("v"):
            WRIST_ROT.percent_output(-1 * WRIST_ROT_SPEED)
            msg = "Wrist rotate left"
        elif key == ord("["):
            # kit.servo[CLAW_CHANNEL].throttle = CLAW_SPEED
            msg = "Claw open"
        elif key == ord("]"):
            # kit.servo[CLAW_CHANNEL].throttle = -CLAW_SPEED
            msg = "Claw close"
        elif key == ord(" "):
            msg = reset_all()
        elif key == ord("Q"):  # capital Q to quit
            reset_all()
            break
        else:
            msg = f"Unknown key: {key}"

        stdscr.addstr(2, 0, f"Last action: {msg}      ")
        stdscr.refresh()
        time.sleep(0.05)

if __name__ == "__main__":
    curses.wrapper(main)