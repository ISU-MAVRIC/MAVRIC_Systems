from config import *
from spark_can.spark_can import SparkBus
from spark_can.spark_controller import Controller
# from adafruit_servokit import ServoKit
import time
import keyboard

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
    
    SHOULDER_PITCH.percent_output(shoulder_pitch)
    SHOULDER_ROT.percent_output(shoulder_rot)
    ELBOW_PITCH.percent_output(elbow_pitch)
    WRIST_PITCH.percent_output(wrist_pitch)
    WRIST_ROT.percent_output(wrist_rot)
    # kit.servo[CLAW_CHANNEL].throttle = claw


def update_controls():
    arm_controls()
    if 'q' in pressed_keys or 'e' in pressed_keys:
        state_rotation()
    elif 'q' not in pressed_keys and 'e' not in pressed_keys:
        reset_steer_pos()
    else:
        state_movement()


def on_press(key):
    pressed_keys.add(key.name)
    update_controls()


def on_release(key):
    pressed_keys.discard(key.name)
    update_controls()


def main():
    keyboard.on_press(on_press)
    keyboard.on_release(on_release)


if __name__ == "__main__":
    main()