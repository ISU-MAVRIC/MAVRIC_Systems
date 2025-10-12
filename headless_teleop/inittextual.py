import time
from typing import Optional
import sys
import os

from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Static
from textual.reactive import reactive

from SparkCANLib import SparkCAN, SparkController as Controller
from config import *

# Initialize hardware
try:
    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=16)
except Exception as e:
    kit = None
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

    # Wait for wheels to reach position, but no longer than 2 seconds
    target = -STEER_ROTATION_POS
    start_time = time.time()
    while abs(steer_motors["FRS"].position - target) > POS_MARGIN_ERROR:
        if time.time() - start_time > 2.0:
            break
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
    if kit is not None:
        kit.continuous_servo[CLAW_CHANNEL].throttle = 0
    return "All controls reset"


class TeleopApp(App):
    """Textual-based TUI for MAVRIC teleoperation.

    This app replaces the previous curses UI while reusing the same
    hardware control helpers defined above.
    """

    CSS_PATH = None

    # current action message shown in the UI
    action_message: reactive[str] = reactive("")

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Static(
            "Controls: w/s drive | a/d steer | q/e rotate | z/x shoulder rot | y/h shoulder pitch | u/j elbow | i/k wrist pitch | c/v wrist rot | [/] claw | space reset | Q quit",
            id="controls",
            markup=False,
        )
        yield Static("Last action: ", id="action")
        yield Footer()

    def on_mount(self) -> None:
        reset_all()

    def _set_action(self, text: Optional[str]) -> None:
        if text:
            self.action_message = text

    async def on_key(self, event) -> None:  # textual key events
        key = event.key
        # quit
        if key == "Q":
            reset_all()
            await self.action_quit()
            return

        # map keys to actions (mirror previous mapping)
        mapping = {
            "w": (set_drive_speeds, (MIN_DRIVE_SPEED,), "Drive forward"),
            "s": (set_drive_speeds, (-MIN_DRIVE_SPEED,), "Drive backward"),
            "a": (set_steer_pos, (STEER_LEFT_POS,), "Steer left"),
            "d": (set_steer_pos, (STEER_RIGHT_POS,), "Steer right"),
            "z": (lambda: arm_motors["SHOULDER_ROT"].percent_output(-SHOULDER_ROT_SPEED), (), "Shoulder rotate left"),
            "x": (lambda: arm_motors["SHOULDER_ROT"].percent_output(SHOULDER_ROT_SPEED), (), "Shoulder rotate right"),
            "y": (lambda: arm_motors["SHOULDER_PITCH"].percent_output(SHOULDER_PITCH_SPEED), (), "Shoulder pitch up"),
            "h": (lambda: arm_motors["SHOULDER_PITCH"].percent_output(-SHOULDER_PITCH_SPEED), (), "Shoulder pitch down"),
            "u": (lambda: arm_motors["ELBOW_PITCH"].percent_output(-ELBOW_PITCH_SPEED), (), "Elbow up"),
            "j": (lambda: arm_motors["ELBOW_PITCH"].percent_output(ELBOW_PITCH_SPEED), (), "Elbow down"),
            "i": (lambda: arm_motors["WRIST_PITCH"].percent_output(WRIST_PITCH_SPEED), (), "Wrist pitch up"),
            "k": (lambda: arm_motors["WRIST_PITCH"].percent_output(-WRIST_PITCH_SPEED), (), "Wrist pitch down"),
            "c": (lambda: arm_motors["WRIST_ROT"].percent_output(-WRIST_ROT_SPEED), (), "Wrist rotate right"),
            "v": (lambda: arm_motors["WRIST_ROT"].percent_output(WRIST_ROT_SPEED), (), "Wrist rotate left"),
            "[": (lambda: setattr(kit.continuous_servo[CLAW_CHANNEL], "throttle", CLAW_SPEED), (), "Claw open"),
            "]": (lambda: setattr(kit.continuous_servo[CLAW_CHANNEL], "throttle", -CLAW_SPEED), (), "Claw close"),
            "space": (reset_all, (), "Reset all"),
        }

        if key in ("q", "e"):
            # rotation: set rotation pose then spin
            if key == "q":
                set_rotation_pos()
                set_rotation_speed(-STEER_ROTATION_SPEED)
                self._set_action("Rotate left")
            else:
                set_rotation_pos()
                set_rotation_speed(STEER_ROTATION_SPEED)
                self._set_action("Rotate right")
            return

        if key in mapping:
            func, args, msg = mapping[key]
            try:
                result = func(*args) if args else func()
            except Exception:
                # hardware not available or action failed in simulation - show message
                result = "Action failed"
            self._set_action(result if result is not None else msg)
            return

        self._set_action(f"Unknown key: {key}")


    def watch_action_message(self, value: str) -> None:
        """Update the action display whenever the reactive value changes."""
        action_widget = self.query_one("#action", Static)
        action_widget.update(f"Last action: {value}")


if __name__ == "__main__":
    TeleopApp().run()
