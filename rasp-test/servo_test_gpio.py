"""Interactive servo test using Raspberry Pi GPIO pins and RPi.GPIO.

Usage:
  - Install RPi.GPIO on the Pi (usually preinstalled):
      sudo apt update
      sudo apt install python3-rpi.gpio
  - Run:
      python3 rasp-test/servo_test_gpio.py

Prompt format:
  <pin> <angle>    e.g. "18 90" sets GPIO18 to 90 degrees (BCM numbering)
  <angle>          single number applies to default pin (GPIO18)
  q / quit         exit

Notes:
  - Uses BCM pin numbering by default. Change GPIO.setmode(GPIO.BOARD)
    if you prefer board numbering.
  - Default angle->duty mapping uses 2.5% (0°) -> 12.5% (180°) which
    matches many SG90/HS-XXX servos. Adjust MIN_DC / MAX_DC as needed.
"""

import sys
import time

try:
    import RPi.GPIO as GPIO
except Exception as e:
    print("Error: RPi.GPIO not available. This script must run on a Raspberry Pi with RPi.GPIO installed.")
    raise

# Use BCM numbering by default
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# PWM frequency for servo (Hz)
FREQUENCY = 50
# Duty cycle range (percent) for 0..180 degrees. Adjust for your servo.
MIN_DC = 2.5
MAX_DC = 12.5

# Default pin to use when user enters a single angle value
DEFAULT_PIN = 18

# Keep PWM objects per pin so multiple pins can be controlled
_pwms = {}


def angle_to_duty(angle: float) -> float:
    """Convert 0-180 angle to duty cycle percentage.

    Linear mapping: MIN_DC -> 0 deg, MAX_DC -> 180 deg
    """
    if angle < 0 or angle > 180:
        raise ValueError("angle must be between 0 and 180")
    span = MAX_DC - MIN_DC
    return MIN_DC + (angle / 180.0) * span


def ensure_pwm(pin: int):
    """Create and start a PWM object for `pin` if not already started."""
    if pin in _pwms:
        return _pwms[pin]
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, FREQUENCY)
    pwm.start(0)
    _pwms[pin] = pwm
    # Small delay to let PWM stabilize
    time.sleep(0.05)
    return pwm


def apply_angle(pin: int, angle: float) -> None:
    """Apply the given angle to the servo connected to `pin` (BCM)."""
    pwm = ensure_pwm(pin)
    dc = angle_to_duty(angle)
    pwm.ChangeDutyCycle(dc)


def cleanup():
    for pwm in _pwms.values():
        try:
            pwm.ChangeDutyCycle(0)
            pwm.stop()
        except Exception:
            pass
    _pwms.clear()
    GPIO.cleanup()


def interactive_loop():
    print("GPIO servo interactive test (BCM). Default pin:", DEFAULT_PIN)
    print("Enter: <pin> <angle>  e.g. '18 90'  or '<angle>' to use default pin")
    print("Type 'q' or 'quit' to exit.")

    while True:
        try:
            user = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not user:
            continue
        if user.lower() in ("q", "quit", "exit"):
            print("Exiting.")
            break

        parts = user.split()
        try:
            if len(parts) == 1:
                # single number -> angle for default pin
                angle = float(parts[0])
                pin = DEFAULT_PIN
            else:
                pin = int(parts[0])
                angle = float(parts[1])
            apply_angle(pin, angle)
            print(f"Set GPIO{pin} -> angle {angle}")
            # keep pulse for a short time to let servo move, then clear pulse
            time.sleep(0.5)
            # Optionally reduce duty to 0 to avoid jitter; comment out if servo needs holding torque
            try:
                _pwms[pin].ChangeDutyCycle(0)
            except Exception:
                pass
        except ValueError as e:
            print(f"Invalid input: {e}")
        except RuntimeError as e:
            print(f"GPIO error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")


if __name__ == "__main__":
    try:
        interactive_loop()
    finally:
        cleanup()
        sys.exit(0)
