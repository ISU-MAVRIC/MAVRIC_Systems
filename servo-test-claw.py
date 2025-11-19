import time
import sys
from adafruit_servokit import ServoKit

# Try to create an explicit I2C object (bus 1 on Raspberry Pi) so the
# PCA9685 driver uses the correct bus. If Blinka/board isn't available,
# fall back to the default ServoKit constructor.
try:
	import board
	import busio

	i2c = busio.I2C(board.SCL, board.SDA)
	kit = ServoKit(channels=16, i2c=i2c)
except Exception:
	# This may happen on systems without Blinka. ServoKit will try to
	# open the system I2C bus (usually bus 1 on Raspberry Pi) by default.
	print("Warning: couldn't create explicit I2C object; using default ServoKit.")
	kit = ServoKit(channels=16)


def apply_angle(channel: int, angle: int) -> None:
	if not (0 <= channel <= 15):
		raise ValueError("channel must be between 0 and 15")
	# if not (0 <= angle <= 180):
	# 	raise ValueError("angle must be between 0 and 180")
	kit.servo[channel].angle = angle


def interactive_loop():
	print("Servo interactive test (bus 1).")
	print("Enter: <channel> <angle>  e.g. '0 90'  or 'q' to quit")
	kit.servo[1].set_pulse_width_range(900, 2100)
	kit.servo[1].actuation_range = 120
	# kit.servo[1].actuation_range = 1890
	while True:
		try:
			user = input("> ").strip()
		except (EOFError, KeyboardInterrupt):
			print("\nExiting.")
			return

		if not user:
			continue
		if user.lower() in ("q", "quit", "exit"):
			print("Exiting.")
			return

		parts = user.split()
		try:
			if len(parts) == 1:
				# single number -> apply to channel 0
				angle = int(parts[0])
				channel = 0
			else:
				channel = int(parts[0])
				angle = int(parts[1])
			apply_angle(channel, angle)
			print(f"Set channel {channel} -> angle {angle}")
			# small delay to give servo time to move
			time.sleep(0.2)
		except ValueError as e:
			print(f"Invalid input: {e}")
		except AttributeError as e:
			print(f"Servo library error: {e}")
		except Exception as e:
			print(f"Unexpected error: {e}")


if __name__ == "__main__":
	try:
		interactive_loop()
	except Exception:
		print("Fatal error, exiting.")
		sys.exit(1)