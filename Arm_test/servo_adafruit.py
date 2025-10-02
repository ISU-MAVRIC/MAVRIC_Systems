import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

kit.continuous_servo[1].throttle = 1
time.sleep(5)
kit.continuous_servo[1].throttle = -1
time.sleep(15)
kit.continuous_servo[1].throttle = 0
