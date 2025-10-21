import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

# Continuous servo on channel 1
# kit.continuous_servo[1].throttle = 1
# time.sleep(5)
# kit.continuous_servo[1].throttle = -1
# time.sleep(15)
# kit.continuous_servo[1].throttle = 0

# Standard servo on channel 0
kit.servo[0].angle = 0
time.sleep(2)
kit.servo[0].angle = 90
time.sleep(2)
kit.servo[0].angle = 180    