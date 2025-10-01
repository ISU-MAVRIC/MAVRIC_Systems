import time
from adafruit_lib import ServoKit

kit = ServoKit(channels=16)

kit.continuous_servo[0].throttle = 1
time.sleep(1)
kit.continuous_servo[0].throttle = -1
time.sleep(1)
kit.continuous_servo[0].throttle = 0


# kit.servo[0].angle = 180
# time.sleep(1)
# kit.servo[0].angle = 0