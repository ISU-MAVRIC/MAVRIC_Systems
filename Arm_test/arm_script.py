from SparkCANLib import SparkController, SparkCAN
import numpy as np

bus = SparkCAN.SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

arm_motor = bus.init_controller(5)

while (True):
    arm_motor.percent_output(0.05)