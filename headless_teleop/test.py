import curses
import time
from SparkCANLib import SparkCAN, SparkController as Controller
from adafruit_servokit import ServoKit
from  config import *

bus = SparkCAN.SparkBus()

FLS = bus.init_controller(FLS_ID)
FRS = bus.init_controller(FRS_ID)
BLS = bus.init_controller(BLS_ID)
BRS = bus.init_controller(BRS_ID)

# FLS.position_output(0.5)
FLS.percent_output(0.01)
# time.sleep(5)
# FRS.position_output(0.8)
# BLS.position_output(-0.8)
# BRS.position_output(-0.8)


while True:
    # print(f"FLS.position: {FLS.position}, FRS.position: {FRS.position}, BLS.position: {BLS.position}, BRS.position: {BRS.position}")
    # time.sleep(0.1)
    pass