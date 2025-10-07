import curses
import time
from spark_can.spark_can import SparkBus
from spark_can.spark_controller import Controller
from adafruit_servokit import ServoKit
from  config import *

bus = SparkBus()

FLS = bus.init_controller(FLS_ID)
FRS = bus.init_controller(FRS_ID)
BLS = bus.init_controller(BLS_ID)
BRS = bus.init_controller(BRS_ID)

while True:
    print(f"FLS.position: {FLS.position}, FRS.position: {FRS.position}, BLS.position: {BLS.position}, BRS.position: {BRS.position}")
    time.sleep(0.1)