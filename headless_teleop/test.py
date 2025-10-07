import curses
import time
from spark_can.spark_can import SparkBus
from spark_can.spark_controller import Controller
from adafruit_servokit import ServoKit
from  config import *

bus = SparkBus()

FLD = Controller(bus, FLD_ID)
FRD = Controller(bus, FRD_ID)
BLD = Controller(bus, BLD_ID)
BRD = Controller(bus, BRD_ID)

FLD.percent_output(0.1)
FRD.percent_output(0.1)
BLD.percent_output(-0.1)
BRD.percent_output(-0.1)

while True:
    pass