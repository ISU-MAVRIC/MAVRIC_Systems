from config import *
from spark_can import SparkBus, Controller
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
bus = SparkBus()

FLD = Controller(bus, FLD_ID)
FRD = Controller(bus, FRD_ID)
BLD = Controller(bus, BLD_ID)
BRD = Controller(bus, BRD_ID)

FLS = Controller(bus, FLS_ID)
FRS = Controller(bus, FRS_ID)
BLS = Controller(bus, BLD_ID)
BRS = Controller(bus, BRD_ID)

SHOULDER_PITCH = Controller(bus, SHOULDER_PITCH_ID)
SHOULDER_ROT = Controller(bus, SHOULDER_ROT_ID)
ELBOW_PITCH = Controller(bus, ELBOW_PITCH_ID)
WRIST_PITCH = Controller(bus, WRIST_PITCH_ID)
WRIST_ROT = Controller(bus, WRIST_ROT_ID)


def main():
    pass

if __name__ == "__main__":
    main()