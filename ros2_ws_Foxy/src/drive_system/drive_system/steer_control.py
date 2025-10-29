"""
steer_control.py

Desc: Uses the CANbus interface to set the velocity of the steer motors.
Author: Andrew Boun, Aaron Miller
Date: 10/20/2025
"""

from mavric_msg.msg import SteerTrain
from utils.SparkCANLib.SparkCAN import SparkBus
from typing import Optional

# CAN IDs for Drive Controllers
FLS = 7
FRS = 10
BLS = 9
BRS = 2

INVERTED = -1


class SteerControl:
    """
    Uses the CANbus interface to set the position of the steer motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """

    def __init__(self, bus: SparkBus):
        self.bus = bus
        # Initialize logger before any log calls and keep attribute name consistent

        self.FLMotor = self.bus.init_controller(FLS)
        self.FRMotor = self.bus.init_controller(FRS)
        self.BLMotor = self.bus.init_controller(BLS)
        self.BRMotor = self.bus.init_controller(BRS)

    def set_position(self, msg: SteerTrain):
        """
        Sets the positions of the motors based on the ROS values.

        Args:
            msg (SteerTrain): The values from ROS indicating the velocity of each motor.
        """

        self.FLMotor.position_output(msg.front_left)
        self.FRMotor.position_output(msg.front_right)
        self.BLMotor.position_output(msg.back_left)
        self.BRMotor.position_output(msg.back_right)
