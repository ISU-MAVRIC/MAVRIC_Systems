from cysar.msg import SteerTrain
from SparkCANLib.SparkCAN import SparkBus
from typing import Optional

# CAN IDs for Drive Controllers
FLS = 7
FRS = 10
BLS = 9
BRS = 8

INVERTED = -1


class SteerControl:
    """
    Uses the CANbus interface to set the velocity of the motors.

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

    def set_velocity(self, msg: SteerTrain):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (SteerTrain): The values from ROS indicating the velocity of each motor.
        """

        self.FLMotor.percent_output(msg.front_left)
        self.FRMotor.percent_output(msg.front_right)
        self.BLMotor.percent_output(INVERTED * msg.back_left)
        self.BRMotor.percent_output(INVERTED * msg.back_right)
