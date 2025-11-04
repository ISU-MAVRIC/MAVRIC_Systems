from mavric_msg.msg import SteerTrain
from utils.SparkCANLib.SparkCAN import SparkBus
from typing import Optional

# CAN IDs for Drive Controllers
FLS = 7
FRS = 10
BLS = 9
BRS = 2

INVERTED = -1

c_str_Scale = 0.15
c_str_lfDir = 1
c_str_lbDir = 1
c_str_rfDir = 1
c_str_rbDir = 0.9


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

        self.FLMotor.percent_output(msg.steer_front_left * c_str_Scale * c_str_lfDir)
        self.FRMotor.percent_output(msg.steer_front_right * c_str_Scale * c_str_rfDir)
        self.BLMotor.percent_output(msg.steer_back_left * c_str_Scale * c_str_lbDir)
        self.BRMotor.percent_output(msg.steer_back_right * c_str_Scale * c_str_rbDir)
