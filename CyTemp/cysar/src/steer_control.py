from cysar.msg import SteerTrain
from SparkCANLib.SparkCAN import SparkBus

# CAN IDs for Drive Controllers
FLD = 7
FRD = 10
BLD = 9
BRD = 8

INVERTED = -1

class SteerControl():
    """
    Uses the CANbus interface to set the velocity of the motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """
    def __init__(self, bus : SparkBus):
        self.bus = bus
        self.FLMotor = self.bus.init_controller(FLD)
        self.FRMotor = self.bus.init_controller(FRD)
        self.BLMotor = self.bus.init_controller(BLD)
        self.BRMotor = self.bus.init_controller(BRD)

    def set_velocity(self, msg : SteerTrain):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (DriveTrain): The values from ROS indicating the velocity of each motor.
        """
        self.FLMotor.percent_output(msg.front_left)
        self.FRMotor.percent_output(msg.front_right)
        self.BLMotor.percent_output(INVERTED * msg.back_left)
        self.BRMotor.percent_output(INVERTED * msg.back_right)
