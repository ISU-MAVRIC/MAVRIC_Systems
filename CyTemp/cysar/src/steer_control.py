from cysar.msg import SteerTrain
from SparkCANLib.SparkCAN import SparkBus
import rclpy
import rclpy.logging
from typing import Optional

# CAN IDs for Drive Controllers
FLS = 7
FRS = 10
BLS = 9
BRS = 8

INVERTED = -1

class SteerControl():
    """
    Uses the CANbus interface to set the velocity of the motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """
    def __init__(self, bus: SparkBus):
        self.bus = bus
        # Initialize logger before any log calls and keep attribute name consistent
        self.logger = rclpy.logging.get_logger('cysar.steer_control')

        self.FLMotor = self.bus.init_controller(FLS)
        self.logger.info(f"Initialized Front Left Steer (FLS) controller with CAN ID {FLS}")

        self.FRMotor = self.bus.init_controller(FRS)
        self.logger.info(f"Initialized Front Right Steer (FRS) controller with CAN ID {FRS}")

        self.BLMotor = self.bus.init_controller(BLS)
        self.logger.info(f"Initialized Back Left Steer (BLS) controller with CAN ID {BLS}")

        self.BRMotor = self.bus.init_controller(BRS)
        self.logger.info(f"Initialized Back Right Steer (BRS) controller with CAN ID {BRS}")

    def set_velocity(self, msg : SteerTrain):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (SteerTrain): The values from ROS indicating the velocity of each motor.
        """
        self.logger.info(f"Setting Front Left Motor velocity to {msg.front_left:.3f}")
        self.FLMotor.percent_output(msg.front_left)

        self.logger.info(f"Setting Front Right Motor velocity to {msg.front_right:.3f}")
        self.FRMotor.percent_output(msg.front_right)

        self.logger.info(
            f"Setting Back Left Motor velocity to {msg.back_left:.3f} (inverted: {(INVERTED * msg.back_left):.3f})"
        )
        self.BLMotor.percent_output(INVERTED * msg.back_left)

        self.logger.info(
            f"Setting Back Right Motor velocity to {msg.back_right:.3f} (inverted: {(INVERTED * msg.back_right):.3f})"
        )
        self.BRMotor.percent_output(INVERTED * msg.back_right)
