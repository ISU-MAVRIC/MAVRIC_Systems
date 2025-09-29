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
        logger (Optional[rclpy.logging.Logger]): ROS 2 logger to use. If not provided,
            a module-level logger named 'cysar.steer_control' will be created.
    """
    def __init__(self, bus: SparkBus):
        self.bus = bus
        # self._logger = logger or rclpy.logging.get_logger('cysar.steer_control')

        self.FLMotor = self.bus.init_controller(FLS)
        # self._logger.info(f"Initialized Front Left Steer (FLS) controller with CAN ID {FLS}")

        self.FRMotor = self.bus.init_controller(FRS)
        # self._logger.info(f"Initialized Front Right Steer (FRS) controller with CAN ID {FRS}")

        self.BLMotor = self.bus.init_controller(BLS)
        # self._logger.info(f"Initialized Back Left Steer (BLS) controller with CAN ID {BLS}")

        self.BRMotor = self.bus.init_controller(BRS)
        # self._logger.info(f"Initialized Back Right Steer (BRS) controller with CAN ID {BRS}")

    def set_velocity(self, msg : SteerTrain):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (SteerTrain): The values from ROS indicating the velocity of each motor.
        """
        # self._logger.info(f"Setting Front Left Motor velocity to {msg.front_left:.3f}")
        self.FLMotor.percent_output(msg.front_left)
        # self._logger.info(f"Setting Front Right Motor velocity to {msg.front_right:.3f}")
        self.FRMotor.percent_output(msg.front_right)
        # self._logger.info(
        #     f"Setting Back Left Motor velocity to {msg.back_left:.3f} (inverted: {(INVERTED * msg.back_left):.3f})"
        #)
        self.BLMotor.percent_output(INVERTED * msg.back_left)
        # self._logger.info(
        #     f"Setting Back Right Motor velocity to {msg.back_right:.3f} (inverted: {(INVERTED * msg.back_right):.3f})"
        # )
        self.BRMotor.percent_output(INVERTED * msg.back_right)
