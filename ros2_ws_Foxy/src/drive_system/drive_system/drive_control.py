#!/usr/bin/python3

"""
drive_control.py

Desc: Uses the CANbus interface to set the velocity of the motors.
Author: Isaac Denning
Date: 10/21/23
"""
import rclpy
from rclpy.node import Node
from mavric_msg.msg import DriveTrain
from utils.SparkCANLib.SparkCAN import SparkBusManager

# CAN IDs for Drive Controllers
FLD = 1
FRD = 6
BLD = 5
BRD = 3

INVERTED = -1


class DriveControl(Node):
    """
    Uses the CANbus interface to set the velocity of the motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """

    def __init__(self) -> None:
        super().__init__("drive_control")
        
        self.bus = SparkBusManager.get_instance()
        self.FLMotor = self.bus.init_controller(FLD)
        self.FRMotor = self.bus.init_controller(FRD)
        self.BLMotor = self.bus.init_controller(BLD)
        self.BRMotor = self.bus.init_controller(BRD)
        
        # self.drive_train_subscription = self.create_subscription(
        #     DriveTrain, "drive_train", self.set_velocity, 10
        # )

    def set_velocity(self, msg: DriveTrain):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (DriveTrain): The values from ROS indicating the velocity of each motor.
        """
        self.FLMotor.velocity_output(msg.front_left)
        self.FRMotor.velocity_output(INVERTED * msg.front_right)
        self.BLMotor.velocity_output(msg.back_left)
        self.BRMotor.velocity_output(INVERTED * msg.back_right)

# def main(args=None):
#     rclpy.init(args=args)

#     # Create the node
#     drive_control = DriveControl()

#     # Run the node
#     rclpy.spin(drive_control)

#     # Destroy it when done
#     drive_control.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
