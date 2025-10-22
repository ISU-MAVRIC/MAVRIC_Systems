#!/usr/bin/python3

"""
can_control.py

Desc: Sets up the can bus and subscribers (with listener functions) for the drive system (drive_train, steer_train).
Author: Andrew Boun, Aaron Miller
Date: 10/20/2025
"""

import rclpy
from rclpy.node import Node
from mavric_msg.msg import DriveTrain
from utils.SparkCANLib import SparkController, SparkCAN
from drive_system.drive_control import DriveControl
from mavric_msg.msg import SteerTrain
from drive_system.steer_control import SteerControl


class CanControl(Node):
    """
    Sets up the can bus and subscribers (with listener functions) for the drive system (drive_train, steer_train).
    """

    def __init__(self) -> None:
        super().__init__("can_control")
        self.bus = SparkCAN.SparkBus(
            channel="can0", bustype="socketcan", bitrate=1000000
        )

        # Drive Control can bus
        self.drive_control = DriveControl(self.bus)
        self.drive_train_subscription = self.create_subscription(
            DriveTrain, "drive_train", self.drive_listener, 10
        )

        # Steer Control can bus
        self.steer_control = SteerControl(self.bus)
        self.steer_train_subscription = self.create_subscription(
            SteerTrain, "steer_train", self.steer_listener, 10
        )

    def drive_listener(self, msg: DriveTrain) -> None:
        """
        Called whenever new drive train data is received from ROS.
        """
        self.drive_control.set_velocity(msg)

    def steer_listener(self, msg: SteerTrain) -> None:
        """
        Called whenever new steer train data is received from ROS.
        """
        self.steer_control.set_velocity(msg)


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    can_control = CanControl()

    # Run the node
    rclpy.spin(can_control)

    # Destroy it when done
    can_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
