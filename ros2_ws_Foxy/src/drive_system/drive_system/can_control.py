#!/usr/bin/python3

"""
can_control.py

Desc: Sets up the can bus and calls the controllers for the corresponding parts
        when (drive, flipper, arm) when ROS data is received.
Author: Isaac Denning
Date: 10/21/23
"""

import rclpy
from rclpy.node import Node
from mavric_msg.msg import DriveTrain, SteerTrain, Arm, ArmScales
from std_msgs.msg import Float64
from utils.SparkCANLib import SparkController, SparkCAN
from drive_system.drive_control import DriveControl
from drive_system.steer_control import SteerControl
from drive_system.arm_control import ArmControl
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy



class CanControl(Node):
    """
    Sets up the can bus and calls the controllers for the corresponding parts
        when (drive, flippper, arm) when ROS data is received.
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

        self.arm_control = ArmControl(self.bus)
        self.arm_subscription = self.create_subscription(
            Arm, "arm_control", self.arm_listener, 10
        )


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.drive_scale_sub = self.create_subscription(
            Float64, "drive_scale", self.drive_scale_listener, qos_profile=qos_profile
        )

        self.arm_scale_sub = self.create_subscription(
            ArmScales, "arm_scales", self.arm_scale_listener, qos_profile=qos_profile
        )

    def drive_scale_listener(self, msg) -> None:
        self.drive_control.set_scale(msg.data)
    
    def arm_scale_listener(self, msg) -> None:
        self.arm_control.set_scale(msg)

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

    def arm_listener(self, msg: Arm) -> None:
        """
        Called whenever new arm control data is received from ROS.
        """
        self.arm_control.set_velocity(msg)


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
