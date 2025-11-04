#!/usr/bin/python3

"""
drive_control.py

Desc: ROS2 node for drive subsystem control. Converts high-level DriveTrain 
      commands to low-level CAN motor commands.
Author: MAVRIC Team
Date: 2025-11-02
"""

import rclpy
from rclpy.node import Node
from mavric_msg.msg import DriveTrain, CANCommand

# CAN IDs for Drive Controllers
FLD = 1  # Front Left Drive
FRD = 6  # Front Right Drive
BLD = 5  # Back Left Drive
BRD = 3  # Back Right Drive

INVERTED = -1

c_Scale_Max = 1.2*20
c_Scale = c_Scale_Max

class DriveControlNode(Node):
    """
    ROS2 node for drive subsystem control.
    
    Subscribes to high-level DriveTrain messages and publishes
    low-level CANCommand messages to the CAN manager.
    """

    def __init__(self) -> None:
        super().__init__("drive_control")

        # Declare parameters
        self.declare_parameter("motor_ids", [FLD, FRD, BLD, BRD])
        self.declare_parameter("invert_motors", [FRD, BRD])

        # Get parameters
        self.motor_ids = self.get_parameter("motor_ids").value
        self.invert_motors = self.get_parameter("invert_motors").value

        # Create publisher for CAN commands
        self.pub_can_commands = self.create_publisher(
            CANCommand, "can_commands", 10
        )

        # Create subscriber for drive train commands
        self.sub_drive_train = self.create_subscription(
            DriveTrain, "drive_train", self.drive_train_callback, 10
        )

        self.get_logger().info(
            f"DriveControl node initialized with motor IDs: {self.motor_ids}"
        )

    def drive_train_callback(self, msg: DriveTrain) -> None:
        """
        Process high-level drive train commands and publish CANCommand messages.

        Args:
            msg (DriveTrain): High-level drive command with 4 motor values
        """
        # Define motor-to-value mapping
        motor_commands = [
            (self.motor_ids[0], msg.front_left),    # FLD
            (self.motor_ids[1], msg.front_right),   # FRD
            (self.motor_ids[2], msg.back_left),     # BLD
            (self.motor_ids[3], msg.back_right),    # BRD
        ]

        # Create and publish CANCommand for each motor
        for motor_id, value in motor_commands:
            # Apply inversion if configured
            if motor_id in self.invert_motors:
                value = value * INVERTED

            cmd = CANCommand(
                command_type = CANCommand.VELOCITY_OUTPUT,
                controller_id = motor_id,
                value = value * c_Scale
            )

            self.pub_can_commands.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    drive_control = DriveControlNode()

    try:
        rclpy.spin(drive_control)
    except KeyboardInterrupt:
        pass
    finally:
        drive_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
