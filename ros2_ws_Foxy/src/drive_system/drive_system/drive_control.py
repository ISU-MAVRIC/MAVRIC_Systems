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
from mavric_msg.msg import DriveTrain, CANCommand, CANCommandBatch, ScaleFeedback
from std_msgs.msg import Float64
from utils.command_filter import CommandDeduplicator
from utils.can_publisher import CANCommandPublisher

# CAN IDs for Drive Controllers
FLD = 1  # Front Left Drive
FRD = 6  # Front Right Drive
BLD = 5  # Back Left Drive
BRD = 3  # Back Right Drive

INVERTED = -1

c_Scale_Max = 1.2*20
c_Scale = c_Scale_Max

class DriveControlNode(Node):
    def __init__(self) -> None:
        super().__init__("drive_control")

        # Declare parameters
        self.declare_parameter("motor_ids", [FLD, FRD, BLD, BRD])
        self.declare_parameter("invert_motors", [FRD, BRD])
        self.declare_parameter("command_deadband", 0.01)  # Slightly larger for velocity commands

        # Get parameters
        self.motor_ids = self.get_parameter("motor_ids").value
        self.invert_motors = self.get_parameter("invert_motors").value
        command_deadband = self.get_parameter("command_deadband").value

        # Initialize command deduplicator
        deduplicator = CommandDeduplicator(deadband=command_deadband)

        # Create publisher for CAN commands
        pub_can_batch = self.create_publisher(
            CANCommandBatch, "can_commands_batch", 10
        )

        # Initialize CAN command publisher helper
        self.can_publisher = CANCommandPublisher(
            publisher=pub_can_batch,
            invert_motors=self.invert_motors,
            deduplicator=deduplicator,
            command_type=CANCommand.VELOCITY_OUTPUT,
        )

        # Create subscriber for drive train commands
        self.sub_drive_train = self.create_subscription(
            DriveTrain, "drive_train", self._drive_train_callback, 10
        )
        self.get_logger().info(
            f"DriveControl node initialized with motor IDs: {self.motor_ids}"
        )

        # Subscriber for updating drive scales
        self.sub_drive_scale = self.create_subscription(
            ScaleFeedback,
            "scale_command",
            self._set_scale,
            10,
        )

    def _drive_train_callback(self, msg: DriveTrain) -> None:
        motor_commands = [
            (self.motor_ids[0], msg.front_left * c_Scale),    # FLD
            (self.motor_ids[1], msg.front_right * c_Scale),   # FRD
            (self.motor_ids[2], msg.back_left * c_Scale),     # BLD
            (self.motor_ids[3], msg.back_right * c_Scale),    # BRD
        ]

        # Publish batch of commands via helper
        self.can_publisher.publish_batch(motor_commands)

    def _set_scale(self, msg: ScaleFeedback) -> None:
        global c_Scale, c_Scale
        new_scale = max(0.0, min(1.0, msg.drive))
        c_Scale = c_Scale_Max * new_scale


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
