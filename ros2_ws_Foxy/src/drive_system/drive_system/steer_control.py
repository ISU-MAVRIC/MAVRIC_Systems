#!/usr/bin/python3

"""
steer_control.py

Desc: ROS2 node for steer subsystem control. Converts high-level SteerTrain
      commands to low-level CAN motor commands.
Author: MAVRIC Team
Date: 2025-11-02
"""

import rclpy
from rclpy.node import Node
from mavric_msg.msg import SteerTrain, CANCommand, CANCommandBatch
from utils.command_filter import CommandDeduplicator
from utils.can_publisher import CANCommandPublisher


# CAN IDs for Steer Controllers
FLS = 7   # Front Left Steer
FRS = 10  # Front Right Steer
BLS = 9   # Back Left Steer
BRS = 2   # Back Right Steer

INVERTED = -1

c_str_Scale = 0.15
c_str_lfDir = 1
c_str_lbDir = 1
c_str_rfDir = 1
c_str_rbDir = 0.9

class SteerControlNode(Node):
    """
    ROS2 node for steer subsystem control.

    Subscribes to high-level SteerTrain messages and publishes
    low-level CANCommand messages to the CAN manager.
    Uses batched commands for improved synchronization.
    """

    def __init__(self) -> None:
        super().__init__("steer_control")

        # Declare parameters
        self.declare_parameter("motor_ids", [FLS, FRS, BLS, BRS])
        self.declare_parameter("command_deadband", 0.01) 

        # Get parameters
        self.motor_ids = self.get_parameter("motor_ids").value
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
            invert_motors=[],  # No motor inversions for steer
            deduplicator=deduplicator,
            command_type=CANCommand.POSITION_OUTPUT,
        )

        # Create subscriber for steer train commands
        self.sub_steer_train = self.create_subscription(
            SteerTrain, "steer_train", self.steer_train_callback, 10
        )

        self.get_logger().info(
            f"SteerControl node initialized with motor IDs: {self.motor_ids}"
        )

    def steer_train_callback(self, msg: SteerTrain) -> None:
        """
        Process high-level steer train commands and publish CANCommand messages.

        Args:
            msg (SteerTrain): High-level steer command with 4 motor values
        """
        # Define motor-to-value mapping with directional scaling
        motor_commands = [
            (self.motor_ids[0], msg.steer_front_left * c_str_lfDir * c_str_Scale),    # FLS
            (self.motor_ids[1], msg.steer_front_right * c_str_rfDir * c_str_Scale),   # FRS
            (self.motor_ids[2], msg.steer_back_left * c_str_lbDir * c_str_Scale),     # BLS
            (self.motor_ids[3], msg.steer_back_right * c_str_rbDir * c_str_Scale),    # BRS
        ]

        # Publish batch of commands via helper
        self.can_publisher.publish_batch(motor_commands)


def main(args=None):
    rclpy.init(args=args)
    steer_control = SteerControlNode()

    try:
        rclpy.spin(steer_control)
    except KeyboardInterrupt:
        pass
    finally:
        steer_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
