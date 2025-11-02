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
from mavric_msg.msg import SteerTrain, CANCommand

# CAN IDs for Steer Controllers
FLS = 7   # Front Left Steer
FRS = 10  # Front Right Steer
BLS = 9   # Back Left Steer
BRS = 2   # Back Right Steer

INVERTED = -1


class SteerControlNode(Node):
    """
    ROS2 node for steer subsystem control.

    Subscribes to high-level SteerTrain messages and publishes
    low-level CANCommand messages to the CAN manager.
    """

    def __init__(self) -> None:
        super().__init__("steer_control")

        # Declare parameters
        self.declare_parameter("motor_ids", [FLS, FRS, BLS, BRS])
        self.declare_parameter("invert_motors", [BLS, BRS])

        # Get parameters
        self.motor_ids = self.get_parameter("motor_ids").value
        self.invert_motors = self.get_parameter("invert_motors").value

        # Create publisher for CAN commands
        self.pub_can_commands = self.create_publisher(
            CANCommand, "can_commands", 10
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
        # Define motor-to-value mapping
        motor_commands = [
            (self.motor_ids[0], msg.front_left),    # FLS
            (self.motor_ids[1], msg.front_right),   # FRS
            (self.motor_ids[2], msg.back_left),     # BLS
            (self.motor_ids[3], msg.back_right),    # BRS
        ]

        # Create and publish CANCommand for each motor
        for motor_id, value in motor_commands:
            # Apply inversion if configured
            if motor_id in self.invert_motors:
                value = value * INVERTED

            cmd = CANCommand()
            cmd.command_type = CANCommand.POSITION_OUTPUT
            cmd.controller_id = motor_id
            cmd.value = value
            cmd.timestamp = self.get_clock().now().to_msg()

            self.pub_can_commands.publish(cmd)


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
