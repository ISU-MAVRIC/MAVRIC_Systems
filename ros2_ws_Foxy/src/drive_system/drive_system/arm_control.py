#!/usr/bin/python3

"""
arm_control.py

Desc: ROS2 node for arm subsystem control. Converts high-level Arm commands
      to low-level CAN motor commands (for CAN motors) and PWM servo commands
      (for claw).
Author: MAVRIC Team
Date: 2025-11-02
"""

import rclpy
from rclpy.node import Node
from mavric_msg.msg import Arm, CANCommand

# CAN IDs for Arm Controllers
SHOULDER_PITCH = 11
SHOULDER_ROT = 12
ELBOW_PITCH = 13
WRIST_PITCH = 14
WRIST_ROT = 15
CLAW_SERVO_CHANNEL = 1

INVERTED = -1

# Arm Scales
c_ShoulderPitch = 1         # Define individual arm rates
c_ShoulderRot = 1           # If one axis is faster/slower than the others, change these values
c_ElbowPitch = 1
c_WristPitch = 1
c_WristRot = 1

class ArmControlNode(Node):
    """
    ROS2 node for arm subsystem control.

    Subscribes to high-level Arm messages and publishes low-level CANCommand
    messages to the CAN manager for CAN motors. Directly controls PWM servo
    for claw.
    """

    def __init__(self) -> None:
        super().__init__("arm_control")

        # Declare parameters
        self.declare_parameter("can_motor_ids", [SHOULDER_PITCH, SHOULDER_ROT, ELBOW_PITCH, WRIST_PITCH, WRIST_ROT])
        self.declare_parameter("invert_motors", [WRIST_ROT])
        self.declare_parameter("servo_channel", CLAW_SERVO_CHANNEL)

        # Get parameters
        self.can_motor_ids = self.get_parameter("can_motor_ids").value
        self.invert_motors = self.get_parameter("invert_motors").value
        servo_channel = self.get_parameter("servo_channel").value

        # Initialize ServoKit for PWM claw control
        try:
            from adafruit_servokit import ServoKit
            self.kit = ServoKit(channels=16)
            self.servo_channel = servo_channel
            self.get_logger().info(f"ServoKit initialized for claw on channel {servo_channel}")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize ServoKit: {e}. Claw will not work.")
            self.kit = None

        # Create publisher for CAN commands
        self.pub_can_commands = self.create_publisher(
            CANCommand, "can_commands", 10
        )

        # Create subscriber for arm commands
        self.sub_arm = self.create_subscription(
            Arm, "arm_control", self.arm_callback, 10
        )

        self.get_logger().info(
            f"ArmControl node initialized with CAN motor IDs: {self.can_motor_ids}"
        )

    def arm_callback(self, msg: Arm) -> None:
        """
        Process high-level arm commands and publish CANCommand messages.

        Args:
            msg (Arm): High-level arm command with 5 CAN motors + 1 PWM servo
        """
        # Define CAN motor-to-value mapping
        can_motor_commands = [
            (self.can_motor_ids[0], msg.shoulder_pitch * c_ShoulderPitch),   # SHOULDER_PITCH
            (self.can_motor_ids[1], msg.shoulder_rot * c_ShoulderRot),         # SHOULDER_ROT
            (self.can_motor_ids[2], msg.elbow_pitch * c_ElbowPitch),            # ELBOW_PITCH
            (self.can_motor_ids[3], msg.wrist_pitch * c_WristPitch),            # WRIST_PITCH
            (self.can_motor_ids[4], msg.wrist_rot * c_WristRot),                  # WRIST_ROT
        ]

        # Create and publish CANCommand for each CAN motor
        for motor_id, value in can_motor_commands:
            # Apply inversion if configured
            if motor_id in self.invert_motors:
                value = value * INVERTED

            cmd = CANCommand(
                command_type = CANCommand.PERCENT_OUTPUT,
                controller_id = motor_id,
                value = value/100.0
            )

            self.pub_can_commands.publish(cmd)

        # Handle PWM servo for claw
        if self.kit is not None:
            try:
                self.kit.continuous_servo[self.servo_channel].throttle = msg.claw
            except Exception as e:
                self.get_logger().warn(f"Failed to control claw servo: {e}")


def main(args=None):
    rclpy.init(args=args)
    arm_control = ArmControlNode()

    try:
        rclpy.spin(arm_control)
    except KeyboardInterrupt:
        pass
    finally:
        arm_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
