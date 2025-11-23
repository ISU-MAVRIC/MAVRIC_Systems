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
from mavric_msg.msg import (Arm, CANCommand, CANCommandBatch,
                            ScaleFeedback, ServoCommand)
from rclpy.node import Node
from utils.can_publisher import CANCommandPublisher
from utils.servo_publisher import ServoCommandPublisher

# CAN IDs for Arm Controllers
SHOULDER_PITCH = 11
SHOULDER_ROT = 12
ELBOW_PITCH = 13
WRIST_PITCH = 14
WRIST_ROT = 15
CLAW_SERVO_CHANNEL = 1

CLAW_START_ANGLE = 120.0
CLAW_MIN_ANGLE = 0.0
CLAW_MAX_ANGLE = 300.0
CLAW_STEP_SIZE = 0.5
CLAW_RELAX_AMOUNT = 4.0  # Amount to back off when releasing grip

INVERTED = -1

# Arm Scales
c_ShoulderPitch = 1         # Define individual arm rates
c_ShoulderRot = 1           # If one axis is faster/slower than the others, change these values
c_ElbowPitch = 1
c_WristPitch = 1
c_WristRot = 1

class ArmControlNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_control")

        # Declare parameters
        self.declare_parameter("can_motor_ids", [SHOULDER_PITCH, SHOULDER_ROT, ELBOW_PITCH, WRIST_PITCH, WRIST_ROT])
        self.declare_parameter("invert_motors", [WRIST_PITCH])
        self.declare_parameter("servo_channel", CLAW_SERVO_CHANNEL)
        self.declare_parameter("starting_claw_angle", CLAW_START_ANGLE)
        self.declare_parameter("claw_min_angle", CLAW_MIN_ANGLE)
        self.declare_parameter("claw_max_angle", CLAW_MAX_ANGLE)
        self.declare_parameter("claw_step_size", CLAW_STEP_SIZE)
        self.declare_parameter("relax_amount", CLAW_RELAX_AMOUNT)
        self.declare_parameter("command_deadband", 0.01)  # Threshold for duplicate detection

        # Get parameters
        self.can_motor_ids = self.get_parameter("can_motor_ids").value
        self.invert_motors = self.get_parameter("invert_motors").value
        self.servo_channel = self.get_parameter("servo_channel").value
        self.current_claw_angle = self.get_parameter("starting_claw_angle").value
        self.claw_min_angle = self.get_parameter("claw_min_angle").value
        self.claw_max_angle = self.get_parameter("claw_max_angle").value
        self.claw_step_size = self.get_parameter("claw_step_size").value
        self.relax_amount = self.get_parameter("relax_amount").value
        command_deadband = self.get_parameter("command_deadband").value

        # Track claw state
        self.was_closing = False  # Track if we were actively closing the claw
        self.closing_time = None  # Track when we stopped closing
        self.has_relaxed = False  # Track if we've already applied the relax amount

        # Create publisher for CAN commands
        pub_can_batch = self.create_publisher(
            CANCommandBatch, "can_commands_batch", 10
        )
        # Initialize CAN command publisher helper
        self.can_publisher = CANCommandPublisher(
            publisher=pub_can_batch,
            invert_motors=self.invert_motors,
            deadband=command_deadband,
            command_type=CANCommand.PERCENT_OUTPUT,
        )

        # Create Publisher for Servo commands
        self.pub_servo_command = self.create_publisher(
            ServoCommand, "servo_commands", 10
        )
        # Initialize Servo command publisher helper
        self.servo_publisher = ServoCommandPublisher(
            publisher=self.pub_servo_command,
            deadband=command_deadband,
            default_servo_type=ServoCommand.CONTINUOUS_SERVO,
        )

        self.sub_arm_scales = self.create_subscription(
            ScaleFeedback,
            "scale_feedback",
            self._set_scale,
            10,
        )

        # Create subscriber for arm commands
        self.sub_arm = self.create_subscription(
            Arm, "arm_control", self._arm_callback, 10
        )
        self.get_logger().info(
            f"ArmControl node initialized with CAN motor IDs: {self.can_motor_ids}"
        )


    def _arm_callback(self, msg: Arm) -> None:
        can_motor_commands = [
            (self.can_motor_ids[0], msg.shoulder_pitch * c_ShoulderPitch / 100.0),   # SHOULDER_PITCH
            (self.can_motor_ids[1], msg.shoulder_rot * c_ShoulderRot / 100.0),         # SHOULDER_ROT
            (self.can_motor_ids[2], msg.elbow_pitch * c_ElbowPitch / 100.0),            # ELBOW_PITCH
            (self.can_motor_ids[3], msg.wrist_pitch * c_WristPitch / 100.0),            # WRIST_PITCH
            (self.can_motor_ids[4], msg.wrist_rot * c_WristRot / 100.0),                  # WRIST_ROT
        ]

        # Publish batch of commands via helper
        self.can_publisher.publish_batch(can_motor_commands, CANCommand.PERCENT_OUTPUT)
        
        # Claw Control
        self.update_claw(msg.claw)

        # Step 3: Publish the ANGLE, not the THROTTLE
        self.servo_publisher.publish_single(
            channel=self.servo_channel,
            value=self.current_claw_angle,
            servo_type=ServoCommand.STANDARD_SERVO,
        )
        # self.get_logger().info(f"Claw at angle: {self.current_claw_angle} degrees")

    def update_claw(self, claw_direction: float):
        # 1. IF CLOSING (Gripping)
        if claw_direction < 0:
            self.was_closing = True  # Remember we are actively gripping
            self.closing_time = None  # Reset the timer
            self.has_relaxed = False  # Reset the relax flag
            self.current_claw_angle -= self.claw_step_size
            
            # Clamp Limit
            if self.current_claw_angle < self.claw_min_angle:
                self.current_claw_angle = self.claw_min_angle

        # 2. IF OPENING (Releasing)
        elif claw_direction > 0:
            self.was_closing = False # Reset the grip tracker
            self.closing_time = None  # Reset the timer
            self.has_relaxed = False  # Reset the relax flag
            self.current_claw_angle += self.claw_step_size
            
            # Clamp Limit
            if self.current_claw_angle > self.claw_max_angle:
                self.current_claw_angle = self.claw_max_angle

        # 3. BUTTON RELEASED (Holding)
        else:
            # This is the "Active Relief" Logic with 2-second delay
            if self.was_closing:
                # User just let go of the Close button.
                # Start the timer if not already started
                if self.closing_time is None:
                    self.closing_time = self.get_clock().now()
                
                # Check if 2 seconds have passed
                elapsed_time = (self.get_clock().now() - self.closing_time).nanoseconds / 1e9
                if elapsed_time >= 1.0 and not self.has_relaxed:
                    # Back off slightly to stop the stall buzzing after 2 seconds
                    self.current_claw_angle += self.relax_amount
                    self.has_relaxed = True  # Mark that we've relaxed
                    self.was_closing = False  # Turn off the flag
                
            # If we weren't just closing, do nothing. 
    
    def _set_scale(self, msg: ScaleFeedback) -> None:
        global c_ShoulderPitch, c_ShoulderRot, c_ElbowPitch, c_WristPitch, c_WristRot
        c_ShoulderPitch = msg.shoulder_pitch
        c_ShoulderRot = msg.shoulder_rot
        c_ElbowPitch = msg.elbow_pitch
        c_WristPitch = msg.wrist_pitch
        c_WristRot = msg.wrist_rot  


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
