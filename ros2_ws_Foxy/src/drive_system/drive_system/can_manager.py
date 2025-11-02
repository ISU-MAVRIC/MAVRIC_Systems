#!/usr/bin/python3

"""
can_manager.py

Desc: Singleton CAN bus manager node. Handles all CAN communication and maintains
      motor state. Other nodes send commands and receive status via ROS messages.
Author: MAVRIC Team
Date: 2025-11-02
"""

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from mavric_msg.msg import CANCommand, CANStatus
from utils.SparkCANLib import SparkController, SparkCAN
from typing import Dict, Optional
import traceback


class MotorState:
    """Tracks state for a single motor controller"""

    def __init__(self, controller_id: int):
        self.controller_id = controller_id
        self.position = 0.0
        self.velocity = 0.0


class CANManager(Node):
    """
    Singleton CAN bus manager node.

    Owns the single SparkBus instance and coordinates all CAN communication.
    - Subscribes to CANCommand messages from control nodes
    - Publishes CANStatus messages with motor state
    - Maintains motor state dictionary
    """

    def __init__(self) -> None:
        super().__init__("can_manager")

        # Declare parameters
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_bustype", "socketcan")
        self.declare_parameter("can_bitrate", 1000000)
        self.declare_parameter("status_publish_rate", 50)  # Hz

        # Get parameters
        can_channel = self.get_parameter("can_channel").value
        can_bustype = self.get_parameter("can_bustype").value
        can_bitrate = self.get_parameter("can_bitrate").value
        status_rate = self.get_parameter("status_publish_rate").value

        # Initialize SparkBus (singleton instance)
        self.get_logger().info(
            f"Initializing CAN bus: channel={can_channel}, bustype={can_bustype}, bitrate={can_bitrate}"
        )
        self.bus = SparkCAN.SparkBus(
            channel=can_channel, bustype=can_bustype, bitrate=can_bitrate
        )

        # Motor state tracking
        self.motor_states: Dict[int, MotorState] = {}
        self.controllers: Dict[int, SparkController.Controller] = {}

        # Create subscriber for CAN commands
        self.sub_can_commands = self.create_subscription(
            CANCommand, "can_commands", self.can_command_callback, 10
        )

        # Create publisher for CAN status
        self.pub_can_status = self.create_publisher(CANStatus, "can_status", 10)

        # Create timer for status publishing
        status_interval = 1.0 / status_rate
        self.status_timer: Timer = self.create_timer(
            status_interval, self.status_publish_timer_callback
        )

        self.get_logger().info(
            f"CAN Manager initialized. Status publishing at {status_rate}Hz"
        )


    def get_or_init_controller(self, controller_id: int) -> Optional[SparkController.Controller]:
        if controller_id in self.controllers:
            return self.controllers[controller_id]

        controller = self.bus.init_controller(controller_id)
        self.controllers[controller_id] = controller
        self.motor_states[controller_id] = MotorState(controller_id)

        return controller


    def can_command_callback(self, msg: CANCommand) -> None:
        controller = self.get_or_init_controller(msg.controller_id)
        state = self.motor_states[msg.controller_id]

        if msg.command_type == CANCommand.PERCENT_OUTPUT:
            controller.percent_output(msg.value)

        elif msg.command_type == CANCommand.VELOCITY_OUTPUT:
            controller.velocity_output(msg.value)

        elif msg.command_type == CANCommand.POSITION_OUTPUT:
            controller.position_output(msg.value)


    def status_publish_timer_callback(self) -> None:
        for controller_id, state in self.motor_states.items():
            controller = self.controllers[controller_id]

            # Read status from controller
            position = controller.position
            velocity = controller.velocity

            state.position = position
            state.velocity = velocity

            # Create and publish status message
            status_msg = CANStatus(
                controller_id=controller_id,
                position=float(state.position),
                velocity=float(state.velocity)
            )
            self.pub_can_status.publish(status_msg)

    def get_motor_position(self, controller_id: int) -> Optional[float]:
        return self.controllers[controller_id].position

    def get_motor_velocity(self, controller_id: int) -> Optional[float]:
        return self.controllers[controller_id].velocity

    def get_motor_last_percent_output(self, controller_id: int) -> Optional[float]:
        return self.motor_states[controller_id].last_percent_output

    def get_all_motor_states(self) -> Dict[int, MotorState]:
        return self.motor_states.copy()


def main(args=None):
    rclpy.init(args=args)

    # Create and run the node
    can_manager = CANManager()

    try:
        rclpy.spin(can_manager)
    except KeyboardInterrupt:
        pass
    finally:
        can_manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
