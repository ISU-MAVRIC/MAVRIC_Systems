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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from mavric_msg.msg import CANCommand, CANStatus, CANCommandBatch
from utils.SparkCANLib import SparkController, SparkCAN
from typing import Dict, Optional


class CANManager(Node):
    """
    Singleton CAN bus manager node.

    Owns the single SparkBus instance and coordinates all CAN communication.
    - Subscribes to CANCommand messages from control nodes
    - Subscribes to CANCommandBatch messages for synchronized multi-motor commands
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

        # Controller dictionary
        self.controllers: Dict[int, SparkController.Controller] = {}

        # Use reentrant callback group for parallel processing
        self.callback_group = ReentrantCallbackGroup()

        # Create subscriber for batched CAN commands (optimized path)
        self.sub_can_batch = self.create_subscription(
            CANCommandBatch,
            "can_commands_batch",
            self.can_batch_callback,
            10,
            callback_group=self.callback_group
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

        return controller

    def can_batch_callback(self, msg: CANCommandBatch) -> None:
        """Process batched CAN commands (optimized path) - all commands executed in tight loop"""
        for cmd in msg.commands:
            self._execute_command(cmd)

    def _execute_command(self, msg: CANCommand) -> None:
        """Execute a single CAN command"""
        controller = self.get_or_init_controller(msg.controller_id)

        if msg.command_type == CANCommand.PERCENT_OUTPUT:
            controller.percent_output(msg.value)

        elif msg.command_type == CANCommand.VELOCITY_OUTPUT:
            controller.velocity_output(msg.value)

        elif msg.command_type == CANCommand.POSITION_OUTPUT:
            controller.position_output(msg.value)

    def status_publish_timer_callback(self) -> None:
        for controller_id in self.controllers.keys():
            controller = self.controllers[controller_id]

            # Create and publish status message
            status_msg = CANStatus(
                controller_id=controller_id,
                position=float(controller.position),
                velocity=float(controller.velocity)
            )
            self.pub_can_status.publish(status_msg)

    def get_motor_position(self, controller_id: int) -> float:
        return self.controllers[controller_id].position

    def get_motor_velocity(self, controller_id: int) -> float:
        return self.controllers[controller_id].velocity


def main(args=None):
    rclpy.init(args=args)

    # Create and run the node with multi-threaded executor for better performance
    can_manager = CANManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(can_manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        can_manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
