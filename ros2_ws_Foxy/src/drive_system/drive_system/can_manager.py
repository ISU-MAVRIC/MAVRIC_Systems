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
        self.last_percent_output = 0.0
        self.last_velocity_output = 0.0
        self.last_position_output = 0.0
        self.position = 0.0
        self.velocity = 0.0
        self.current = 0.0
        self.voltage = 0.0
        self.last_update = None


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
        """
        Get existing controller or initialize new one on-demand.

        Args:
            controller_id: CAN ID of controller (0-15)

        Returns:
            Controller object or None if initialization fails
        """
        if controller_id in self.controllers:
            return self.controllers[controller_id]

        try:
            # Initialize controller via SparkBus
            controller = self.bus.init_controller(controller_id)
            self.controllers[controller_id] = controller
            self.motor_states[controller_id] = MotorState(controller_id)

            self.get_logger().info(f"Initialized controller {controller_id}")
            return controller
        except Exception as e:
            self.get_logger().error(
                f"Failed to initialize controller {controller_id}: {e}"
            )
            traceback.print_exc()
            return None

    def can_command_callback(self, msg: CANCommand) -> None:
        """
        Handle incoming CAN commands from control nodes.

        Args:
            msg: CANCommand message
        """
        try:
            controller = self.get_or_init_controller(msg.controller_id)
            if controller is None:
                self.get_logger().warn(
                    f"Command ignored: controller {msg.controller_id} unavailable"
                )
                return

            state = self.motor_states[msg.controller_id]

            if msg.command_type == CANCommand.PERCENT_OUTPUT:
                controller.percent_output(msg.value)
                state.last_percent_output = msg.value
                self.get_logger().debug(
                    f"Controller {msg.controller_id}: percent_output({msg.value})"
                )

            elif msg.command_type == CANCommand.VELOCITY_OUTPUT:
                controller.velocity_output(msg.value)
                state.last_velocity_output = msg.value
                self.get_logger().debug(
                    f"Controller {msg.controller_id}: velocity_output({msg.value})"
                )

            elif msg.command_type == CANCommand.POSITION_OUTPUT:
                controller.position_output(msg.value)
                state.last_position_output = msg.value
                self.get_logger().debug(
                    f"Controller {msg.controller_id}: position_output({msg.value})"
                )

            elif msg.command_type == CANCommand.REQUEST_STATUS:
                self.get_logger().debug(
                    f"Controller {msg.controller_id}: status requested"
                )
                # Status will be published in next timer callback

            else:
                self.get_logger().warn(
                    f"Unknown command type: {msg.command_type} for controller {msg.controller_id}"
                )

        except Exception as e:
            self.get_logger().error(f"Error processing CAN command: {e}")
            traceback.print_exc()

    def status_publish_timer_callback(self) -> None:
        """
        Periodic callback to publish motor status for all initialized controllers.
        """
        try:
            for controller_id, state in self.motor_states.items():
                controller = self.controllers[controller_id]

                # Read status from controller
                try:
                    position = controller.position
                    velocity = controller.velocity

                    # Extract current and voltage from status 0x61
                    current = 0.0
                    voltage = 0.0
                    if controller.statuses[0x61] is not None:
                        status_data = controller.statuses[0x61].data
                        if len(status_data) > 1:
                            current = status_data[1]
                        if len(status_data) > 2:
                            voltage = status_data[2]

                    state.position = position
                    state.velocity = velocity
                    state.current = current
                    state.voltage = voltage
                    state.last_update = self.get_clock().now()

                except Exception as e:
                    self.get_logger().warn(
                        f"Failed to read status from controller {controller_id}: {e}"
                    )
                    continue

                # Create and publish status message
                status_msg = CANStatus(
                    controller_id=controller_id,
                    position=state.position,
                    velocity=state.velocity,
                    current=state.current,
                    voltage=state.voltage,
                    last_percent_output=state.last_percent_output,
                    timestamp=state.last_update.to_msg(),
                )

                self.pub_can_status.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error in status publish timer: {e}")
            traceback.print_exc()

    def get_motor_position(self, controller_id: int) -> Optional[float]:
        """Get current position of a motor."""
        if controller_id in self.controllers:
            try:
                return self.controllers[controller_id].position
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to get position for controller {controller_id}: {e}"
                )
        return None

    def get_motor_velocity(self, controller_id: int) -> Optional[float]:
        """Get current velocity of a motor."""
        if controller_id in self.controllers:
            try:
                return self.controllers[controller_id].velocity
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to get velocity for controller {controller_id}: {e}"
                )
        return None

    def get_motor_last_percent_output(self, controller_id: int) -> Optional[float]:
        """Get last commanded percent output."""
        if controller_id in self.motor_states:
            return self.motor_states[controller_id].last_percent_output
        return None

    def get_all_motor_states(self) -> Dict[int, MotorState]:
        """Get copy of all motor states."""
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
