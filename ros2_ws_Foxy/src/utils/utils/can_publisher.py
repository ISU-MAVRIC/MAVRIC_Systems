#!/usr/bin/python3

"""
can_publisher.py

Desc: Helper class for publishing CAN commands from control nodes.
      Reduces code duplication across drive_control, arm_control, and steer_control.
Author: MAVRIC Team
Date: 2025-11-04
"""

from typing import List, Tuple, Optional, Any, Union
from mavric_msg.msg import CANCommand, CANCommandBatch
from .command_filter import CommandDeduplicator


class CANCommandPublisher:
    """
    Helper class for publishing batched CAN commands.
    
    Encapsulates the common logic for:
    - Building motor command tuples
    - Applying motor inversions
    - Deduplicating commands
    - Publishing batched commands
    """

    def __init__(
        self,
        publisher: Any,
        invert_motors: List[int],
        deadband: float,
        command_type: int = CANCommand.VELOCITY_OUTPUT,
    ):
        """
        Initialize the CAN command publisher helper.

        Args:
            publisher: ROS2 publisher for CANCommandBatch messages
            invert_motors: List of motor IDs that should be inverted
            deadband: value threshold for command deduplication
            command_type: Default CAN command type (e.g., VELOCITY_OUTPUT, PERCENT_OUTPUT)
        """
        self.publisher = publisher
        self.invert_motors = invert_motors
        self.deduplicator = CommandDeduplicator(deadband=deadband)
        self.command_type = command_type

    def publish_batch(
        self,
        motor_commands: Union[List[Tuple[int, float]], List[Tuple[int, float, int]]],
        command_type: Optional[int] = None,
    ) -> None:
        """
        Publish a batch of motor commands.

        Args:
            motor_commands: List of (motor_id, value) tuples
            command_type: Override default command type for this batch (optional)
            publish_if_empty: If True, publish even if batch is empty (default: False)
        """
        batch = CANCommandBatch()
        cmd_type = command_type if command_type is not None else self.command_type

        # motor_commands entries can be either:
        #  - (motor_id, value)
        #  - (motor_id, value, per_command_type)
        for item in motor_commands:
            # Normalize tuple lengths and guard against invalid entries
            if len(item) == 3:
                motor_id, value, per_cmd_type = item
            elif len(item) == 2:
                motor_id, value = item
                per_cmd_type = None
        

            # Apply motor inversion if configured
            if motor_id in self.invert_motors:
                value = value * -1

            # Only add to batch if value changed significantly
            if self.deduplicator.should_send(motor_id, value):
                effective_cmd_type = per_cmd_type if per_cmd_type is not None else cmd_type
                cmd = CANCommand(
                    command_type=effective_cmd_type,
                    controller_id=motor_id,
                    value=value,
                )
                batch.commands.append(cmd)

        # Publish if batch has commands
        if batch.commands:
            self.publisher.publish(batch)

    def publish_single(
        self,
        motor_id: int,
        value: float,
        command_type: Optional[int] = None,
    ) -> bool:
        """
        Publish a single motor command.

        Args:
            motor_id: The motor ID
            value: The command value
            command_type: Override default command type (optional)

        Returns:
            True if command was published, False if deduplicated
        """
        # Apply motor inversion if configured
        if motor_id in self.invert_motors:
            value = value * -1

        # Check deduplicator
        if not self.deduplicator.should_send(motor_id, value):
            return False

        cmd_type = command_type if command_type is not None else self.command_type
        cmd = CANCommand(
            command_type=cmd_type,
            controller_id=motor_id,
            value=value,
        )

        batch = CANCommandBatch(commands=[cmd])
        self.publisher.publish(batch)
        return True