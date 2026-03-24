#!/usr/bin/python3

"""
servo_publisher.py

Desc: Helper class for publishing ServoCommand messages from control nodes.
      Mirrors the CAN command publisher helper but for servo (PWM) outputs.
Author: MAVRIC Team
Date: 2025-11-10
"""

from typing import List, Tuple, Optional, Any, Union
from mavric_msg.msg import ServoCommand
from .command_filter import CommandDeduplicator


class ServoCommandPublisher:
    """
    Helper class for publishing servo commands.

    Encapsulates common logic for:
    - Applying deduplication (deadband)
    - Supporting per-command servo type overrides
    - Publishing ServoCommand messages
    """

    def __init__(
        self,
        publisher: Any,
        deadband: float,
        default_servo_type: int = ServoCommand.STANDARD_SERVO,
    ):
        """
        Initialize the Servo command publisher helper.

        Args:
            publisher: ROS2 publisher for ServoCommand messages
            deadband: value threshold for command deduplication
            default_servo_type: Default servo type to use when none provided
        """
        self.publisher = publisher
        self.deduplicator = CommandDeduplicator(deadband=deadband)
        self.default_servo_type = default_servo_type

    def publish_batch(
        self,
        servo_commands: Union[List[Tuple[int, float]], List[Tuple[int, float, int]]],
        servo_type: Optional[int] = None,
    ) -> None:
        """
        Publish a batch of servo commands.

        Args:
            servo_commands: List of (channel, value) or (channel, value, servo_type)
            servo_type: Default servo_type for the batch if per-command override not provided
        """
        effective_batch_type = servo_type if servo_type is not None else self.default_servo_type

        for item in servo_commands:
            # Normalize tuple lengths
            if len(item) == 3:
                channel, value, per_type = item
            elif len(item) == 2:
                channel, value = item
                per_type = None


            # Deduplicate based on channel/value
            if not self.deduplicator.should_send(channel, value):
                continue

            cmd_type = per_type if per_type is not None else effective_batch_type
            msg = ServoCommand(servo_type=cmd_type, channel=channel, value=value)
            self.publisher.publish(msg)

    def publish_single(
        self,
        channel: int,
        value: float,
        servo_type: Optional[int] = None,
    ) -> bool:
        """
        Publish a single servo command with deduplication.

        Args:
            channel: servo channel (0-15)
            value: servo value (angle or throttle depending on servo_type)
            servo_type: optional override of default servo type

        Returns:
            True if published, False if deduplicated
        """
        if not self.deduplicator.should_send(channel, value):
            return False

        cmd_type = servo_type if servo_type is not None else self.default_servo_type
        msg = ServoCommand(servo_type=cmd_type, channel=channel, value=value)
        self.publisher.publish(msg)
        return True
