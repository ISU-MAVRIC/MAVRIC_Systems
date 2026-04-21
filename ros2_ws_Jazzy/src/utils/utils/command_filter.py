#!/usr/bin/python3

"""
command_filter.py

Desc: Utilities for filtering duplicate or near-duplicate commands to reduce
      unnecessary CAN bus traffic and motor controller updates.
Author: MAVRIC Team
Date: 2025-11-04
"""

from typing import Dict, Optional


class CommandDeduplicator:
    """
    Tracks previously sent command values and filters out duplicates based on
    a configurable deadband threshold.
    
    Useful for reducing CAN bus traffic when joystick inputs or other control
    sources send repeated identical values.
    
    Example:
        dedup = CommandDeduplicator(deadband=0.001)
        
        if dedup.should_send(motor_id=5, value=0.5):
            # Send command to motor
            pass
    """

    def __init__(self, deadband: float = 0.001):
        """
        Initialize the command deduplicator.
        
        Args:
            deadband: Minimum difference required to consider a command "changed".
                     Values closer than this threshold are considered duplicates.
                     Default 0.001 (0.1% for percent output commands).
        """
        self.deadband = deadband
        self.last_commands: Dict[int, float] = {}

    def should_send(self, controller_id: int, value: float) -> bool:
        """
        Determine if a command should be sent based on deadband filtering.
        
        Args:
            controller_id: CAN ID or other unique identifier for the controller
            value: New command value to check
            
        Returns:
            True if command is different enough to send, False if it's a duplicate
        """
        # First command for this controller - always send
        if controller_id not in self.last_commands:
            self.last_commands[controller_id] = value
            return True
        
        # Check if value changed beyond deadband threshold
        if abs(value - self.last_commands[controller_id]) >= self.deadband:
            self.last_commands[controller_id] = value
            return True
        
        # Value unchanged (within deadband) - don't send
        return False

    def reset(self, controller_id: Optional[int] = None) -> None:
        """
        Reset tracked command state.
        
        Args:
            controller_id: If provided, reset only this controller.
                          If None, reset all controllers.
        """
        if controller_id is None:
            self.last_commands.clear()
        elif controller_id in self.last_commands:
            del self.last_commands[controller_id]

    def get_last_value(self, controller_id: int) -> Optional[float]:
        """
        Get the last sent value for a controller.
        
        Args:
            controller_id: CAN ID or other unique identifier
            
        Returns:
            Last sent value, or None if no command has been sent yet
        """
        return self.last_commands.get(controller_id)

    def set_deadband(self, deadband: float) -> None:
        """
        Update the deadband threshold.
        
        Args:
            deadband: New deadband value
        """
        self.deadband = deadband
