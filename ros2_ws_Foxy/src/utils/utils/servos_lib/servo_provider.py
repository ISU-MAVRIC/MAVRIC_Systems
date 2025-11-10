from .abstract_servo import AbstractServoKit
from rclpy.logging import get_logger


class ServoProvider:
    """Provides a ServoKit instance (singleton), real or mock based on availability."""
    
    _instance = None

    @staticmethod
    def get_servo_kit(channels: int = 16, address: int = 0x40) -> AbstractServoKit:
        if ServoProvider._instance is None:
            try:
                from .real_servo import RealServoKit

                ServoProvider._instance = RealServoKit(channels=channels, address=address)
                logger = get_logger("ServoProvider")
                logger.info("Using RealServoKit for servo control.")
            except AttributeError:
                from .mock_servo import MockServoKit
                
                ServoProvider._instance = MockServoKit(channels=channels, address=address)
                logger = get_logger("ServoProvider")
                logger.warn("RealServoKit not available, using MockServoKit for servo control.")
        
        return ServoProvider._instance
    
    @staticmethod
    def reset():
        """Reset the singleton instance. Useful for testing."""
        ServoProvider._instance = None
