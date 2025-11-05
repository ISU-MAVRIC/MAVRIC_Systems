from .abstract_servo import AbstractServoKit
from rclpy.logging import get_logger


class ServoProvider:
    """Provides a ServoKit instance, real or mock based on availability."""

    @staticmethod
    def get_servo_kit(channels: int = 16, address: int = 0x40) -> AbstractServoKit:
        try:
            from .real_servo import RealServoKit

            tmp =  RealServoKit(channels=channels, address=address)
            logger = get_logger("ServoProvider")
            logger.info("Using RealServoKit for servo control.")
            return tmp
        except AttributeError:
            from .mock_servo import MockServoKit
            
            tmp =  MockServoKit(channels=channels, address=address)
            logger = get_logger("ServoProvider")
            logger.warn("RealServoKit not available, using MockServoKit for servo control.")
            return tmp
