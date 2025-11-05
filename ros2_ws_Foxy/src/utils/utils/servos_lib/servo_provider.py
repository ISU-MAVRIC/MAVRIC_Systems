from .abstract_servo import AbstractServoKit


class ServoProvider:
    """Provides a ServoKit instance, real or mock based on availability."""

    @staticmethod
    def get_servo_kit(channels: int = 16) -> AbstractServoKit:
        try:
            from .real_servo import RealServoKit

            return RealServoKit(channels=channels)
        except AttributeError:
            from .mock_servo import MockServoKit

            return MockServoKit(channels=channels)
