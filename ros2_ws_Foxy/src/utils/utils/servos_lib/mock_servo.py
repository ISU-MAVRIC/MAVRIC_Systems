from .abstract_servo import AbstractServoKit, AbstractServo, AbstractContinuousServo
from typing import List, Any

class MockServo(AbstractServo):
    def __init__(self, actuation_range: int = 180):
        self._angle = None
        self._actuation_range = actuation_range

    @property
    def angle(self) -> float:
        return self._angle

    @angle.setter
    def angle(self, value: float) -> None:
        self._angle = value

    @property
    def actuation_range(self) -> float:
        return self._actuation_range

    @actuation_range.setter
    def actuation_range(self, value: float) -> None:
        self._actuation_range = value

    def set_pulse_width_range(self, min_pulse: int = 750, max_pulse: int = 2250) -> None:
        pass


# --- Mock Continuous Servo Channel ---
class MockContinuousServo(AbstractContinuousServo):
    def __init__(self):
        self._throttle = None

    @property
    def throttle(self) -> float:
        return self._throttle

    @throttle.setter
    def throttle(self, value: float) -> None:
        self._throttle = value
    
    def set_pulse_width_range(self, min_pulse: int = 750, max_pulse: int = 2250) -> None:
        pass


# --- Mock ServoKit (drop-in replacement) ---
class MockServoKit(AbstractServoKit):
    def __init__(
        self,
        channels: int,
        i2c: Any = None,
        address: int = 0x40,
        reference_clock_speed: int = 25000000,
        frequency: int = 50,
    ):
        self.channels = channels
        self.address = address
        self._servo = [MockServo() for _ in range(channels)]
        self._continuous_servo = [MockContinuousServo() for _ in range(channels)]

    @property
    def servo(self) -> List[AbstractServo]:
        return self._servo

    @property
    def continuous_servo(self) -> List[AbstractContinuousServo]:
        return self._continuous_servo
