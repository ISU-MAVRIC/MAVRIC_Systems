from abc import ABC, abstractmethod
from typing import Optional, List


# --- Abstract Interfaces ---
class AbstractServo(ABC):
    """Abstract interface for a positional servo channel."""

    @property
    @abstractmethod
    def angle(self) -> Optional[float]:
        pass

    @angle.setter
    @abstractmethod
    def angle(self, value: float) -> None:
        pass

    @property
    def actuation_range(self) -> float:
        pass

    @actuation_range.setter
    def actuation_range(self, value: float) -> None:
        pass

    def set_pulse_width_range(self, min_pulse: int = 750, max_pulse: int = 2250) -> None:
        pass


class AbstractContinuousServo(ABC):
    """Abstract interface for a continuous rotation servo channel."""

    @property
    @abstractmethod
    def throttle(self) -> Optional[float]:
        pass

    @throttle.setter
    @abstractmethod
    def throttle(self, value: Optional[float]) -> None:
        pass

    def set_pulse_width_range(self, min_pulse: int = 750, max_pulse: int = 2250) -> None:
        pass


class AbstractServoKit(ABC):
    """Abstract interface for a ServoKit-like controller."""

    @property
    @abstractmethod
    def servo(self) -> List[AbstractServo]:
        pass

    @property
    @abstractmethod
    def continuous_servo(self) -> List[AbstractContinuousServo]:
        pass
