from .abstract_servo import AbstractServoKit, AbstractServo, AbstractContinuousServo
from adafruit_servokit import ServoKit
from typing import Optional, List
from busio import I2C


# --- Real Positional Servo Channel ---
class RealServo(AbstractServo):
    def __init__(self, servo):
        self._servo = servo

    @property
    def angle(self) -> float:
        return self._servo.angle

    @angle.setter
    def angle(self, value: float) -> None:
        self._servo.angle = value
    
    @property
    def actuation_range(self) -> float:
        return self._servo.actuation_range

    @actuation_range.setter
    def actuation_range(self, value: float) -> None:
        self._servo.actuation_range = value
    
    def set_pulse_width_range(self, min_pulse: int = 750, max_pulse: int = 2250) -> None:
        self._servo.set_pulse_width_range(min_pulse, max_pulse)


# --- Real Continuous Servo Channel ---
class RealContinuousServo(AbstractContinuousServo):
    def __init__(self, continuous_servo):
        self._continuous_servo = continuous_servo

    @property
    def throttle(self) -> float:
        return self._continuous_servo.throttle

    @throttle.setter
    def throttle(self, value: float) -> None:
        self._continuous_servo.throttle = value
    
    def set_pulse_width_range(self, min_pulse: int = 750, max_pulse: int = 2250) -> None:
        self._continuous_servo.set_pulse_width_range(min_pulse, max_pulse)


# --- Real ServoKit (drop-in replacement) ---
class RealServoKit(AbstractServoKit):
    def __init__(
        self,
        *,
        channels: int,
        i2c: Optional[I2C] = None,
        address: int = 0x40,
        reference_clock_speed: int = 25000000,
        frequency: int = 50,
    ):
        self._kit = ServoKit(
            channels=channels,
            i2c=i2c,
            reference_clock_speed=reference_clock_speed,
            frequency=frequency,
            address=address,
        )
        self._channels = channels
        # Lazy initialization to avoid "channel already in use" errors
        self._servo = [None] * channels
        self._continuous = [None] * channels

    @property
    def servo(self) -> List[AbstractServo]:
        # Lazily initialize servo wrappers when accessed
        for i in range(self._channels):
            if self._servo[i] is None:
                self._servo[i] = RealServo(self._kit.servo[i])
        return self._servo

    @property
    def continuous_servo(self) -> List[AbstractContinuousServo]:
        # Lazily initialize continuous servo wrappers when accessed
        for i in range(self._channels):
            if self._continuous[i] is None:
                self._continuous[i] = RealContinuousServo(self._kit.continuous_servo[i])
        return self._continuous
