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
        self._servo = [RealServo(self._kit.servo[i]) for i in range(channels)]
        # Store kit and channels for lazy initialization of continuous servos
        self._kit_ref = self._kit
        self._channels = channels
        self._continuous = None  # Lazy load on first access

    @property
    def servo(self) -> List[AbstractServo]:
        return self._servo

    @property
    def continuous_servo(self) -> List[AbstractContinuousServo]:
        # Lazy initialize continuous servos only when accessed
        if self._continuous is None:
            self._continuous = [
                RealContinuousServo(self._kit_ref.continuous_servo[i]) for i in range(self._channels)
            ]
        return self._continuous
