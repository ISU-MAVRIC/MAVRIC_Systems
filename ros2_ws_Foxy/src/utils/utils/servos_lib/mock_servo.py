from abstract_servo import AbstractServoKit, AbstractServo, AbstractContinuousServo


class MockServo(AbstractServo):
    def __init__(self):
        self.angle = None

    @property
    def angle(self) -> float:
        return self.angle

    @angle.setter
    def angle(self, value: float) -> None:
        self.angle = value


# --- Mock Continuous Servo Channel ---
class MockContinuousServo(AbstractContinuousServo):
    def __init__(self):
        self.throttle = None

    @property
    def throttle(self) -> float:
        return self.throttle

    @throttle.setter
    def throttle(self, value: float) -> None:
        self.throttle = value


# --- Mock ServoKit (drop-in replacement) ---
class MockServoKit(AbstractServoKit):
    def __init__(
        self,
        *,
        channels: int,
        i2c: None,
        address: int = 0x40,
        reference_clock_speed: int = 25000000,
        frequency: int = 50,
    ):
        self.channels = channels
        self.address = address
        self.servo = [MockServo() for _ in range(channels)]
        self.continuous_servo = [MockContinuousServo() for _ in range(channels)]

    @property
    def servo(self) -> list[AbstractServo]:
        return self.servo

    @property
    def continuous_servo(self) -> list[AbstractContinuousServo]:
        return self.continuous_servo
