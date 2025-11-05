import rclpy
from rclpy.node import Node
from mavric_msg.msg import ServoCommand
from utils.servos_lib import ServoProvider

class ServoManager(Node):
    def __init__(self) -> None:
        super().__init__("servo_manager")

        # Declare parameters
        self.declare_parameter("address", 0x40)

        self.address = self.get_parameter("address").value

        # Initialize ServoKit
        self.kit = ServoProvider.get_servo_kit(address=self.address)

        # Create subscriber for servo commands
        self.sub_servo_command = self.create_subscription(
            ServoCommand,
            "servo_commands",
            self.servo_command_callback,
            10
        )

    def servo_command_callback(self, msg: ServoCommand) -> None:
        # Process command based on servo type
        if msg.servo_type == ServoCommand.STANDARD_SERVO:
            servo = self.kit.servo[msg.channel]
            servo.angle = msg.value
        elif msg.servo_type == ServoCommand.CONTINUOUS_SERVO:
            servo = self.kit.continuous_servo[msg.channel]
            servo.throttle = msg.value
        else:
            self.get_logger().warn(f"Unknown servo type {msg.servo_type} for channel {msg.channel}")


def main(args=None):
    rclpy.init(args=args)
    servo_manager = ServoManager()

    try:
        rclpy.spin(servo_manager)
    except KeyboardInterrupt:
        pass
    finally:
        servo_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()