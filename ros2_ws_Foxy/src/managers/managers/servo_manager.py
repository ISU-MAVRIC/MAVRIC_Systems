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

        # Configure Servos based on provided parameters
        self.declare_parameter("configured_channels", [])
        channels_to_config = self.get_parameter("configured_channels").value
        self.configure_servos(channels_to_config)

        # Initialize ServoKit

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

    def configure_servos(self, channels):
            """
            Iterates through the requested channels, declares parameters for them,
            and applies the values to the ServoKit instance.
            """
            for channel in channels:
                param_prefix = f"servo_config.channel_{channel}"
                
                self.declare_parameters(
                    namespace='',
                    parameters=[
                        (f"{param_prefix}.min_pulse", 750),
                        (f"{param_prefix}.max_pulse", 2250),
                        (f"{param_prefix}.actuation_range", 180),
                    ]
                )

                min_pulse = self.get_parameter(f"{param_prefix}.min_pulse").value
                max_pulse = self.get_parameter(f"{param_prefix}.max_pulse").value
                act_range = self.get_parameter(f"{param_prefix}.actuation_range").value

                # APPLY TO HARDWARE
                servo = self.kit.servo[channel]
                servo.set_pulse_width_range(min_pulse, max_pulse)
                servo.actuation_range = act_range
                
                self.get_logger().info(
                    f"Channel {channel} Configured: Pulse({min_pulse}, {max_pulse}), Range({act_range})"
                )


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
