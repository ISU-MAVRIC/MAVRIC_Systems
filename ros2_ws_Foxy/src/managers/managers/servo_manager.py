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

        # Declare parameters for each possible servo channel configuration
        self.configure_servos()

        # Create subscriber for servo commands
        self.sub_servo_command = self.create_subscription(
            ServoCommand, "servo_commands", self.servo_command_callback, 10
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
            self.get_logger().warn(
                f"Unknown servo type {msg.servo_type} for channel {msg.channel}"
            )

    def configure_servos(self):
        """
        Configures servos by checking for channel-specific parameters.
        For each channel 0-15, checks if configuration parameters exist.
        """
        # PCA9685 supports 16 channels (0-15)
        for channel in range(16):
            param_prefix = f"channel_{channel}"
            
            self.declare_parameter(f"{param_prefix}.min_pulse", -1)
            self.declare_parameter(f"{param_prefix}.max_pulse", -1)
            self.declare_parameter(f"{param_prefix}.actuation_range", -1)
            self.declare_parameter(f"{param_prefix}.start_angle", -1)
            
            min_pulse = self.get_parameter(f"{param_prefix}.min_pulse").value
            max_pulse = self.get_parameter(f"{param_prefix}.max_pulse").value
            act_range = self.get_parameter(f"{param_prefix}.actuation_range").value
            start_angle = self.get_parameter(f"{param_prefix}.start_angle").value
            
            if min_pulse == -1 or max_pulse == -1 or act_range == -1:
                # Parameters not set for this channel, skip configuration
                continue
                
            servo = self.kit.servo[channel]
            servo.set_pulse_width_range(min_pulse, max_pulse)
            servo.actuation_range = act_range
            if start_angle != -1: servo.angle = start_angle
            
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
