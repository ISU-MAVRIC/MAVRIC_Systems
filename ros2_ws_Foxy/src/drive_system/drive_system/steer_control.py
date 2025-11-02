import rclpy
from rclpy.node import Node
from mavric_msg.msg import SteerTrain
from utils.SparkCANLib.SparkCAN import SparkBusManager
from typing import Optional

# CAN IDs for Drive Controllers
FLS = 7
FRS = 10
BLS = 9
BRS = 2

INVERTED = -1


class SteerControl(Node):
    """
    Uses the CANbus interface to set the velocity of the motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """

    def __init__(self) -> None:
        super().__init__("steer_control")
        
        self.bus = SparkBusManager.get_instance()
        self.FLMotor = self.bus.init_controller(FLS)
        self.FRMotor = self.bus.init_controller(FRS)
        self.BLMotor = self.bus.init_controller(BLS)
        self.BRMotor = self.bus.init_controller(BRS)

        # self.steer_train_subscription = self.create_subscription(
        #     SteerTrain, "steer_train", self.set_position, 10
        # )

    def set_position(self, msg: SteerTrain):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (SteerTrain): The values from ROS indicating the velocity of each motor.
        """

        self.FLMotor.position_output(msg.front_left)
        self.FRMotor.position_output(msg.front_right)
        self.BLMotor.position_output(INVERTED * msg.back_left)
        self.BRMotor.position_output(INVERTED * msg.back_right)

# def main(args=None):
#     rclpy.init(args=args)

#     # Create the node
#     steer_control = SteerControl()

#     # Run the node
#     rclpy.spin(steer_control)

#     # Destroy it when done
#     steer_control.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()