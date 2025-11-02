import rclpy
from rclpy.node import Node
from mavric_msg.msg import Arm
from utils.SparkCANLib.SparkCAN import SparkBusManager
# from adafruit_servokit import ServoKit
from typing import Optional

# CAN IDs for Drive Controllers
shoulder_pitch = 11
shoulder_rot = 12
elbow_pitch = 13
wrist_pitch = 14
wrist_rot  = 15
claw = 1 # PWM Controller BUS

INVERTED = -1


class ArmControl(Node):
    """
    Uses the CANbus interface to set the velocity of the motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """

    def __init__(self):
        super().__init__("arm_control")
        
        self.bus = SparkBusManager.get_instance()
        self.SPMotor = self.bus.init_controller(shoulder_pitch)
        self.SRMotor = self.bus.init_controller(shoulder_rot)
        self.EPMotor = self.bus.init_controller(elbow_pitch)
        self.WPMotor = self.bus.init_controller(wrist_pitch)
        self.WRMotor = self.bus.init_controller(wrist_rot)
        # self.kit = ServoKit(channels=16)

        # self.arm_subscription = self.create_subscription(
        #     Arm, "arm_control", self.set_velocity, 10
        # )

    def set_velocity(self, msg: Arm):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (SteerTrain): The values from ROS indicating the velocity of each motor.
        """

        self.SPMotor.percent_output(msg.shoulder_pitch)
        self.SRMotor.percent_output(msg.shoulder_rot)
        self.EPMotor.percent_output(msg.elbow_pitch)
        self.WPMotor.percent_output(msg.wrist_pitch)
        self.WRMotor.percent_output(INVERTED * msg.wrist_rot)
        # self.kit.continuous_servo[1].throttle = msg.claw

# def main(args=None):
#     rclpy.init(args=args)

#     # Create the node
#     arm_control = ArmControl()

#     # Run the node
#     rclpy.spin(arm_control)

#     # Destroy it when done
#     arm_control.destroy_node()
#     rclpy.shutdown()
        
# if __name__ == "__main__":
#     main()