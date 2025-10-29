#!/usr/bin/python3

"""
can_control_sim.py

Desc: Simulated CAN control for development without hardware
Author: Isaac Denning
Date: 10/29/25
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavric_msg.msg import DriveTrain, SteerTrain, Arm

class CanControlSim(Node):
    """
    Simulated CAN control for development without hardware
    """

    def __init__(self) -> None:
        super().__init__("can_control_sim")
        
        # Log that we're in simulation mode
        self.get_logger().info("CAN Control running in SIMULATION MODE (no hardware)")
        
        # Drive Control subscribers
        self.drive_train_subscription = self.create_subscription(
            DriveTrain, "/Drive/Drive_Command", self.drive_listener, 10
        )

        # Steer Control subscribers
        self.steer_train_subscription = self.create_subscription(
            SteerTrain, "/Drive/Steer_Command", self.steer_listener, 10
        )

        # Arm Control - Individual joint subscribers
        self.arm_subscriptions = {}
        arm_topics = {
            "shoulder_rot": "/Arm/Shoulder_Rotation_Command",
            "shoulder_pitch": "/Arm/Shoulder_Pitch_Command", 
            "elbow_pitch": "/Arm/Elbow_Pitch_Command",
            "wrist_pitch": "/Arm/Wrist_Pitch_Command",
            "wrist_rot": "/Arm/Wrist_Rotation_Command",
            "claw": "/Arm/Claw_Command",
            "luminometer": "/Arm/Luminometer_Command",
            "lumibutton": "/Arm/LumiButton_Command", 
            "lumilid": "/Arm/LumiLid_Command",
            "cache": "/Arm/Cache_Command",
            "drill": "/Arm/Drill_Command",
            "drillactuator": "/Arm/DrillActuator_Command"
        }
        
        for joint, topic in arm_topics.items():
            self.arm_subscriptions[joint] = self.create_subscription(
                Float64, topic, self.create_arm_callback(joint), 10
            )

    def create_arm_callback(self, joint_name):
        """Create a callback function for a specific arm joint"""
        def arm_callback(msg):
            self.arm_listener(joint_name, msg.data)
        return arm_callback

    def drive_listener(self, msg: DriveTrain) -> None:
        """
        Log drive commands (simulated) - UPDATED FOR 4 MOTORS
        """
        self.get_logger().info(
            f"DRIVE SIM: FL:{msg.front_left:.2f} FR:{msg.front_right:.2f} "
            f"BL:{msg.back_left:.2f} BR:{msg.back_right:.2f}"
        )

    def steer_listener(self, msg: SteerTrain) -> None:
        """
        Log steer commands (simulated)
        """
        self.get_logger().info(
            f"STEER SIM: strLf:{msg.str_lf:.2f} strLb:{msg.str_lb:.2f} "
            f"strRf:{msg.str_rf:.2f} strRb:{msg.str_rb:.2f}"
        )

    def arm_listener(self, joint: str, value: float) -> None:
        """
        Log arm commands (simulated)
        """
        self.get_logger().info(f"ARM SIM: {joint} = {value:.2f}")


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    can_control_sim = CanControlSim()

    # Run the node
    rclpy.spin(can_control_sim)

    # Destroy it when done
    can_control_sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()