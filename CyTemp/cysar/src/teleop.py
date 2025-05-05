#!/usr/bin/python3

"""
teleop.py

Desc: Main functionality of overall program. Takes ROS joystick values and 
        decides what to publish for drive, flipper, and arm. 
Author: Isaac Denning
Date: 10/21/23
"""

import rclpy
from rclpy.node import Node
from cysar.msg import DriveTrain
import numpy as np

class Teleop(Node):
    """
    Main functionality of overall program. Takes ROS joystick values and 
    decides what to publish for drive, flipper, and arm.
    """
    def __init__(self) -> None:
        super().__init__('Teleop')
        # Variables
        self.drive_train = DriveTrain()
        self.mode : str = "Drive"

        # Publisher/Subscribers
        self.drive_train_publisher = self.create_publisher(DriveTrain, 'drive_train', 10)

        # Paramerters   
        self.declare_parameter('max_speed', '1.0')
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.get_logger().info(f"""max_speed: {self.max_speed}, """)
        


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    teleop = Teleop()

    # Run the node
    rclpy.spin(teleop)

    # Destroy it when done
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
