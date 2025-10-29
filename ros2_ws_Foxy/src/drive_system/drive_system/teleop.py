#!/usr/bin/python3

"""
teleop.py - Message forwarder (if needed)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavric_msg.msg import DriveTrain, SteerTrain

class Teleop(Node):
    """
    Forwards messages from old topics to new topics (if needed)
    """

    def __init__(self) -> None:
        super().__init__("teleop_forwarder")
        
        # Create publishers for the correct topics
        self.drive_pub = self.create_publisher(DriveTrain, "/Drive/Drive_Command", 10)
        self.steer_pub = self.create_publisher(SteerTrain, "/Drive/Steer_Command", 10)
        
        # If you need to subscribe to old topics and forward them:
        # self.drive_sub = self.create_subscription(DriveTrain, "drive_train", self.drive_forward, 10)
        # self.steer_sub = self.create_subscription(SteerTrain, "steer_train", self.steer_forward, 10)

        self.get_logger().info("Teleop forwarder started")

    # Optional: Add forwarding methods if needed
    # def drive_forward(self, msg):
    #     self.drive_pub.publish(msg)
    # 
    # def steer_forward(self, msg):
    #     self.steer_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()