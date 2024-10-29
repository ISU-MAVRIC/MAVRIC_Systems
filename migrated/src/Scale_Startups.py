#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float64
from mavric.msg import ArmData, ScaleFeedback

class ScaleManager(Node):
    def __init__(self):
        super().__init__('scale_manager')

        # Global variables
        self.arm_values = ArmData()
        self.drive_value = Float64()

        # Publishers
        self.drive_sens_pub = self.create_publisher(Float64, 'Drive/Drive_Sensitivity', 10)
        self.arm_sens_pub = self.create_publisher(ArmData, 'Arm/Arm_Sensitivity', 10)
        self.feedback_pub = self.create_publisher(ScaleFeedback, 'SensFeedback', 10)

        # Subscribers
        self.create_subscription(Float64, 'Drive/Drive_Sensitivity', self.drive_cb, 10)
        self.create_subscription(ArmData, 'Arm/Arm_Sensitivity', self.arm_cb, 10)

        # Parameters
        self.arm_values.shoulder_rot = self.get_parameter_or('ShoulderRot', 1.0)
        self.arm_values.shoulder_pitch = self.get_parameter_or('ShoulderPitch', 1.0)
        self.arm_values.elbow_pitch = self.get_parameter_or('ElbowPitch', 1.0)
        self.arm_values.wrist_pitch = self.get_parameter_or('WristPitch', 1.0)
        self.arm_values.wrist_rot = self.get_parameter_or('WristRot', 1.0)
        self.drive_value = Float64(data=self.get_parameter_or('Drive_Sens', 1.0))

        # Initial Publishing
        time.sleep(5)
        self.publish_initial_values()

        # Timer for continuous feedback publishing
        self.create_timer(5.0, self.publish_feedback)

    def get_parameter_or(self, param_name, default_value):
        self.declare_parameter(param_name, default_value)
        return self.get_parameter(param_name).get_parameter_value().double_value

    def publish_initial_values(self):
        self.drive_sens_pub.publish(self.drive_value)
        self.arm_sens_pub.publish(self.arm_values)

    def drive_cb(self, data):
        self.drive_value = data.data

    def arm_cb(self, data):
        self.arm_values = data

    def publish_feedback(self):
        values = ScaleFeedback()
        values.drive = self.drive_value
        values.shoulder_pitch = self.arm_values.shoulder_pitch
        values.shoulder_rot = self.arm_values.shoulder_rot
        values.elbow_pitch = self.arm_values.elbow_pitch
        values.wrist_pitch = self.arm_values.wrist_pitch
        values.wrist_rot = self.arm_values.wrist_rot
        self.feedback_pub.publish(values)

def main(args=None):
    rclpy.init(args=args)
    scale_manager_node = ScaleManager()
    rclpy.spin(scale_manager_node)
    scale_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
