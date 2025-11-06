#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavric_msg.msg import ArmScales, ScaleFeedback

class ScaleTuning(Node):
    """
    Sets up the listeners for arm and drive scaling messages,
    sets up publisher to verify active scaling values,
    and defines default scalings and startup behavior.
    """

    def __init__(self) -> None:
        super().__init__("scale_tuning")
        
        # Initialize scaling values as instance variables
        self.drive_scale = Float64(data=1.0)
        self.arm_scales = ArmScales()
        self.arm_scales.shoulder_rot = 0.25
        self.arm_scales.shoulder_pitch = 0.75
        self.arm_scales.elbow_pitch = 0.5
        self.arm_scales.wrist_pitch = 0.3
        self.arm_scales.wrist_rot = 0.75

        # Drive Scale Subscriber - FIXED: Correct topic name
        self.driveScale_subscription = self.create_subscription(
            Float64, "drive_scale", self.driveScale_listener, 10
        )

        # Arm Scales Subscriber - FIXED: Correct topic name
        self.armScales_subscription = self.create_subscription(
            ArmScales, "arm_scales", self.armScale_listener, 10
        )

        # Publishers
        self.driveScale_publisher = self.create_publisher(Float64, "drive_scale", 10)
        self.armScale_publisher = self.create_publisher(ArmScales, "arm_scales", 10)
        self.scaleFeedback_publisher = self.create_publisher(ScaleFeedback, "scale_feedback", 10)

        # Publish defaults
        self.driveScale_publisher.publish(self.drive_scale)
        self.armScale_publisher.publish(self.arm_scales)

        # Feedback timer
        self.timer = self.create_timer(5.0, self.publish_feedback)

    def driveScale_listener(self, msg: Float64) -> None:
        """Called whenever new drive scaling is received from basestation."""
        self.drive_scale = msg
        self.get_logger().info(f"Received drive scale: {msg.data}")

    def armScale_listener(self, msg: ArmScales) -> None:
        """Called whenever new arm scaling is received from basestation."""
        self.arm_scales = msg
        self.get_logger().info("Received arm scales")

    def publish_feedback(self) -> None:
        """Publishes the current scaling values as feedback."""
        feedback = ScaleFeedback()
        feedback.drive = self.drive_scale.data
        feedback.shoulder_rot = self.arm_scales.shoulder_rot
        feedback.shoulder_pitch = self.arm_scales.shoulder_pitch
        feedback.elbow_pitch = self.arm_scales.elbow_pitch
        feedback.wrist_pitch = self.arm_scales.wrist_pitch
        feedback.wrist_rot = self.arm_scales.wrist_rot
        self.scaleFeedback_publisher.publish(feedback)


def main(args=None):
    rclpy.init(args=args)
    scale_tuning = ScaleTuning()
    rclpy.spin(scale_tuning)
    scale_tuning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()