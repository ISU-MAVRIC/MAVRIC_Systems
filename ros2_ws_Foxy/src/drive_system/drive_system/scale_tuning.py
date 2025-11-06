#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavric_msg.msg import ArmScales, ScaleFeedback
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ScaleTuning(Node):
    """
    Sets up the listeners for arm and drive scaling messages,
    sets up publisher to verify active scaling values,
    and defines default scalings and startup behavior.
    """

    def __init__(self) -> None:
        super().__init__("scale_tuning")
        # Declare parameters (defaults) and store them on the node instance
        self.declare_parameter('drive_scale', 1.0)
        self.declare_parameter('arm_scales.shoulder_rot', 0.25)
        self.declare_parameter('arm_scales.shoulder_pitch', 0.75)
        self.declare_parameter('arm_scales.elbow_pitch', 0.5)
        self.declare_parameter('arm_scales.wrist_pitch', 0.3)
        self.declare_parameter('arm_scales.wrist_rot', 0.75)

        # Initialize values from parameters
        self.drive_scale = Float64(data=self.get_parameter('drive_scale').value)

        self.arm_scales = ArmScales()
        self.arm_scales.shoulder_rot = self.get_parameter('arm_scales.shoulder_rot').value
        self.arm_scales.shoulder_pitch = self.get_parameter('arm_scales.shoulder_pitch').value
        self.arm_scales.elbow_pitch = self.get_parameter('arm_scales.elbow_pitch').value
        self.arm_scales.wrist_pitch = self.get_parameter('arm_scales.wrist_pitch').value
        self.arm_scales.wrist_rot = self.get_parameter('arm_scales.wrist_rot').value

        # Drive subscription — update node state and immediately publish feedback
        self.driveScale_subscription = self.create_subscription(
            Float64,
            "drive_scale",
            self._on_drive_scale,
            10,
        )

        # Arm scales subscription — update node state and immediately publish feedback
        self.armScales_subscription = self.create_subscription(
            ArmScales,
            "arm_scales",
            self._on_arm_scales,
            10,
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.driveScale_publisher = self.create_publisher(Float64, "drive_scale", qos_profile=qos_profile)
        self.armScale_publisher = self.create_publisher(ArmScales, "arm_scales", qos_profile=qos_profile)
        self.scaleFeedback_publisher = self.create_publisher(ScaleFeedback, "scale_feedback", qos_profile=qos_profile)

        # Publish defaults
        self.driveScale_publisher.publish(self.drive_scale)
        self.armScale_publisher.publish(self.arm_scales)
        self.publish_feedback()

        # Feedback timer
        # self.timer = self.create_timer(5.0, self.publish_feedback)

    def _on_drive_scale(self, msg: Float64) -> None:
        """Callback for incoming drive_scale messages — update and publish immediate feedback."""
        self.drive_scale = msg
        self.publish_feedback()

    def _on_arm_scales(self, msg: ArmScales) -> None:
        """Callback for incoming arm_scales messages — update and publish immediate feedback."""
        self.arm_scales = msg
        self.publish_feedback()

    def publish_feedback(self) -> None:
        """Publishes the current scaling values as feedback."""
        feedback = ScaleFeedback()
        feedback.drive = self.drive_scale.data if isinstance(self.drive_scale, Float64) else self.drive_scale
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
