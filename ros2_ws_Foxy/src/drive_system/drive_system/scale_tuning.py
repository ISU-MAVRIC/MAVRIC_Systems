#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavric_msg.msg import ArmScales, ScaleFeedback
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

arm_scales = ArmScales()
drive_scale = Float64()

class ScaleTuning(Node):
    """
    Sets up the listeners for arm and drive scaling messages,
    sets up publisher to verify active scaling values,
    and defines default scalings and startup behavior.
    """

    def __init__(self) -> None:
        super().__init__("scale_tuning")
        
        # Startup values
        global arm_scales, drive_scale
        drive_scale = Float64(data=1.0)
        arm_scales.shoulder_rot = 0.25
        arm_scales.shoulder_pitch = 0.75
        arm_scales.elbow_pitch = 0.5
        arm_scales.wrist_pitch = 0.3
        arm_scales.wrist_rot = 0.75

        # Drive Scale Subscriber
        self.driveScale_subscription = self.create_subscription(
            Float64, "drive_scale", self.driveScale_listener, 10
        )

        # Arm Scales Subscriber
        self.armScales_subscription = self.create_subscription(
            ArmScales, "arm_scales", self.armScale_listener, 10
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
        self.scaleFeedback_publisher = self.create_publisher(ScaleFeedback, "scale_feedback", 10)

        # Publish defaults
        self.driveScale_publisher.publish(drive_scale)
        self.armScale_publisher.publish(arm_scales)

        # Feedback timer
        self.timer = self.create_timer(5.0, self.publish_feedback)


    def driveScale_listener(self, msg: Float64) -> None:
        """Called whenever new drive scaling is received from basestation."""
        global drive_scale
        drive_scale = msg

    def armScale_listener(self, msg: ArmScales) -> None:
        """Called whenever new arm scaling is received from basestation."""
        global arm_scales
        arm_scales = msg  

    def publish_feedback(self) -> None:
        """Publishes the current scaling values as feedback."""
        global drive_scale, arm_scales
        feedback = ScaleFeedback()
        feedback.drive = drive_scale.data if isinstance(drive_scale, Float64) else drive_scale
        feedback.shoulder_rot = arm_scales.shoulder_rot
        feedback.shoulder_pitch = arm_scales.shoulder_pitch
        feedback.elbow_pitch = arm_scales.elbow_pitch
        feedback.wrist_pitch = arm_scales.wrist_pitch
        feedback.wrist_rot = arm_scales.wrist_rot
        self.scaleFeedback_publisher.publish(feedback)


def main(args=None):
    rclpy.init(args=args)
    scale_tuning = ScaleTuning()
    rclpy.spin(scale_tuning)
    scale_tuning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
