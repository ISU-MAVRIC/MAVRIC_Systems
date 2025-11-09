#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavric_msg.msg import ScaleFeedback
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
        self.declare_parameter('drive', 1.0)
        self.declare_parameter('shoulder_rot', 0.25)
        self.declare_parameter('shoulder_pitch', 0.75)
        self.declare_parameter('elbow_pitch', 0.5)
        self.declare_parameter('wrist_pitch', 0.3)
        self.declare_parameter('wrist_rot', 0.75)

        self.msg_scale = ScaleFeedback(
            drive = self.get_parameter('drive').value,
            shoulder_rot = self.get_parameter('shoulder_rot').value,
            shoulder_pitch = self.get_parameter('shoulder_pitch').value,
            elbow_pitch = self.get_parameter('elbow_pitch').value,
            wrist_pitch = self.get_parameter('wrist_pitch').value,
            wrist_rot = self.get_parameter('wrist_rot').value
        )
        
        # Define QOS, Keep last sent msg
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.scaleFeedback_publisher = self.create_publisher(ScaleFeedback, "scale_feedback", qos_profile=qos_profile)

        # Subscriber to Update values
        self.scaleFeedback_sub = self.create_subscription(
            ScaleFeedback,
            "scale_feedback",
            self._update_scales,
            10
        )

        # Publish defaults
        self._publish_feedback(self.msg_scale)

    def _update_scales(self, msg: ScaleFeedback) -> None:
        """Updates scale values only if any value differs by more than 0.01."""
        if (abs(msg.drive - self.msg_scale.drive) < 0.01 and
            abs(msg.shoulder_rot - self.msg_scale.shoulder_rot) < 0.01 and
            abs(msg.shoulder_pitch - self.msg_scale.shoulder_pitch) < 0.01 and
            abs(msg.elbow_pitch - self.msg_scale.elbow_pitch) < 0.01 and
            abs(msg.wrist_pitch - self.msg_scale.wrist_pitch) < 0.01 and
            abs(msg.wrist_rot - self.msg_scale.wrist_rot) < 0.01):
            return
        
        self.msg_scale = msg
        self._publish_feedback(msg)

    def _publish_feedback(self, msg: ScaleFeedback) -> None:
        """Publishes the current scaling values as feedback."""
        self.scaleFeedback_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    scale_tuning = ScaleTuning()
    rclpy.spin(scale_tuning)
    scale_tuning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
