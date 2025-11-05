#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavric_msg.msg import ArmScales, ScaleFeedback

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
        
        #Startup values
        drive_scale = 1.0
        arm_scales.shoulder_rot = 0.25
        arm_scales.shoulder_pitch = 0.75
        arm_scales.shoulder_pitch = 0.75
        arm_scales.wrist_pitch = 0.3
        arm_scales.wrist_rot = 0.75

        #Drive Scale Subscriber
        self.driveScale_subscription = self.create_subscription(
            Float64, "drive_scale", self.driveScale_listener, 10
        )

        #Arm Scales Subscriber
        self.armScales_subscription = self.create_subscription(
            ArmScales, "arm_scales", self.armScales_listener, 10
        )

        #Drive Scale Publisher
        self.driveScale_publisher = self.create_publisher(
            Float64, "drive_scale", 10
        )

        #Arm Scale Publisher
        self.armScale_publisher = self.create_publisher(
            ArmScales, "arm_scales", 10
        )

        #Scale Feedback Publisher
        self.scaleFeedback_publisher = self.create_publisher(
            ScaleFeedback, "scale_feedback", 10
        )

        #Publish defaults
        self.driveScale_publisher(drive_scale)
        self.armScale_publisher(arm_scales)

        #Publish feedback for basestation every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_feedback)


    def driveScale_listener(self, msg: Float64) -> None:
        """
        Called whenever new drive scaling is received from basestation.
        """
        global drive_scale
        drive_scale = msg

    def armScale_listener(self, msg: ArmScales) -> None:
        """
        Called whenever new arm scaling is received from basestation.
        """
        global arm_scales
        arm_scales = msg  
 
    def publish_feedback(self) -> None:
        """Publishes the current scaling values as feedback."""
        feedback = ScaleFeedback()
        feedback.drive = self.drive_scale
        feedback.shoulder_rot = self.arm_scales.shoulder_rot
        feedback.shoulder_pitch = self.arm_scales.shoulder_pitch
        feedback.shoulder_pitch = self.arm_scales.shoulder_pitch
        feedback.wrist_pitch = self.arm_scales.wrist_pitch
        feedback.wrist_rot = self.arm_scales.wrist_rot
        self.scaleFeedback_publisher.publish(feedback)  

def main(args=None):
    rclpy.init(args=args)

    #Create the node
    scale_tuning = ScaleTuning()

    # Run the node
    rclpy.spin(scale_tuning)

    #Destroy it when done
    scale_tuning.destroy_node()
    rclpy.shutodown()


if __name__ == '__main__':
    main()