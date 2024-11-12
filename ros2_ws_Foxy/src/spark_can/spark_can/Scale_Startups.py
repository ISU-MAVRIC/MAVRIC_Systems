# scale_startups.py

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float64
from spark_can.msg import ArmData, ScaleFeedback

arm_values = ArmData()
drive_value = Float64()

def drive_cb(msg):
    global drive_value
    drive_value = msg.data

def arm_cb(msg):
    global arm_values
    arm_values = msg

def main(args=None):
    global drive_value, arm_values
    rclpy.init(args=args)
    node = Node("Scale_Manager")
    drive_sens = node.create_publisher(Float64, "Drive/Drive_Sensitivity", 10)
    arm_sens = node.create_publisher(ArmData, "Arm/Arm_Sensitivity", 10)
    feedback = node.create_publisher(ScaleFeedback, "SensFeedback", 10)
    drive_sub = node.create_subscription(Float64, "Drive/Drive_Sensitivity", drive_cb, 10)
    arm_sub = node.create_subscription(ArmData, "Arm/Arm_Sensitivity", arm_cb, 10)
    node.declare_parameter("ShoulderRot", 1.0)
    node.declare_parameter("ShoulderPitch", 1.0)
    node.declare_parameter("ElbowPitch", 1.0)
    node.declare_parameter("WristPitch", 1.0)
    node.declare_parameter("WristRot", 1.0)
    node.declare_parameter("Drive_Sens", 1.0)
    arm_values.ShoulderRot = node.get_parameter("ShoulderRot").value
    arm_values.ShoulderPitch = node.get_parameter("ShoulderPitch").value
    arm_values.ElbowPitch = node.get_parameter("ElbowPitch").value
    arm_values.WristPitch = node.get_parameter("WristPitch").value
    arm_values.WristRot = node.get_parameter("WristRot").value
    drive_value = node.get_parameter("Drive_Sens").value
    time.sleep(5)
    drive_sens.publish(Float64(data=drive_value))
    arm_sens.publish(arm_values)
    try:
        while rclpy.ok():
            values = ScaleFeedback()
            values.Drive = drive_value
            values.ShoulderPitch = arm_values.ShoulderPitch
            values.ShoulderRot = arm_values.ShoulderRot
            values.ElbowPitch = arm_values.ElbowPitch
            values.WristRot = arm_values.WristPitch
            feedback.publish(values)
            time.sleep(5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


# class ScaleManager(Node):
#     def __init__(self):
#         super().__init__('Scale_Manager')

#         # Declare and get parameters
#         self.declare_parameter('ShoulderRot', 1.0)
#         self.declare_parameter('ShoulderPitch', 1.0)
#         self.declare_parameter('ElbowPitch', 1.0)
#         self.declare_parameter('WristPitch', 1.0)
#         self.declare_parameter('WristRot', 1.0)
#         self.declare_parameter('Drive_Sens', 1.0)

#         self.arm_values = ArmData()
#         self.arm_values.ShoulderRot = self.get_parameter('ShoulderRot').get_parameter_value().double_value
#         self.arm_values.ShoulderPitch = self.get_parameter('ShoulderPitch').get_parameter_value().double_value
#         self.arm_values.ElbowPitch = self.get_parameter('ElbowPitch').get_parameter_value().double_value
#         self.arm_values.WristPitch = self.get_parameter('WristPitch').get_parameter_value().double_value
#         self.arm_values.WristRot = self.get_parameter('WristRot').get_parameter_value().double_value
#         self.drive_value = self.get_parameter('Drive_Sens').get_parameter_value().double_value

#         # Publishers
#         self.drive_sens_pub = self.create_publisher(Float64, 'Drive/Drive_Sensitivity', 10)
#         self.arm_sens_pub = self.create_publisher(ArmData, 'Arm/Arm_Sensitivity', 10)
#         self.feedback_pub = self.create_publisher(ScaleFeedback, 'SensFeedback', 10)

#         # Subscribers
#         self.create_subscription(Float64, 'Drive/Drive_Sensitivity', self.drive_cb, 10)
#         self.create_subscription(ArmData, 'Arm/Arm_Sensitivity', self.arm_cb, 10)

#         # Timers
#         self.initial_timer = self.create_timer(5.0, self.initial_publish)
#         self.feedback_timer = self.create_timer(5.0, self.publish_feedback)

#     def initial_publish(self):
#         self.drive_sens_pub.publish(Float64(data=self.drive_value))
#         self.arm_sens_pub.publish(self.arm_values)
#         self.initial_timer.cancel()  # Stop the timer after initial publish

#     def drive_cb(self, msg):
#         self.drive_value = msg.data

#     def arm_cb(self, msg):
#         self.arm_values = msg

#     def publish_feedback(self):
#         values = ScaleFeedback()
#         values.Drive = self.drive_value
#         values.ShoulderPitch = self.arm_values.ShoulderPitch
#         values.ShoulderRot = self.arm_values.ShoulderRot
#         values.ElbowPitch = self.arm_values.ElbowPitch
#         values.WristRot = self.arm_values.WristRot
#         values.WristPitch = self.arm_values.WristPitch
#         self.feedback_pub.publish(values)

# def main(args=None):
#     rclpy.init(args=args)
#     scale_manager = ScaleManager()
#     rclpy.spin(scale_manager)
#     scale_manager.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()