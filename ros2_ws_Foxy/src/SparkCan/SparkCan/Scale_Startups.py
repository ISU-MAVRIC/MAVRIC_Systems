# scale_startups.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavric_msgs.msg import ArmData, ScaleFeedback  # Replace with actual message imports

class ScaleManager(Node):
    def __init__(self):
        super().__init__('Scale_Manager')

        # Declare and get parameters
        self.declare_parameter('ShoulderRot', 1.0)
        self.declare_parameter('ShoulderPitch', 1.0)
        self.declare_parameter('ElbowPitch', 1.0)
        self.declare_parameter('WristPitch', 1.0)
        self.declare_parameter('WristRot', 1.0)
        self.declare_parameter('Drive_Sens', 1.0)

        self.arm_values = ArmData()
        self.arm_values.ShoulderRot = self.get_parameter('ShoulderRot').get_parameter_value().double_value
        self.arm_values.ShoulderPitch = self.get_parameter('ShoulderPitch').get_parameter_value().double_value
        self.arm_values.ElbowPitch = self.get_parameter('ElbowPitch').get_parameter_value().double_value
        self.arm_values.WristPitch = self.get_parameter('WristPitch').get_parameter_value().double_value
        self.arm_values.WristRot = self.get_parameter('WristRot').get_parameter_value().double_value
        self.drive_value = self.get_parameter('Drive_Sens').get_parameter_value().double_value

        # Publishers
        self.drive_sens_pub = self.create_publisher(Float64, 'Drive/Drive_Sensitivity', 10)
        self.arm_sens_pub = self.create_publisher(ArmData, 'Arm/Arm_Sensitivity', 10)
        self.feedback_pub = self.create_publisher(ScaleFeedback, 'SensFeedback', 10)

        # Subscribers
        self.create_subscription(Float64, 'Drive/Drive_Sensitivity', self.drive_cb, 10)
        self.create_subscription(ArmData, 'Arm/Arm_Sensitivity', self.arm_cb, 10)

        # Timers
        self.initial_timer = self.create_timer(5.0, self.initial_publish)
        self.feedback_timer = self.create_timer(5.0, self.publish_feedback)

    def initial_publish(self):
        self.drive_sens_pub.publish(Float64(data=self.drive_value))
        self.arm_sens_pub.publish(self.arm_values)
        self.initial_timer.cancel()  # Stop the timer after initial publish

    def drive_cb(self, msg):
        self.drive_value = msg.data

    def arm_cb(self, msg):
        self.arm_values = msg

    def publish_feedback(self):
        values = ScaleFeedback()
        values.Drive = self.drive_value
        values.ShoulderPitch = self.arm_values.ShoulderPitch
        values.ShoulderRot = self.arm_values.ShoulderRot
        values.ElbowPitch = self.arm_values.ElbowPitch
        values.WristRot = self.arm_values.WristRot
        values.WristPitch = self.arm_values.WristPitch
        self.feedback_pub.publish(values)

def main(args=None):
    rclpy.init(args=args)
    scale_manager = ScaleManager()
    rclpy.spin(scale_manager)
    scale_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()