from mavric_msg.msg import Arm
from utils.SparkCANLib.SparkCAN import SparkBus
from adafruit_servokit import ServoKit
from typing import Optional

# CAN IDs for Drive Controllers
shoulder_pitch = 11
shoulder_rot = 12
elbow_pitch = 13
wrist_pitch = 14
wrist_rot  = 15
claw = 1 # PWM Controller BUS

INVERTED = -1


class ArmControl:
    """
    Uses the CANbus interface to set the velocity of the motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """

    def __init__(self, bus: SparkBus):
        self.bus = bus
        self.kit = ServoKit(channels=16)
        # Initialize logger before any log calls and keep attribute name consistent

        self.SPMotor = self.bus.init_controller(shoulder_pitch)
        self.SRMotor = self.bus.init_controller(shoulder_rot)
        self.EPMotor = self.bus.init_controller(elbow_pitch)
        self.WPMotor = self.bus.init_controller(wrist_pitch)
        self.WRMotor = self.bus.init_controller(wrist_rot)

    def set_velocity(self, msg: Arm):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (SteerTrain): The values from ROS indicating the velocity of each motor.
        """

        self.SPMotor.percent_output(msg.shoulder_pitch)
        self.SRMotor.percent_output(msg.shoulder_rot)
        self.EPMotor.percent_output(msg.elbow_pitch)
        self.WPMotor.percent_output(msg.wrist_pitch)
        self.WRMotor.percent_output(INVERTED * msg.wrist_rot)
        self.kit.continuous_servo[1].throttle = msg.claw
        
