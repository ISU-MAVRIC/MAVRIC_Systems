from mavric_msg.msg import Arm, ArmScales
from utils.SparkCANLib.SparkCAN import SparkBus
#from adafruit_servokit import ServoKit
from typing import Optional

# CAN IDs for Drive Controllers
shoulder_pitch = 11
shoulder_rot = 12
elbow_pitch = 13
wrist_pitch = 14
wrist_rot  = 15
claw = 1 # PWM Controller BUS

INVERTED = -1

# Arm Scales
c_ShoulderPitch = 1         # Define individual arm rates
c_ShoulderRot = 1           # If one axis is faster/slower than the others, change these values
c_ElbowPitch = 1
c_WristPitch = 1
c_WristRot = 1

# Arm Directions
c_ShoulderRotDir = 1        
c_ShoulderPitchDir = 1     # If axis is moving wrong way, invert these 
c_ElbowPitchDir = 1
c_WristPitchDir = 1
c_WristRotDir = -1


class ArmControl:
    """
    Uses the CANbus interface to set the velocity of the motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """

    def __init__(self, bus: SparkBus):
        self.bus = bus
        #self.kit = ServoKit(channels=16)
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

        self.SPMotor.percent_output(msg.shoulder_pitch * c_ShoulderPitch * c_ShoulderPitchDir/100)
        self.SRMotor.percent_output(msg.shoulder_rot * c_ShoulderRot * c_ShoulderRotDir/100)
        self.EPMotor.percent_output(msg.elbow_pitch * c_ElbowPitch * c_ElbowPitchDir/100)
        self.WPMotor.percent_output(msg.wrist_pitch * c_WristPitch * c_WristPitchDir/100)
        self.WRMotor.percent_output(msg.wrist_rot * c_WristRot * c_WristRotDir/100)
        #self.kit.continuous_servo[1].throttle = msg.claw
    
    def set_scale(self, msg: ArmScales) -> None:
        global c_ShoulderPitch, c_ShoulderRot, c_ElbowPitch, c_WristPitch, c_WristRot
        c_ShoulderPitch = msg.shoulder_pitch
        c_ShoulderRot = msg.shoulder_rot
        c_ElbowPitch = msg.elbow_pitch
        c_WristPitch = msg.wrist_pitch
        c_WristRot = msg.wrist_rot  
        
