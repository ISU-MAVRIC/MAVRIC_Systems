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
        
        # Initialize arm motors
        self.SPMotor = self.bus.init_controller(shoulder_pitch)
        self.SRMotor = self.bus.init_controller(shoulder_rot)
        self.EPMotor = self.bus.init_controller(elbow_pitch)
        self.WPMotor = self.bus.init_controller(wrist_pitch)
        self.WRMotor = self.bus.init_controller(wrist_rot)
        
        # Initialize servo positions
        self.current_claw_position = 0
        self.current_luminometer_position = 0
        self.current_lumibutton_position = 0
        self.current_lumilid_position = 0
        self.current_cache_position = 0

    def set_velocity(self, msg: Arm):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (Arm): The values from ROS indicating the velocity/position of each component.
        """
        try:
            # Set motor velocities (convert from -100/100 range to -1/1 for percent output)
            if hasattr(msg, 'shoulder_pitch'):
                self.SPMotor.percent_output(msg.shoulder_pitch / 100.0)
            
            if hasattr(msg, 'shoulder_rot'):
                self.SRMotor.percent_output(msg.shoulder_rot / 100.0)
            
            if hasattr(msg, 'elbow_pitch'):
                self.EPMotor.percent_output(msg.elbow_pitch / 100.0)
            
            if hasattr(msg, 'wrist_pitch'):
                self.WPMotor.percent_output(msg.wrist_pitch / 100.0)
            
            if hasattr(msg, 'wrist_rot'):
                self.WRMotor.percent_output(INVERTED * msg.wrist_rot / 100.0)
            
            # Set servo positions
            if hasattr(msg, 'claw'):
                # Map from -100/100 to 0/1 for servo throttle
                claw_throttle = (msg.claw + 100) / 200.0
                self.kit.continuous_servo[claw].throttle = claw_throttle
            
            if hasattr(msg, 'luminometer'):
                # Assuming luminometer is a standard servo (0-180 degrees)
                self.kit.servo[0].angle = max(0, min(180, msg.luminometer + 90))
            
            if hasattr(msg, 'lumibutton'):
                self.kit.servo[1].angle = max(0, min(180, msg.lumibutton + 90))
            
            if hasattr(msg, 'lumilid'):
                self.kit.servo[2].angle = max(0, min(180, msg.lumilid + 90))
            
            if hasattr(msg, 'cache'):
                self.kit.servo[3].angle = max(0, min(180, msg.cache + 90))
            
            if hasattr(msg, 'drill'):
                # Drill speed control
                drill_speed = max(-1.0, min(1.0, msg.drill / 100.0))
                self.kit.continuous_servo[4].throttle = drill_speed
            
            if hasattr(msg, 'drillactuator'):
                # Drill actuator control
                drill_actuator_speed = max(-1.0, min(1.0, msg.drillactuator / 100.0))
                self.kit.continuous_servo[5].throttle = drill_actuator_speed
                
        except Exception as e:
            print(f"Error in arm control: {e}")