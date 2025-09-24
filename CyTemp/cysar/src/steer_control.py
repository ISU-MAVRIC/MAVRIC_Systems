from cysar.msg import SteerTrain
from SparkCANLib.SparkCAN import SparkBus
import rospy

# CAN IDs for Drive Controllers
FLS = 7
FRS = 10
BLS = 9
BRS = 8

INVERTED = -1

class SteerControl():
    """
    Uses the CANbus interface to set the velocity of the motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """
    def __init__(self, bus : SparkBus):
        self.bus = bus
        self.FLMotor = self.bus.init_controller(FLS)
        rospy.loginfo("Initialized Front Left Steer (FLS) controller with CAN ID %d", FLS)
        self.FRMotor = self.bus.init_controller(FRS)
        rospy.loginfo("Initialized Front Right Steer (FRS) controller with CAN ID %d", FRS)
        self.BLMotor = self.bus.init_controller(BLS)
        rospy.loginfo("Initialized Back Left Steer (BLS) controller with CAN ID %d", BLS)
        self.BRMotor = self.bus.init_controller(BRS)
        rospy.loginfo("Initialized Back Right Steer (BRS) controller with CAN ID %d", BRS)

    def set_velocity(self, msg : SteerTrain):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (DriveTrain): The values from ROS indicating the velocity of each motor.
        """
        rospy.loginfo("Setting Front Left Motor velocity to %f", msg.front_left)
        self.FLMotor.percent_output(msg.front_left)
        rospy.loginfo("Setting Front Right Motor velocity to %f", msg.front_right)
        self.FRMotor.percent_output(msg.front_right)
        rospy.loginfo("Setting Back Left Motor velocity to %f (inverted: %f)", msg.back_left, INVERTED * msg.back_left)
        self.BLMotor.percent_output(INVERTED * msg.back_left)
        rospy.loginfo("Setting Back Right Motor velocity to %f (inverted: %f)", msg.back_right, INVERTED * msg.back_right)
        self.BRMotor.percent_output(INVERTED * msg.back_right)
