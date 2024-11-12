#!/usr/bin/env python3
'''
Description: Reads arm and Drive topics and controlls the relevant motors
Original Authors: Jacob Peskuski, Gabe Carlson, Nathan Logston
Ported to ROS 2 by: ...

###PLAN FOR IMPROVEMENT AND PORTING###
    -Migrate to a class based system and node declaration
    -Remove unnecessary global variables
    -Add more comments and documentation
    -Refactor code to remove duplicate code

Topics:
    Publishers:
        SteerPosition
        JointPosition
        JointVelocity
    Subscribers:
        Drive_Train
        Steer_Train
        Drive_Sensitivity
        ShoulderPitch
        ShoulderRot
        ElbowPitch
        WristPitch
        WristRot
        Arm_Sensitivity
'''


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Float64
from msg_drive.msg import Steer, Drivetrain, Steertrain, ArmData
from spark_can.spark_can.spark_can import SparkBus



# Drive Scales
c_Scale_Max = 1.2*20
c_Scale = c_Scale_Max
c_str_Scale = 0.15
# Arm Scales
c_ShoulderPitch = 1         # Define individual arm rates
c_ShoulderRot = 1           # If one axis is faster/slower than the others, change these values
c_ElbowPitch = 1
c_WristPitch = 1
c_WristRot = 1
# Science Scales
c_Drill = 1
c_DrillActuator = 1

# Drive Directions
c_lfDir = 1
c_lmDir = 1
c_lbDir = 1
c_rfDir = -1
c_rmDir = -1
c_rbDir = -1
c_str_lfDir = 1
c_str_lbDir = 1
c_str_rfDir = 1
c_str_rbDir = 0.9
# Arm Directions
c_ShoulderRotDir = 1        
c_ShoulderPitchDir = 1     # If axis is moving wrong way, invert these 
c_ElbowPitchDir = -1
c_WristPitchDir = -1
c_WristRotDir = 1
# Science Directions
c_DrillDir = -1
c_DrillActuatorDir = 1

#set up globals for spark outputs. These should be zero
lf = 0
lm = 0
lb = 0
rf = 0
rm = 0
rb = 0
slf = 0
slb = 0
srf = 0
srb = 0
ShoulderRot = 0
ShoulderPitch = 0
ElbowPitch = 0
WristPitch = 0
WristRot = 0
Drill = 0
DrillActuator = 0


# Setup Sparkmax on can bus
sparkBus = SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)
# Wheels
spark_lf = sparkBus.init_controller(1)
spark_lm = sparkBus.init_controller(2)
spark_lb = sparkBus.init_controller(3)
spark_rf = sparkBus.init_controller(4)
spark_rm = sparkBus.init_controller(5)
spark_rb = sparkBus.init_controller(6)
# Steering
spark_str_lf = sparkBus.init_controller(7)
spark_str_lb = sparkBus.init_controller(8)
spark_str_rf = sparkBus.init_controller(9)
spark_str_rb = sparkBus.init_controller(10)
# Arm
spark_shoulderPitch = sparkBus.init_controller(11)
spark_shoulderRot = sparkBus.init_controller(12)
spark_elbowPitch = sparkBus.init_controller(13)
spark_wristPitch = sparkBus.init_controller(14)
spark_wristRot = sparkBus.init_controller(15)
# Science
spark_Drill = sparkBus.init_controller(16)
spark_DrillActuator = sparkBus.init_controller(17)


# Axis Feedback Publishers
def strpub():
  steerMsg = Steer()
  steerMsg.lf = int(spark_str_lf.position)
  steerMsg.lb = int(spark_str_lb.position)
  steerMsg.rf = int(spark_str_rf.position)
  steerMsg.rb = int(spark_str_rb.position)
  str_pub.publish(steerMsg)

def arm_feedback():
    Pos_msg = ArmData()
    Vel_msg = ArmData()

    Pos_msg.shoulder_rot = spark_shoulderRot.position
    Pos_msg.shoulder_pitch = spark_shoulderPitch.position
    Pos_msg.elbow_pitch = spark_elbowPitch.position
    Pos_msg.wrist_pitch = spark_wristPitch.position
    Pos_msg.wrist_rot = spark_wristRot.position

    Vel_msg.shoulder_rot = spark_shoulderRot.velocity
    Vel_msg.shoulder_pitch = spark_shoulderPitch.velocity
    Vel_msg.elbow_pitch = spark_elbowPitch.velocity
    Vel_msg.wrist_pitch = spark_wristPitch.velocity
    Vel_msg.wrist_rot = spark_wristRot.velocity

    Pos_pub.publish(Pos_msg)
    Vel_pub.publish(Vel_msg)
  

'''
### CALLBACK FUNCTIONS ###
Each callback function is run whenever a topic changes it's value.
When the value changes, each callback function updates its relevant axis output.
Then as a safety measure, makes sure the percent out is between the values -100 and 100.
'''
def strCallback(data):
  global slf, slb, srf, srb
  slf = data.str_lf
  if (slf > 100):
    slf = 100
  if (slf < -100):
    slf = -100

  slb = data.str_lb
  if (slb > 100):
    slb = 100
  if (slb < -100):
    slb = -100

  srf = data.str_rf
  if (srf > 100):
    srf = 100
  if (srf < -100):
    srf = -100

  srb = data.str_rb
  if (srb > 100):
    srb = 100
  if (srb < -100):
    srb = -100


def driveCallback(data):
	global lf, lm, lb, rf, rm, rb
	lf = data.lf
	lm = data.lm
	lb = data.lb
	rf = data.rf
	rm = data.rm
	rb = data.rb
	print(lf, lm, lb, rf, rm, rb)

	if (lf > 100):
		lf = 100
	if (lf < -100):
		lf = -100

	if (lm > 100):
		lm = 100
	if (lm < -100):
		lm = -100

	if (lb > 100):
		lb = 100
	if (lb < -100):
		lb = -100

	if (rf > 100):
		rf = 100
	if (rf < -100):
		rf = -100

	if (rm > 100):
		rm = 100
	if (rm < -100):
		rm = -100

	if (rb > 100):
		rb = 100
	if (rb < -100):
		rb = -100

def driveSens_cb(data):
    global c_Scale
    temp = data.data
    if temp > 1.0:
      temp = 1.0
    if temp < 0:
      temp = 0
    c_Scale = c_Scale_Max*temp

def SR_cb(data):
    global ShoulderRot
    ShoulderRot = data.data
    if ShoulderRot > 100:
        ShoulderRot = 100
    if ShoulderRot < -100:
        ShoulderRot = -100

def SP_cb(data):
    global ShoulderPitch
    ShoulderPitch = data.data
    if ShoulderPitch > 100:
        ShoulderPitch = 100
    if ShoulderPitch < -100:
        ShoulderPitch = -100

def EP_cb(data):
    global ElbowPitch
    ElbowPitch = data.data
    if ElbowPitch > 100:
        ElbowPitch = 100
    if ElbowPitch < -100:
        ElbowPitch = -100

def WP_cb(data):
    global WristPitch
    WristPitch = data.data
    if WristPitch > 100:
        WristPitch = 100
    if WristPitch < -100:
        WristPitch = -100

def WR_cb(data):
    global WristRot
    WristRot = data.data
    if WristRot > 100:
        WristRot = 100
    if WristRot < -100:
        WristRot = -100

def armSens_cb(data):
    global c_ShoulderRot, c_ShoulderPitch, c_ElbowPitch, c_WristPitch, c_WristRot
    c_ShoulderRot = data.shoulder_rot
    c_ShoulderPitch = data.shoulder_pitch
    c_ElbowPitch = data.elbow_pitch
    c_WristPitch = data.wrist_pitch
    c_WristRot = data.wrist_rot
    if c_ShoulderRot > 1:
        c_ShoulderRot = 1
    elif c_ShoulderRot < 0:
        c_ShoulderRot = 0

    if c_ShoulderPitch > 1:
        c_ShoulderPitch = 1
    elif c_ShoulderPitch < 0:
        c_ShoulderPitch = 0

    if c_ElbowPitch > 1:
        c_ElbowPitch = 1
    elif c_ElbowPitch < 0:
        c_ElbowPitch = 0

    if c_WristPitch > 1:
        c_WristPitch = 1
    elif c_WristPitch < 0:
        c_WristPitch = 0

    if c_WristRot > 1:
        c_WristRot = 1
    elif c_WristRot < 0:
        c_WristRot = 0

def Drill_cb(data):
    global Drill
    Drill = data.data
    if Drill > 100:
        Drill = 100
    elif Drill < -100:
        Drill = -100

def DrillActuator_cb(data):
    global DrillActuator
    DrillActuator = data.data
    if DrillActuator > 100:
        DrillActuator = 100
    elif DrillActuator < -100:
        DrillActuator = -100

def setOutputs(lf, lm, lb, rf, rm, rb, str_lf, str_lb, str_rf, str_rb):
	spark_lf.velocity_output(lf * c_Scale * c_lfDir)
	spark_lm.velocity_output(lm * c_Scale * c_lmDir)
	spark_lb.velocity_output(lb * c_Scale * c_lbDir)
	spark_rf.velocity_output(rf * c_Scale * c_rfDir)
	spark_rm.velocity_output(rm * c_Scale * c_rmDir)
	spark_rb.velocity_output(rb * c_Scale * c_rbDir)

	spark_str_lf.position_output(str_lf * c_str_lfDir * c_str_Scale)
	spark_str_lb.position_output(str_lb * c_str_lbDir * c_str_Scale)
	spark_str_rf.position_output(str_rf * c_str_rfDir * c_str_Scale)
	spark_str_rb.position_output(str_rb * c_str_rbDir * c_str_Scale)

'''
### SNAPPING FUNCTION ###
This function is used in science system to "snap" the swab holder's position 
whenever it gets close to some specified swab positions.
'''   
def snap_function(publisher,input):
    snap_points = [9.405, 4.214, -1.07, -5.714, -10.214, -15, -19.19, -23.618]
    if input == 0:
        closest = min(snap_points, key=lambda x:abs(x-publisher.position))
        if abs(publisher.position - closest) < 3:
            publisher.position_output(closest)
        else:
            publisher.percent_output(input)
    else:
        publisher.percent_output(input)


def talker(args=None):

    # Declare global variables, TODO Question: Why are these global?
    global str_pub, lf, lm, lb, rf, rm, rb, c_Scale, c_str_Scale
    global c_lfDir, c_lmDir, c_lbDir, c_rfDir, c_rmDir, c_rbDir
    global ShoulderRot, ShoulderPitch, ElbowPitch, WristPitch, WristRot
    global Pos_pub, Vel_pub
    global snapping, snap_points, snap_status
    
    # Initialize the node
    rclpy.init(args=args)
    node = rclpy.create_node('CAN_ATS')
    node.get_logger().info('Created CAN_ATS node')

    # 
    """
        Declare parameters, and get values

        Format:
                <variable> = node.declare_parameter(
                            '<parameter>',
                            <default value>
                            ).value

                assert isinstance(
                        <variable>, 
                        <type>)
    """
    snappingValue = node.declare_parameter('~snapping', 0.0).value > 0
    assert isinstance(snappingValue, int)
    snapping = True if snappingValue > 0 else False

    """
        Subscribers for drivetrain
        
        Format:
                node.create_subscription(
                    <message type>,
                    <topic name>,
                    <callback function>,
                    <QoS profile>)
                
                node.create_subscription(
                    <message type>,
                    <topic name>,
                    <callback function>,
                    <queue_size>)

    """
    sub = node.create_subscription(Drivetrain, 'Drive_Train', driveCallback, 10)
    str_sub = node.create_subscription(Steertrain, 'Steer_Train', strCallback, 10)
    drive_sens = node.create_subscription(Float64, 'Drive/Drive_Sensitivity', drive_sens_cb, 10)
    
    """
        Create Publisher for Drive/Steer_Feedback

        Format:
                node.create_publisher(
                    <message type>,
                    <topic name>,
                    <QoS profile>)
                
                node.create_publisher(
                    <message type>,
                    <topic name>,
                    <queue_size>)
    """
    str_pub = node.create_publisher(Float64, 'Drive/Steer_Feedback', 10)


    # Subscribers for Armtrain
    SR_sub = node.create_subscription(Float64, 
                                      "Arm/ShoulderRot", 
                                      SR_cb, 
                                      10)
    SP_sub = node.create_subscription(Float64, "Arm/ShoulderPitch", SP_cb, 10)
    EP_sub = node.create_subscription(Float64, "Arm/ElbowPitch", EP_cb, 10)
    WP_sub = node.create_subscription(Float64, "Arm/WristPitch", WP_cb, 10)
    WR_sub = node.create_subscription(Float64, "Arm/WristRot", WR_cb, 10)
    arm_sens = node.create_subscription(ArmData, "Arm/Arm_Sensitivity", armSens_cb, 10)
    Pos_pub = node.create_publisher(ArmData, 
                                    "Arm/JointPosition", 
                                    10)
    Vel_pub = node.create_publisher(ArmData, "Arm/JointVelocity", 10)

    # Subscribers for Drill and Drill actuator for science.
    Drill_sub = node.create_subscriptionubscriber(Float64, "Science/Drill", Drill_cb, queue_size=10)
    DrillActuator_sub = node.create_subscription(Float64, "Science/DrillActuator", DrillActuator_cb, queue_size=10)


    setOutputs(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    # Create a rate object to control the loop rate
    rosRate = node.create_rate(30)

    # Loop until the node is shutdown
    while rclpy.ok():
        rclpy.spin_once(node)
        setOutputs(lf, lm, lb, rf, rm, rb, slf, slb, srf, srb)
        strpub()
        spark_shoulderRot.percent_output(c_ShoulderRot * ShoulderRot * c_ShoulderRotDir/100)
        spark_shoulderPitch.percent_output(c_ShoulderPitch * ShoulderPitch * c_ShoulderPitchDir/100)
        spark_elbowPitch.percent_output(c_ElbowPitch * ElbowPitch * c_ElbowPitchDir/100)
        spark_wristPitch.percent_output(c_WristPitch * WristPitch * c_WristPitchDir/100)
        if snapping:
            snap_function(spark_wristRot, c_WristRot * WristRot * c_WristRotDir/100)
        else:
            spark_wristRot.percent_output(c_WristRot * WristRot * c_WristRotDir/100)
        spark_Drill.percent_output(c_Drill * Drill * c_DrillDir/100)
        spark_DrillActuator.percent_output(c_DrillActuator * DrillActuator * c_DrillActuator/100)
        
        arm_feedback()
        rosRate.sleep()


""" class SparkCANDriveTrainTalker(Node):
    def __init__(self):
        super().__init__('CAN_ATS')

        snappingValue = self.declare_parameter('~snapping', 0.0).value > 0
        assert isinstance(snappingValue, int)
        snapping = True if snappingValue > 0 else False
        
        #Subscribers for drivetrain
        self.sub = self.create_subscription(Drivetrain, 'Drive_Train', driveCallback, 10)
        self.str_sub = self.create_subscription(Steertrain, 'Steer_Train', strCallback, 10)
        self.drive_sens = self.create_subscription(Float64, 'Drive/Drive_Sensitivity', drive_sens_cb, 10)
        self.str_pub = self.create_publisher(Float64, 'Drive/Steer_Feedback', 10)

        # Subscribers for Armtrain
        self.SR_sub = self.create_subscription(Float64, "Arm/ShoulderRot", SR_cb, 10)
        self.SP_sub = self.create_subscription(Float64, "Arm/ShoulderPitch", SP_cb, 10)
        self.EP_sub = self.create_subscription(Float64, "Arm/ElbowPitch", EP_cb, 10)
        self.WP_sub = self.create_subscription(Float64, "Arm/WristPitch", WP_cb, 10)
        self.WR_sub = self.create_subscription(Float64, "Arm/WristRot", WR_cb, 10)
        self.arm_sens = self.create_subscription(ArmData, "Arm/Arm_Sensitivity", armSens_cb, 10)
        self.Pos_pub = self.create_publisher(ArmData, "Arm/JointPosition", 10)
        self.Vel_pub = self.create_publisher(ArmData, "Arm/JointVelocity", 10)

        # Subscribers for Drill and Drill actuator for science.
        self.Drill_sub = self.create_subscription(Float64, "Science/Drill", Drill_cb, 10)
        self.DrillActuator_sub = self.create_subscription(Float64, "Science/DrillActuator", DrillActuator_cb, 10)

        setOutputs(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


        timer_period = 1.0 / 30.0 # 30 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        setOutputs(lf, lm, lb, rf, rm, rb, slf, slb, srf, srb)
        strpub()
        spark_shoulderRot.percent_output(c_ShoulderRot * ShoulderRot * c_ShoulderRotDir/100)
        spark_shoulderPitch.percent_output(c_ShoulderPitch * ShoulderPitch * c_ShoulderPitchDir/100)
        spark_elbowPitch.percent_output(c_ElbowPitch * ElbowPitch * c_ElbowPitchDir/100)
        spark_wristPitch.percent_output(c_WristPitch * WristPitch * c_WristPitchDir/100)
        if snapping:
            snap_function(spark_wristRot, c_WristRot * WristRot * c_WristRotDir/100)
        else:
            spark_wristRot.percent_output(c_WristRot * WristRot * c_WristRotDir/100)
        spark_Drill.percent_output(c_Drill * Drill * c_DrillDir/100)
        spark_DrillActuator.percent_output(c_DrillActuator * DrillActuator * c_DrillActuator/100)
        
        arm_feedback()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.i += 1 """

if __name__ == "__main___":
    talker()
