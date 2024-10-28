#!/usr/bin/env python3
#!/usr/bin/env python3

'''
Description: Reads arm and Drive topics and controls the relevant motors
Author: Jacob Peskuski, Gabe Carlson, Nathan Logston (converted to ROS 2)
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavric_interfaces.msg import Steer, Drivetrain, Steertrain, ArmData
from SparkCAN import SparkBus

# Drive Scales
c_Scale_Max = 1.2 * 20
c_Scale = c_Scale_Max
c_str_Scale = 0.15
# Arm Scales
c_ShoulderPitch = 1
c_ShoulderRot = 1
c_ElbowPitch = 1
c_WristPitch = 1
c_WristRot = 1

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
c_ShoulderPitchDir = 1
c_ElbowPitchDir = -1
c_WristPitchDir = -1
c_WristRotDir = 1

# Science Directions
c_DrillDir = -1
c_DrillActuatorDir = 1

# Initial global variables for spark outputs
lf = lm = lb = rf = rm = rb = 0
slf = slb = srf = srb = 0
ShoulderRot = ShoulderPitch = ElbowPitch = WristPitch = WristRot = 0
Drill = DrillActuator = 0

# Setup Sparkmax on CAN bus
sparkBus = SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

# Controllers
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


class CAN_DTS_Node(Node):

    def __init__(self):
        super().__init__('CAN_DTS')

        # Create publishers
        self.pos_pub = self.create_publisher(ArmData, 'Arm/JointPosition', 10)
        self.vel_pub = self.create_publisher(ArmData, 'Arm/JointVelocity', 10)
        self.str_pub = self.create_publisher(Steer, 'Drive/Steer_Feedback', 10)

        # Create subscribers for drivetrain
        self.create_subscription(Drivetrain, 'Drive_Train', self.drive_callback, 10)
        self.create_subscription(Steertrain, 'Steer_Train', self.steer_callback, 10)
        self.create_subscription(Float64, 'Drive/Drive_Sensitivity', self.drive_sensitivity_callback, 10)

        # Create subscribers for Armtrain
        self.create_subscription(Float64, 'Arm/ShoulderRot', self.sr_callback, 10)
        self.create_subscription(Float64, 'Arm/ShoulderPitch', self.sp_callback, 10)
        self.create_subscription(Float64, 'Arm/ElbowPitch', self.ep_callback, 10)
        self.create_subscription(Float64, 'Arm/WristPitch', self.wp_callback, 10)
        self.create_subscription(Float64, 'Arm/WristRot', self.wr_callback, 10)
        self.create_subscription(ArmData, 'Arm/Arm_Sensitivity', self.arm_sensitivity_callback, 10)

        # Create subscribers for Science (Drill and Drill actuator)
        self.create_subscription(Float64, 'Science/Drill', self.drill_callback, 10)
        self.create_subscription(Float64, 'Science/DrillActuator', self.drill_actuator_callback, 10)

        self.timer = self.create_timer(1 / 30, self.control_loop)  # 30Hz

    def control_loop(self):
        # Drive outputs
        set_outputs(lf, lm, lb, rf, rm, rb, slf, slb, srf, srb)

        # Publish steer positions
        self.publish_steer_feedback()

        # Set arm joint control outputs
        spark_shoulderRot.percent_output(c_ShoulderRot * ShoulderRot * c_ShoulderRotDir / 100)
        spark_shoulderPitch.percent_output(c_ShoulderPitch * ShoulderPitch * c_ShoulderPitchDir / 100)
        spark_elbowPitch.percent_output(c_ElbowPitch * ElbowPitch * c_ElbowPitchDir / 100)
        spark_wristPitch.percent_output(c_WristPitch * WristPitch * c_WristPitchDir / 100)
        spark_wristRot.percent_output(c_WristRot * WristRot * c_WristRotDir / 100)
        spark_Drill.percent_output(c_Drill * Drill * c_DrillDir / 100)
        spark_DrillActuator.percent_output(c_DrillActuator * DrillActuator * c_DrillActuatorDir / 100)

        # Publish arm feedback
        self.arm_feedback()

    def publish_steer_feedback(self):
        steer_msg = Steer()
        steer_msg.lf = int(spark_str_lf.position)
        steer_msg.lb = int(spark_str_lb.position)
        steer_msg.rf = int(spark_str_rf.position)
        steer_msg.rb = int(spark_str_rb.position)
        self.str_pub.publish(steer_msg)

    def arm_feedback(self):
        pos_msg = ArmData()
        vel_msg = ArmData()

        pos_msg.ShoulderRot = spark_shoulderRot.position
        pos_msg.ShoulderPitch = spark_shoulderPitch.position
        pos_msg.ElbowPitch = spark_elbowPitch.position
        pos_msg.WristPitch = spark_wristPitch.position
        pos_msg.WristRot = spark_wristRot.position

        vel_msg.ShoulderRot = spark_shoulderRot.velocity
        vel_msg.ShoulderPitch = spark_shoulderPitch.velocity
        vel_msg.ElbowPitch = spark_elbowPitch.velocity
        vel_msg.WristPitch = spark_wristPitch.velocity
        vel_msg.WristRot = spark_wristRot.velocity

        self.pos_pub.publish(pos_msg)
        self.vel_pub.publish(vel_msg)

    def steer_callback(self, data):
        global slf, slb, srf, srb
        slf = self.limit_value(data.strLf)
        slb = self.limit_value(data.strLb)
        srf = self.limit_value(data.strRf)
        srb = self.limit_value(data.strRb)

    def drive_callback(self, data):
        global lf, lm, lb, rf, rm, rb
        lf = self.limit_value(data.lf)
        lm = self.limit_value(data.lm)
        lb = self.limit_value(data.lb)
        rf = self.limit_value(data.rf)
        rm = self.limit_value(data.rm)
        rb = self.limit_value(data.rb)

    def drive_sensitivity_callback(self, data):
        global c_Scale
        c_Scale = c_Scale_Max * self.limit_value(data.data, 1.0)

    def sr_callback(self, data):
        global ShoulderRot
        ShoulderRot = self.limit_value(data.data)

    def sp_callback(self, data):
        global ShoulderPitch
        ShoulderPitch = self.limit_value(data.data)

    def ep_callback(self, data):
        global ElbowPitch
        ElbowPitch = self.limit_value(data.data)

    def wp_callback(self, data):
        global WristPitch
        WristPitch = self.limit_value(data.data)

    def wr_callback(self, data):
        global WristRot
        WristRot = self.limit_value(data.data)

    def arm_sensitivity_callback(self, data):
        global c_ShoulderRot, c_ShoulderPitch, c_ElbowPitch, c_WristPitch, c_WristRot
        c_ShoulderRot = self.limit_value(data.ShoulderRot, 1.0)
        c_ShoulderPitch = self.limit_value(data.ShoulderPitch, 1.0)
        c_ElbowPitch = self.limit_value(data.ElbowPitch, 1.0)
        c_WristPitch = self.limit_value(data.WristPitch, 1.0)
        c_WristRot = self.limit_value(data.WristRot, 1.0)

    def drill_callback(self, data):
        global Drill
        Drill = self.limit_value(data.data)

    def drill_actuator_callback(self, data):
        global DrillActuator
        DrillActuator = self.limit_value(data.data)

    def limit_value(self, value, max_val=100.0):
        return max(-max_val, min(value, max_val))


def set_outputs(lf, lm, lb, rf, rm, rb, slf, slb, srf, srb):
    # Implement output logic for each controller
    spark_lf.percent_output(lf * c_Scale * c_lfDir / 100)
    spark_lm.percent_output(lm * c_Scale * c_lmDir / 100)
    spark_lb.percent_output(lb * c_Scale * c_lbDir / 100)
    spark_rf.percent_output(rf * c_Scale * c_rfDir / 100)
    spark_rm.percent_output(rm * c_Scale * c_rmDir / 100)
    spark_rb.percent_output(rb * c_Scale * c_rbDir / 100)

    spark_str_lf.percent_output(slf * c_str_Scale * c_str_lfDir / 100)
    spark_str_lb.percent_output(slb * c_str_Scale * c_str_lbDir / 100)
    spark_str_rf.percent_output(srf * c_str_Scale * c_str_rfDir / 100)
    spark_str_rb.percent_output(srb * c_str_Scale * c_str_rbDir / 100)


def main(args=None):
    rclpy.init(args=args)
    node = CAN_DTS_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

'''
====================HERE IS THE OLD CODE JIC YOU NEED TO REFERENCE=================

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

    Pos_msg.ShoulderRot = spark_shoulderRot.position
    Pos_msg.ShoulderPitch = spark_shoulderPitch.position
    Pos_msg.ElbowPitch = spark_elbowPitch.position
    Pos_msg.WristPitch = spark_wristPitch.position
    Pos_msg.WristRot = spark_wristRot.position

    Vel_msg.ShoulderRot = spark_shoulderRot.velocity
    Vel_msg.ShoulderPitch = spark_shoulderPitch.velocity
    Vel_msg.ElbowPitch = spark_elbowPitch.velocity
    Vel_msg.WristPitch = spark_wristPitch.velocity
    Vel_msg.WristRot = spark_wristRot.velocity

    Pos_pub.publish(Pos_msg)
    Vel_pub.publish(Vel_msg)
  


### CALLBACK FUNCTIONS ###
Each callback function is run whenever a topic changes it's value.
When the value changes, each callback function updates its relevant axis output.
Then as a safety measure, makes sure the percent out is between the values -100 and 100.

def strCallback(data):
  global slf, slb, srf, srb
  slf = data.strLf
  if (slf > 100):
    slf = 100
  if (slf < -100):
    slf = -100

  slb = data.strLb
  if (slb > 100):
    slb = 100
  if (slb < -100):
    slb = -100

  srf = data.strRf
  if (srf > 100):
    srf = 100
  if (srf < -100):
    srf = -100

  srb = data.strRb
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
    c_ShoulderRot = data.ShoulderRot
    c_ShoulderPitch = data.ShoulderPitch
    c_ElbowPitch = data.ElbowPitch
    c_WristPitch = data.WristPitch
    c_WristRot = data.WristRot
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


### SNAPPING FUNCTION ###
This function is used in science system to "snap" the swab holder's position 
whenever it gets close to some specified swab positions.

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

def talker():
    global str_pub, lf, lm, lb, rf, rm, rb, c_Scale, c_str_Scale
    global c_lfDir, c_lmDir, c_lbDir, c_rfDir, c_rmDir, c_rbDir
    global ShoulderRot, ShoulderPitch, ElbowPitch, WristPitch, WristRot
    global Pos_pub, Vel_pub
    global snapping, snap_points, snap_status
    rospy.init_node("CAN_DTS")
    if rospy.get_param('~Snapping', 0.0) > 0:
        snapping = True
    else:
        snapping = False

    # Subscribers for drivetrain
    sub = rospy.Subscriber("Drive_Train", Drivetrain, driveCallback, queue_size = 10)
    str_sub = rospy.Subscriber("Steer_Train", Steertrain, strCallback, queue_size = 10)
    drive_sens = rospy.Subscriber("Drive/Drive_Sensitivity", Float64, driveSens_cb, queue_size=10)
    str_pub = rospy.Publisher("Drive/Steer_Feedback", Steer, queue_size=10)

    # Subscribers for Armtrain
    SR_sub = rospy.Subscriber("Arm/ShoulderRot", Float64, SR_cb, queue_size=10)
    SP_sub = rospy.Subscriber("Arm/ShoulderPitch", Float64, SP_cb, queue_size=10)
    EP_sub = rospy.Subscriber("Arm/ElbowPitch", Float64, EP_cb, queue_size=10)
    WP_sub = rospy.Subscriber("Arm/WristPitch", Float64, WP_cb, queue_size=10)
    WR_sub = rospy.Subscriber("Arm/WristRot", Float64, WR_cb, queue_size=10)
    arm_sens = rospy.Subscriber("Arm/Arm_Sensitivity", ArmData, armSens_cb, queue_size=10)
    Pos_pub = rospy.Publisher("Arm/JointPosition", ArmData, queue_size=10)
    Vel_pub = rospy.Publisher("Arm/JointVelocity", ArmData, queue_size=10)

    # Subscribers for Drill and Drill actuator for science.
    Drill_sub = rospy.Subscriber("Science/Drill", Float64, Drill_cb, queue_size=10)
    DrillActuator_sub = rospy.Subscriber("Science/DrillActuator", Float64, DrillActuator_cb, queue_size=10)

    setOutputs(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    rosRate = rospy.Rate(30)    # hz

    while not rospy.is_shutdown():
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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
'''