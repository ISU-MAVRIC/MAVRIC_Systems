from SparkCANLib import SparkController, SparkCAN

bus = SparkCAN.SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

arm_motor = bus.init_controller(15)
# l_motor = bus.init_controller(1)

while (True):
    arm_motor.percent_output(0.40)

    # print (arm_motor.position)

    # l_motor.percent_output(0.05)