from SparkCANLib import SparkController, SparkCAN

bus = SparkCAN.SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

arm_motor = bus.init_controller(5)

arm_motor.percent_output(0.05)