from SparkCANLib import SparkController, SparkCAN

bus = SparkCAN.SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

motor = bus.init_controller(1)
motor2 = bus.init_controller(6)
motor3 = bus.init_controller(5)
motor4 = bus.init_controller(3)

motor.percent_output(0.40)
motor2.percent_output(-0.40)
motor3.percent_output(0.40)
motor4.percent_output(-0.40)

while True:
    pass



# import rev

# motor = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
# motor.set(0.5)  # run at 50% power