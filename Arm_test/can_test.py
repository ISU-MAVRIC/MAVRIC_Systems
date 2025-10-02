from SparkCANLib import SparkController, SparkCAN

bus = SparkCAN.SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

motor = bus.init_controller(15)

motor.percent_output(-0.40)

while True:
    pass



# import rev

# motor = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
# motor.set(0.5)  # run at 50% power