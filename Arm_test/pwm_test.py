from adafruit_lib import PCA9685

channel = 0 # channel number servo is pluged into on PCA9685
busnum = 1 # i2c bus number
frequency = 50 # default frequency for servos
address = 0x40 # default address for PCA9685
period = 1. / frequency
clk_error = 1.04166667

pwm = PCA9685.PCA9685(address, busnum=busnum)
pwm.set_pwm_freq(frequency / clk_error)


percent = 0.2 # time / period
pwm.set_pwm(channel, percent, int(percent*4095+0.5))  # 1.5ms