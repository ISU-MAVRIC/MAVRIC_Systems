from adafruit_lib import PCA9685

channel = 1
frequency = 50
address = 0x40
period = 1. / frequency
clk_error = 1.04166667

pwm = PCA9685.PCA9685(address, simulation=True)
pwm.set_pwm_freq(frequency / clk_error)


percent = 0.0 # time / period
pwm.set_pwm(channel, percent, int(percent*4095+0.5))  # 1.5ms