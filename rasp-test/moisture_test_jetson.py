import adafruit_platformdetect.board as ap_board
import adafruit_blinka.microcontroller.nvidia.jetsonnano.pin as pin
import busio
from adafruit_ms8607 import MS8607

i2c = busio.I2C(pin.SCL, pin.SDA)
sensor = MS8607(i2c)

print("Pressure: %.2f hPa" % sensor.pressure)
print("Temperature: %.2f C" % sensor.temperature)
print("Humidity: %.2f %% rH" % sensor.relative_humidity)