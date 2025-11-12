import adafruit_bus_device.i2c_device as i2c_device

# Properly override BEFORE any instance is created
def _no_probe(self):
    pass
i2c_device.I2CDevice._I2CDevice__probe_for_device = _no_probe  # override the mangled private name

import board, busio
from adafruit_ms8607 import MS8607

i2c = busio.I2C(board.SCL, board.SDA)
sensor = MS8607(i2c)

print("Pressure: %.2f hPa" % sensor.pressure)
print("Temperature: %.2f C" % sensor.temperature)
print("Humidity: %.2f %% rH" % sensor.relative_humidity)
