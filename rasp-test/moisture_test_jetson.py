import adafruit_bus_device.i2c_device as i2c_device
i2c_device.I2CDevice.__probe_for_device = lambda self: None  # disable zero-length write check

import board, busio
from adafruit_ms8607 import MS8607

i2c = busio.I2C(board.SCL, board.SDA)  # Jetson uses /dev/i2c-1
sensor = MS8607(i2c)

print("Pressure:", sensor.pressure, "hPa")
print("Temperature:", sensor.temperature, "°C")
print("Humidity:", sensor.relative_humidity, "% rH")
