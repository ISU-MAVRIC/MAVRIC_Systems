#IMUTest.py
#This code was used to test an Adafruit BNO055 IMU with a Raspberry Pi
#Author: MAVRIC Team
#Date: 12/3/2025

import time
import board
import busio
import adafruit_bno055

# Initialize I2C and IMU
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("BNO055 detected. Reading data...")

while True:
    accel = sensor.acceleration
    gyro = sensor.gyro
    mag = sensor.magnetic
    euler = sensor.euler

    print("Acceleration:", accel)
    print("Gyroscope:", gyro)
    print("Magnetometer:", mag)
    print("Euler Angles:", euler)
    print("-" * 40)

    time.sleep(3)
