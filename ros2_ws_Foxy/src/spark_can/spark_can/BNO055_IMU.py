#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
'''
import adafruit_bno055 as BNO055
import board
'''
i2c = board.I2C()
bno = BNO055.BNO055_I2C(i2c)

pwm_offset_ms = 0
HEADING_OFFSET = 0

def angle_to_ms(angle):
    # convert from (-90 to 90) to about (1ms to 2ms)
    percent = (angle / 180.0) + 0.5
    return ((1 + percent) * 0.001) + (pwm_offset_ms * 0.001)

def servo_cal_roll(pub_servo, node):
    # allow IMU to auto-calibrate roll angle
    pub_servo.publish(Float64(data=angle_to_ms(-60)))
    node.get_clock().sleep(rclpy.duration.Duration(seconds=0.5))

    pub_servo.publish(Float64(data=angle_to_ms(60)))
    node.get_clock().sleep(rclpy.duration.Duration(seconds=0.5))

    pub_servo.publish(Float64(data=angle_to_ms(0)))
    node.get_clock().sleep(rclpy.duration.Duration(seconds=0.5))

class BNO055IMUPublisher(Node):
    def __init__(self):
        super().__init__('BNO055_IMU')
        global pwm_offset_ms
        pwm_offset_ms = self.declare_parameter("pwm_offset_ms", 0).get_parameter_value().double_value

        self.pub_sys_cal = self.create_publisher(Float64, "IMU/SysCalibration", 10)
        self.pub_cal = self.create_publisher(Vector3, "IMU/SensorCalibrations", 10)
        self.pub_angle = self.create_publisher(Vector3, "IMU/FusedAngle", 10)
        self.pub_servo = self.create_publisher(Float64, "IMU/IMU_Cal_Servo", 10)

        self.rate = self.create_rate(10)

        # Servo calibration
        servo_cal_roll(self.pub_servo, self)

        self.cal = True
        self.timer = self.create_timer(0.1, self.publish_imu_data)

    def publish_imu_data(self):
        global HEADING_OFFSET

        # Obtain calibration status and Euler angles
        sys_cal, gyro_cal, accel_cal, mag_cal = bno.calibration_status
        yaw, pitch, roll = bno.euler

        if self.cal and yaw is not None:
            HEADING_OFFSET = -yaw
            self.cal = False
        
        yaw += HEADING_OFFSET
        yaw %= 360

        # Publish calibration status and angles
        self.pub_sys_cal.publish(Float64(data=sys_cal))
        self.pub_cal.publish(Vector3(x=gyro_cal, y=accel_cal, z=mag_cal))
        self.pub_angle.publish(Vector3(x=roll, y=pitch, z=yaw))

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = BNO055IMUPublisher()
    
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
