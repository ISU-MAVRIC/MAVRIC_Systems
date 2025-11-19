#!/usr/bin/python3

"""
GPS_control.py

Desc: ROS2 node for interfacing with GPS hardware and publishing data to topic(s).
Author: MAVRIC Team
Date: 2025-11-02
"""

import rclpy, serial
from mavric_msg.msg import GPS
from std_msgs.msg import Bool
from rclpy.node import Node
from ublox_gps import UbloxGps
#must have spidev installed for '/dev/ttyACM0' to work
#though spidev is linux only...

#Will use a timer to publish GPS coords at a frequency, can update later to 
#use "serial-read-driven publishing" where a message is published everytime a full coordinate of data is
#read from the GPS.
class GPSControlNode(Node):
    def __init__(self) -> None:
        super().__init__("GPS_control")

        #Publishers
        self.gps_publisher = self.create_publisher(GPS, "GPS_Data", 10)
        self.fix_pub = self.create_publisher(Bool, 'GPS_Fix', 10)
        
        #Open serial port
        self.port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
        gps = UbloxGps(self.port)

        #Timer to publish at 1Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("GPS node started, publishing at 1Hz")


def timer_callback(self):
    try:
            geo = self.gps.geo_coords()
            time = self.gps.date_time()

            msg = GPS()
            msg.latitude = geo.lat
            msg.longitude = geo.lon
            msg.altitude = geo.height
            msg.speed = geo.gSpeed
            msg.heading = geo.headMot
            msg.num_satellites = geo.numSV
            msg.time_h = int(time.hour)
            msg.time_m = int(time.min)
            msg.time_s = float(time.sec)

            # Publish fix OK or not
            fix = Bool()
            fix.data = (geo.flags.gnssFixOK != 0)
            self.fix_pub.publish(fix)

            # Publish GPS message
            self.gps_pub.publish(msg)

    except Exception as err:
            self.get_logger().warn(f"GPS read error: {err}")


def main(args=None):
    rclpy.init(args=args)
    #instantiation
    gps = GPSControlNode()
    rclpy.spin(gps)
    gps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
