#!/usr/bin/python3

"""
GPS_control.py

Desc: ROS2 node for interfacing with GPS hardware and publishing data to topic(s).
Author: MAVRIC Team
Date: 2025-11-02
"""

import rclpy, serial
from mavric_msg.msg import (messagetypes)
from rclpy.node import Node
from ublox_gps import UbloxGps
#must have spidev installed for '/dev/ttyACM0' to work
#but spidev is linux only...

#Will use a timer to publish GPS coords at a frequency, can update later to 
#use "serial-read-driven publishing" where a message is published everytime a full coordinate of data is
#read from the GPS.
class GPSControlNode(Node):
    def __init__(self) -> None:
        super().__init__("GPS_control")

    port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
    gps = UbloxGps(port)
    #...create publishers and add a timer for pubishing.


def main(args=None):
    rclpy.init(args=args)
    #instantiation
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
