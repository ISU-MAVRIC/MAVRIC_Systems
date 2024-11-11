#!/usr/bin/env python3

# https://github.com/sparkfun/Qwiic_Ublox_Gps_Py/tree/master/examples

import rclpy
from rclpy.node import Node
import serial
#changed from "mavric.msg" to "SparkCan.msg"
from SparkCan.msg import GPS
from std_msgs.msg import Bool
from ublox_gps import UbloxGps

class GPSStreamer(Node):
    def __init__(self):
        super().__init__('GPS_Streamer')
        
        # Publishers
        self.gps_pub = self.create_publisher(GPS, "GPS_Data", 10)
        self.fix_pub = self.create_publisher(Bool, "GPS_Fix", 10)
        
        # Set the publish rate to 1 Hz
        self.timer = self.create_timer(1.0, self.publish_gps_data)
        
        # Initialize the GPS
        self.port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=10)
        self.gps = UbloxGps(self.port)
        
    def publish_gps_data(self):
        try:
            # Retrieve GPS data
            geo = self.gps.geo_coords()
            time = self.gps.date_time()
            
            # Prepare GPS message
            gps_msg = GPS()
            gps_msg.latitude = geo.lat
            gps_msg.longitude = geo.lon
            gps_msg.altitude = geo.height
            gps_msg.speed = geo.gSpeed
            gps_msg.heading = geo.headMot
            gps_msg.num_satellites = geo.numSV
            gps_msg.time_h = int(time.hour)
            gps_msg.time_m = int(time.min)
            gps_msg.time_s = float(time.sec)
            
            # Check for GPS fix
            fix_status = Bool()
            fix_status.data = geo.flags.gnssFixOK != 0
            
            # Publish messages
            self.fix_pub.publish(fix_status)
            self.gps_pub.publish(gps_msg)

        except (ValueError, IOError) as err:
            self.get_logger().error(f"GPS error: {err}")

    def destroy_node(self):
        # Close the serial port on shutdown
        self.port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gps_streamer = GPSStreamer()
    
    try:
        rclpy.spin(gps_streamer)
    except KeyboardInterrupt:
        pass
    finally:
        gps_streamer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
