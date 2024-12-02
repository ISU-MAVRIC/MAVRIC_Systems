#!/usr/bin/env python3
'''
Description:
    Reads voltages from analog to digital converter and 
    converts them to raw battery voltage for the drive
    and system batteries.
Author: 
    Nathan Logston

Topics:
    Publishers:
        Voltage_Monitor
            Batt1
            Batt2
    Subscribers:
        None
'''
import rclpy
from rclpy.node import Node
import board
from std_msgs.msg import Float64

#changed "mavric.msg" to "SparkCan.msg"
from msg_sensor.msg import Voltage
# import adafruit_ads1x15.ads1115 as ADS
# from adafruit_ads1x15 import analog_in
import Adafruit_ADS1x15 as ADS

class ADCPublisher(Node):
    def __init__(self):
        super().__init__('ADC_Pub')
        
        # Publishers
        self.volt_pub = self.create_publisher(Voltage, "ADC", 10)
        self.voltperCell_pub = self.create_publisher(Voltage, "ADCpCell", 10)
        
        # Set the publish rate to 0.25 Hz (every 4 seconds)
        self.timer = self.create_timer(4.0, self.publish_voltage)
        
        # Initialize Voltage messages
        self.voltages = Voltage()
        self.voltpcell = Voltage()
        
        # Initialize ADC
        self.adc = ADS.ADS1115(busnum=1)
        
        # Resistors and constants
        self.B1R1 = 5197
        self.B1R2 = 995
        self.B2R1 = 5202
        self.B2R2 = 997
        self.B1C = 6
        self.B2C = 4
        
        # Ratios are V_Bat/V_divider
        self.B1Ratio = (self.B1R1 + self.B1R2) / self.B1R2
        self.B2Ratio = (self.B2R1 + self.B2R2) / self.B2R2
        self.VMax1 = 4.09
        self.VMax2 = 4.1
        self.ADCMax = 32767

    def publish_voltage(self):
        adc1 = self.adc.read_adc(0) * self.VMax1 / self.ADCMax
        adc2 = self.adc.read_adc(1) * self.VMax2
