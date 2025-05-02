#!/usr/bin/python3

"""
operator_inerface.py

Desc: Connection between base station and carl
Author: Isaac Denning
Date: 10/21/23
"""

import rclpy
from rclpy.node import Node
from cysar.msg import Joystick
import socket

class OperatorInterface(Node):
    """
    Creates connection between base station and ROS.
    """
    def __init__(self) -> None:
        super().__init__('Operator_Interface')
        self.joystick = Joystick()
        self.socket = None
        self.conn = None
        self.joystick_publisher = self.create_publisher(Joystick, 'joystick', 10)
        self.timer = self.create_timer(0.05, self.talker)

    def connect(self) -> None:
        """
        Creates the connection between the base station and carl.
        """
        # Define the IP address and port number to listen on
        ip_address = "2610:130:110:1525:47e7:9414:7e67:15e4"
        port = 4143

        # Create a TCP socket
        self.socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)

        # Bind the socket to the IP address and port
        self.socket.bind((ip_address, port))

        # Listen for incoming connections
        self.socket.listen()

        # Accept a connection
        self.conn, addr = self.socket.accept()


        
        self.get_logger().info(f'Connected: {self.conn.getpeername()}\n')

    def disconnect(self) -> None:
        """
        Disconnects the the base station and carl.
        """
        try:
            self.conn.close()
            self.socket.close()
        except:
            pass

    def talker(self) -> None:
        """
        Loop for retrieving joystick inputs and publishing them to ROS.
        """
        try:
            # Receive data
            dataRaw = self.conn.recv(1664)
            data = bytearray(dataRaw)
            self.joystick.stick_left_x = (data[0] - 127) / 127
            self.joystick.stick_left_y = (data[1] - 127) / 127
            self.joystick.stick_right_x = (data[2] - 127) / 127
            self.joystick.stick_right_y = (data[3] - 127) / 127
            self.joystick.trigger_left = data[4] / 254
            self.joystick.trigger_right = data[5] /254
            self.joystick.button_a = data[6] == 1
            self.joystick.button_b = data[7] == 1
            self.joystick.button_x = data[8] == 1
            self.joystick.button_y = data[9] == 1
            self.joystick.bumper_left = data[10] == 1
            self.joystick.bumper_right = data[11] == 1
            self.joystick.button_back = data[12] == 1
            self.joystick.button_xbox = data[13] == 1
            self.joystick.button_start = data[14] == 1
            self.joystick.button_left_stick = data[15] == 1
            self.joystick.button_right_stick = data[16] == 1
            self.joystick.d_pad_up = data[17] == 1
            self.joystick.d_pad_down = data[18] == 1
            self.joystick.d_pad_left = data[19] == 1
            self.joystick.d_pad_right = data[20] == 1
            
            # Publish data to ROS
            self.joystick_publisher.publish(self.joystick)
        except:
            # In case of recieve fail, go back to home
            self.joystick.stick_left_x = 0.0
            self.joystick.stick_left_y = 0.0
            self.joystick.stick_right_x = 0.0
            self.joystick.stick_right_y = 0.0
            self.joystick.trigger_left = 0.0
            self.joystick.trigger_right = 0.0
            self.joystick.button_a = False
            self.joystick.button_b = False
            self.joystick.button_x = False
            self.joystick.button_y = False
            self.joystick.bumper_left = False
            self.joystick.bumper_right = False
            self.joystick.button_back = False
            self.joystick.button_xbox = False
            self.joystick.button_start = False
            self.joystick.button_right_stick = False
            self.joystick.button_right_stick = False
            self.joystick.d_pad_up = False
            self.joystick.d_pad_down = False
            self.joystick.d_pad_left = False
            self.joystick.d_pad_right = False
            
            # Publish data to ROS
            self.joystick_publisher.publish(self.joystick)


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    operator_interface = OperatorInterface()

    # Connect to the base station
    operator_interface.connect()

    # Run the node
    rclpy.spin(operator_interface)

    # Destroy it when done
    operator_interface.disconnect()
    operator_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
