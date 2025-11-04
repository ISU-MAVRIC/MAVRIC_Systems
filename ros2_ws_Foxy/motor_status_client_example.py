#!/usr/bin/env python3
"""
Example client for querying motor status via service.

This demonstrates how to use the GetMotorStatus service
for UI, autonomous systems, or debugging.

Usage:
    python3 motor_status_client_example.py
"""

import rclpy
from rclpy.node import Node
from mavric_msg.srv import GetMotorStatus


class MotorStatusClient(Node):
    """Example client node for motor status queries"""

    def __init__(self):
        super().__init__('motor_status_client')
        
        # Create service client
        self.cli = self.create_client(GetMotorStatus, 'get_motor_status')
        
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for motor status service...')
        
        self.get_logger().info('Motor status service ready!')

    def get_all_motor_status(self):
        """Query status for all motors"""
        request = GetMotorStatus.Request()
        # Empty controller_ids = get all motors
        
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Received status for {len(response.statuses)} motors:')
            for status in response.statuses:
                self.get_logger().info(
                    f'  Motor {status.controller_id}: '
                    f'pos={status.position:.3f}, vel={status.velocity:.3f}'
                )
            return response.statuses
        else:
            self.get_logger().error('Service call failed')
            return None

    def get_specific_motors(self, motor_ids):
        """Query status for specific motors
        
        Args:
            motor_ids: List of motor IDs to query (e.g., [1, 2, 7])
        """
        request = GetMotorStatus.Request()
        request.controller_ids = motor_ids
        
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Received status for motors {motor_ids}:')
            for status in response.statuses:
                self.get_logger().info(
                    f'  Motor {status.controller_id}: '
                    f'pos={status.position:.3f}, vel={status.velocity:.3f}'
                )
            return response.statuses
        else:
            self.get_logger().error('Service call failed')
            return None


def main():
    rclpy.init()
    
    client = MotorStatusClient()
    
    # Example 1: Get all motor status
    print("\n=== Getting status for ALL motors ===")
    client.get_all_motor_status()
    
    # Example 2: Get specific motors (e.g., front wheels)
    print("\n=== Getting status for specific motors (1, 6, 7, 10) ===")
    client.get_specific_motors([1, 6, 7, 10])
    
    # Example 3: Continuous polling at 10Hz (like a UI would do)
    print("\n=== Polling at 10Hz for 3 seconds ===")
    timer_count = 0
    def poll_callback():
        nonlocal timer_count
        client.get_all_motor_status()
        timer_count += 1
        if timer_count >= 30:  # 3 seconds at 10Hz
            raise KeyboardInterrupt
    
    timer = client.create_timer(0.1, poll_callback)  # 10Hz
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
