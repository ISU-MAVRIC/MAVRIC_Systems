#!/usr/bin/env python3
"""
Quick test script for motor status service.
Run this after starting can_manager to verify service is working.

Usage:
    python3 test_motor_status_service.py
"""

import rclpy
from rclpy.node import Node
from mavric_msg.srv import GetMotorStatus
import sys


def test_service():
    """Quick test of motor status service"""
    
    rclpy.init()
    node = Node('test_status_service')
    
    # Create client
    client = node.create_client(GetMotorStatus, 'get_motor_status')
    
    # Wait for service
    print("Waiting for motor status service...")
    if not client.wait_for_service(timeout_sec=5.0):
        print("❌ ERROR: Service not available after 5 seconds")
        print("   Make sure can_manager node is running:")
        print("   ros2 run drive_system can_manager")
        node.destroy_node()
        rclpy.shutdown()
        return False
    
    print("✅ Service available!")
    
    # Test 1: Get all motors
    print("\nTest 1: Querying all motors...")
    request = GetMotorStatus.Request()
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    
    if future.result() is not None:
        response = future.result()
        if len(response.statuses) > 0:
            print(f"✅ Received {len(response.statuses)} motor statuses:")
            for status in response.statuses:
                print(f"   Motor {status.controller_id}: pos={status.position:.3f}, vel={status.velocity:.3f}")
        else:
            print("⚠️  No motors initialized yet")
            print("   Send some commands first to initialize motors")
    else:
        print("❌ Service call failed")
        node.destroy_node()
        rclpy.shutdown()
        return False
    
    # Test 2: Get specific motors
    if len(response.statuses) > 0:
        test_ids = [response.statuses[0].controller_id]
        print(f"\nTest 2: Querying specific motor {test_ids}...")
        
        request = GetMotorStatus.Request()
        request.controller_ids = test_ids
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response = future.result()
            print(f"✅ Received {len(response.statuses)} motor status:")
            for status in response.statuses:
                print(f"   Motor {status.controller_id}: pos={status.position:.3f}, vel={status.velocity:.3f}")
        else:
            print("❌ Service call failed")
            node.destroy_node()
            rclpy.shutdown()
            return False
    
    print("\n✅ All tests passed! Motor status service is working correctly.")
    
    node.destroy_node()
    rclpy.shutdown()
    return True


if __name__ == '__main__':
    try:
        success = test_service()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nTest interrupted")
        sys.exit(1)
