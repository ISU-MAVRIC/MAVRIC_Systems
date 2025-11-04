#!/usr/bin/env python3
"""
Performance Test Script for Drive System Optimization

This script measures the latency and synchronization of motor commands.
Run this while the drive system is active to measure performance.

Usage:
    python3 test_performance.py [--duration SECONDS]
"""

import rclpy
from rclpy.node import Node
from mavric_msg.msg import SteerTrain, CANCommandBatch
from std_msgs.msg import Header
import time
import argparse
import statistics


class PerformanceTester(Node):
    def __init__(self):
        super().__init__('performance_tester')
        
        # Statistics storage
        self.batch_timestamps = []
        self.batch_sizes = []
        self.start_time = time.time()
        
        # Subscribe to batch commands to measure
        self.sub_batch = self.create_subscription(
            CANCommandBatch,
            'can_commands_batch',
            self.batch_callback,
            10
        )
        
        # Publish test steer commands
        self.pub_steer = self.create_publisher(SteerTrain, 'steer_train', 10)
        
        # Timer to publish test commands at 60Hz
        self.test_timer = self.create_timer(1.0/60.0, self.publish_test_command)
        
        self.get_logger().info("Performance tester started. Publishing test commands at 60Hz...")
        
    def publish_test_command(self):
        """Publish a test steering command"""
        msg = SteerTrain()
        msg.steer_front_left = 0.5
        msg.steer_front_right = 0.5
        msg.steer_back_left = 0.5
        msg.steer_back_right = 0.5
        self.pub_steer.publish(msg)
        
    def batch_callback(self, msg: CANCommandBatch):
        """Record batch message statistics"""
        current_time = time.time()
        self.batch_timestamps.append(current_time)
        self.batch_sizes.append(len(msg.commands))
        
    def print_statistics(self):
        """Print performance statistics"""
        if len(self.batch_timestamps) < 2:
            self.get_logger().warn("Not enough data collected")
            return
            
        # Calculate message rate
        duration = self.batch_timestamps[-1] - self.batch_timestamps[0]
        message_count = len(self.batch_timestamps)
        rate = message_count / duration if duration > 0 else 0
        
        # Calculate inter-message intervals
        intervals = []
        for i in range(1, len(self.batch_timestamps)):
            intervals.append(self.batch_timestamps[i] - self.batch_timestamps[i-1])
        
        # Calculate statistics
        avg_interval = statistics.mean(intervals) if intervals else 0
        std_interval = statistics.stdev(intervals) if len(intervals) > 1 else 0
        min_interval = min(intervals) if intervals else 0
        max_interval = max(intervals) if intervals else 0
        
        avg_batch_size = statistics.mean(self.batch_sizes) if self.batch_sizes else 0
        
        # Print results
        self.get_logger().info("=" * 60)
        self.get_logger().info("PERFORMANCE TEST RESULTS")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Test Duration:        {duration:.2f} seconds")
        self.get_logger().info(f"Messages Received:    {message_count}")
        self.get_logger().info(f"Average Rate:         {rate:.2f} Hz")
        self.get_logger().info(f"Target Rate:          60.00 Hz")
        self.get_logger().info(f"Rate Accuracy:        {(rate/60.0)*100:.1f}%")
        self.get_logger().info("")
        self.get_logger().info(f"Avg Batch Size:       {avg_batch_size:.1f} commands")
        self.get_logger().info("")
        self.get_logger().info("Inter-Message Timing:")
        self.get_logger().info(f"  Average:            {avg_interval*1000:.2f} ms")
        self.get_logger().info(f"  Std Deviation:      {std_interval*1000:.2f} ms")
        self.get_logger().info(f"  Min:                {min_interval*1000:.2f} ms")
        self.get_logger().info(f"  Max:                {max_interval*1000:.2f} ms")
        self.get_logger().info(f"  Jitter:             {(max_interval-min_interval)*1000:.2f} ms")
        self.get_logger().info("=" * 60)
        
        # Evaluation
        if rate > 55 and rate < 65:
            self.get_logger().info("✅ Rate is within acceptable range")
        else:
            self.get_logger().warn("⚠️  Rate is outside expected range (55-65 Hz)")
            
        if avg_batch_size >= 4:
            self.get_logger().info("✅ Batch mode is active (4 motors per message)")
        else:
            self.get_logger().warn("⚠️  Batch mode may not be active")
            
        if std_interval < 0.01:  # Less than 10ms jitter
            self.get_logger().info("✅ Low jitter - good synchronization")
        else:
            self.get_logger().warn("⚠️  High jitter detected")


def main():
    parser = argparse.ArgumentParser(description='Test drive system performance')
    parser.add_argument('--duration', type=int, default=10, 
                       help='Test duration in seconds (default: 10)')
    args = parser.parse_args()
    
    rclpy.init()
    tester = PerformanceTester()
    
    print(f"\n🧪 Running performance test for {args.duration} seconds...\n")
    
    try:
        # Run for specified duration
        start = time.time()
        while time.time() - start < args.duration:
            rclpy.spin_once(tester, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        # Print statistics
        print("\n")
        tester.print_statistics()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
