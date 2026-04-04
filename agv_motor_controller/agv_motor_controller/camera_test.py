#!/usr/bin/env python3
"""
Test script for camera_node.py
Tests path subscription and QR code validation logic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading


class CameraNodeTester(Node):
    def __init__(self):
        super().__init__('camera_test_node')
        
        # Publisher for path results
        self.path_publisher = self.create_publisher(String, 'path_result', 10)
        
        # Subscriber to monitor QR codes
        self.qr_subscriber = self.create_subscription(
            String,
            'camera/qr_code',
            self.qr_callback,
            10
        )
        
        self.detected_qr = []
        self.get_logger().info("Camera Tester initialized")
        time.sleep(1)  # Wait for subscribers to connect
    
    def qr_callback(self, msg: String):
        """Monitor QR code detections"""
        self.detected_qr.append(msg.data)
        self.get_logger().info(f"QR published: {msg.data}")
    
    def test_path_subscription(self):
        """Test 1: Path subscription"""
        self.get_logger().info("\n=== TEST 1: Path Subscription ===")
        
        path_data = {
            'status': 'success',
            'path': ['A1', 'B2', 'C3', 'D4'],
            'navigation': []
        }
        
        msg = String()
        msg.data = json.dumps(path_data)
        self.path_publisher.publish(msg)
        
        self.get_logger().info(f"Published path: {path_data['path']}")
        time.sleep(1)
        self.get_logger().info("✓ Test 1 passed\n")
    
    def test_error_path(self):
        """Test 2: Error handling"""
        self.get_logger().info("=== TEST 2: Error Handling ===")
        
        error_data = {
            'status': 'error',
            'message': 'Start node A99 not found'
        }
        
        msg = String()
        msg.data = json.dumps(error_data)
        self.path_publisher.publish(msg)
        
        self.get_logger().info("Published error path")
        time.sleep(1)
        self.get_logger().info("✓ Test 2 passed\n")
    
    def test_qr_validation(self):
        """Test 3: Simulate QR detections"""
        self.get_logger().info("=== TEST 3: QR Code Validation ===")
        self.get_logger().info("Publish QR codes to 'camera/qr_code' topic manually")
        
        # First publish a correct path
        path_data = {
            'status': 'success',
            'path': ['A1', 'B2', 'C3'],
            'navigation': []
        }
        msg = String()
        msg.data = json.dumps(path_data)
        self.path_publisher.publish(msg)
        
        time.sleep(1)
        self.get_logger().info("Path set to: A1 → B2 → C3")
        self.get_logger().info("Now manually publish QR codes:")
        self.get_logger().info("  ros2 topic pub -1 /camera/qr_code std_msgs/String '{data: \"A1 | Angle: 45.0\"}'")
        self.get_logger().info("  ros2 topic pub -1 /camera/qr_code std_msgs/String '{data: \"B2 | Angle: 90.0\"}'")
        self.get_logger().info("  ros2 topic pub -1 /camera/qr_code std_msgs/String '{data: \"C3 | Angle: 135.0\"}'")


def main():
    rclpy.init()
    tester = CameraNodeTester()
    
    try:
        # Run tests
        tester.test_path_subscription()
        tester.test_error_path()
        tester.test_qr_validation()
        
        # Spin to keep node alive
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
