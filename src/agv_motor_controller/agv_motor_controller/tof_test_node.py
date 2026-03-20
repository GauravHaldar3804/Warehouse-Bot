#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
from adafruit_vl53l0x import VL53L0X
import time


class TOFTestNode(Node):
    def __init__(self):
        super().__init__('tof_test_node')
        
        # Create publisher for distance measurements (in cm)
        self.publisher = self.create_publisher(Float32, 'tof_distance', 10)
        
        # Timer for periodic measurements (100ms interval = 10 Hz)
        self.timer = self.create_timer(0.1, self.read_distance)
        
        try:
            # Initialize I2C bus
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # Initialize VL53L0X sensor
            self.sensor = VL53L0X(i2c)
            
            self.get_logger().info("VL53L0X TOF Sensor initialized successfully!")
            self.get_logger().info("Distance measurements will be published to 'tof_distance' topic in cm")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TOF sensor: {e}")
            self.get_logger().error("Make sure the sensor is connected to I2C pins")
            raise

    def read_distance(self):
        """Read distance from VL53L0X sensor and publish it."""
        try:
            # Read distance in millimeters
            distance_mm = self.sensor.range
            
            # Convert to centimeters
            distance_cm = distance_mm / 10.0
            
            # Create and publish message
            msg = Float32()
            msg.data = distance_cm
            self.publisher.publish(msg)
            
            # Print to terminal
            self.get_logger().info(f"Distance: {distance_cm:.2f} cm ({distance_mm} mm)")
            
        except Exception as e:
            self.get_logger().error(f"Error reading sensor: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TOFTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
