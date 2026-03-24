#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import board
import busio
import digitalio
from adafruit_vl53l0x import VL53L0X
import time


class TOFTestNode(Node):
    def __init__(self):
        super().__init__('tof_test_node')
        
        # GPIO pins for XSHUT control (change to your GPIO pins if needed)
        self.XSHUT_PIN_1 = board.D4   # XSHUT pin for sensor 1
        self.XSHUT_PIN_2 = board.D5   # XSHUT pin for sensor 2
        
        # I2C addresses
        self.SENSOR1_ADDRESS = 0x29   # Default VL53L0X address
        self.SENSOR2_ADDRESS = 0x30   # Changed address for second sensor
        
        # Create publishers for distance measurements (in cm)
        self.publisher_1 = self.create_publisher(Float32, 'tof_distance_1', 10)
        self.publisher_2 = self.create_publisher(Float32, 'tof_distance_2', 10)
        
        # Alternative: If you want combined measurement in Range message
        self.range_pub_1 = self.create_publisher(Range, 'tof_range_1', 10)
        self.range_pub_2 = self.create_publisher(Range, 'tof_range_2', 10)
        
        # Timer for periodic measurements (100ms interval = 10 Hz)
        self.timer = self.create_timer(0.1, self.read_distances)
        
        self.sensor1 = None
        self.sensor2 = None
        
        try:
            self.initialize_dual_sensors()
            self.get_logger().info("Dual TOF Sensors initialized successfully!")
            self.get_logger().info("Sensor 1 (0x29) publishes to 'tof_distance_1' and 'tof_range_1'")
            self.get_logger().info("Sensor 2 (0x30) publishes to 'tof_distance_2' and 'tof_range_2'")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TOF sensors: {e}")
            self.get_logger().error("Make sure both sensors are connected to I2C pins with proper XSHUT GPIO connections")
            raise

    def initialize_dual_sensors(self):
        """Initialize two TOF sensors using XSHUT multiplexing.
        
        Note: VL53L0X sensors cannot change I2C addresses.
        Both run at 0x29, but we control them with XSHUT pins.
        """
        # Initialize I2C bus
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Setup XSHUT GPIO pins
        self.xshut1 = digitalio.DigitalInOut(self.XSHUT_PIN_1)
        self.xshut2 = digitalio.DigitalInOut(self.XSHUT_PIN_2)
        self.xshut1.direction = digitalio.Direction.OUTPUT
        self.xshut2.direction = digitalio.Direction.OUTPUT
        
        # Power down both sensors
        self.xshut1.value = False
        self.xshut2.value = False
        time.sleep(0.2)
        
        # Initialize sensor 1 - enable only sensor 1
        self.xshut1.value = True
        self.xshut2.value = False
        time.sleep(0.3)
        self.sensor1 = VL53L0X(self.i2c)
        self.get_logger().info(f"Sensor 1 initialized at address 0x{self.SENSOR1_ADDRESS:02x}")
        
        # Initialize sensor 2 - enable only sensor 2
        self.xshut1.value = False
        self.xshut2.value = True
        time.sleep(0.3)
        self.sensor2 = VL53L0X(self.i2c)
        self.get_logger().info(f"Sensor 2 initialized at address 0x{self.SENSOR2_ADDRESS:02x} (using XSHUT multiplexing)")
        
        # Enable both sensors for normal operation
        self.xshut1.value = True
        self.xshut2.value = True
        time.sleep(0.1)

    def read_distances(self):
        """Read distance from both TOF sensors using XSHUT multiplexing and publish them."""
        try:
            # Read from sensor 1 - enable only sensor 1
            if self.sensor1:
                self.xshut1.value = True
                self.xshut2.value = False
                time.sleep(0.05)  # Small delay for sensor to stabilize
                
                distance_mm_1 = self.sensor1.range
                distance_cm_1 = distance_mm_1 / 10.0
                
                # Publish as Float32
                msg_float = Float32()
                msg_float.data = distance_cm_1
                self.publisher_1.publish(msg_float)
                
                # Publish as Range message
                msg_range = Range()
                msg_range.header.stamp = self.get_clock().now().to_msg()
                msg_range.header.frame_id = "tof_link_1"
                msg_range.radiation_type = Range.INFRARED
                msg_range.field_of_view = 0.471  # ~27 degrees in radians for VL53L0X
                msg_range.min_range = 0.0
                msg_range.max_range = 2.0  # 2 meters
                msg_range.range = distance_cm_1 / 100.0  # Convert to meters
                self.range_pub_1.publish(msg_range)
                
                self.get_logger().debug(f"Sensor 1: {distance_cm_1:.2f} cm")
            
            # Read from sensor 2 - enable only sensor 2
            if self.sensor2:
                self.xshut1.value = False
                self.xshut2.value = True
                time.sleep(0.05)  # Small delay for sensor to stabilize
                
                distance_mm_2 = self.sensor2.range
                distance_cm_2 = distance_mm_2 / 10.0
                
                # Publish as Float32
                msg_float = Float32()
                msg_float.data = distance_cm_2
                self.publisher_2.publish(msg_float)
                
                # Publish as Range message
                msg_range = Range()
                msg_range.header.stamp = self.get_clock().now().to_msg()
                msg_range.header.frame_id = "tof_link_2"
                msg_range.radiation_type = Range.INFRARED
                msg_range.field_of_view = 0.471
                msg_range.min_range = 0.0
                msg_range.max_range = 2.0
                msg_range.range = distance_cm_2 / 100.0
                self.range_pub_2.publish(msg_range)
                
                self.get_logger().debug(f"Sensor 2: {distance_cm_2:.2f} cm")
            
            # Keep both enabled for next cycle
            self.xshut1.value = True
            self.xshut2.value = True
                
            self.get_logger().info(f"Sensor 1: {distance_cm_1:.2f} cm | Sensor 2: {distance_cm_2:.2f} cm")
            
        except Exception as e:
            self.get_logger().error(f"Error reading sensors: {e}")


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
