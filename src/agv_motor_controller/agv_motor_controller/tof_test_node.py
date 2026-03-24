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
        
        # Create publishers for distance measurements (in cm)
        self.publisher_1 = self.create_publisher(Float32, 'tof_distance_1', 10)
        self.publisher_2 = self.create_publisher(Float32, 'tof_distance_2', 10)
        
        # Timer for periodic measurements (100ms interval = 10 Hz)
        self.timer = self.create_timer(0.1, self.read_distance)
        
        self.sensor1 = None
        self.sensor2 = None
        self.i2c = None
        
        try:
            # Initialize I2C bus
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # Initialize Sensor 1
            print("Initializing Sensor 1 at address 0x29...", flush=True)
            self.sensor1 = VL53L0X(self.i2c)
            self.get_logger().info("Sensor 1 initialized successfully!")
            
            # Try to initialize Sensor 2
            # Note: We'll try to initialize it and see what happens
            print("Trying to initialize Sensor 2...", flush=True)
            try:
                self.sensor2 = VL53L0X(self.i2c)
                self.get_logger().info("Sensor 2 initialized successfully!")
                print("Sensor 2 initialized!", flush=True)
            except Exception as e:
                print(f"Sensor 2 initialization failed: {e}", flush=True)
                self.get_logger().warning(f"Could not initialize Sensor 2: {e}")
                self.sensor2 = None
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TOF sensors: {e}")
            raise

    def read_distance(self):
        """Read distance from both TOF sensors and publish them."""
        try:
            distance_cm_1 = 0
            distance_cm_2 = 0
            
            # Read from Sensor 1
            if self.sensor1 is not None:
                try:
                    distance_mm_1 = self.sensor1.range
                    distance_cm_1 = distance_mm_1 / 10.0
                    
                    # Publish
                    msg = Float32()
                    msg.data = distance_cm_1
                    self.publisher_1.publish(msg)
                    
                    print(f"Sensor 1: {distance_cm_1:.2f} cm", end=" | ", flush=True)
                except Exception as e:
                    print(f"ERROR Sensor 1: {e}", end=" | ", flush=True)
            
            # Read from Sensor 2
            if self.sensor2 is not None:
                try:
                    distance_mm_2 = self.sensor2.range
                    distance_cm_2 = distance_mm_2 / 10.0
                    
                    # Publish
                    msg = Float32()
                    msg.data = distance_cm_2
                    self.publisher_2.publish(msg)
                    
                    print(f"Sensor 2: {distance_cm_2:.2f} cm", flush=True)
                except Exception as e:
                    print(f"ERROR Sensor 2: {e}", flush=True)
            else:
                print("Sensor 2: Not initialized", flush=True)
            
        except Exception as e:
            print(f"ERROR in read_distance: {e}", flush=True)
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
