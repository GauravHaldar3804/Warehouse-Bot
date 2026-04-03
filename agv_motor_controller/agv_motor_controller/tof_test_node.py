#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
import digitalio
from adafruit_vl53l0x import VL53L0X
import time


class TOFTestNode(Node):
    def __init__(self):
        super().__init__('tof_test_node')
        
        # GPIO pins for XSHUT control
        self.XSHUT_PIN_1 = board.D27  # GPIO 27 - Sensor 1
        self.XSHUT_PIN_2 = board.D23  # GPIO 23 - Sensor 2
        
        # Create publishers for distance measurements (in cm)
        self.publisher_1 = self.create_publisher(Float32, 'tof_distance_1', 10)
        self.publisher_2 = self.create_publisher(Float32, 'tof_distance_2', 10)
        
        # Timer for periodic measurements (100ms interval = 10 Hz)
        self.timer = self.create_timer(0.1, self.read_distance)
        
        self.sensor1 = None
        self.sensor2 = None
        self.i2c = None
        self.xshut1 = None
        self.xshut2 = None
        
        try:
            # Setup XSHUT pins
            self.xshut1 = digitalio.DigitalInOut(self.XSHUT_PIN_1)
            self.xshut2 = digitalio.DigitalInOut(self.XSHUT_PIN_2)
            self.xshut1.direction = digitalio.Direction.OUTPUT
            self.xshut2.direction = digitalio.Direction.OUTPUT
            
            # Initialize I2C bus
            self.i2c = busio.I2C(board.SCL, board.SDA)
            time.sleep(0.5)  # Give I2C bus time to stabilize
            
            # Power down both sensors
            self.xshut1.value = False
            self.xshut2.value = False
            time.sleep(0.5)
            
            # Initialize Sensor 1 (only S1 powered)
            self.xshut1.value = True
            self.xshut2.value = False
            time.sleep(0.5)  # Extended wait for sensor to power up
            self.sensor1 = VL53L0X(self.i2c)
            self.get_logger().info("Sensor 1 initialized")
            
            # Initialize Sensor 2 (only S2 powered)
            self.xshut1.value = False
            self.xshut2.value = True
            time.sleep(0.5)  # Extended wait for sensor to power up
            self.sensor2 = VL53L0X(self.i2c)
            self.get_logger().info("Sensor 2 initialized")
            
            # Keep both sensors powered on for normal operation
            self.xshut1.value = True
            self.xshut2.value = True
            time.sleep(0.2)
            self.get_logger().info("TOF sensors ready")
            
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")
            raise

    def read_distance(self):
        """Read distance from both TOF sensors using XSHUT switching."""
        try:
            distance_cm_1 = 0
            distance_cm_2 = 0
            
            # Read from Sensor 1 (keep S1 on, S2 off)
            if self.sensor1 is not None:
                try:
                    self.xshut1.value = True
                    self.xshut2.value = False
                    time.sleep(0.08)
                    distance_mm_1 = self.sensor1.range
                    distance_cm_1 = distance_mm_1 / 10.0
                    msg = Float32()
                    msg.data = distance_cm_1
                    self.publisher_1.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"Sensor 1 error: {e}")
            
            # Read from Sensor 2 (keep S2 on, S1 off)
            if self.sensor2 is not None:
                try:
                    self.xshut1.value = False
                    self.xshut2.value = True
                    time.sleep(0.08)
                    distance_mm_2 = self.sensor2.range
                    distance_cm_2 = distance_mm_2 / 10.0
                    msg = Float32()
                    msg.data = distance_cm_2
                    self.publisher_2.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"Sensor 2 error: {e}")
            
            # Display readings
            print(f"TOF Sensors [cm]  |  S1: {distance_cm_1:6.2f}  |  S2: {distance_cm_2:6.2f}", flush=True)
            
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
