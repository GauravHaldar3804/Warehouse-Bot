#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
import digitalio
from adafruit_vl53l0x import VL53L0X
import time


class TOFSensor1Node(Node):
    def __init__(self):
        super().__init__('tof_sensor_1_node')
        
        # GPIO pins for XSHUT control
        self.XSHUT_PIN_1 = board.D27  # GPIO 27 - Sensor 1
        self.XSHUT_PIN_2 = board.D23  # GPIO 23 - Sensor 2
        
        # Create publisher for distance measurements (in cm)
        self.publisher = self.create_publisher(Float32, 'tof_distance_1', 10)
        
        # Timer for periodic measurements (100ms interval = 10 Hz)
        self.timer = self.create_timer(0.1, self.read_distance)
        
        self.sensor = None
        self.i2c = None
        self.xshut1 = None
        self.xshut2 = None
        
        try:
            # Setup XSHUT pins for both sensors
            self.xshut1 = digitalio.DigitalInOut(self.XSHUT_PIN_1)
            self.xshut2 = digitalio.DigitalInOut(self.XSHUT_PIN_2)
            self.xshut1.direction = digitalio.Direction.OUTPUT
            self.xshut2.direction = digitalio.Direction.OUTPUT
            
            # Initialize I2C bus
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # Power down both sensors
            self.xshut1.value = False
            self.xshut2.value = False
            time.sleep(0.2)
            
            # Power up only Sensor 1, keep Sensor 2 off
            self.xshut1.value = True
            self.xshut2.value = False
            time.sleep(0.3)
            
            # Initialize Sensor 1 (will be at default address 0x29)
            self.sensor = VL53L0X(self.i2c)
            self.get_logger().info("TOF Sensor 1 initialized at address 0x29")
            
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")
            raise

    def read_distance(self):
        """Read distance from TOF sensor 1."""
        try:
            if self.sensor is not None:
                try:
                    # Keep Sensor 1 powered, Sensor 2 off
                    self.xshut1.value = True
                    self.xshut2.value = False
                    
                    distance_mm = self.sensor.range
                    distance_cm = distance_mm / 10.0
                    
                    msg = Float32()
                    msg.data = distance_cm
                    self.publisher.publish(msg)
                    
                    print(f"Sensor 1: {distance_cm:6.2f} cm", flush=True)
                except Exception as e:
                    self.get_logger().error(f"Sensor 1 error: {e}")
            
        except Exception as e:
            self.get_logger().error(f"Error reading sensor: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TOFSensor1Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
