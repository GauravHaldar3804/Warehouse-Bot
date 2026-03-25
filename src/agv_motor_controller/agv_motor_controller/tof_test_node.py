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
        self.XSHUT_PIN_1 = board.D27  # GPIO 27
        self.XSHUT_PIN_2 = board.D23   # GPIO 23
        
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
            print("Setting up XSHUT GPIO pins...", flush=True)
            # Setup XSHUT pins
            self.xshut1 = digitalio.DigitalInOut(self.XSHUT_PIN_1)
            self.xshut2 = digitalio.DigitalInOut(self.XSHUT_PIN_2)
            self.xshut1.direction = digitalio.Direction.OUTPUT
            self.xshut2.direction = digitalio.Direction.OUTPUT
            print("XSHUT pins configured", flush=True)
            
            # Initialize I2C bus
            print("Initializing I2C bus...", flush=True)
            self.i2c = busio.I2C(board.SCL, board.SDA)
            print("I2C bus ready", flush=True)
            
            # Power down both sensors
            print("Powering down all sensors...", flush=True)
            self.xshut1.value = False
            self.xshut2.value = False
            time.sleep(0.2)
            
            # Initialize Sensor 1
            print("Enabling and initializing Sensor 1...", flush=True)
            self.xshut1.value = True
            self.xshut2.value = False
            time.sleep(0.1)
            self.sensor1 = VL53L0X(self.i2c)
            self.get_logger().info("Sensor 1 initialized successfully!")
            print("Sensor 1 OK", flush=True)
            
            # Initialize Sensor 2
            print("Enabling and initializing Sensor 2...", flush=True)
            self.xshut1.value = False
            self.xshut2.value = True
            time.sleep(0.1)
            self.sensor2 = VL53L0X(self.i2c)
            self.get_logger().info("Sensor 2 initialized successfully!")
            print("Sensor 2 OK", flush=True)
            
            # Enable both sensors for reading
            print("Enabling both sensors...", flush=True)
            self.xshut1.value = True
            self.xshut2.value = True
            time.sleep(0.1)
            print("Both sensors online!", flush=True)
            
        except Exception as e:
            print(f"ERROR during init: {e}", flush=True)
            self.get_logger().error(f"Failed to initialize TOF sensors: {e}")
            import traceback
            traceback.print_exc()
            raise

    def read_distance(self):
        """Read distance from both TOF sensors using XSHUT switching."""
        try:
            print("\n=== Reading ===", flush=True)
            distance_cm_1 = 0
            distance_cm_2 = 0
            
            # Read from Sensor 1 (disable sensor 2)
            print("Step 1: Setting XSHUT for Sensor 1...", flush=True)
            if self.sensor1 is not None:
                try:
                    print("  Setting pins: S1=True, S2=False", flush=True)
                    self.xshut1.value = True
                    self.xshut2.value = False
                    print("  Pins set, sleeping 0.05s...", flush=True)
                    time.sleep(0.05)  
                    
                    print("  About to read Sensor 1.range...", flush=True)
                    distance_mm_1 = self.sensor1.range
                    print(f"  Got: {distance_mm_1} mm", flush=True)
                    distance_cm_1 = distance_mm_1 / 10.0
                    
                    # Publish
                    msg = Float32()
                    msg.data = distance_cm_1
                    self.publisher_1.publish(msg)
                    
                    print(f"Sensor 1: {distance_cm_1:.2f} cm", end=" | ", flush=True)
                except Exception as e:
                    print(f"ERROR Sensor 1: {e}", end=" | ", flush=True)
                    import traceback
                    traceback.print_exc()
            
            # Read from Sensor 2 (disable sensor 1)
            print("\nStep 2: Setting XSHUT for Sensor 2...", flush=True)
            if self.sensor2 is not None:
                try:
                    print("  Setting pins: S1=False, S2=True", flush=True)
                    self.xshut1.value = False
                    self.xshut2.value = True
                    print("  Pins set, sleeping 0.05s...", flush=True)
                    time.sleep(0.05)  
                    
                    print("  About to read Sensor 2.range...", flush=True)
                    distance_mm_2 = self.sensor2.range
                    print(f"  Got: {distance_mm_2} mm", flush=True)
                    distance_cm_2 = distance_mm_2 / 10.0
                    
                    # Publish
                    msg = Float32()
                    msg.data = distance_cm_2
                    self.publisher_2.publish(msg)
                    
                    print(f"Sensor 2: {distance_cm_2:.2f} cm", flush=True)
                except Exception as e:
                    print(f"ERROR Sensor 2: {e}", flush=True)
                    import traceback
                    traceback.print_exc()
            else:
                print("Sensor 2: Not initialized", flush=True)
            
            # Re-enable both sensors for next cycle
            print("Step 3: Enable both sensors...", flush=True)
            self.xshut1.value = True
            self.xshut2.value = True
            print("Done with this cycle", flush=True)
            
        except Exception as e:
            print(f"ERROR in read_distance: {e}", flush=True)
            import traceback
            traceback.print_exc()
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
