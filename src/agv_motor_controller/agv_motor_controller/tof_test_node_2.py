import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import board
import busio
import time
import RPi.GPIO as GPIO

import adafruit_vl53l0x

# GPIO pins for XSHUT
XSHUT_1 = 22
XSHUT_2 = 27
XSHUT_3 = 23  # Uncomment when 3rd sensor is available

class DualVL53L0X(Node):
    def __init__(self):
        super().__init__('dual_vl53l0x_node')

        # Publishers
        self.pub1 = self.create_publisher(Float32, '/tof1/distance', 10)
        self.pub2 = self.create_publisher(Float32, '/tof2/distance', 10)
        self.pub3 = self.create_publisher(Float32, '/tof3/distance', 10)

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(XSHUT_1, GPIO.OUT)
        GPIO.setup(XSHUT_2, GPIO.OUT)
        GPIO.setup(XSHUT_3, GPIO.OUT)

        # Turn OFF both sensors
        GPIO.output(XSHUT_1, GPIO.LOW)
        GPIO.output(XSHUT_2, GPIO.LOW)
        GPIO.output(XSHUT_3, GPIO.LOW)
        time.sleep(0.5)

        # Initialize I2C
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # --- Sensor 1 ---
        GPIO.output(XSHUT_1, GPIO.HIGH)
        time.sleep(0.5)

        self.sensor1 = adafruit_vl53l0x.VL53L0X(self.i2c)
        self.sensor1.set_address(0x30)

        # --- Sensor 2 ---
        GPIO.output(XSHUT_2, GPIO.HIGH)
        time.sleep(0.5)

        self.sensor2 = adafruit_vl53l0x.VL53L0X(self.i2c)
        self.sensor2.set_address(0x31)

        # # --- Sensor 3 ---
        GPIO.output(XSHUT_3, GPIO.HIGH)
        time.sleep(0.5)
        
        self.sensor3 = adafruit_vl53l0x.VL53L0X(self.i2c)
        self.sensor3.set_address(0x32)

        self.get_logger().info("Sensors initialized successfully")

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.read_sensors)

    def read_sensors(self):
        msg1 = Float32()
        msg2 = Float32()
        msg3 = Float32()

        try:
            dist1 = self.sensor1.range  # in mm
            msg1.data = float(dist1)
        except:
            msg1.data = -1.0
            dist1 = -1

        try:
            dist2 = self.sensor2.range
            msg2.data = float(dist2)
        except:
            msg2.data = -1.0
            dist2 = -1

        try:
            dist3 = self.sensor3.range
            msg3.data = float(dist3)
        except:
            msg3.data = -1.0
            dist3 = -1

        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)
        
        # Print values to terminal
        # print(f"TOF Distance [mm]  |  Sensor 1: {dist1:6.1f}  |  Sensor 2: {dist2:6.1f}", flush=True)
        print(f"TOF Distance [mm]  |  Sensor 1: {dist1:6.1f}  |  Sensor 2: {dist2:6.1f}  |  Sensor 3: {dist3:6.1f}", flush=True)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualVL53L0X()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()