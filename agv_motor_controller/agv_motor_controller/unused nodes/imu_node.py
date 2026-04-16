#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import time
import math

class IMUSensor(Node):
    def __init__(self):
        super().__init__('imu_sensor')
        self.publisher = self.create_publisher(Imu, 'imu_data', 10)
        self.timer = self.create_timer(0.1, self.read_imu)

        self.bus = smbus.SMBus(1)
        self.mpu_address = 0x68
        self.bus.write_byte_data(self.mpu_address, 0x6B, 0)  # Wake up MPU6050
        self.get_logger().info("IMU Sensor Node Started!")

        # Initial values for angle calculation
        self.last_time = time.time()
        self.angle_x = 0.0
        self.angle_y = 0.0

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.mpu_address, addr)
        low = self.bus.read_byte_data(self.mpu_address, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def read_imu(self):
        # Read accelerometer raw values
        acc_x = self.read_raw_data(0x3B) / 16384.0  # Convert to g
        acc_y = self.read_raw_data(0x3D) / 16384.0
        acc_z = self.read_raw_data(0x3F) / 16384.0

        # Compute angles from accelerometer
        acc_angle_x = math.atan2(acc_y, acc_z) * (180 / math.pi)
        acc_angle_y = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)) * (180 / math.pi)

        # Read gyroscope raw values
        gyro_x = self.read_raw_data(0x43) / 131.0  # Convert to degrees/sec
        gyro_y = self.read_raw_data(0x45) / 131.0
        gyro_z = self.read_raw_data(0x47) / 131.0

        # Calculate time elapsed
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Compute angles using complementary filter
        self.angle_x = 0.98 * (self.angle_x + gyro_x * dt) + 0.02 * acc_angle_x
        self.angle_y = 0.98 * (self.angle_y + gyro_y * dt) + 0.02 * acc_angle_y

        # Publish data
        msg = Imu()
        msg.orientation.x = self.angle_x
        msg.orientation.y = self.angle_y
        msg.orientation.z = gyro_z  # Gyro Z is kept as rate
        self.publisher.publish(msg)

        self.get_logger().info(f"Published IMU Data: Angle X={self.angle_x:.2f}, Angle Y={self.angle_y:.2f}, Gyro Z={gyro_z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()