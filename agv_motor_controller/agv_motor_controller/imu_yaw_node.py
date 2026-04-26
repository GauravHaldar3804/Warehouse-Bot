#!/usr/bin/env python3

import math
import time

import board
import busio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import adafruit_mpu6050


class IMUYawNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_node')

        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('yaw_topic', 'imu/yaw_deg')
        self.declare_parameter('yaw_rate_topic', 'imu/yaw_rate_deg_s')
        self.declare_parameter('gyro_bias_samples', 300)
        self.declare_parameter('gyro_bias_sample_delay_sec', 0.005)
        self.declare_parameter('log_rate_hz', 2.0)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.yaw_topic = str(self.get_parameter('yaw_topic').value)
        self.yaw_rate_topic = str(self.get_parameter('yaw_rate_topic').value)
        self.gyro_bias_samples = int(self.get_parameter('gyro_bias_samples').value)
        self.gyro_bias_sample_delay_sec = float(self.get_parameter('gyro_bias_sample_delay_sec').value)
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)

        self.yaw_pub = self.create_publisher(Float32, self.yaw_topic, 10)
        self.yaw_rate_pub = self.create_publisher(Float32, self.yaw_rate_topic, 10)

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_mpu6050.MPU6050(self.i2c)

        self.yaw_deg = 0.0
        self.prev_time = time.monotonic()
        self.sample_count = 0
        self.log_every_n = max(1, int(round(self.publish_rate_hz / max(0.1, self.log_rate_hz))))

        self.gyro_z_bias_rad_s = self._calibrate_gyro_z_bias()

        timer_period = max(0.005, 1.0 / max(0.1, self.publish_rate_hz))
        self.timer = self.create_timer(timer_period, self.update_and_publish)

        self.get_logger().info(
            f'IMU yaw node started: rate={self.publish_rate_hz:.1f}Hz, '
            f'yaw_topic={self.yaw_topic}, yaw_rate_topic={self.yaw_rate_topic}, '
            f'gyro_z_bias={self.gyro_z_bias_rad_s:.6f} rad/s'
        )

    def _normalize_angle_deg(self, angle_deg: float) -> float:
        return ((angle_deg + 180.0) % 360.0) - 180.0

    def _calibrate_gyro_z_bias(self) -> float:
        self.get_logger().info(
            f'Calibrating gyro Z bias with {self.gyro_bias_samples} samples. Keep robot still...'
        )

        samples = max(1, self.gyro_bias_samples)
        total = 0.0
        good = 0

        for _ in range(samples):
            try:
                _, _, gz = self.sensor.gyro
                total += float(gz)
                good += 1
            except Exception as exc:
                self.get_logger().warn(f'Gyro bias sample failed: {exc}')
            time.sleep(max(0.0, self.gyro_bias_sample_delay_sec))

        if good == 0:
            self.get_logger().warn('No valid gyro samples for bias calibration; using 0.0 rad/s')
            return 0.0

        bias = total / float(good)
        self.get_logger().info(f'Gyro Z bias calibrated: {bias:.6f} rad/s from {good} samples')
        return bias

    def update_and_publish(self):
        try:
            _, _, gyro_z_rad_s = self.sensor.gyro

            now = time.monotonic()
            dt = now - self.prev_time
            self.prev_time = now

            if dt <= 0.0:
                return

            yaw_rate_deg_s = (float(gyro_z_rad_s) - self.gyro_z_bias_rad_s) * (180.0 / math.pi)
            self.yaw_deg = self._normalize_angle_deg(self.yaw_deg + yaw_rate_deg_s * dt)

            self.yaw_pub.publish(Float32(data=float(self.yaw_deg)))
            self.yaw_rate_pub.publish(Float32(data=float(yaw_rate_deg_s)))

            self.sample_count += 1
            if self.sample_count % self.log_every_n == 0:
                self.get_logger().info(
                    f'Yaw={self.yaw_deg:.2f} deg | YawRate={yaw_rate_deg_s:.2f} deg/s'
                )

        except Exception as exc:
            self.get_logger().warn(f'IMU read failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = IMUYawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
