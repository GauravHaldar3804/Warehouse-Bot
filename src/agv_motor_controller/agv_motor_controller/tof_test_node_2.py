#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO
from sensor_msgs.msg import Range


class DualVL53L0XNode(Node):
    def __init__(self):
        super().__init__('tof_test_node_2')

        self.xshut1 = 16
        self.xshut2 = 20
        self.publish_rate = 15.0   # lowered a bit for stability

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.xshut1, GPIO.OUT)
        GPIO.setup(self.xshut2, GPIO.OUT)

        # Fully reset both sensors
        GPIO.output(self.xshut1, GPIO.LOW)
        GPIO.output(self.xshut2, GPIO.LOW)
        time.sleep(0.2)

        i2c = busio.I2C(board.SCL, board.SDA)

        # Sensor 1 - default address 0x29
        GPIO.output(self.xshut1, GPIO.HIGH)
        time.sleep(0.05)
        try:
            self.vl53_1 = adafruit_vl53l0x.VL53L0X(i2c)
            self.get_logger().info('Sensor 1 initialized @ 0x29')
        except Exception as e:
            self.get_logger().error(f'Failed to init Sensor 1: {e}')

        # Sensor 2 - change address safely
        GPIO.output(self.xshut2, GPIO.HIGH)
        time.sleep(0.05)
        try:
            # First create with default address, then change
            temp_sensor = adafruit_vl53l0x.VL53L0X(i2c)
            temp_sensor.set_address(0x30)
            self.vl53_2 = adafruit_vl53l0x.VL53L0X(i2c, address=0x30)
            self.get_logger().info('Sensor 2 initialized @ 0x30')
        except Exception as e:
            self.get_logger().error(f'Failed to init Sensor 2: {e}')

        self.pub1 = self.create_publisher(Range, '/vl53l0x/sensor1/range', 10)
        self.pub2 = self.create_publisher(Range, '/vl53l0x/sensor2/range', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info('Dual VL53L0X node running...')

    def publish_range(self, distance_mm, publisher, frame_id):
        if distance_mm is None or distance_mm <= 0:
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.44
        msg.min_range = 0.03
        msg.max_range = 1.2
        msg.range = distance_mm / 1000.0
        publisher.publish(msg)

    def timer_callback(self):
        try:
            dist1 = self.vl53_1.distance if hasattr(self, 'vl53_1') else None
            dist2 = self.vl53_2.distance if hasattr(self, 'vl53_2') else None

            self.publish_range(dist1, self.pub1, 'vl53l0x_sensor1')
            self.publish_range(dist2, self.pub2, 'vl53l0x_sensor2')
        except Exception as e:
            self.get_logger().warn(f'Read error: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualVL53L0XNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()