import rclpy
from rclpy.node import Node
import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685


class MotorTestNode(Node):

    def __init__(self):
        super().__init__('motor_test_node')

        # ===== GPIO Setup for BTS7960 Enable Pins =====
        self.R_EN = 17   # Change if needed
        self.L_EN = 27   # Change if needed

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.R_EN, GPIO.OUT)
        GPIO.setup(self.L_EN, GPIO.OUT)

        GPIO.output(self.R_EN, GPIO.HIGH)
        GPIO.output(self.L_EN, GPIO.HIGH)

        # ===== PCA9685 Setup =====
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000  # 1kHz PWM

        # Assign channels
        self.RPWM_CHANNEL = 0
        self.LPWM_CHANNEL = 1

        self.get_logger().info("Starting motor test...")

        self.test_motor()

    def set_motor_speed(self, forward_speed=0.0, reverse_speed=0.0):
        """
        forward_speed and reverse_speed range: 0.0 to 1.0
        """

        forward_duty = int(forward_speed * 65535)
        reverse_duty = int(reverse_speed * 65535)

        self.pca.channels[self.RPWM_CHANNEL].duty_cycle = forward_duty
        self.pca.channels[self.LPWM_CHANNEL].duty_cycle = reverse_duty

    def stop_motor(self):
        self.pca.channels[self.RPWM_CHANNEL].duty_cycle = 0
        self.pca.channels[self.LPWM_CHANNEL].duty_cycle = 0

    def test_motor(self):
        # Forward
        self.get_logger().info("Motor Forward")
        self.set_motor_speed(forward_speed=0.5)
        time.sleep(3)

        # Stop
        self.get_logger().info("Motor Stop")
        self.stop_motor()
        time.sleep(2)

        # Reverse
        self.get_logger().info("Motor Reverse")
        self.set_motor_speed(reverse_speed=0.5)
        time.sleep(3)

        # Stop
        self.get_logger().info("Motor Final Stop")
        self.stop_motor()

        GPIO.cleanup()
        self.pca.deinit()

        self.get_logger().info("Motor test completed.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    rclpy.spin(node)