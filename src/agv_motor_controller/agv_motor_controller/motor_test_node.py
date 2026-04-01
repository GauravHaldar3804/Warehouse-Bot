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

        # ===== GPIO Setup for BTS7960 Enable Pin =====
        # All 4 motors share enable pin 17
        self.ENABLE_PIN = 17

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT)
        GPIO.output(self.ENABLE_PIN, GPIO.HIGH)

        # ===== PCA9685 Setup =====
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000  # 1kHz PWM

        # Assign channels for 4 motors (each motor needs 2 channels: forward & reverse)
        self.MOTOR_CHANNELS = {
            1: {'forward': 0, 'reverse': 1},
            2: {'forward': 2, 'reverse': 3},
            3: {'forward': 4, 'reverse': 5},
            4: {'forward': 6, 'reverse': 7}
        }

        self.get_logger().info("Starting motor test with 4 motors (all enables on pin 17)...")

        self.test_motor()

    def set_motor_speed(self, motor_id, forward_speed=0.0, reverse_speed=0.0):
        """
        Set speed for a specific motor.
        motor_id: 1-4
        forward_speed and reverse_speed range: 0.0 to 1.0
        """
        if motor_id not in self.MOTOR_CHANNELS:
            self.get_logger().error(f"Invalid motor_id: {motor_id}")
            return

        forward_duty = int(forward_speed * 65535)
        reverse_duty = int(reverse_speed * 65535)

        channels = self.MOTOR_CHANNELS[motor_id]
        self.pca.channels[channels['forward']].duty_cycle = forward_duty
        self.pca.channels[channels['reverse']].duty_cycle = reverse_duty

    def stop_motor(self, motor_id):
        """Stop a specific motor."""
        if motor_id not in self.MOTOR_CHANNELS:
            self.get_logger().error(f"Invalid motor_id: {motor_id}")
            return

        channels = self.MOTOR_CHANNELS[motor_id]
        self.pca.channels[channels['forward']].duty_cycle = 0
        self.pca.channels[channels['reverse']].duty_cycle = 0

    def stop_all_motors(self):
        """Stop all motors."""
        for motor_id in self.MOTOR_CHANNELS:
            self.stop_motor(motor_id)

    def test_motor(self):
        # Test Motor 1 only (PCA Channels 0 & 1) - one at a time
        motor_id = 1
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Testing Motor {motor_id} (Channels 0 & 1)")
        self.get_logger().info("=" * 50)

        # Test Forward (Channel 0)
        self.get_logger().info(f"\n[1/4] Motor {motor_id} - FORWARD (Channel 0 active)")
        self.get_logger().info("Speed: 80%")
        self.set_motor_speed(motor_id, forward_speed=0.8)
        time.sleep(3)

        # Stop
        self.get_logger().info(f"\n[2/4] Motor {motor_id} - STOP")
        self.stop_motor(motor_id)
        time.sleep(2)

        # Test Reverse (Channel 1)
        self.get_logger().info(f"\n[3/4] Motor {motor_id} - REVERSE (Channel 1 active)")
        self.get_logger().info("Speed: 80%")
        self.set_motor_speed(motor_id, reverse_speed=0.8)
        time.sleep(3)

        # Final Stop
        self.get_logger().info(f"\n[4/4] Motor {motor_id} - FINAL STOP")
        self.stop_motor(motor_id)
        time.sleep(1)

        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("Motor test completed successfully!")
        self.get_logger().info("=" * 50)
        
        self.stop_all_motors()
        GPIO.cleanup()
        self.pca.deinit()

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    rclpy.spin(node)