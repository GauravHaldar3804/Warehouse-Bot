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

        # User-provided physical mapping:
        # 1: Right Front, 2: Right Back, 3: Left Back, 4: Left Front
        self.RF = 1
        self.RB = 2
        self.LB = 3
        self.LF = 4

        # Motion tuning (time-based).
        self.STRAIGHT_SPEED = 0.65
        self.ROTATE_SPEED = 0.60
        self.LATERAL_SPEED = 0.60

        # Approximate distance is achieved by duration at fixed speed.
        self.STRAIGHT_TIME = 3.0
        self.ROTATE_TIME = 2.0
        self.LATERAL_TIME = 2.5

        self.get_logger().info("Starting motion sequence: straight -> clockwise rotate -> right lateral")
        self.run_sequence()

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

    def drive_motor(self, motor_id, signed_speed):
        """Positive signed_speed is forward, negative is reverse."""
        speed = max(-1.0, min(1.0, signed_speed))
        if speed >= 0.0:
            self.set_motor_speed(motor_id, forward_speed=speed, reverse_speed=0.0)
        else:
            self.set_motor_speed(motor_id, forward_speed=0.0, reverse_speed=abs(speed))

    def run_pattern(self, motor_speeds, duration, label):
        """
        Run a timed movement pattern.
        motor_speeds keys are motor IDs, values are signed speed (-1.0 to 1.0).
        """
        self.get_logger().info(f"{label} for {duration:.1f}s")
        for motor_id, speed in motor_speeds.items():
            self.drive_motor(motor_id, speed)
        time.sleep(duration)
        self.stop_all_motors()
        time.sleep(0.5)

    def run_sequence(self):
        # 1) Go straight forward.
        self.run_pattern(
            {
                self.RF: self.STRAIGHT_SPEED,
                self.RB: self.STRAIGHT_SPEED,
                self.LB: self.STRAIGHT_SPEED,
                self.LF: self.STRAIGHT_SPEED,
            },
            self.STRAIGHT_TIME,
            "Straight forward"
        )

        # 2) Rotate clockwise in place (right side reverse, left side forward).
        self.run_pattern(
            {
                self.RF: -self.ROTATE_SPEED,
                self.RB: -self.ROTATE_SPEED,
                self.LB: self.ROTATE_SPEED,
                self.LF: self.ROTATE_SPEED,
            },
            self.ROTATE_TIME,
            "Clockwise rotation"
        )

        # 3) Lateral right for mecanum/omni layout.
        self.run_pattern(
            {
                self.RF: -self.LATERAL_SPEED,
                self.RB: self.LATERAL_SPEED,
                self.LB: -self.LATERAL_SPEED,
                self.LF: self.LATERAL_SPEED,
            },
            self.LATERAL_TIME,
            "Lateral right"
        )

        self.get_logger().info("Motion sequence completed.")
        self.stop_all_motors()

        GPIO.cleanup()
        self.pca.deinit()

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    rclpy.spin(node)