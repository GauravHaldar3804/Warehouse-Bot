import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time


class EncoderTestNode(Node):

    def __init__(self):
        super().__init__('encoder_test_node')

        # ===== CHANGE PINS IF NEEDED =====
        self.ENCODER_A = 23
        self.ENCODER_B = 24

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.tick_count = 0

        # Detect BOTH edges on channel A
        GPIO.add_event_detect(self.ENCODER_A, GPIO.BOTH, callback=self.encoder_callback)

        self.timer = self.create_timer(0.5, self.print_ticks)

        self.get_logger().info("Encoder test started...")

    def encoder_callback(self, channel):
        a_state = GPIO.input(self.ENCODER_A)
        b_state = GPIO.input(self.ENCODER_B)

        # Quadrature direction detection
        if a_state == b_state:
            self.tick_count += 1
        else:
            self.tick_count -= 1

    def print_ticks(self):
        self.get_logger().info(f"Tick Count: {self.tick_count}")

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()