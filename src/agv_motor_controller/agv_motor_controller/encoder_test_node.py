import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO


class EncoderTestNode(Node):

    def __init__(self):
        super().__init__('encoder_test_node')

        self.ENCODER_A = 23
        self.ENCODER_B = 24

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.tick_count = 0

        # Store previous state
        self.prev_state = (GPIO.input(self.ENCODER_A) << 1) | GPIO.input(self.ENCODER_B)

        # Detect change on BOTH channels
        GPIO.add_event_detect(self.ENCODER_A, GPIO.BOTH, callback=self.encoder_callback)
        GPIO.add_event_detect(self.ENCODER_B, GPIO.BOTH, callback=self.encoder_callback)

        self.timer = self.create_timer(0.5, self.print_ticks)

        self.get_logger().info("Improved encoder test started...")

    def encoder_callback(self, channel):
        a = GPIO.input(self.ENCODER_A)
        b = GPIO.input(self.ENCODER_B)

        current_state = (a << 1) | b

        # Quadrature state transition table
        transition = (self.prev_state << 2) | current_state

        if transition in [0b0001, 0b0111, 0b1110, 0b1000]:
            self.tick_count += 1
        elif transition in [0b0010, 0b0100, 0b1101, 0b1011]:
            self.tick_count -= 1

        self.prev_state = current_state

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