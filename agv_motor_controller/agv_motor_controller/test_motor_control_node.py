import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685

class TestMotorControlNode(Node):

    def __init__(self):
        super().__init__('test_motor_control_node')

        # Enable pin
        self.ENABLE_PIN = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT)
        GPIO.output(self.ENABLE_PIN, GPIO.HIGH)

        # PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000

        self.MOTOR_CHANNELS = {
            1: {'forward': 0, 'reverse': 1},
            2: {'forward': 2, 'reverse': 3},
            3: {'forward': 4, 'reverse': 5},
            4: {'forward': 6, 'reverse': 7}
        }

        self.base_speed = 0.7
        self.Kp = 0.8

        self.create_subscription(Float32, '/line_error', self.callback, 10)

        self.get_logger().info("Test Motor Control Node Started")

    def set_motor(self, motor_id, speed):
        ch = self.MOTOR_CHANNELS[motor_id]

        if speed >= 0:
            self.pca.channels[ch['forward']].duty_cycle = int(speed * 65535)
            self.pca.channels[ch['reverse']].duty_cycle = 0
        else:
            self.pca.channels[ch['forward']].duty_cycle = 0
            self.pca.channels[ch['reverse']].duty_cycle = int(abs(speed) * 65535)

    def callback(self, msg):
        error = msg.data

        if error == 9999:
            left = 0
            right = 0
        else:
            correction = self.Kp * error
            left = self.base_speed - correction
            right = self.base_speed + correction

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # Left motors (1,2), Right motors (3,4)
        self.set_motor(1, left)
        self.set_motor(2, left)
        self.set_motor(3, right)
        self.set_motor(4, right)

        self.get_logger().info(f"L:{left:.2f} R:{right:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = TestMotorControlNode()
    rclpy.spin(node)
    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()
