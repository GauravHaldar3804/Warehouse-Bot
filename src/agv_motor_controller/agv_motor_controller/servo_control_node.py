import rclpy
from rclpy.node import Node
import board
import busio
import time
from adafruit_pca9685 import PCA9685
from adafruit_servo import Servo

# PCA9685 I2C Address
I2C_ADDRESS = 0x40

# Servo channels on PCA9685
SERVO_1_CHANNEL = 4
SERVO_2_CHANNEL = 5


class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.get_logger().info('Servo Control Node started')

        # Initialize I2C
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Initialize PCA9685
        self.pca = PCA9685(self.i2c, address=I2C_ADDRESS)
        self.pca.frequency = 50  # Standard servo frequency

        self.get_logger().info('PCA9685 initialized at address 0x40')
        self.get_logger().info(f'Frequency set to {self.pca.frequency} Hz')

        # Initialize Servo objects for intuitive angle control
        self.servo1 = Servo(self.pca.channels[SERVO_1_CHANNEL], min_pulse=1000, max_pulse=2000)
        self.servo2 = Servo(self.pca.channels[SERVO_2_CHANNEL], min_pulse=1000, max_pulse=2000)
        
        self.get_logger().info('Servo 1 initialized on channel 4')
        self.get_logger().info('Servo 2 initialized on channel 5')

        # Start the servo control sequence
        self.run_servo_sequence()

    def set_servo_angle(self, servo, servo_num, angle):
        """
        Set servo to a specific angle in degrees.
        
        Args:
            servo (Servo): Servo object to control
            servo_num (int): Servo number for logging
            angle (int): Angle in degrees (0-180)
        """
        servo.angle = angle
        self.get_logger().info(f'Servo {servo_num}: Set to {angle}°')

    def run_servo_sequence(self):
        """Execute the servo control sequence."""
        try:
            # Step 1: Both servos to 0 degrees
            self.get_logger().info('Step 1: Moving both servos to 0°')
            self.set_servo_angle(self.servo1, 1, 0)
            self.set_servo_angle(self.servo2, 2, 0)
            time.sleep(1.5)

            # Step 2: Servo 1 to 180 degrees
            self.get_logger().info('Step 2: Moving Servo 1 to 180°')
            self.set_servo_angle(self.servo1, 1, 180)
            time.sleep(1.5)

            # Step 3: Servo 2 to 180 degrees
            self.get_logger().info('Step 3: Moving Servo 2 to 180°')
            self.set_servo_angle(self.servo2, 2, 180)
            time.sleep(1.5)

            # Step 4: Both servos to 90 degrees
            self.get_logger().info('Step 4: Moving both servos to 90°')
            self.set_servo_angle(self.servo1, 1, 90)
            self.set_servo_angle(self.servo2, 2, 90)
            time.sleep(1.5)

            self.get_logger().info('Servo sequence completed!')

        except Exception as e:
            self.get_logger().error(f'Error in servo sequence: {str(e)}')

        # Shutdown after sequence
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
