import rclpy
from rclpy.node import Node
import board
import busio
import time
from adafruit_pca9685 import PCA9685

# PCA9685 I2C Address
I2C_ADDRESS = 0x40

# Servo channels on PCA9685
SERVO_1_CHANNEL = 4
SERVO_2_CHANNEL = 5

# Servo PWM calibration - SG90 servos
# SG90 uses: 1.0 - 2.0 ms pulse width
MIN_PULSE_MS = 0.5     # Pulse width for 0°
MAX_PULSE_MS = 2.5      # Pulse width for 180°


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

        # Start the servo control sequence
        self.run_servo_sequence()

    def angle_to_duty_cycle(self, angle):
        """
        Convert angle (0-180°) to duty cycle value for PCA9685.
        
        Args:
            angle (int/float): Angle in degrees (0-180)
            
        Returns:
            int: Duty cycle value (0-65535)
        """
        # Map angle 0-180 to pulse width MIN_PULSE_MS-MAX_PULSE_MS
        # For 50Hz: period = 20ms, each ms = 3276.75 (65535/20)
        ms_per_unit = 3276.75
        pulse_ms = MIN_PULSE_MS + (angle / 180.0) * (MAX_PULSE_MS - MIN_PULSE_MS)
        duty_cycle = int(pulse_ms * ms_per_unit)
        return duty_cycle

    def set_servo_angle(self, channel, servo_num, angle):
        """
        Set servo to a specific angle in degrees.
        
        Args:
            channel (int): PCA9685 channel number (4 or 5)
            servo_num (int): Servo number for logging (1 or 2)
            angle (int/float): Angle in degrees (0-180)
        """
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.pca.channels[channel].duty_cycle = duty_cycle
        self.get_logger().info(f'Servo {servo_num}: Set to {angle}°')

    def run_servo_sequence(self):
        """Execute the servo control sequence."""
        try:
            # Step 1: Both servos to 0 degrees
            self.get_logger().info('Step 1: Moving both servos to 0°')
            self.set_servo_angle(SERVO_1_CHANNEL, 1, 0)
            self.set_servo_angle(SERVO_2_CHANNEL, 2, 0)
            time.sleep(1.5)

            # Step 2: Servo 1 to 180 degrees
            self.get_logger().info('Step 2: Moving Servo 1 to 180°')
            self.set_servo_angle(SERVO_1_CHANNEL, 1, 180)
            time.sleep(1.5)

            # Step 3: Servo 2 to 180 degrees
            self.get_logger().info('Step 3: Moving Servo 2 to 180°')
            self.set_servo_angle(SERVO_2_CHANNEL, 2, 180)
            time.sleep(1.5)

            # Step 4: Both servos to 90 degrees
            self.get_logger().info('Step 4: Moving both servos to 90°')
            self.set_servo_angle(SERVO_1_CHANNEL, 1, 90)
            self.set_servo_angle(SERVO_2_CHANNEL, 2, 90)
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
