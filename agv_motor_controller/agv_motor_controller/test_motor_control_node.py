import rclpy
from rclpy.node import Node
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685
import serial

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

        # Physical mapping provided earlier:
        # 1: Right Front, 2: Right Back, 3: Left Back, 4: Left Front
        self.RIGHT_MOTORS = [1, 2]
        self.LEFT_MOTORS = [3, 4]

        self.base_speed = 0.7
        self.Kp = 0.8
        self.rotate_speed = 0.5

        self.packet_count = 0
        self.last_error = 0.0

        # Open serial port (Arduino connection).
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Read serial continuously.
        self.timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info("Test Motor Control Node Started (integrated serial mode)")

    def set_motor(self, motor_id, speed):
        ch = self.MOTOR_CHANNELS[motor_id]

        if speed >= 0:
            self.pca.channels[ch['forward']].duty_cycle = int(speed * 65535)
            self.pca.channels[ch['reverse']].duty_cycle = 0
        else:
            self.pca.channels[ch['forward']].duty_cycle = 0
            self.pca.channels[ch['reverse']].duty_cycle = int(abs(speed) * 65535)

    def stop_all_motors(self):
        for motor_id in self.MOTOR_CHANNELS:
            ch = self.MOTOR_CHANNELS[motor_id]
            self.pca.channels[ch['forward']].duty_cycle = 0
            self.pca.channels[ch['reverse']].duty_cycle = 0

    def set_side_speeds(self, left, right):
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        for motor_id in self.LEFT_MOTORS:
            self.set_motor(motor_id, left)
        for motor_id in self.RIGHT_MOTORS:
            self.set_motor(motor_id, right)

    def rotate_clockwise(self):
        # Clockwise: left side forward, right side reverse.
        self.set_side_speeds(self.rotate_speed, -self.rotate_speed)
        self.get_logger().info(f"CALIB MODE: rotating clockwise at {self.rotate_speed:.2f}")

    def apply_line_control(self, error):
        if error == 9999:
            left = 0.0
            right = 0.0
        else:
            correction = self.Kp * error
            left = self.base_speed - correction
            right = self.base_speed + correction

        self.set_side_speeds(left, right)
        self.get_logger().info(f"CTRL: err={error:.2f} L:{left:.2f} R:{right:.2f}")

    def log_serial_packet(self, raw_line, error, encoders):
        self.packet_count += 1
        self.get_logger().info(
            f"[SERIAL {self.packet_count:05d}] "
            f"raw='{raw_line}' | "
            f"line_error={error:.2f} | "
            f"encoders=[E1:{encoders[0]:.0f}, E2:{encoders[1]:.0f}, E3:{encoders[2]:.0f}, E4:{encoders[3]:.0f}]"
        )

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            if not line:
                return

            # Calibration stream from Arduino: rotate in one direction while it arrives.
            if line.startswith('#'):
                self.get_logger().info(f"CALIB: {line}")
                self.rotate_clockwise()
                return

            data = line.split(',')
            if len(data) < 5:
                self.get_logger().warn(f"Invalid serial packet: {line}")
                return

            try:
                error = float(data[0])
                encoders = [float(v.strip()) for v in data[1:5]]
            except ValueError:
                self.get_logger().warn(f"Parse error in serial packet: {line}")
                return

            self.last_error = error

            # Print all values; encoders are intentionally not used for control yet.
            self.log_serial_packet(line, error, encoders)
            self.apply_line_control(error)

        except Exception as e:
            self.get_logger().warn(f"Error reading serial: {e}")

    def destroy_node(self):
        try:
            self.stop_all_motors()
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
            GPIO.cleanup()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestMotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Stopping all motors...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
