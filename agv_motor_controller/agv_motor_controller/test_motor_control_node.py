import rclpy
from rclpy.node import Node
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685
import serial
import time

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
        self.Kp = 1.2
        self.rotate_speed = 0.5
        self.calibration_lateral_speed = 1.0
        self.calibration_switch_interval_seconds = 0.8
        self.calibration_direction = 1
        self.calibration_last_switch_time = 0.0
        self.post_calibration_stop_seconds = 10.0
        self.calibration_active = False
        self.post_calibration_pause_until = 0.0
        self.post_calibration_pause_announced = False

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

    def set_lateral_speed(self, lateral_speed):
        # Positive speed moves right; negative speed moves left.
        lateral_speed = max(-1.0, min(1.0, lateral_speed))
        self.set_motor(1, -lateral_speed)  # Right Front
        self.set_motor(2, lateral_speed)   # Right Back
        self.set_motor(3, -lateral_speed)  # Left Back
        self.set_motor(4, lateral_speed)   # Left Front

    def run_calibration_lateral_motion(self, now):
        if now - self.calibration_last_switch_time >= self.calibration_switch_interval_seconds:
            self.calibration_direction *= -1
            self.calibration_last_switch_time = now
            direction_label = "RIGHT" if self.calibration_direction > 0 else "LEFT"
            self.get_logger().info(f"CALIB MODE: switching lateral direction to {direction_label}")

        self.set_lateral_speed(self.calibration_direction * self.calibration_lateral_speed)

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
            now = time.monotonic()

            if not line:
                return

            # Calibration stream from Arduino: alternate lateral right/left while it arrives.
            if line.startswith('#'):
                if not self.calibration_active:
                    self.calibration_active = True
                    self.post_calibration_pause_until = 0.0
                    self.post_calibration_pause_announced = False
                    self.calibration_direction = 1
                    self.calibration_last_switch_time = now
                    self.get_logger().info("CALIB MODE: starting lateral alternating motion")
                self.get_logger().info(f"CALIB: {line}")
                self.run_calibration_lateral_motion(now)
                return

            if self.calibration_active:
                self.calibration_active = False
                self.stop_all_motors()
                self.post_calibration_pause_until = now + self.post_calibration_stop_seconds
                self.post_calibration_pause_announced = False

            if self.post_calibration_pause_until > 0.0 and now < self.post_calibration_pause_until:
                self.stop_all_motors()
                if not self.post_calibration_pause_announced:
                    self.get_logger().info(
                        f"CALIB DONE: stopped for {self.post_calibration_stop_seconds:.1f}s before control resumes"
                    )
                    self.post_calibration_pause_announced = True
                return

            if self.post_calibration_pause_until > 0.0 and now >= self.post_calibration_pause_until:
                self.post_calibration_pause_until = 0.0
                self.post_calibration_pause_announced = False
                self.get_logger().info("CALIB DONE: resuming line control")

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
