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
        self.calibration_segment_seconds = 2.0
        self.calibration_total_seconds = 10.0
        self.calibration_direction = 1
        self.calibration_start_time = 0.0
        self.post_calibration_stop_seconds = 10.0
        self.calibration_active = False
        self.post_calibration_pause_until = 0.0
        self.post_calibration_pause_announced = False

        self.last_error = 0.0

        # Open serial port (Arduino connection).
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Read serial continuously.
        self.timer = self.create_timer(0.05, self.read_serial)

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

        return left, right

    def speed_to_pwm(self, speed):
        speed = max(-1.0, min(1.0, speed))
        return int(speed * 255)

    def set_lateral_speed(self, lateral_speed):
        # Positive speed moves right; negative speed moves left.
        lateral_speed = max(-1.0, min(1.0, lateral_speed))
        self.set_motor(1, -lateral_speed)  # Right Front
        self.set_motor(2, lateral_speed)   # Right Back
        self.set_motor(3, -lateral_speed)  # Left Back
        self.set_motor(4, lateral_speed)   # Left Front

    def run_calibration_lateral_motion(self, now):
        elapsed = now - self.calibration_start_time

        if elapsed >= self.calibration_total_seconds:
            return False

        segment_index = int(elapsed // self.calibration_segment_seconds)
        direction = 1 if segment_index % 2 == 0 else -1

        if direction != self.calibration_direction:
            self.calibration_direction = direction

        self.set_lateral_speed(direction * self.calibration_lateral_speed)
        return True

    def apply_line_control(self, error):
        if error == 9999:
            left = 0.0
            right = 0.0
        else:
            correction = self.Kp * error
            left = self.base_speed - correction
            right = self.base_speed + correction

        left_cmd, right_cmd = self.set_side_speeds(left, right)
        self.get_logger().info(
            f"line_error={error:.2f}, left_pwm={self.speed_to_pwm(left_cmd)}, right_pwm={self.speed_to_pwm(right_cmd)}"
        )

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            now = time.monotonic()

            # Start calibration when Arduino sends calibration marker.
            if line.startswith('#') and not self.calibration_active:
                if not self.calibration_active:
                    self.calibration_active = True
                    self.post_calibration_pause_until = 0.0
                    self.post_calibration_pause_announced = False
                    self.calibration_direction = 1
                    self.calibration_start_time = now

            if self.calibration_active:
                if self.run_calibration_lateral_motion(now):
                    return

                self.calibration_active = False
                self.stop_all_motors()
                self.post_calibration_pause_until = now + self.post_calibration_stop_seconds
                self.post_calibration_pause_announced = False
                return

            if not line:
                return

            if self.post_calibration_pause_until > 0.0 and now < self.post_calibration_pause_until:
                self.stop_all_motors()
                return

            if self.post_calibration_pause_until > 0.0 and now >= self.post_calibration_pause_until:
                self.post_calibration_pause_until = 0.0

            data = line.split(',')
            if len(data) < 5:
                return

            try:
                error = float(data[0])
                _ = [float(v.strip()) for v in data[1:5]]
            except ValueError:
                return

            self.last_error = error
            self.apply_line_control(error)

        except Exception:
            return

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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
