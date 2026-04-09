import rclpy
from rclpy.node import Node
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685
import serial
import time

class AGVMotorControlNode(Node):
    def __init__(self):
        super().__init__('agv_motor_control_node')

        # Enable Pin
        self.ENABLE_PIN = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT)
        GPIO.output(self.ENABLE_PIN, GPIO.HIGH)

        # PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000

        self.MOTOR_CHANNELS = {
            1: {'forward': 0, 'reverse': 1},  # RF
            2: {'forward': 2, 'reverse': 3},  # RB
            3: {'forward': 4, 'reverse': 5},  # LB
            4: {'forward': 6, 'reverse': 7}   # LF
        }

        self.base_speed = 0.65
        self.Kp = 1.8
        self.Ki = 0.02
        self.Kd = 0.8

        self.last_error = 0.0
        self.integral = 0.0

        # Serial
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        self.calibration_active = False
        self.calibration_start_time = 0.0
        self.calibration_duration = 10.0
        self.calibration_segment_seconds = 2.0
        self.calibration_lateral_speed = 0.8
        self.calibration_last_segment_index = -1

        self.get_logger().info("AGV Node Started - Waiting for Arduino calibration...")

        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

    def set_motor(self, motor_id, speed):
        ch = self.MOTOR_CHANNELS[motor_id]
        speed = max(-1.0, min(1.0, speed))
        pwm = int(abs(speed) * 65535)

        if speed >= 0:
            self.pca.channels[ch['forward']].duty_cycle = pwm
            self.pca.channels[ch['reverse']].duty_cycle = 0
        else:
            self.pca.channels[ch['forward']].duty_cycle = 0
            self.pca.channels[ch['reverse']].duty_cycle = pwm

    def stop_all_motors(self):
        for i in range(1, 5):
            self.set_motor(i, 0)

    def set_lateral_speed(self, speed):
        """Positive = move right, Negative = move left"""
        speed = max(-1.0, min(1.0, speed))
        self.set_motor(1, -speed)   # RF
        self.set_motor(2,  speed)   # RB
        self.set_motor(3, -speed)   # LB
        self.set_motor(4,  speed)   # LF

    def apply_line_control(self, error):
        if error == 9999.0 or abs(error) > 10:
            self.stop_all_motors()
            return

        self.integral += error * 0.02
        self.integral = max(min(self.integral, 30), -30)

        derivative = error - self.last_error
        correction = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        left  = self.base_speed - correction
        right = self.base_speed + correction

        # Set side speeds
        for m in [3, 4]:   # Left motors
            self.set_motor(m, left)
        for m in [1, 2]:   # Right motors
            self.set_motor(m, right)

        left_cmd = max(-1.0, min(1.0, left))
        right_cmd = max(-1.0, min(1.0, right))
        left_pwm = int(abs(left_cmd) * 65535)
        right_pwm = int(abs(right_cmd) * 65535)
        self.get_logger().info(
            f"line_error={error:.2f}, left_pwm={left_pwm}, right_pwm={right_pwm}, left_cmd={left_cmd:.3f}, right_cmd={right_cmd:.3f}"
        )

        self.last_error = error

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            now = time.monotonic()

            if line.startswith('#'):
                self.get_logger().info(f"arduino_msg: {line}")

            # === Start Calibration when Arduino sends this message ===
            if "#CALIBRATING 10 SECONDS" in line and not self.calibration_active:
                self.get_logger().info("=== SENSOR CALIBRATION STARTED ===")
                self.calibration_active = True
                self.calibration_start_time = now
                self.calibration_last_segment_index = -1
                self.stop_all_motors()

            # === Perform Lateral Movement during calibration ===
            if self.calibration_active:
                elapsed = now - self.calibration_start_time

                if elapsed >= self.calibration_duration:
                    self.calibration_active = False
                    self.stop_all_motors()
                    self.get_logger().info("=== CALIBRATION COMPLETED - Starting Line Following ===")
                    return

                # Alternate: 2s left, 2s right, repeated until calibration ends.
                segment_index = int(elapsed // self.calibration_segment_seconds)
                direction = -1 if segment_index % 2 == 0 else 1

                if segment_index != self.calibration_last_segment_index:
                    side = "LEFT" if direction < 0 else "RIGHT"
                    self.get_logger().info(f"calibration_motion: {side} (segment={segment_index}, elapsed={elapsed:.2f}s)")
                    self.calibration_last_segment_index = segment_index

                self.set_lateral_speed(direction * self.calibration_lateral_speed)
                return

            # === Normal Line Following Mode ===
            if line.startswith('#'):
                return

            data = line.split(',')
            if len(data) < 5:
                return

            error = float(data[0])
            sensors = [float(v.strip()) for v in data[1:5]]

            self.get_logger().info(
                f"line_sensors={sensors[0]:.2f},{sensors[1]:.2f},{sensors[2]:.2f},{sensors[3]:.2f}"
            )

            self.apply_line_control(error)

        except Exception:
            pass

    def destroy_node(self):
        self.stop_all_motors()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AGVMotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()