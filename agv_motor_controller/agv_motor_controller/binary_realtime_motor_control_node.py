import struct
import threading
import time

import board
import busio
import rclpy
import RPi.GPIO as GPIO
import serial
from adafruit_pca9685 import PCA9685
from rclpy.node import Node


class BinaryRealtimeMotorControlNode(Node):
    HEADER = b"\xAA\x55"
    PAYLOAD_FMT = "<B H I f 4i 8H"
    PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)

    def __init__(self):
        super().__init__("binary_realtime_motor_control_node")

        self.ENABLE_PIN = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT)
        GPIO.output(self.ENABLE_PIN, GPIO.HIGH)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000

        self.MOTOR_CHANNELS = {
            1: {"forward": 0, "reverse": 1},
            2: {"forward": 2, "reverse": 3},
            3: {"forward": 4, "reverse": 5},
            4: {"forward": 6, "reverse": 7},
        }

        self.base_speed = 0.50
        self.Kp = 2.0
        self.Ki = 0.02
        self.Kd = 0.8

        self.white_threshold = 700
        self.search_speed = 0.45
        self.search_inner_speed = 0.08

        self.last_error = 0.0
        self.integral = 0.0
        self.last_seen_error_sign = 1

        self.calibration_active = False
        self.calibration_start_time = 0.0
        self.calibration_segment_seconds = 2.0
        self.calibration_lateral_speed = 0.8
        self.stream_ready = False

        self.packet_buffer = bytearray()
        self.latest_packet = None
        self.latest_packet_time = 0.0
        self.packet_lock = threading.Lock()

        self.last_seq = None
        self.drop_count = 0
        self.crc_fail_count = 0
        self.last_stats_log = 0.0

        self.ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0, write_timeout=0)

        self.running = True
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info("Binary realtime motor control node started")

    @staticmethod
    def crc16_ccitt(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def set_motor(self, motor_id, speed):
        ch = self.MOTOR_CHANNELS[motor_id]
        speed = max(-1.0, min(1.0, speed))
        pwm = int(abs(speed) * 65535)

        if speed >= 0:
            self.pca.channels[ch["forward"]].duty_cycle = pwm
            self.pca.channels[ch["reverse"]].duty_cycle = 0
        else:
            self.pca.channels[ch["forward"]].duty_cycle = 0
            self.pca.channels[ch["reverse"]].duty_cycle = pwm

    def stop_all_motors(self):
        for i in range(1, 5):
            self.set_motor(i, 0.0)

    def set_lateral_speed(self, speed):
        speed = max(-1.0, min(1.0, speed))
        self.set_motor(1, -speed)
        self.set_motor(2, speed)
        self.set_motor(3, -speed)
        self.set_motor(4, speed)

    def apply_line_control(self, error, sensor_norm):
        line_lost = error == 9999.0 or abs(error) > 10
        if not line_lost:
            line_lost = all(v <= self.white_threshold for v in sensor_norm)

        if line_lost:
            if self.last_seen_error_sign < 0:
                left_cmd = self.search_inner_speed
                right_cmd = self.search_speed
            else:
                left_cmd = self.search_speed
                right_cmd = self.search_inner_speed

            for m in [3, 4]:
                self.set_motor(m, left_cmd)
            for m in [1, 2]:
                self.set_motor(m, right_cmd)
            return

        self.integral += error * 0.01
        self.integral = max(min(self.integral, 30), -30)

        derivative = error - self.last_error
        correction = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        left = self.base_speed - correction
        right = self.base_speed + correction

        left = max(0.0, min(1.0, left))
        right = max(0.0, min(1.0, right))

        for m in [3, 4]:
            self.set_motor(m, left)
        for m in [1, 2]:
            self.set_motor(m, right)

        if abs(error) > 0.02:
            self.last_seen_error_sign = -1 if error < 0 else 1

        self.last_error = error

    def handle_text_line(self, line: str, now: float):
        if line.startswith("#CALIBRATING") and not self.calibration_active:
            self.calibration_active = True
            self.stream_ready = False
            self.calibration_start_time = now
            self.stop_all_motors()
            self.get_logger().info("Calibration started")
            return

        if line.startswith("#STREAM_START") and self.calibration_active:
            self.calibration_active = False
            self.stream_ready = True
            self.stop_all_motors()
            self.packet_buffer.clear()
            self.get_logger().info("Calibration complete, binary stream active")
            return

    def try_parse_one_frame(self):
        while True:
            header_idx = self.packet_buffer.find(self.HEADER)
            if header_idx < 0:
                self.packet_buffer.clear()
                return None
            if header_idx > 0:
                del self.packet_buffer[:header_idx]

            if len(self.packet_buffer) < 5:
                return None

            payload_len = self.packet_buffer[2]
            frame_len = 2 + 1 + payload_len + 2
            if len(self.packet_buffer) < frame_len:
                return None

            frame = bytes(self.packet_buffer[:frame_len])
            del self.packet_buffer[:frame_len]

            payload = frame[3:3 + payload_len]
            crc_rx = struct.unpack("<H", frame[3 + payload_len:3 + payload_len + 2])[0]
            crc_calc = self.crc16_ccitt(frame[2:3 + payload_len])
            if crc_rx != crc_calc:
                self.crc_fail_count += 1
                continue

            if payload_len != self.PAYLOAD_SIZE:
                continue

            unpacked = struct.unpack(self.PAYLOAD_FMT, payload)
            version = unpacked[0]
            if version != 1:
                continue

            seq = unpacked[1]
            packet = {
                "seq": seq,
                "millis": unpacked[2],
                "error": unpacked[3],
                "encoders": unpacked[4:8],
                "sensor_norm": unpacked[8:16],
            }
            return packet

    def reader_loop(self):
        while self.running:
            try:
                now = time.monotonic()

                if not self.stream_ready:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line.startswith("#"):
                        self.handle_text_line(line, now)
                    continue

                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    time.sleep(0.001)
                    continue

                self.packet_buffer.extend(chunk)

                while True:
                    packet = self.try_parse_one_frame()
                    if packet is None:
                        break

                    seq = packet["seq"]
                    if self.last_seq is not None:
                        expected = (self.last_seq + 1) & 0xFFFF
                        if seq != expected:
                            missed = (seq - expected) & 0xFFFF
                            self.drop_count += missed
                    self.last_seq = seq

                    with self.packet_lock:
                        self.latest_packet = packet
                        self.latest_packet_time = now

                if now - self.last_stats_log > 2.0:
                    if self.drop_count > 0 or self.crc_fail_count > 0:
                        self.get_logger().warn(
                            f"serial_stats drops={self.drop_count} crc_fail={self.crc_fail_count}"
                        )
                    self.last_stats_log = now

            except Exception:
                time.sleep(0.005)

    def control_loop(self):
        now = time.monotonic()

        if self.calibration_active:
            elapsed = now - self.calibration_start_time
            segment_index = int(elapsed // self.calibration_segment_seconds)
            direction = -1 if segment_index % 2 == 0 else 1
            self.set_lateral_speed(direction * self.calibration_lateral_speed)
            return

        if not self.stream_ready:
            self.stop_all_motors()
            return

        with self.packet_lock:
            packet = self.latest_packet
            packet_time = self.latest_packet_time

        if packet is None:
            self.stop_all_motors()
            return

        if now - packet_time > 0.2:
            self.stop_all_motors()
            return

        error = float(packet["error"])
        sensor_norm = [int(v) for v in packet["sensor_norm"]]
        self.apply_line_control(error, sensor_norm)

    def destroy_node(self):
        self.running = False
        try:
            if self.reader_thread.is_alive():
                self.reader_thread.join(timeout=0.5)
        except Exception:
            pass

        try:
            self.stop_all_motors()
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
            GPIO.cleanup()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BinaryRealtimeMotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
