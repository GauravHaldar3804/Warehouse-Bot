#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray


class MotorCommandSerialBridge(Node):
    def __init__(self):
        super().__init__('motor_command_serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('command_topic', 'motor_command')
        self.declare_parameter('encoder_topic', 'encoder_counts')
        self.declare_parameter('encoder_text_topic', 'encoder_counts_text')
        self.declare_parameter('serial_read_rate_hz', 200.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.command_topic = self.get_parameter('command_topic').value
        self.encoder_topic = self.get_parameter('encoder_topic').value
        self.encoder_text_topic = self.get_parameter('encoder_text_topic').value
        self.serial_read_rate_hz = float(self.get_parameter('serial_read_rate_hz').value)

        self.ser = None
        self._last_encoder = None
        self._serial_rx_buffer = ''
        self._last_malformed_warn_time = 0.0

        self.allowed_commands = {
            'LEFT', 'RIGHT', 'STRAIGHT', 'UTURN', 'STOP', 'START',
            'OBSTACLE', 'CLEAR'
        }

        self.subscription = self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            10,
        )

        self.encoder_pub = self.create_publisher(Int32MultiArray, self.encoder_topic, 20)
        self.encoder_text_pub = self.create_publisher(String, self.encoder_text_topic, 20)

        self.reconnect_timer = self.create_timer(1.0, self.ensure_serial_connection)
        read_period = max(0.002, 1.0 / max(1.0, self.serial_read_rate_hz))
        self.read_timer = self.create_timer(read_period, self.read_serial)
        self.ensure_serial_connection()

        self.get_logger().info(
            f"Bridge ready: topic='{self.command_topic}' -> serial='{self.serial_port}' @ {self.baud_rate}, "
            f"encoders=({self.encoder_topic}, {self.encoder_text_topic}), read_rate={self.serial_read_rate_hz:.1f}Hz"
        )

    def ensure_serial_connection(self):
        if self.ser and self.ser.is_open:
            return

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except Exception as exc:
            self.ser = None
            self.get_logger().warn(f"Waiting for Arduino serial ({self.serial_port}): {exc}")

    def parse_encoder_line(self, line: str):
        # Expected format: ENC,<rf>,<rb>,<lb>,<lf>
        if not line.startswith('ENC,'):
            return None

        parts = line.split(',')
        if len(parts) != 5:
            self._warn_malformed_line(line)
            return None

        try:
            return [int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])]
        except ValueError:
            self._warn_malformed_line(line)
            return None

    def _warn_malformed_line(self, line: str):
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_malformed_warn_time) >= 1.0:
            self.get_logger().warn(f"Malformed encoder line: '{line}'")
            self._last_malformed_warn_time = now

    def publish_encoder_counts(self, counts):
        if counts is None:
            return

        msg = Int32MultiArray()
        msg.data = counts
        self.encoder_pub.publish(msg)

        text_msg = String()
        text_msg.data = f"rf={counts[0]},rb={counts[1]},lb={counts[2]},lf={counts[3]}"
        self.encoder_text_pub.publish(text_msg)

        self._last_encoder = counts

    def read_serial(self):
        if not self.ser or not self.ser.is_open:
            return

        # Drain pending bytes and parse complete newline-delimited frames.
        max_lines = 100
        lines_read = 0

        try:
            available = self.ser.in_waiting
            if available <= 0:
                return

            chunk = self.ser.read(available).decode('utf-8', errors='ignore')
            if not chunk:
                return

            self._serial_rx_buffer += chunk

            while '\n' in self._serial_rx_buffer and lines_read < max_lines:
                line, self._serial_rx_buffer = self._serial_rx_buffer.split('\n', 1)
                line = line.strip()
                if not line:
                    lines_read += 1
                    continue

                counts = self.parse_encoder_line(line)
                if counts is not None:
                    self.publish_encoder_counts(counts)
                lines_read += 1
        except Exception as exc:
            self.get_logger().error(f"Serial read failed: {exc}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def command_callback(self, msg: String):
        command = msg.data.strip().upper()
        if not command:
            return

        if command not in self.allowed_commands:
            self.get_logger().warn(f"Ignoring unsupported motor command: '{msg.data}'")
            return

        if not self.ser or not self.ser.is_open:
            self.get_logger().warn(f"Serial not connected, cannot send command: {command}")
            return

        try:
            self.ser.write((command + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent to Arduino: {command}")
        except Exception as exc:
            self.get_logger().error(f"Serial write failed for '{command}': {exc}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
