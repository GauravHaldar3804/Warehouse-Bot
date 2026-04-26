#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MotorCommandSerialBridge(Node):
    def __init__(self):
        super().__init__('motor_command_serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('command_topic', 'motor_command')

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.command_topic = self.get_parameter('command_topic').value

        self.ser = None

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

        self.reconnect_timer = self.create_timer(1.0, self.ensure_serial_connection)
        self.ensure_serial_connection()

        self.get_logger().info(
            f"Bridge ready: topic='{self.command_topic}' -> serial='{self.serial_port}' @ {self.baud_rate}"
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
