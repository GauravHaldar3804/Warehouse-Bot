import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Open serial port (Arduino connection)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Publishers
        self.line_pub = self.create_publisher(Float32, '/line_error', 10)
        self.enc_pub = self.create_publisher(String, '/encoder_data', 10)

        # Timer to read serial continuously
        self.timer = self.create_timer(0.05, self.read_serial)

        # Sequence number for readable serial logs.
        self.packet_count = 0

    def log_serial_packet(self, raw_line, error, encoders):
        """Print a parsed serial packet in a consistent human-readable format."""
        self.packet_count += 1
        self.get_logger().info(
            f"[SERIAL {self.packet_count:05d}] "
            f"raw='{raw_line}' | "
            f"line_error={error:.2f} | "
            f"encoders=[E1:{encoders[0]:.0f}, E2:{encoders[1]:.0f}, E3:{encoders[2]:.0f}, E4:{encoders[3]:.0f}]"
        )

    def read_serial(self):
        try:
            # Read and decode safely
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            # Ignore empty lines
            if not line:
                return

            # Print calibration/debug lines from firmware.
            if line.startswith('#'):
                self.get_logger().info(f"CALIB: {line}")
                return

            data = line.split(',')

            # Expect at least error + 4 encoder values
            if len(data) < 5:
                return

            # Safe conversion
            try:
                error = float(data[0])
                encoders = [float(v.strip()) for v in data[1:5]]
            except ValueError:
                return

            # Publish line error
            msg = Float32()
            msg.data = error
            self.line_pub.publish(msg)

            # Publish encoder data
            enc_msg = String()
            enc_msg.data = ','.join(data[1:5])
            self.enc_pub.publish(enc_msg)

            # Print serial data in a proper readable format.
            self.log_serial_packet(line, error, encoders)

        except Exception as e:
            self.get_logger().warn(f"Error reading serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
