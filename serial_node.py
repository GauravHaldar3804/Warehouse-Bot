import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200)

        self.line_pub = self.create_publisher(Float32, '/line_error', 10)
        self.enc_pub = self.create_publisher(String, '/encoder_data', 10)

        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()

            # Ignore debug lines
            if not line or line.startswith('#'):
                return

            data = line.split(',')

            if len(data) < 5:
                return

            error = float(data[0])

            msg = Float32()
            msg.data = error
            self.line_pub.publish(msg)

            enc_msg = String()
            enc_msg.data = ','.join(data[1:5])
            self.enc_pub.publish(enc_msg)

        except Exception as e:
            self.get_logger().warn(f"Error reading serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
