import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TestLineControlNode(Node):

    def __init__(self):
        super().__init__('test_line_control_node')

        self.create_subscription(Float32, '/line_error', self.callback, 10)

        self.base_speed = 120
        self.Kp = 40

        self.get_logger().info("Test Line Control Node Started")

    def callback(self, msg):
        error = msg.data

        if error == 9999:
            left = 0
            right = 0
        else:
            correction = self.Kp * error
            left = self.base_speed - correction
            right = self.base_speed + correction

        left = max(0, min(255, left))
        right = max(0, min(255, right))

        self.get_logger().info(f"ERROR:{error:.2f}  L:{left:.1f}  R:{right:.1f}")


def main(args=None):
    rclpy.init(args=args)
    node = TestLineControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

