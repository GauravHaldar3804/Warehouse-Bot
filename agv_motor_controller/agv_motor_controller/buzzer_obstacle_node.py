import time

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ObstacleBuzzerNode(Node):
    def __init__(self):
        super().__init__('obstacle_buzzer_node')

        self.buzzer_pin = 17
        self.beep_on_seconds = 0.2
        self.beep_off_seconds = 0.2

        self.obstacle_detected = False
        self._beep_state_on = False
        self._last_toggle = time.monotonic()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.buzzer_pin, GPIO.OUT, initial=GPIO.LOW)

        self.create_subscription(Bool, '/tof/obstacle_detected', self.obstacle_callback, 10)
        self.timer = self.create_timer(0.05, self.beep_loop)

        self.get_logger().info('Obstacle buzzer node started on GPIO17')

    def obstacle_callback(self, msg: Bool):
        self.obstacle_detected = msg.data
        if not self.obstacle_detected:
            self._beep_state_on = False
            GPIO.output(self.buzzer_pin, GPIO.LOW)

    def beep_loop(self):
        now = time.monotonic()

        if not self.obstacle_detected:
            return

        interval = self.beep_on_seconds if self._beep_state_on else self.beep_off_seconds
        if (now - self._last_toggle) >= interval:
            self._beep_state_on = not self._beep_state_on
            GPIO.output(self.buzzer_pin, GPIO.HIGH if self._beep_state_on else GPIO.LOW)
            self._last_toggle = now

    def destroy_node(self):
        GPIO.output(self.buzzer_pin, GPIO.LOW)
        GPIO.cleanup(self.buzzer_pin)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleBuzzerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
