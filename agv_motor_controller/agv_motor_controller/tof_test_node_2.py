import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool

import board
import busio
import time
import RPi.GPIO as GPIO

import adafruit_vl53l0x

# GPIO pins for XSHUT
XSHUT_1 = 22
XSHUT_2 = 27
XSHUT_3 = 23  # Uncomment when 3rd sensor is available
BUZZER_PIN = 24

class DualVL53L0X(Node):
    def __init__(self):
        super().__init__('dual_vl53l0x_node')

        self.declare_parameter('buzzer_active_low', False)
        self.buzzer_active_low = bool(self.get_parameter('buzzer_active_low').value)

        # Publishers
        self.pub1 = self.create_publisher(Float32, '/tof1/distance', 10)
        self.pub2 = self.create_publisher(Float32, '/tof2/distance', 10)
        self.pub3 = self.create_publisher(Float32, '/tof3/distance', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/tof/obstacle_detected', 10)
        self.motor_command_pub = self.create_publisher(String, 'motor_command', 10)
        self.motor_command_sub = self.create_subscription(String, 'motor_command', self.motor_command_callback, 10)

        # Obstacle gating: 40 cm threshold, 2 sec persistence
        self.obstacle_threshold_mm = 400.0
        self.persistence_seconds = 0.5
        self.obstacle_active = False
        self.obstacle_below_since = None
        self.clear_since = None
        self.running_assumed = False
        self.resume_needed = False
        self._ignore_next_command = None

        # Buzzer pattern: beep while obstacle is active.
        self.beep_on_seconds = 0.2
        self.beep_off_seconds = 0.2
        self._beep_state_on = False
        self._last_beep_toggle = time.monotonic()
        self.buzzer_obstacle_active = False

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(XSHUT_1, GPIO.OUT)
        GPIO.setup(XSHUT_2, GPIO.OUT)
        GPIO.setup(XSHUT_3, GPIO.OUT)
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        self.set_buzzer_output(False)

        # Turn OFF both sensors
        GPIO.output(XSHUT_1, GPIO.LOW)
        GPIO.output(XSHUT_2, GPIO.LOW)
        GPIO.output(XSHUT_3, GPIO.LOW)
        time.sleep(0.5)

        # Initialize I2C
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # --- Sensor 1 ---
        GPIO.output(XSHUT_1, GPIO.HIGH)
        time.sleep(0.5)

        self.sensor1 = adafruit_vl53l0x.VL53L0X(self.i2c)
        self.sensor1.set_address(0x30)

        # --- Sensor 2 ---
        GPIO.output(XSHUT_2, GPIO.HIGH)
        time.sleep(0.5)

        self.sensor2 = adafruit_vl53l0x.VL53L0X(self.i2c)
        self.sensor2.set_address(0x31)

        # # --- Sensor 3 ---
        GPIO.output(XSHUT_3, GPIO.HIGH)
        time.sleep(0.5)
        
        self.sensor3 = adafruit_vl53l0x.VL53L0X(self.i2c)
        self.sensor3.set_address(0x32)

        self.get_logger().info("Sensors initialized successfully")

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.read_sensors)

    def publish_motor_command(self, command: str):
        self._ignore_next_command = command
        msg = String()
        msg.data = command
        self.motor_command_pub.publish(msg)
        self.get_logger().info(f"TOF safety command: {command}")

    def motor_command_callback(self, msg: String):
        command = msg.data.strip().upper()
        if not command:
            return

        # Ignore loopback from this same node's publish call.
        if self._ignore_next_command == command:
            self._ignore_next_command = None
            return

        if command == 'START':
            self.running_assumed = True
        elif command == 'STOP':
            self.running_assumed = False

    def read_sensors(self):
        msg1 = Float32()
        msg2 = Float32()
        msg3 = Float32()

        try:
            dist1 = self.sensor1.range  # in mm
            msg1.data = float(dist1)
        except:
            msg1.data = -1.0
            dist1 = -1

        try:
            dist2 = self.sensor2.range
            msg2.data = float(dist2)
        except:
            msg2.data = -1.0
            dist2 = -1

        try:
            dist3 = self.sensor3.range
            msg3.data = float(dist3)
        except:
            msg3.data = -1.0
            dist3 = -1

        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)

        now = time.monotonic()
        distances = [dist1, dist2, dist3]
        valid_distances = [d for d in distances if d >= 0]

        any_below = any(d <= self.obstacle_threshold_mm for d in valid_distances)
        all_clear = len(valid_distances) == 3 and all(d > self.obstacle_threshold_mm for d in valid_distances)
        self.buzzer_obstacle_active = any_below

        if not self.obstacle_active:
            if any_below:
                if self.obstacle_below_since is None:
                    self.obstacle_below_since = now
                elif (now - self.obstacle_below_since) >= self.persistence_seconds:
                    self.publish_motor_command('STOP')
                    self.resume_needed = self.running_assumed
                    self.running_assumed = False
                    self.obstacle_active = True
                    self.clear_since = None
            else:
                self.obstacle_below_since = None
        else:
            if all_clear:
                if self.clear_since is None:
                    self.clear_since = now
                elif (now - self.clear_since) >= self.persistence_seconds:
                    if self.resume_needed:
                        self.publish_motor_command('START')
                        self.running_assumed = True
                    self.obstacle_active = False
                    self.resume_needed = False
                    self.obstacle_below_since = None
            else:
                self.clear_since = None

        obstacle_msg = Bool()
        obstacle_msg.data = self.obstacle_active
        self.obstacle_pub.publish(obstacle_msg)
        self.update_buzzer(now)
        
        # Print values to terminal
        # print(f"TOF Distance [mm]  |  Sensor 1: {dist1:6.1f}  |  Sensor 2: {dist2:6.1f}", flush=True)
        print(f"TOF Distance [mm]  |  Sensor 1: {dist1:6.1f}  |  Sensor 2: {dist2:6.1f}  |  Sensor 3: {dist3:6.1f}", flush=True)

    def update_buzzer(self, now: float):
        if not self.buzzer_obstacle_active:
            self._beep_state_on = False
            self.set_buzzer_output(False)
            self._last_beep_toggle = now
            return

        interval = self.beep_on_seconds if self._beep_state_on else self.beep_off_seconds
        if (now - self._last_beep_toggle) >= interval:
            self._beep_state_on = not self._beep_state_on
            self.set_buzzer_output(self._beep_state_on)
            self._last_beep_toggle = now

    def set_buzzer_output(self, enabled: bool):
        gpio_on = GPIO.LOW if self.buzzer_active_low else GPIO.HIGH
        gpio_off = GPIO.HIGH if self.buzzer_active_low else GPIO.LOW
        GPIO.output(BUZZER_PIN, gpio_on if enabled else gpio_off)

    def destroy_node(self):
        self.set_buzzer_output(False)
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualVL53L0X()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()