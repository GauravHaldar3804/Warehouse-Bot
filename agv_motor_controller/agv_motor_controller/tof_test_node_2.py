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
        self.declare_parameter('sensor_init_retries', 5)
        self.declare_parameter('sensor_init_delay_sec', 0.5)
        self.buzzer_active_low = bool(self.get_parameter('buzzer_active_low').value)
        self.sensor_init_retries = int(self.get_parameter('sensor_init_retries').value)
        self.sensor_init_delay_sec = float(self.get_parameter('sensor_init_delay_sec').value)

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
        self._last_sensor_warn_time = {
            'sensor1': 0.0,
            'sensor2': 0.0,
            'sensor3': 0.0,
        }
        self._sensor_warn_interval_sec = 2.0

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

        self.sensor1 = self._init_sensor_with_retry(XSHUT_1, 0x30, 'sensor1')
        self.sensor2 = self._init_sensor_with_retry(XSHUT_2, 0x31, 'sensor2')
        self.sensor3 = self._init_sensor_with_retry(XSHUT_3, 0x32, 'sensor3')

        initialized = sum(s is not None for s in [self.sensor1, self.sensor2, self.sensor3])
        if initialized == 0:
            self.get_logger().error('No TOF sensors initialized. Node will keep running and retry in read loop.')
        else:
            self.get_logger().info(f'Sensors initialized successfully: {initialized}/3')

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.read_sensors)

    def _init_sensor_with_retry(self, xshut_pin: int, new_addr: int, name: str):
        for attempt in range(1, self.sensor_init_retries + 1):
            try:
                GPIO.output(xshut_pin, GPIO.LOW)
                time.sleep(self.sensor_init_delay_sec)
                GPIO.output(xshut_pin, GPIO.HIGH)
                time.sleep(self.sensor_init_delay_sec)

                sensor = adafruit_vl53l0x.VL53L0X(self.i2c)
                sensor.set_address(new_addr)
                self.get_logger().info(f'{name} initialized at 0x{new_addr:02X} (attempt {attempt})')
                return sensor
            except Exception as exc:
                self.get_logger().warn(
                    f'{name} init failed (attempt {attempt}/{self.sensor_init_retries}): {exc}'
                )
                time.sleep(self.sensor_init_delay_sec)

        self.get_logger().error(f'{name} failed to initialize after {self.sensor_init_retries} attempts')
        return None

    def _read_sensor_range(self, sensor, sensor_name: str):
        if sensor is None:
            return -1.0, -1

        try:
            dist = sensor.range
            return float(dist), int(dist)
        except Exception as exc:
            now = time.monotonic()
            if (now - self._last_sensor_warn_time[sensor_name]) >= self._sensor_warn_interval_sec:
                self.get_logger().warn(f'{sensor_name} read failed: {exc}')
                self._last_sensor_warn_time[sensor_name] = now
            return -1.0, -1

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

        msg1.data, dist1 = self._read_sensor_range(self.sensor1, 'sensor1')
        msg2.data, dist2 = self._read_sensor_range(self.sensor2, 'sensor2')
        msg3.data, dist3 = self._read_sensor_range(self.sensor3, 'sensor3')

        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)

        now = time.monotonic()
        distances = [dist1, dist2, dist3]
        valid_distances = [d for d in distances if d >= 0]

        any_below = any(d <= self.obstacle_threshold_mm for d in valid_distances)
        all_clear = len(valid_distances) >= 2 and all(d > self.obstacle_threshold_mm for d in valid_distances)
        self.buzzer_obstacle_active = any_below

        if not self.obstacle_active:
            if any_below:
                if self.obstacle_below_since is None:
                    self.obstacle_below_since = now
                elif (now - self.obstacle_below_since) >= self.persistence_seconds:
                    self.publish_motor_command('STOP')
                    self.publish_motor_command('OBSTACLE')
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
                    self.publish_motor_command('CLEAR')
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