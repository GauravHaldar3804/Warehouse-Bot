import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import threading
from collections import deque


class EncoderTestNode(Node):

    def __init__(self):
        super().__init__('encoder_test_node')

        # GPIO pins for encoder channels
        self.ENCODER_A = 23
        self.ENCODER_B = 24
        
        # Encoder parameters
        self.TICKS_PER_REV = 3  # Typical: 3 ticks per revolution for hall effect
        self.WHEEL_DIAMETER = 0.065  # meters (adjust for your wheel)
        
        # State tracking
        self.tick_count = 0
        self.last_tick_time = time.time()
        self.lock = threading.Lock()
        
        # Debouncing settings (in seconds)
        self.DEBOUNCE_TIME = 0.01  # 10ms debounce
        self.last_interrupt_time = 0
        
        # Speed calculation
        self.rpm = 0.0
        self.speed_mps = 0.0
        self.tick_history = deque(maxlen=10)  # Store last 10 tick times
        
        # GPIO setup
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Use both RISING and FALLING edges for better accuracy
            # Increased bouncetime to 20ms for hall effect sensors
            GPIO.add_event_detect(
                self.ENCODER_A,
                GPIO.BOTH,
                callback=self.encoder_callback,
                bouncetime=20
            )
            GPIO.add_event_detect(
                self.ENCODER_B,
                GPIO.BOTH,
                callback=self.encoder_callback,
                bouncetime=20
            )
            
            self.get_logger().info("Hall effect encoder initialized successfully")
            
        except RuntimeError as e:
            self.get_logger().error(f"GPIO setup error: {e}")
            raise

        # Timer for periodic updates
        self.timer = self.create_timer(0.5, self.print_encoder_data)

    def encoder_callback(self, channel):
        """
        Handle encoder interrupts with debouncing.
        Works with hall effect sensors using quadrature encoding.
        """
        current_time = time.time()
        
        # Software debouncing: ignore if interrupt comes too soon
        if (current_time - self.last_interrupt_time) < self.DEBOUNCE_TIME:
            return
        
        self.last_interrupt_time = current_time
        
        with self.lock:
            # Read both channels
            a_state = GPIO.input(self.ENCODER_A)
            b_state = GPIO.input(self.ENCODER_B)
            
            # Quadrature encoding logic for direction detection
            # This determines rotation direction
            if channel == self.ENCODER_A:
                if a_state == 1 and b_state == 0:
                    self.tick_count += 1
                elif a_state == 1 and b_state == 1:
                    self.tick_count += 1
                elif a_state == 0 and b_state == 1:
                    self.tick_count -= 1
                elif a_state == 0 and b_state == 0:
                    self.tick_count -= 1
            else:  # channel == ENCODER_B
                if b_state == 1 and a_state == 0:
                    self.tick_count += 1
                elif b_state == 1 and a_state == 1:
                    self.tick_count += 1
                elif b_state == 0 and a_state == 1:
                    self.tick_count -= 1
                elif b_state == 0 and a_state == 0:
                    self.tick_count -= 1
            
            # Store tick timestamp for speed calculation
            self.tick_history.append(current_time)

    def calculate_speed(self):
        """
        Calculate RPM and linear speed from tick history.
        """
        with self.lock:
            if len(self.tick_history) < 2:
                return
            
            # Calculate time interval over last measurements
            time_diff = self.tick_history[-1] - self.tick_history[0]
            
            if time_diff > 0:
                # Number of ticks in this interval
                ticks_in_interval = len(self.tick_history) - 1
                
                # Calculate RPM
                # RPM = (ticks / TICKS_PER_REV) / (time_diff / 60)
                revolutions = ticks_in_interval / self.TICKS_PER_REV
                time_minutes = time_diff / 60.0
                self.rpm = revolutions / time_minutes if time_minutes > 0 else 0
                
                # Calculate linear speed (m/s)
                # speed = RPM * (π * D) / 60
                self.speed_mps = abs(self.rpm) * (3.14159 * self.WHEEL_DIAMETER) / 60.0

    def print_encoder_data(self):
        """
        Print encoder data including ticks, RPM, and speed.
        """
        with self.lock:
            current_tick = self.tick_count
        
        self.calculate_speed()
        
        # Determine direction
        direction = "Forward" if self.tick_count >= 0 else "Backward"
        
        self.get_logger().info(
            f"Ticks: {current_tick:6d} | "
            f"RPM: {self.rpm:6.2f} | "
            f"Speed: {self.speed_mps:6.3f} m/s | "
            f"Direction: {direction}"
        )

    def destroy_node(self):
        """
        Clean up GPIO and shutdown node.
        """
        try:
            GPIO.cleanup()
            self.get_logger().info("GPIO cleanup completed")
        except Exception as e:
            self.get_logger().error(f"Error during GPIO cleanup: {e}")
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()