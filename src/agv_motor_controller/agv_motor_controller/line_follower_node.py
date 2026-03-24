#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
import time


class LineFollowerSensorArray(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Create publisher for sensor array (8 channels)
        self.publisher = self.create_publisher(Float32MultiArray, 'line_follower_sensors', 10)
        
        # Timer for periodic measurements (50ms interval = 20 Hz)
        self.timer = self.create_timer(0.05, self.read_sensors)
        
        # MUX control pins (GPIO pins for S0-S3)
        self.S0 = 17
        self.S1 = 27
        self.S2 = 22
        self.S3 = 23
        
        # Number of channels
        self.NUM_CHANNELS = 8
        
        # Store sensor values
        self.sensor_values = [0.0] * self.NUM_CHANNELS
        
        try:
            # Setup GPIO for MUX control pins
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.S0, GPIO.OUT)
            GPIO.setup(self.S1, GPIO.OUT)
            GPIO.setup(self.S2, GPIO.OUT)
            GPIO.setup(self.S3, GPIO.OUT)
            
            # Initialize I2C and ADS1115 ADC
            i2c = busio.I2C(board.SCL, board.SDA)
            ads = ADS.ADS1115(i2c)
            
            # Use channel 0 of ADS1115 (connected to MUX output)
            self.channel = AnalogIn(ads, ADS.P0)
            
            self.get_logger().info("Line Follower Sensor Array initialized successfully!")
            self.get_logger().info("Reading 8 channels (C0-C7) from CD74HC4067 MUX")
            self.get_logger().info("Publishing to 'line_follower_sensors' topic")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensors: {e}")
            raise

    def select_channel(self, channel):
        """
        Select MUX channel using binary encoding.
        Channel 0-7 uses S0-S3 pins.
        """
        # Convert channel number to binary and set GPIO pins
        GPIO.output(self.S0, (channel >> 0) & 1)
        GPIO.output(self.S1, (channel >> 1) & 1)
        GPIO.output(self.S2, (channel >> 2) & 1)
        GPIO.output(self.S3, (channel >> 3) & 1)
        
        # Small delay for MUX to settle
        time.sleep(0.001)

    def read_sensors(self):
        """Read all 8 channels from MUX and publish."""
        try:
            # Read all 8 channels
            for channel in range(self.NUM_CHANNELS):
                self.select_channel(channel)
                
                # Read voltage from ADS1115
                voltage = self.channel.voltage
                
                # Convert voltage to 0-1023 scale (simulating 10-bit ADC)
                # ADS1115 is 16-bit, so convert to a more familiar scale
                adc_value = int((voltage / 4.096) * 1023)
                
                # Clamp value between 0-1023
                adc_value = max(0, min(1023, adc_value))
                
                self.sensor_values[channel] = float(adc_value)
            
            # Create and publish message
            msg = Float32MultiArray()
            msg.data = self.sensor_values
            self.publisher.publish(msg)
            
            # Print formatted output
            self.print_formatted_output()
            
        except Exception as e:
            self.get_logger().error(f"Error reading sensors: {e}")

    def print_formatted_output(self):
        """Print sensor values in a nice formatted way for debugging."""
        output = "\n" + "="*80 + "\n"
        output += "LINE FOLLOWER SENSOR ARRAY - CD74HC4067 MUX\n"
        output += "="*80 + "\n"
        
        # Create a visual representation
        output += "Channel │ Value │ Analog Bar Chart\n"
        output += "────────┼───────┼──────────────────────────────────────\n"
        
        for i, value in enumerate(self.sensor_values):
            # Normalize value for bar chart (0-50 characters)
            bar_length = int((value / 1023) * 50)
            bar = "█" * bar_length + "░" * (50 - bar_length)
            
            output += f"   C{i}   │ {value:4.0f}  │ {bar}\n"
        
        output += "="*80 + "\n"
        
        # Summary statistics
        avg_value = sum(self.sensor_values) / len(self.sensor_values)
        max_value = max(self.sensor_values)
        min_value = min(self.sensor_values)
        
        output += f"Average: {avg_value:.1f}  │  Max: {max_value:.0f}  │  Min: {min_value:.0f}\n"
        output += "="*80
        
        self.get_logger().info(output)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerSensorArray()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup GPIO
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
