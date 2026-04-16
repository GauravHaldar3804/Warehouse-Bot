#!/usr/bin/env python3

import json
import math

import board
import busio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from adafruit_ina219 import INA219


class INA219BatteryNode(Node):
    def __init__(self):
        super().__init__('ina219_battery_node')

        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('battery_full_voltage', 12.6)
        self.declare_parameter('battery_empty_voltage', 10.8)
        self.declare_parameter('design_capacity_ah', 10.0)
        self.declare_parameter('state_topic', 'battery_state')
        self.declare_parameter('metrics_topic', 'battery_metrics')

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.battery_full_voltage = float(self.get_parameter('battery_full_voltage').value)
        self.battery_empty_voltage = float(self.get_parameter('battery_empty_voltage').value)
        self.design_capacity_ah = float(self.get_parameter('design_capacity_ah').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.metrics_topic = str(self.get_parameter('metrics_topic').value)

        self.battery_pub = self.create_publisher(BatteryState, self.state_topic, 10)
        self.metrics_pub = self.create_publisher(String, self.metrics_topic, 10)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.ina219 = INA219(i2c, addr=self.i2c_address)

        timer_period = max(0.05, 1.0 / max(0.1, self.publish_rate_hz))
        self.timer = self.create_timer(timer_period, self.publish_battery_state)

        self.get_logger().info(
            f"INA219 battery node started: addr=0x{self.i2c_address:02X}, "
            f"rate={self.publish_rate_hz:.2f}Hz, topics=({self.state_topic}, {self.metrics_topic})"
        )
        self.get_logger().info(
            "Parameters: "
            f"publish_rate_hz={self.publish_rate_hz:.2f}, "
            f"i2c_address=0x{self.i2c_address:02X}, "
            f"battery_full_voltage={self.battery_full_voltage:.2f}, "
            f"battery_empty_voltage={self.battery_empty_voltage:.2f}, "
            f"design_capacity_ah={self.design_capacity_ah:.2f}, "
            f"state_topic={self.state_topic}, "
            f"metrics_topic={self.metrics_topic}"
        )

    def _estimate_percentage(self, voltage: float) -> float:
        denom = self.battery_full_voltage - self.battery_empty_voltage
        if denom <= 0.001:
            return 0.0
        return max(0.0, min(1.0, (voltage - self.battery_empty_voltage) / denom))

    def _status_from_current(self, current_a: float, percentage: float) -> int:
        if percentage >= 0.98 and abs(current_a) < 0.1:
            return BatteryState.POWER_SUPPLY_STATUS_FULL
        if current_a < -0.05:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        if current_a > 0.05:
            return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

    def publish_battery_state(self):
        try:
            bus_v = float(self.ina219.bus_voltage)
            shunt_mv = float(self.ina219.shunt_voltage)
            current_ma = float(self.ina219.current)
            power_mw = float(self.ina219.power)

            load_v = bus_v + (shunt_mv / 1000.0)
            current_a = current_ma / 1000.0
            power_w = power_mw / 1000.0
            percentage = self._estimate_percentage(load_v)

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.voltage = load_v
            msg.current = current_a
            msg.charge = math.nan
            msg.capacity = self.design_capacity_ah
            msg.design_capacity = self.design_capacity_ah
            msg.percentage = percentage
            msg.power_supply_status = self._status_from_current(current_a, percentage)
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            msg.present = True
            msg.location = 'main_battery_pack'
            msg.serial_number = 'INA219'

            self.battery_pub.publish(msg)

            metrics = {
                'voltage_v': round(load_v, 3),
                'bus_voltage_v': round(bus_v, 3),
                'shunt_mv': round(shunt_mv, 3),
                'current_a': round(current_a, 3),
                'power_w': round(power_w, 3),
                'percentage': round(percentage, 3),
            }
            self.metrics_pub.publish(String(data=json.dumps(metrics)))

        except Exception as exc:
            self.get_logger().warn(f'INA219 read failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = INA219BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
