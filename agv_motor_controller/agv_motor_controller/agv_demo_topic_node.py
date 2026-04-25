#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Bool, Float32


class AgvDemoTopicNode(Node):
    """Publish demo AGV data on runtime topics when hardware is unavailable."""

    def __init__(self):
        super().__init__('agv_demo_topic_node')

        self.battery_state_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.battery_metrics_pub = self.create_publisher(String, 'battery_metrics', 10)
        self.battery_status_pub = self.create_publisher(String, 'battery_status_text', 10)

        self.motor_cmd_pub = self.create_publisher(String, 'motor_command', 10)

        self.tof1_pub = self.create_publisher(Float32, '/tof1/distance', 10)
        self.tof2_pub = self.create_publisher(Float32, '/tof2/distance', 10)
        self.tof3_pub = self.create_publisher(Float32, '/tof3/distance', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/tof/obstacle_detected', 10)

        self.tick = 0
        self.last_obstacle_active = False

        self.create_timer(1.0, self.publish_battery)
        self.create_timer(0.2, self.publish_tof)
        self.get_logger().info('AGV demo topic node started')

    def publish_battery(self):
        # Battery oscillates from ~35% to ~95% for demo visibility.
        pct = 0.65 + 0.30 * math.sin(self.tick / 14.0)
        pct = max(0.0, min(1.0, pct))
        voltage = 10.8 + (12.6 - 10.8) * pct
        current = 0.8 + 0.5 * math.sin(self.tick / 8.0)
        power = voltage * current

        state = BatteryState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.voltage = float(voltage)
        state.current = float(current)
        state.charge = math.nan
        state.capacity = 10.0
        state.design_capacity = 10.0
        state.percentage = float(pct)
        state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        state.present = True
        state.location = 'main_battery_pack'
        state.serial_number = 'DEMO-INA219'
        self.battery_state_pub.publish(state)

        pct_display = int(round(pct * 100.0))
        metrics = {
            'voltage_v': round(voltage, 3),
            'current_a': round(current, 3),
            'power_w': round(power, 3),
            'percentage': round(pct, 3),
            'percentage_text': f'{pct_display}%',
            'level': 'High' if pct >= 0.8 else ('Medium' if pct >= 0.5 else 'Low'),
            'status_text': 'Discharging',
            'summary_text': f'Battery {pct_display}% | {voltage:.2f}V | {current:.2f}A',
            'sensor_ok': True,
        }
        self.battery_metrics_pub.publish(String(data=json.dumps(metrics)))
        self.battery_status_pub.publish(String(data=metrics['summary_text']))

        self.tick += 1

    def publish_tof(self):
        # Periodically generate near obstacle events.
        obstacle_active = (self.tick % 25) in [18, 19, 20]

        base_far = 0.75
        near = 0.16

        d1 = near if obstacle_active else base_far + 0.05 * math.sin(self.tick / 3.0)
        d2 = near if obstacle_active else base_far + 0.04 * math.cos(self.tick / 4.0)
        d3 = near if obstacle_active else base_far + 0.06 * math.sin(self.tick / 5.0)

        self.tof1_pub.publish(Float32(data=float(max(0.05, d1))))
        self.tof2_pub.publish(Float32(data=float(max(0.05, d2))))
        self.tof3_pub.publish(Float32(data=float(max(0.05, d3))))
        self.obstacle_pub.publish(Bool(data=obstacle_active))

        # Mimic the ToF node's safety command behavior: publish only on state change.
        if obstacle_active and not self.last_obstacle_active:
            self.motor_cmd_pub.publish(String(data='STOP'))
        elif (not obstacle_active) and self.last_obstacle_active:
            self.motor_cmd_pub.publish(String(data='START'))

        self.last_obstacle_active = obstacle_active



def main(args=None):
    rclpy.init(args=args)
    node = AgvDemoTopicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
