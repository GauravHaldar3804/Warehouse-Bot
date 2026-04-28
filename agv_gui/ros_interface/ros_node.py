import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Bool


class DashboardRosNode(Node):
	"""ROS bridge node for AGV GUI pages."""

	def __init__(self):
		super().__init__('agv_dashboard_node')

		self._status = {
			'connection': 'ROS Node Active',
			'battery': '--',
			'position': 'HOME-1',
			'target': '--',
			'state': 'Waiting for mission',
			'last_motor_command': 'NONE',
			'last_update_age_sec': None,
			'obstacle_detected': False,
			'battery_voltage': '--',
			'battery_current': '--',
			'mission_progress': '--',
			'system_tof': 'Unknown',
			'system_camera': 'Unknown',
			'system_motor': 'Unknown',
			'system_battery': 'Unknown',
		}
		self._last_tof_time = 0.0
		self._last_camera_time = 0.0
		self._last_motor_time = 0.0
		self._last_battery_time = 0.0
		self._last_update_time = 0.0

		self._camera_qos = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			history=HistoryPolicy.KEEP_LAST,
			depth=1,
		)

		self.create_subscription(BatteryState, 'battery_state', self._on_battery_state, 10)
		self.create_subscription(String, 'camera/qr_code', self._on_qr_code, self._camera_qos)
		self.create_subscription(BatteryState, 'battery_state', self._on_battery_state, 10)
		self.create_subscription(String, 'battery_metrics', self._on_battery_metrics, 10)
		self.create_subscription(String, 'camera/qr_code', self._on_qr_code, self._camera_qos)
		self.path_query_publisher = self.create_publisher(String, 'path_query', 10)
		self.motor_command_publisher = self.create_publisher(String, 'motor_command', 10)

		self.create_subscription(String, 'path_result', self._on_path_result, 10)
		self.create_subscription(String, 'motor_command', self._on_motor_command, 10)
		self.create_subscription(Bool, '/tof/obstacle_detected', self._on_obstacle, 10)

		self.get_logger().info('AGV dashboard ROS bridge started')

	def _touch(self):
		self._last_update_time = time.time()

	def _on_battery_state(self, msg: BatteryState):
		if math.isfinite(msg.percentage):
			self._status['battery'] = f"{int(round(msg.percentage * 100.0))}%"
		elif math.isfinite(msg.voltage):
			self._status['battery'] = f"{msg.voltage:.2f} V"
		if math.isfinite(msg.voltage):
			self._status['battery_voltage'] = f"{msg.voltage:.2f} V"
		if math.isfinite(msg.current):
			self._status['battery_current'] = f"{msg.current:.2f} A"
		self._status['system_battery'] = 'Active'
		self._last_battery_time = time.time()
		self._touch()

	def _on_battery_metrics(self, msg: String):
		try:
			payload = json.loads(msg.data)
		except Exception:
			return

		pct_text = payload.get('percentage_text')
		if isinstance(pct_text, str) and pct_text:
			self._status['battery'] = pct_text
		self._touch()

	def _on_qr_code(self, msg: String):
		raw = (msg.data or '').strip()
		if not raw:
			return
		node_label = raw.split('|', 1)[0].strip().upper()
		self._status['system_camera'] = 'Active'
		self._last_camera_time = time.time()
		self._status['position'] = node_label
		self._touch()

	def _on_path_result(self, msg: String):
		try:
			payload = json.loads(msg.data)
		except Exception:
			return

		if payload.get('status') == 'success':
			goal = str(payload.get('goal', '--')).upper()
			self._status['target'] = goal if goal else '--'
			self._status['state'] = 'Mission loaded'
			self._touch()

	def _on_motor_command(self, msg: String):
		cmd = (msg.data or '').strip().upper()
		if not cmd:
			return

		self._status['last_motor_command'] = cmd
		if cmd == 'START':
			self._status['state'] = 'Navigating'
		elif cmd == 'STOP':
			self._status['state'] = 'Stopped'
		elif cmd in ['LEFT', 'RIGHT', 'STRAIGHT', 'UTURN']:
			self._status['state'] = f'Executing {cmd}'
		else:
			self._status['state'] = cmd
		self._touch()

	def send_path_query(self, start: str, goal: str):
		start_node = (start or '').strip().upper()
		goal_node = (goal or '').strip().upper()
		if not start_node or not goal_node:
			return False

		payload = json.dumps({'start': start_node, 'goal': goal_node})
		self.path_query_publisher.publish(String(data=payload))
		self.get_logger().info(f'Sent path query: {start_node} -> {goal_node}')

		# Log to activity log
		if self.activity_log:
			self.activity_log.add_log(f"Path query sent: {start_node} → {goal_node}")
		return True

	def send_motor_command(self, command: str):
		cmd = (command or '').strip().upper()
		if not cmd:
			return False

		self.motor_command_publisher.publish(String(data=cmd))
		self.get_logger().info(f'Sent motor command: {cmd}')

		# Log to activity log
		if self.activity_log:
			self.activity_log.add_log(f"Motor command sent: {cmd}")
		return True

	def _on_obstacle(self, msg: Bool):
		if msg.data:
			self._status['state'] = 'Obstacle detected'
		elif self._status['last_motor_command'] == 'START':
			self._status['state'] = 'Navigating'
		self._touch()

	def get_status_snapshot(self):
		snapshot = dict(self._status)
		if self._last_update_time > 0.0:
			age = max(0.0, time.time() - self._last_update_time)
			snapshot['last_update_age_sec'] = age
			if age > 5.0:
				snapshot['connection'] = 'Waiting for topic updates'
			else:
				snapshot['connection'] = 'Topics active'
		else:
			snapshot['last_update_age_sec'] = None
			snapshot['connection'] = 'Waiting for topic updates'

		# Update system component status based on last activity
		current_time = time.time()
		if current_time - self._last_tof_time > 10.0:
			snapshot['system_tof'] = 'Inactive'
		if current_time - self._last_camera_time > 10.0:
			snapshot['system_camera'] = 'Inactive'
		if current_time - self._last_motor_time > 10.0:
			snapshot['system_motor'] = 'Inactive'
		if current_time - self._last_battery_time > 10.0:
			snapshot['system_battery'] = 'Inactive'

		return snapshot
