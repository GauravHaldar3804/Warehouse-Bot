#!/usr/bin/env python3
"""Realtime OpenCV grid visualizer for AGV map nodes."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from .grid_path_planner import GridPathPlanner


class RealtimeGridVisualizerNode(Node):
    """Render the planner grid and node labels in an OpenCV window."""

    def __init__(self):
        super().__init__('realtime_grid_visualizer_node')

        # Printed map is 3x2 cells, which corresponds to 4x3 intersection nodes.
        self.planner = GridPathPlanner(num_cols=4, num_rows=3)

        self.declare_parameter('window_width', 1200)
        self.declare_parameter('window_height', 800)
        self.declare_parameter('padding_px', 60)
        self.declare_parameter('show_labels', True)
        self.declare_parameter('grid_square_cm', 80.0)
        self.declare_parameter('agv_size_cm', 40.0)
        self.declare_parameter('agv_angle_offset_deg', 0.0)

        self.window_width = int(self.get_parameter('window_width').value)
        self.window_height = int(self.get_parameter('window_height').value)
        self.padding_px = int(self.get_parameter('padding_px').value)
        self.show_labels = bool(self.get_parameter('show_labels').value)
        self.grid_square_cm = float(self.get_parameter('grid_square_cm').value)
        self.agv_size_cm = float(self.get_parameter('agv_size_cm').value)
        self.agv_angle_offset_deg = float(self.get_parameter('agv_angle_offset_deg').value)

        # 1 world unit is one grid square (80 cm), so AGV 40 cm => 0.5 world units.
        if self.grid_square_cm <= 0.0:
            self.grid_square_cm = 80.0
        self.agv_size_world = max(0.05, self.agv_size_cm / self.grid_square_cm)

        # Live pose from camera_test topic.
        self.current_node_label = 'HOME-1'
        self.current_angle_deg = 0.0
        self.last_qr_raw = 'N/A'

        self.background_color = (235, 235, 235)
        self.lane_color = (0, 0, 0)
        self.dock_color = (207, 224, 233)

        self.window_name = 'AGV Realtime Grid Visualizer'
        self.timer = self.create_timer(0.1, self.render)

        # Match camera_test publisher QoS (BEST_EFFORT), otherwise no messages are received.
        qr_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.qr_sub = self.create_subscription(
            String,
            'camera/qr_code',
            self.qr_callback,
            qr_qos,
        )

        self.get_logger().info('Realtime grid visualizer started')
        self.get_logger().info('Press q in the OpenCV window to quit')
        self.get_logger().info('Subscribed to camera/qr_code for AGV pose updates')

    def qr_callback(self, msg: String):
        raw = (msg.data or '').strip()
        self.last_qr_raw = raw if raw else 'N/A'
        if not raw:
            return

        # camera_test format: "<NODE> | Angle: <deg>"
        parts = [p.strip() for p in raw.split('|')]
        node_label = parts[0].upper() if parts else ''
        angle_deg = self.current_angle_deg

        if len(parts) > 1 and ':' in parts[1]:
            try:
                angle_text = parts[1].split(':', 1)[1].strip()
                angle_deg = float(angle_text)
            except ValueError:
                pass

        if node_label in self.planner.nodes:
            self.current_node_label = node_label

        self.current_angle_deg = angle_deg

    def compute_layout(self):
        # World extents include lower dock area so the whole printed shape is centered.
        x_min, x_max = -0.25, 3.25
        y_min, y_max = -1.30, 2.10

        world_w = x_max - x_min
        world_h = y_max - y_min
        avail_w = max(1.0, float(self.window_width - 2 * self.padding_px))
        avail_h = max(1.0, float(self.window_height - 2 * self.padding_px))

        self.scale = min(avail_w / world_w, avail_h / world_h)
        draw_w = world_w * self.scale
        draw_h = world_h * self.scale

        self.left = (self.window_width - draw_w) / 2.0
        self.top = (self.window_height - draw_h) / 2.0
        self.x_min = x_min
        self.y_max = y_max

    def world_to_pixel(self, x: float, y: float):
        px = int(self.left + (x - self.x_min) * self.scale)
        py = int(self.top + (self.y_max - y) * self.scale)
        return px, py

    def draw_rounded_rect(self, img, x1, y1, x2, y2, radius, color):
        radius = max(1, radius)
        cv2.rectangle(img, (x1 + radius, y1), (x2 - radius, y2), color, -1)
        cv2.rectangle(img, (x1, y1 + radius), (x2, y2 - radius), color, -1)
        cv2.circle(img, (x1 + radius, y1 + radius), radius, color, -1)
        cv2.circle(img, (x2 - radius, y1 + radius), radius, color, -1)
        cv2.circle(img, (x1 + radius, y2 - radius), radius, color, -1)
        cv2.circle(img, (x2 - radius, y2 - radius), radius, color, -1)

    def draw_base_grid(self, img):
        thickness = max(8, int(self.scale * 0.045))

        # 3x2 outer and divider lines.
        for y in [0.0, 1.0, 2.0]:
            p1 = self.world_to_pixel(0.0, y)
            p2 = self.world_to_pixel(3.0, y)
            cv2.line(img, p1, p2, self.lane_color, thickness)

        for x in [0.0, 1.0, 2.0, 3.0]:
            p1 = self.world_to_pixel(x, -1.15)
            p2 = self.world_to_pixel(x, 2.0)
            cv2.line(img, p1, p2, self.lane_color, thickness)

        # Short vertical stubs inside each cell column, like the printed map.
        for x in [0.5, 1.5, 2.5]:
            top_stub_start = self.world_to_pixel(x, 1.0)
            top_stub_end = self.world_to_pixel(x, 1.45)
            mid_stub_start = self.world_to_pixel(x, 0.0)
            mid_stub_end = self.world_to_pixel(x, 0.45)
            cv2.line(img, top_stub_start, top_stub_end, self.lane_color, thickness)
            cv2.line(img, mid_stub_start, mid_stub_end, self.lane_color, thickness)

        # Bottom dock pads.
        for x in [0.0, 1.0, 2.0, 3.0]:
            left_top = self.world_to_pixel(x - 0.22, -1.12)
            right_bottom = self.world_to_pixel(x + 0.22, -1.30)
            x1, y1 = left_top
            x2, y2 = right_bottom
            x_min = min(x1, x2)
            x_max = max(x1, x2)
            y_min = min(y1, y2)
            y_max = max(y1, y2)
            radius = max(6, int(self.scale * 0.05))
            self.draw_rounded_rect(img, x_min, y_min, x_max, y_max, radius, self.dock_color)

    def draw_axis_labels(self, img):
        # Column labels aligned with the four main vertical lanes.
        for i, col in enumerate(['A', 'B', 'C', 'D']):
            cx, cy = self.world_to_pixel(float(i), 2.0)
            cv2.putText(
                img,
                col,
                (cx - 8, cy - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (70, 70, 70),
                2,
                cv2.LINE_AA,
            )

        # Row labels match planner naming: 1 is top row, 3 is bottom row.
        row_specs = [('1', 2.0), ('2', 1.0), ('3', 0.0)]
        for row_text, y in row_specs:
            cx, cy = self.world_to_pixel(0.0, y)
            cv2.putText(
                img,
                row_text,
                (cx - 34, cy + 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (70, 70, 70),
                2,
                cv2.LINE_AA,
            )

    def node_color(self, label: str):
        if label.startswith('HOME-'):
            return (255, 120, 0)
        if label.startswith('DOC-'):
            return (160, 80, 255)
        if len(label) == 2 and label[0].isalpha() and label[1].isdigit():
            return (0, 180, 0)
        return (0, 140, 255)

    def draw_nodes_and_labels(self, img):
        sorted_nodes = sorted(
            self.planner.nodes.values(),
            key=lambda n: (n.y, n.x, n.label)
        )

        for node in sorted_nodes:
            cx, cy = self.world_to_pixel(node.x, node.y)
            cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1)
            cv2.circle(img, (cx, cy), 5, (40, 40, 40), 1)
            if not self.show_labels:
                continue

            label_color = self.node_color(node.label)
            label_x = cx + 8
            label_y = cy - 6

            # Keep labels readable on top of thick black lanes.
            text_size, _ = cv2.getTextSize(node.label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            text_w, text_h = text_size
            x1 = max(0, label_x - 2)
            y1 = max(0, label_y - text_h - 2)
            x2 = min(self.window_width - 1, label_x + text_w + 2)
            y2 = min(self.window_height - 1, label_y + 3)
            cv2.rectangle(img, (x1, y1), (x2, y2), (248, 248, 248), -1)

            cv2.putText(
                img,
                node.label,
                (label_x, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                label_color,
                1,
                cv2.LINE_AA,
            )

    def agv_corners_world(self, cx: float, cy: float, angle_deg: float):
        half = 0.5 * self.agv_size_world
        # Rectangle in AGV local frame, where +Y is forward.
        local = np.array([
            [-half, -half],
            [half, -half],
            [half, half],
            [-half, half],
        ], dtype=float)

        a = np.deg2rad(angle_deg + self.agv_angle_offset_deg)
        c = float(np.cos(a))
        s = float(np.sin(a))
        rot = np.array([[c, -s], [s, c]], dtype=float)
        world = (local @ rot.T) + np.array([cx, cy], dtype=float)
        return world

    def draw_agv(self, img):
        if self.current_node_label not in self.planner.nodes:
            return

        node = self.planner.nodes[self.current_node_label]
        corners_world = self.agv_corners_world(node.x, node.y, self.current_angle_deg)
        corners_px = np.array(
            [self.world_to_pixel(float(x), float(y)) for x, y in corners_world],
            dtype=np.int32,
        )

        cv2.fillPoly(img, [corners_px], (80, 200, 120))
        cv2.polylines(img, [corners_px], True, (20, 60, 35), 2)

        # Front-direction arrow from center.
        center_px = self.world_to_pixel(node.x, node.y)
        f_len = 0.38 * self.agv_size_world
        a = np.deg2rad(self.current_angle_deg + self.agv_angle_offset_deg)
        fx = node.x - f_len * np.sin(a)
        fy = node.y + f_len * np.cos(a)
        front_px = self.world_to_pixel(float(fx), float(fy))
        cv2.arrowedLine(img, center_px, front_px, (30, 30, 30), 2, tipLength=0.35)

    def render(self):
        self.compute_layout()
        frame = np.full((self.window_height, self.window_width, 3), self.background_color, dtype=np.uint8)

        self.draw_base_grid(frame)
        self.draw_axis_labels(frame)
        self.draw_nodes_and_labels(frame)
        self.draw_agv(frame)

        cv2.putText(
            frame,
            '3x2 Warehouse Grid (Centered)',
            (20, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (80, 80, 80),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f'AGV Node: {self.current_node_label}',
            (20, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (50, 50, 50),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f'AGV Angle: {self.current_angle_deg:.1f} deg',
            (20, 86),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (50, 50, 50),
            1,
            cv2.LINE_AA,
        )

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quit key pressed, shutting down visualizer')
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeGridVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
