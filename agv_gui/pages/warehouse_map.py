from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import threading
import time


class ROSBridgeNode(Node):
    def __init__(self):
        super().__init__('warehouse_map_ros_bridge')
        self.current_node = "HOME1"
        self.last_update = time.time()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            String, 'camera/qr_code', self.qr_callback, qos
        )
        self.get_logger().info('Warehouse Map ROS Bridge: Subscribed to camera/qr_code')


    def qr_callback(self, msg: String):
        raw = (msg.data or '').strip()
        if not raw:
            return
        parts = [p.strip() for p in raw.split('|')]
        node_label = parts[0].upper() if parts else self.current_node
        self.current_node = node_label
        self.last_update = time.time()


class WarehouseMapPage(QWidget):

    def __init__(self, main_window, ros_node=None):
        super().__init__()
        self.main_window = main_window
        self.main_ros_node = ros_node   # from dashboard

        self.setStyleSheet("background-color:#eef2f5;")

        # Main Layout
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 10, 20, 10)

        # Header
        header = QHBoxLayout()
        back_btn = QPushButton("← Back")
        back_btn.setFixedSize(100, 35)
        back_btn.setStyleSheet("""
            QPushButton{background-color:#3498db; color:white; border-radius:8px; font-weight:bold;}
            QPushButton:hover{background-color:#2980b9;}
        """)
        back_btn.clicked.connect(self.main_window.show_home)

        self.status_label = QLabel("ROS Status: Initializing...")
        self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold;")

        title = QLabel("Warehouse Map - Real-time AGV")
        title.setFont(QFont("Arial", 22, QFont.Bold))
        title.setStyleSheet("color:#2c3e50;")

        header.addWidget(back_btn)
        header.addSpacing(20)
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.status_label)
        main_layout.addLayout(header)

        # Map Container
        container = QFrame()
        container.setStyleSheet("QFrame{background:white; border-radius:15px; border:1px solid #d0d7de;}")
        clayout = QVBoxLayout()
        clayout.setContentsMargins(20, 20, 20, 20)

        self.figure = Figure(figsize=(12, 10), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumHeight(800)
        clayout.addWidget(self.canvas)
        container.setLayout(clayout)
        main_layout.addWidget(container)

        self.setLayout(main_layout)

        # Grid settings
        self.grid_size = 4
        self.agv_pos = (0, 3)   # col, row → HOME1
        self.path = []
        self.index = 0
        self.target = None

        # ROS Bridge
        self.ros_bridge = None
        self.executor = None
        self.ros_thread = None
        self.init_ros_bridge()

        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_map)
        self.timer.start(180)

        self.draw_map()

    def init_ros_bridge(self):
        try:
            if not rclpy.ok():
                rclpy.init()

            self.ros_bridge = ROSBridgeNode()
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.ros_bridge)

            self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.ros_thread.start()

            self.status_label.setText("ROS Status: Connected ✓ | Waiting for QR codes")
            self.status_label.setStyleSheet("color: #27ae60; font-weight: bold;")
            
        except Exception as e:
            self.status_label.setText(f"ROS Failed: {str(e)[:50]}")
            print(f"ROS Bridge Error: {e}")

    def get_real_position(self):
        if not self.ros_bridge:
            return None
        
        node = self.ros_bridge.current_node.strip().upper()
        pos_map = {
            "A1": (0,0), "A2":(1,0), "A3":(2,0),
            "B1":(0,1), "B2":(1,1), "B3":(2,1),
            "C1":(0,2), "C2":(1,2), "C3":(2,2),
            "HOME1":(0,3), "HOME2":(1,3), "HOME3":(2,3),
            "HOME-1":(0,3), "HOME-2":(1,3), "HOME-3":(2,3),
        }
        return pos_map.get(node)

    def start_mission(self, pickup, drop):
        pos_map = {
            "A1": (0,0), "A2":(1,0), "A3":(2,0),
            "B1":(0,1), "B2":(1,1), "B3":(2,1),
            "C1":(0,2), "C2":(1,2), "C3":(2,2),
            "HOME1":(0,3), "HOME2":(1,3), "HOME3":(2,3)
        }
        p = pos_map.get(pickup, (0,0))
        d = pos_map.get(drop, (0,3))

        self.path = self.generate_path(self.agv_pos, p) + self.generate_path(p, d)
        self.index = 0
        self.target = d
        print(f"Mission started: {pickup} → {drop}")

    def generate_path(self, start, end):
        path = []
        c, r = start
        tc, tr = end
        while r != tr:
            r += 1 if tr > r else -1
            path.append((c, r))
        while c != tc:
            c += 1 if tc > c else -1
            path.append((c, r))
        return path

    def update_map(self):
        real_pos = self.get_real_position()

        if real_pos:
            self.agv_pos = real_pos
            if self.ros_bridge:
                self.status_label.setText(f"ROS: Connected | Current: {self.ros_bridge.current_node}")
        elif self.path and self.index < len(self.path):
            self.agv_pos = self.path[self.index]
            self.index = (self.index + 1) % len(self.path)   # loop for testing

        self.draw_map()

    def draw_map(self):
        self.figure.clear()
        ax = self.figure.add_subplot(111)

        # Grid
        for i in range(self.grid_size + 1):
            ax.plot([i, i], [0, self.grid_size], 'k-', linewidth=2)
            ax.plot([0, self.grid_size], [i, i], 'k-', linewidth=2)

        # Shelves A1-C3
        for ri, row in enumerate(['1','2','3']):
            for ci, col in enumerate(['A','B','C']):
                x = ci
                y = 3 - ri
                ax.add_patch(patches.Circle((x, y), 0.08, color='#3498db', zorder=3))
                ax.text(x, y - 0.28, f"{col}{row}", ha='center', va='top', 
                       fontsize=11, fontweight='bold')

        # Home row
        for i in range(3):
            ax.add_patch(patches.Circle((i, 0), 0.08, color='#2ecc71', zorder=3))
            ax.text(i, -0.28, f"HOME{i+1}", ha='center', va='top', 
                   fontsize=11, fontweight='bold')

        # AGV
        col, row = self.agv_pos
        ax_x, ax_y = col, 3 - row
        ax.add_patch(patches.Circle((ax_x, ax_y), 0.13, color='#27ae60', zorder=5))
        ax.text(ax_x, ax_y, 'AGV', ha='center', va='center', color='white', 
                fontsize=12, fontweight='bold', zorder=6)

        # Target
        if self.target:
            tx, ty = self.target
            ax.add_patch(patches.Circle((tx, 3-ty), 0.16, fill=False, 
                                      edgecolor='#f39c12', linewidth=3, linestyle='--'))

        ax.set_xlim(-0.7, 3.8)
        ax.set_ylim(-0.8, 3.8)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title('Warehouse Map - Real-time AGV Tracking', fontsize=16, fontweight='bold', pad=15)

        self.canvas.draw_idle()

    def closeEvent(self, event):
        if self.executor:
            self.executor.shutdown()
        if self.ros_bridge:
            self.ros_bridge.destroy_node()
        event.accept()
