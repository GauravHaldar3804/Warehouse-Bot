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
import numpy as np


class WarehouseMapPage(QWidget):

    def __init__(self, main_window, ros_node=None):
        super().__init__()

        self.main_window = main_window
        self.ros_node = ros_node
        self.setStyleSheet("background-color:#eef2f5;")

        # ---------- MAIN LAYOUT ----------
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 10, 20, 10)
        main_layout.setSpacing(10)

        # ---------- HEADER ----------
        header = QHBoxLayout()

        back_btn = QPushButton("← Back")
        back_btn.setFixedSize(100, 35)
        back_btn.setStyleSheet("""
            QPushButton{
                background-color:#3498db;
                color:white;
                border-radius:8px;
                font-weight:bold;
            }
            QPushButton:hover{
                background-color:#2980b9;
            }
        """)
        back_btn.clicked.connect(self.main_window.show_home)

        title = QLabel("Warehouse Map")
        title.setFont(QFont("Arial", 22, QFont.Bold))
        title.setStyleSheet("color:#2c3e50;")

        header.addWidget(back_btn)
        header.addSpacing(20)
        header.addWidget(title)
        header.addStretch()

        main_layout.addLayout(header)

        # ---------- MAP CONTAINER ----------
        container = QFrame()
        container.setStyleSheet("""
            QFrame{
                background:white;
                border-radius:15px;
                border:1px solid #d0d7de;
            }
        """)

        container_layout = QVBoxLayout()
        container_layout.setContentsMargins(20, 20, 20, 20)

        # ---------- MATPLOTLIB CANVAS ----------
        self.figure = Figure(figsize=(12, 10), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumHeight(800)
        
        container_layout.addWidget(self.canvas)
        container.setLayout(container_layout)

        main_layout.addWidget(container, alignment=Qt.AlignCenter)

        self.setLayout(main_layout)

        # ---------- GRID PARAMETERS ----------
        self.cols = ['A', 'B', 'C']  # 3 columns
        self.rows = ['1', '2', '3']  # 3 rows for shelves
        self.home_row = 'HOME'       # Home row
        self.grid_size = 4  # 4 positions (3 shelves + 1 home)
        self.cell_size = 1.0

        # ---------- AGV STATE ----------
        self.agv_pos = (0, 3)  # Starting at Home1
        self.path = []
        self.index = 0
        self.target = None

        # ---------- TIMER ----------
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_agv_position)
        self.timer.start(100)  # Update every 100ms for smooth real-time movement

        self.draw_map()


    # =========================================================
    # 🎯 START MISSION (called from Task Page)
    # =========================================================
    def start_mission(self, pickup, drop):

        position_map = {
            "A1": (0, 0), "A2": (1, 0), "A3": (2, 0),
            "B1": (0, 1), "B2": (1, 1), "B3": (2, 1),
            "C1": (0, 2), "C2": (1, 2), "C3": (2, 2),
            "HOME1": (0, 3), "HOME2": (1, 3), "HOME3": (2, 3)
        }

        # Try to parse grid positions from task management
        if pickup in position_map:
            pickup_pos = position_map[pickup]
        else:
            pickup_pos = (0, 0)

        if drop in position_map:
            drop_pos = position_map[drop]
        else:
            drop_pos = (0, 3)

        # Generate path
        path1 = self.generate_path(self.agv_pos, pickup_pos)
        path2 = self.generate_path(pickup_pos, drop_pos)

        self.path = path1 + path2
        self.index = 0
        self.target = drop_pos

    # =========================================================
    # 🔄 PATH GENERATION
    # =========================================================
    def generate_path(self, start, end):

        path = []
        c, r = start
        tc, tr = end

        # Move rows
        while r != tr:
            r += 1 if tr > r else -1
            path.append((c, r))

        # Move columns
        while c != tc:
            c += 1 if tc > c else -1
            path.append((c, r))

        return path

    # =========================================================
    # 🎨 DRAW MAP (Dynamic Grid Generation)
    # =========================================================
    def draw_map(self):
        """Draw the warehouse map dynamically using matplotlib"""
        self.figure.clear()
        ax = self.figure.add_subplot(111)

        # Draw grid lines
        for i in range(self.grid_size + 1):
            # Vertical lines
            ax.plot([i * self.cell_size, i * self.cell_size],
                    [0, self.grid_size * self.cell_size],
                    'k-', linewidth=2)
            # Horizontal lines
            ax.plot([0, self.grid_size * self.cell_size],
                    [i * self.cell_size, i * self.cell_size],
                    'k-', linewidth=2)

        # Draw shelf nodes (A1-A3, B1-B3, C1-C3)
        for row_idx, row in enumerate(self.rows):
            for col_idx, col in enumerate(self.cols):
                x = col_idx * self.cell_size
                y = (self.grid_size - 1 - row_idx) * self.cell_size
                node_label = f"{col}{row}"

                # Draw node circle
                circle = patches.Circle((x, y), radius=0.08,
                                       color='#3498db', zorder=3)
                ax.add_patch(circle)

                # Add label
                ax.text(x, y - 0.25, node_label,
                       fontsize=11, fontweight='bold',
                       ha='center', va='top',
                       bbox=dict(boxstyle='round,pad=0.3',
                                facecolor='#e8f4f8', alpha=0.8))

        # Draw home nodes (HOME1, HOME2, HOME3)
        home_row_idx = self.grid_size - 1
        for col_idx, col in enumerate(self.cols):
            x = col_idx * self.cell_size
            y = 0  # Bottom row
            node_label = f"HOME{col_idx + 1}"

            # Draw home circle (green)
            circle = patches.Circle((x, y), radius=0.08,
                                   color='#2ecc71', zorder=3)
            ax.add_patch(circle)

            # Add label
            ax.text(x, y - 0.25, node_label,
                   fontsize=11, fontweight='bold',
                   ha='center', va='top',
                   bbox=dict(boxstyle='round,pad=0.3',
                            facecolor='#d5f4e6', alpha=0.8))

        # Draw path if exists
        if len(self.path) > 0:
            for i, (c, r) in enumerate(self.path):
                x = c * self.cell_size
                y = (self.grid_size - 1 - r) * self.cell_size
                
                ax.plot(x, y, 'o', color='#3498db', markersize=8, alpha=0.6)

        # Draw target if exists
        if self.target:
            target_col, target_row = self.target
            x = target_col * self.cell_size
            y = (self.grid_size - 1 - target_row) * self.cell_size

            circle = patches.Circle((x, y), radius=0.15,
                                   fill=False, edgecolor='#f39c12',
                                   linewidth=3, linestyle='--', zorder=4)
            ax.add_patch(circle)

        # Draw AGV at current position
        col, row = self.agv_pos
        agv_x = col * self.cell_size
        agv_y = (self.grid_size - 1 - row) * self.cell_size

        # AGV circle (larger, green)
        agv_circle = patches.Circle((agv_x, agv_y), radius=0.12,
                                    color='#27ae60', zorder=5)
        ax.add_patch(agv_circle)

        # AGV marker/emoji
        ax.text(agv_x, agv_y, 'AGV', fontsize=12, ha='center', va='center', zorder=6, color='white', fontweight='bold')

        # Add column labels (A, B, C) at the bottom
        for i, col in enumerate(self.cols):
            x = i * self.cell_size
            ax.text(x, -0.45, col, fontsize=14, fontweight='bold',
                   ha='center', va='top',
                   bbox=dict(boxstyle='round,pad=0.4',
                            facecolor='#ffeb3b', alpha=0.8))

        # Add row labels (1, 2, 3, HOME) on the left
        for j, row in enumerate(self.rows):
            y = (self.grid_size - 1 - j) * self.cell_size
            ax.text(-0.45, y, row, fontsize=14, fontweight='bold',
                   ha='right', va='center',
                   bbox=dict(boxstyle='round,pad=0.4',
                            facecolor='#81c784', alpha=0.8))

        # Add HOME label
        ax.text(-0.45, 0, self.home_row, fontsize=14, fontweight='bold',
               ha='right', va='center',
               bbox=dict(boxstyle='round,pad=0.4',
                        facecolor='#81c784', alpha=0.8))

        # Set axis properties
        ax.set_xlim(-0.7, self.grid_size - 0.3)
        ax.set_ylim(-0.7, self.grid_size - 0.3)
        ax.set_aspect('equal')
        ax.axis('off')

        # Add title
        ax.set_title('Warehouse Grid Map - Real-time AGV Tracking',
                    fontsize=16, fontweight='bold', pad=20)

        # Draw the canvas
        self.canvas.draw_idle()

    # =========================================================
    # � UPDATE AGV POSITION (Real-time)
    # =========================================================
    def update_agv_position(self):
        """Update AGV position from ROS data or path progress"""
        # Try to get real position from ROS
        if self.ros_node and hasattr(self.ros_node, '_status'):
            try:
                position = self.ros_node._status.get('position', '--')
                if position != '--':
                    # Parse position string if it's a grid coordinate
                    pos_str = str(position).strip().upper()
                    position_map = {
                        "A1": (0, 0), "A2": (1, 0), "A3": (2, 0),
                        "B1": (0, 1), "B2": (1, 1), "B3": (2, 1),
                        "C1": (0, 2), "C2": (1, 2), "C3": (2, 2),
                        "HOME1": (0, 3), "HOME2": (1, 3), "HOME3": (2, 3)
                    }
                    if pos_str in position_map:
                        self.agv_pos = position_map[pos_str]
            except Exception:
                pass

        # Otherwise, continue following the planned path
        if self.path and self.index < len(self.path):
            self.agv_pos = self.path[self.index]
            self.index += 1

        self.draw_map()

    # =========================================================
    # 🚗 MOVE AGV
    # =========================================================
    def move_agv(self):
        """Move AGV along the path (kept for backwards compatibility)"""
        self.update_agv_position()
