import os
import sys
import inspect

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


# Allow running GUI directly from the repo without installing agv_motor_controller.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..'))
_ROS_PKG_PARENT = os.path.join(_REPO_ROOT, 'agv_motor_controller')
if _ROS_PKG_PARENT not in sys.path:
    sys.path.insert(0, _ROS_PKG_PARENT)

import rclpy


class WarehouseMapPage(QWidget):
    def __init__(self, main_window, ros_node=None):
        super().__init__()
        self.main_window = main_window
        self.main_ros_node = ros_node
        self.visualizer_node = None

        self.setStyleSheet('background-color:#eef2f5;')

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 10, 20, 10)

        header = QHBoxLayout()
        back_btn = QPushButton('← Back')
        back_btn.setFixedSize(100, 35)
        back_btn.setStyleSheet(
            'QPushButton{background-color:#3498db; color:white; border-radius:8px; font-weight:bold;}'
            'QPushButton:hover{background-color:#2980b9;}'
        )
        back_btn.clicked.connect(self.main_window.show_home)

        self.status_label = QLabel('Visualizer Status: Initializing...')
        self.status_label.setStyleSheet('color: #e67e22; font-weight: bold;')

        title = QLabel('Warehouse Map - Embedded Realtime Visualizer')
        title.setFont(QFont('Arial', 20, QFont.Bold))
        title.setStyleSheet('color:#2c3e50;')

        header.addWidget(back_btn)
        header.addSpacing(20)
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.status_label)
        main_layout.addLayout(header)

        container = QFrame()
        container.setStyleSheet('QFrame{background:white; border-radius:15px; border:1px solid #d0d7de;}')
        clayout = QVBoxLayout()
        clayout.setContentsMargins(20, 20, 20, 20)

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumHeight(700)
        self.image_label.setStyleSheet('QLabel{background:#f7f9fb; border-radius:10px;}')
        clayout.addWidget(self.image_label)

        container.setLayout(clayout)
        main_layout.addWidget(container)
        self.setLayout(main_layout)

        self.init_visualizer_node()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_visualizer_frame)
        self.timer.start(100)

    def init_visualizer_node(self):
        try:
            # Main app already initializes rclpy in agv_gui/main.py; this keeps direct runs safe.
            if not rclpy.ok():
                rclpy.init(args=None)

            # Import lazily to avoid cv2 Qt plugin side effects before QApplication is created.
            from agv_motor_controller.realtime_grid_visualizer_node import RealtimeGridVisualizerNode

            init_params = inspect.signature(RealtimeGridVisualizerNode.__init__).parameters
            kwargs = {}
            if 'headless' in init_params:
                kwargs['headless'] = True
            if 'node_name' in init_params:
                kwargs['node_name'] = 'embedded_realtime_grid_visualizer_node'

            self.visualizer_node = RealtimeGridVisualizerNode(**kwargs)

            if not hasattr(self.visualizer_node, 'build_frame'):
                raise RuntimeError('Loaded visualizer node does not expose build_frame().')
            self.status_label.setText('Visualizer Status: Connected')
            self.status_label.setStyleSheet('color: #27ae60; font-weight: bold;')
        except Exception as exc:
            self.status_label.setText(f'Visualizer failed: {str(exc)[:80]}')
            self.status_label.setStyleSheet('color: #e74c3c; font-weight: bold;')

    def update_visualizer_frame(self):
        if self.visualizer_node is None:
            return

        try:
            rclpy.spin_once(self.visualizer_node, timeout_sec=0.0)
            frame_bgr = self.visualizer_node.build_frame()
            frame_rgb = frame_bgr[:, :, ::-1].copy()

            h, w, ch = frame_rgb.shape
            qimage = QImage(frame_rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimage)

            target_size = self.image_label.size()
            scaled = pixmap.scaled(target_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(scaled)

            self.status_label.setText(
                f'Visualizer: Active | Node {self.visualizer_node.current_node_label} | '
                f'Angle {self.visualizer_node.current_angle_deg:.1f} deg'
            )
        except Exception as exc:
            self.status_label.setText(f'Visualizer update failed: {str(exc)[:80]}')
            self.status_label.setStyleSheet('color: #e74c3c; font-weight: bold;')

    def start_mission(self, pickup, drop):
        # Keep Task page integration intact; realtime visualizer consumes pose from camera/qr_code.
        self.status_label.setText(f'Visualizer: Mission requested {pickup} -> {drop}')
        self.status_label.setStyleSheet('color: #27ae60; font-weight: bold;')

    def closeEvent(self, event):
        if self.timer.isActive():
            self.timer.stop()

        if self.visualizer_node is not None:
            try:
                self.visualizer_node.destroy_node()
            except Exception:
                pass
            self.visualizer_node = None

        event.accept()
