import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
from sensor_msgs.msg import Image


class CameraRosNode(Node):
    def __init__(self, node_name: str = 'camera_viewer_node'):
        super().__init__(node_name)

        self.latest_frame = None
        self.frame_lock = False

        from cv_bridge import CvBridge
        self.bridge = CvBridge()

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            camera_qos,
        )

        self.get_logger().info('Camera viewer node started, subscribed to camera/image_raw')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv_image
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')


class CameraPage(QWidget):
    def __init__(self, main_window, ros_node=None):
        super().__init__()
        self.main_window = main_window
        self.main_ros_node = ros_node
        self.camera_node = None
        self.obstacle_alert_active = False

        self.setStyleSheet('background-color:#f5f7fb; color:#111827; font-family:Segoe UI, Arial, sans-serif;')

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

        self.status_label = QLabel('Camera Status: Initializing...')
        self.status_label.setStyleSheet('color: #e67e22; font-weight: bold;')

        self.obstacle_alert = QLabel()
        self.obstacle_alert.setStyleSheet('color: white; font-weight: bold; font-size: 14px;')
        self.obstacle_alert.setVisible(False)

        title = QLabel('📷 Live Camera Feed')
        title.setFont(QFont('Arial', 20, QFont.Bold))
        title.setStyleSheet('color:#2c3e50;')

        header.addWidget(back_btn)
        header.addSpacing(20)
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.status_label)
        main_layout.addLayout(header)

        # Add obstacle alert banner
        self.alert_banner = QFrame()
        self.alert_banner.setStyleSheet('QFrame{background:#fee2e2; border-radius:14px; padding:14px; border:1px solid #f5c2c7;}')
        self.alert_banner.setVisible(False)
        alert_layout = QHBoxLayout()
        alert_layout.addWidget(QLabel('⚠ OBSTACLE DETECTED!'))
        self.alert_banner.setLayout(alert_layout)
        main_layout.addWidget(self.alert_banner)


        # Subscribe to obstacle signals
        if self.main_ros_node:
            self.main_ros_node.register_obstacle_callback(self.on_obstacle_detected)

        container = QFrame()
        container.setStyleSheet('QFrame{background:white; border-radius:18px; border:1px solid #e5e7eb;}')
        clayout = QVBoxLayout()
        clayout.setContentsMargins(18, 18, 18, 18)

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumHeight(600)
        self.image_label.setStyleSheet('QLabel{background:#f7f9fb; border-radius:10px;}')
        clayout.addWidget(self.image_label)

        container.setLayout(clayout)
        main_layout.addWidget(container)
        self.setLayout(main_layout)

        self.init_camera_node()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_camera_frame)
        self.timer.start(66)  # ~15 FPS

    def init_camera_node(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self.camera_node = CameraRosNode(node_name='embedded_camera_viewer_node')
            self.status_label.setText('Camera Status: Connected')
            self.status_label.setStyleSheet('color: #27ae60; font-weight: bold;')
        except Exception as exc:
            self.status_label.setText(f'Camera failed: {str(exc)[:80]}')
            self.status_label.setStyleSheet('color: #e74c3c; font-weight: bold;')

    def update_camera_frame(self):
        if self.camera_node is None:
            return

        try:
            rclpy.spin_once(self.camera_node, timeout_sec=0.0)

            if self.camera_node.latest_frame is not None:
                frame_bgr = self.camera_node.latest_frame
                frame_rgb = frame_bgr[:, :, ::-1].copy()

                h, w, ch = frame_rgb.shape
                qimage = QImage(frame_rgb.data, w, h, ch * w, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qimage)

                target_size = self.image_label.size()
                scaled = pixmap.scaled(target_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.image_label.setPixmap(scaled)

                self.status_label.setText('Camera Status: Streaming')
                self.status_label.setStyleSheet('color: #27ae60; font-weight: bold;')
            else:
                self.status_label.setText('Camera Status: Waiting for frames...')
                self.status_label.setStyleSheet('color: #e67e22; font-weight: bold;')
        except Exception as exc:
            self.status_label.setText(f'Camera update failed: {str(exc)[:80]}')
            self.status_label.setStyleSheet('color: #e74c3c; font-weight: bold;')

    def on_obstacle_detected(self, obstacle_present):
        """Called when obstacle detection state changes"""
        self.obstacle_alert_active = obstacle_present
        self.update_obstacle_alert(obstacle_present)

    def update_obstacle_alert(self, show_alert):
        """Update the visibility of the obstacle alert banner"""
        if show_alert:
            self.alert_banner.setVisible(True)
            self.alert_banner.setStyleSheet(
                'QFrame{background-color:#e74c3c; border-radius:8px; padding:15px; '
                'border:2px solid #c0392b;}'
            )
            alert_label = self.alert_banner.findChild(QLabel)
            if alert_label:
                alert_label.setText('⚠  OBSTACLE DETECTED! - Camera Feed Active')
                alert_label.setStyleSheet('color: white; font-weight: bold; font-size: 16px;')
        else:
            self.alert_banner.setVisible(False)

    def closeEvent(self, event):
        if self.timer.isActive():
            self.timer.stop()

        if self.camera_node is not None:
            try:
                self.camera_node.destroy_node()
            except Exception:
                pass
            self.camera_node = None

        event.accept()
