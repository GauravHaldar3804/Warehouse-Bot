from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QPushButton,
    QHBoxLayout, QGridLayout, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer
import random


class SystemStatusPage(QWidget):

    def __init__(self, main_window, ros_node=None):
        super().__init__()

        self.main_window = main_window
        self.ros_node = ros_node
        self.setStyleSheet("background-color:#f5f7fb; color:#111827; font-family:Segoe UI, Arial, sans-serif;")

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(18)

        # HEADER
        header = QHBoxLayout()

        back_btn = QPushButton('← Back')
        back_btn.setFixedSize(100, 35)
        back_btn.setStyleSheet(
            'QPushButton{background-color:#3498db; color:white; border-radius:8px; font-weight:bold;}'
            'QPushButton:hover{background-color:#2980b9;}'
        )
        back_btn.clicked.connect(self.main_window.show_home)

        title = QLabel("System Status")
        title.setFont(QFont("Arial",20,QFont.Bold))

        header.addWidget(back_btn)
        header.addStretch()
        header.addWidget(title)
        header.addStretch()

        main_layout.addLayout(header)

        main_layout.addSpacing(20)

        # STATUS GRID
        grid = QGridLayout()
        grid.setSpacing(20)

        self.controller = self.create_card("Controller","Connected","#2ecc71")
        self.motor = self.create_card("Motor Driver","Unknown","#27ae60")
        self.encoders = self.create_card("Encoders","Unknown","#27ae60")
        self.imu = self.create_card("IMU","Unknown","#3498db")
        self.tof = self.create_card("ToF Sensors","Unknown","#9b59b6")
        self.camera = self.create_card("Camera","Unknown","#e67e22")
        self.battery_sensor = self.create_card("Battery Sensor","Unknown","#f39c12")
        self.ros = self.create_card("ROS Node","Running","#f39c12")

        grid.addWidget(self.controller,0,0)
        grid.addWidget(self.motor,0,1)
        grid.addWidget(self.encoders,0,2)

        grid.addWidget(self.imu,1,0)
        grid.addWidget(self.tof,1,1)
        grid.addWidget(self.camera,1,2)

        grid.addWidget(self.battery_sensor,2,0)
        grid.addWidget(self.ros,2,1)

        main_layout.addLayout(grid)

        main_layout.addStretch()

        self.setLayout(main_layout)

        # TIMER FOR STATUS UPDATE
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(4000)

    # CREATE STATUS CARD
    def create_card(self,title,value,color):

        card = QFrame()
        card.setStyleSheet(f"""
        QFrame{{
            background:white;
            border-left:6px solid {color};
            border-radius:10px;
            padding:15px;
        }}
        """)

        layout = QVBoxLayout()

        label_title = QLabel(title)
        label_title.setFont(QFont("Arial",11))
        label_title.setStyleSheet("color:gray")

        label_value = QLabel(value)
        label_value.setFont(QFont("Arial",16,QFont.Bold))

        layout.addWidget(label_title)
        layout.addWidget(label_value)

        card.setLayout(layout)

        card.value_label = label_value

        return card

    # UPDATE STATUS FROM ROS
    def update_status(self):
        if self.ros_node is None:
            return

        status = self.ros_node.get_status_snapshot()

        # Update component statuses based on ROS activity
        motor_status = status.get('system_motor', 'Unknown')
        self.motor.value_label.setText(motor_status)
        self.set_card_color(self.motor, motor_status)

        tof_status = status.get('system_tof', 'Unknown')
        self.tof.value_label.setText(tof_status)
        self.set_card_color(self.tof, tof_status)

        camera_status = status.get('system_camera', 'Unknown')
        self.camera.value_label.setText(camera_status)
        self.set_card_color(self.camera, camera_status)

        battery_status = status.get('system_battery', 'Unknown')
        self.battery_sensor.value_label.setText(battery_status)
        self.set_card_color(self.battery_sensor, battery_status)

        # Keep others as Unknown for now (no ROS topics)
        self.encoders.value_label.setText("Unknown")
        self.imu.value_label.setText("Unknown")

    # SET CARD COLOR BASED ON STATUS
    def set_card_color(self, card, status):
        if status == 'Active':
            card.value_label.setStyleSheet('color:green')
        elif status == 'Inactive':
            card.value_label.setStyleSheet('color:red')
        else:
            card.value_label.setStyleSheet('color:#2c3e50')
