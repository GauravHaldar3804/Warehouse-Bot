from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QPushButton,
    QHBoxLayout, QGridLayout, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer
import random


class SystemStatusPage(QWidget):

    def __init__(self, main_window):
        super().__init__()

        self.main_window = main_window
        self.setStyleSheet("background-color:#eef2f5;")

        main_layout = QVBoxLayout()

        # HEADER
        header = QHBoxLayout()

        back_btn = QPushButton("← Back")
        back_btn.setFixedWidth(90)
        back_btn.setStyleSheet("""
        QPushButton{
            background-color:#3498db;
            color:white;
            border-radius:6px;
            padding:5px;
        }
        QPushButton:hover{
            background-color:#2980b9;
        }
        """)
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
        self.motor = self.create_card("Motor Driver","OK","#27ae60")
        self.encoders = self.create_card("Encoders","OK","#27ae60")
        self.imu = self.create_card("IMU","OK","#3498db")
        self.tof = self.create_card("ToF Sensors","Active","#9b59b6")
        self.camera = self.create_card("Camera","Streaming","#e67e22")
        self.ros = self.create_card("ROS Node","Running","#f39c12")

        grid.addWidget(self.controller,0,0)
        grid.addWidget(self.motor,0,1)
        grid.addWidget(self.encoders,0,2)

        grid.addWidget(self.imu,1,0)
        grid.addWidget(self.tof,1,1)
        grid.addWidget(self.camera,1,2)

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

    # SIMULATE STATUS CHANGES
    def update_status(self):

        states = ["OK","Active","Running","Connected"]

        self.motor.value_label.setText(random.choice(states))
        self.encoders.value_label.setText(random.choice(states))
        self.imu.value_label.setText(random.choice(states))
        self.tof.value_label.setText(random.choice(states))
