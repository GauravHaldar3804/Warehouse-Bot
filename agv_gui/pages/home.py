from PyQt5.QtWidgets import (
    QWidget, QPushButton, QLabel, QGridLayout,
    QVBoxLayout, QHBoxLayout, QProgressBar, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer
import random


class HomePage(QWidget):

    def __init__(self, main_window):
        super().__init__()

        self.main_window = main_window

        # Background gradient
        self.setStyleSheet("""
        QWidget{
            background:qlineargradient(
                x1:0,y1:0,x2:1,y2:1,
                stop:0 #eef2f5,
                stop:1 #d6e2ec
            );
        }
        """)

        main_layout = QVBoxLayout()

        # ---------------- TITLE ----------------
        title = QLabel("AGV CONTROL DASHBOARD")
        title.setFont(QFont("Arial",22,QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color:#2c3e50; padding:10px")

        main_layout.addWidget(title)

        # ---------------- SUMMARY CARD ----------------
        summary_card = QFrame()
        summary_card.setStyleSheet("""
        QFrame{
            background:white;
            border-radius:14px;
            padding:18px;
            border:1px solid #cfd6dd;
        }
        """)

        summary_layout = QVBoxLayout()

        # AGV STATUS
        self.agv_status = QLabel("● AGV ONLINE")
        self.agv_status.setFont(QFont("Arial",14,QFont.Bold))
        self.agv_status.setStyleSheet("color:#27ae60")

        summary_layout.addWidget(self.agv_status)

        # ---------------- BATTERY ----------------
        battery_row = QHBoxLayout()

        battery_label = QLabel("🔋 Battery")

        self.battery_bar = QProgressBar()
        self.battery_bar.setValue(100)
        self.battery_bar.setFixedHeight(24)

        self.battery_text = QLabel("100%")
        self.battery_text.setMinimumWidth(40)

        battery_row.addWidget(battery_label)
        battery_row.addWidget(self.battery_bar)
        battery_row.addWidget(self.battery_text)

        summary_layout.addLayout(battery_row)

        # ---------------- STATUS CARDS ----------------
        status_row = QHBoxLayout()

        self.position_card = self.create_status_card("📍 Position", "A1")
        self.state_card = self.create_status_card("⚡ State", "Idle")
        self.task_card = self.create_status_card("📦 Task", "None")

        status_row.addWidget(self.position_card)
        status_row.addWidget(self.state_card)
        status_row.addWidget(self.task_card)

        summary_layout.addLayout(status_row)

        summary_card.setLayout(summary_layout)

        main_layout.addWidget(summary_card)

        # ---------------- NAVIGATION BUTTONS ----------------
        grid = QGridLayout()
        grid.setSpacing(20)

        task_btn = QPushButton("📦 Task Management")
        map_btn = QPushButton("🗺 Warehouse Map")
        camera_btn = QPushButton("📷 Camera Feed")
        agv_btn = QPushButton("🤖 AGV Status")
        sys_btn = QPushButton("⚙ System Status")
        log_btn = QPushButton("📜 Activity Log")

        task_btn.clicked.connect(self.main_window.show_task)
        agv_btn.clicked.connect(self.main_window.show_agv_status)
        sys_btn.clicked.connect(self.main_window.show_system_status)
        log_btn.clicked.connect(self.main_window.show_activity_log)
        map_btn.clicked.connect(self.main_window.show_map)
        camera_btn.clicked.connect(self.main_window.show_camera)

        buttons = [task_btn, camera_btn, map_btn, agv_btn, sys_btn, log_btn]

        for b in buttons:
            b.setMinimumHeight(100)
            b.setFont(QFont("Arial",14,QFont.Bold))
            b.setStyleSheet("""
            QPushButton{
                background:white;
                border:2px solid #d0d7de;
                border-radius:15px;
            }
            QPushButton:hover{
                background:#3498db;
                color:white;
                border:2px solid #3498db;
            }
            QPushButton:pressed{
                background:#2c80b4;
            }
            """)

        grid.addWidget(task_btn,0,0)
        grid.addWidget(camera_btn,0,1)

        grid.addWidget(map_btn,1,0)
        grid.addWidget(agv_btn,1,1)

        grid.addWidget(sys_btn,2,0)
        grid.addWidget(log_btn,2,1)

        main_layout.addLayout(grid)

        self.setLayout(main_layout)

        # ---------------- TIMER ----------------
        self.battery_level = 100

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_dashboard)
        self.timer.start(3000)

    # ---------------- STATUS CARD ----------------
    def create_status_card(self, title, value):

        card = QFrame()
        card.setStyleSheet("""
        QFrame{
            background:#f4f7fb;
            border-radius:10px;
            padding:10px;
            border:1px solid #e1e4e8;
        }
        """)

        layout = QVBoxLayout()

        label_title = QLabel(title)
        label_title.setStyleSheet("color:gray")

        value_label = QLabel(value)
        value_label.setFont(QFont("Arial",14,QFont.Bold))

        layout.addWidget(label_title)
        layout.addWidget(value_label)

        card.setLayout(layout)

        card.value_label = value_label

        return card

    # ---------------- UPDATE BATTERY ----------------
    def update_dashboard(self):

        self.battery_level -= random.randint(0,2)

        if self.battery_level > 50:
            color = "#27ae60"
        elif self.battery_level > 20:
            color = "#f39c12"
        else:
            color = "#e74c3c"

        self.battery_bar.setStyleSheet(f"""
        QProgressBar::chunk {{
            background:{color};
        }}
        """)

        self.battery_bar.setValue(self.battery_level)
        self.battery_text.setText(str(self.battery_level) + "%")
