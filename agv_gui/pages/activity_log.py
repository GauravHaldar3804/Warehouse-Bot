from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QPushButton,
    QHBoxLayout, QListWidget, QListWidgetItem
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from datetime import datetime


class ActivityLogPage(QWidget):

    def __init__(self, main_window, ros_node=None):
        super().__init__()

        self.main_window = main_window
        self.ros_node = ros_node
        self.setStyleSheet("background-color:#f5f7fb; color:#111827; font-family:Segoe UI, Arial, sans-serif;")

        main_layout = QVBoxLayout()

        # HEADER
        header = QHBoxLayout()

        back_btn = QPushButton('← Back')
        back_btn.setFixedSize(100, 35)
        back_btn.setStyleSheet(
            'QPushButton{background-color:#3498db; color:white; border-radius:8px; font-weight:bold;}'
            'QPushButton:hover{background-color:#2980b9;}'
        )
        back_btn.clicked.connect(self.main_window.show_home)

        title = QLabel("Activity Log")
        title.setFont(QFont("Arial",20,QFont.Bold))

        header.addWidget(back_btn)
        header.addStretch()
        header.addWidget(title)
        header.addStretch()

        main_layout.addLayout(header)

        main_layout.addSpacing(20)

        # LOG LIST
        self.log_list = QListWidget()

        self.log_list.setStyleSheet("""
        QListWidget{
            background:white;
            border-radius:16px;
            padding:14px;
            font-size:13px;
            border:1px solid #e5e7eb;
        }
        """)

        main_layout.addWidget(self.log_list)

        # TEST BUTTON (simulate events)
        test_btn = QPushButton("Simulate Event")
        test_btn.clicked.connect(self.add_test_log)
        test_btn.setFixedHeight(42)
        test_btn.setStyleSheet("""
        QPushButton{
            background:#ffffff;
            color:#1f2937;
            border:1px solid #d1d5db;
            border-radius:14px;
            padding:10px 14px;
        }
        QPushButton:hover{
            background:#eef2ff;
        }
        """)

        main_layout.addWidget(test_btn)

        self.setLayout(main_layout)

    # ADD LOG ENTRY
    def add_log(self,message):

        time = datetime.now().strftime("%H:%M:%S")

        item = QListWidgetItem(f"[{time}]  {message}")

        self.log_list.addItem(item)
        self.log_list.scrollToBottom()

    # TEST FUNCTION
    def add_test_log(self):

        events = [
            "Mission created A2 → Station B",
            "AGV navigating",
            "Rack marker detected",
            "Picking box",
            "Delivering box",
            "Mission completed"
        ]

        import random
        self.add_log(random.choice(events))
