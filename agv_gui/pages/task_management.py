from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout,
    QComboBox, QGroupBox, QListWidget, QMessageBox
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt


class TaskPage(QWidget):

    def __init__(self, main_window):
        super().__init__()

        # ✅ FIX: assign here
        self.main_window = main_window
        self.setStyleSheet("background-color:#eef2f5;")

        main_layout = QVBoxLayout()

        # ---------------- TOP BAR ----------------
        top_bar = QHBoxLayout()

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

        title = QLabel("Task Management")
        title.setFont(QFont("Arial",18))
        title.setAlignment(Qt.AlignCenter)

        top_bar.addWidget(back_btn)
        top_bar.addStretch()
        top_bar.addWidget(title)
        top_bar.addStretch()

        main_layout.addLayout(top_bar)

        # ---------------- CURRENT TASK ----------------
        self.current_task = QLabel("Current Task: None")
        self.current_task.setStyleSheet("font-size:14px; color:#2c3e50")
        main_layout.addWidget(self.current_task)

        # ---------------- MAIN CONTENT ----------------
        content_layout = QHBoxLayout()

        # -------- Mission Setup --------
        mission_box = QGroupBox("Mission Setup")
        mission_box.setStyleSheet("""
        QGroupBox{
            background-color:white;
            border:1px solid #d0d7de;
            border-radius:10px;
            font-weight:bold;
            padding:10px;
        }
        """)

        mission_layout = QVBoxLayout()

        rack_row = QHBoxLayout()
        rack_label = QLabel("Pickup Rack:")
        self.rack_select = QComboBox()
        self.rack_select.addItems(["A1","A2","A3","B1","B2","B3","C1","C2","C3"])

        rack_row.addWidget(rack_label)
        rack_row.addWidget(self.rack_select)

        drop_row = QHBoxLayout()
        drop_label = QLabel("Drop Station:")
        self.drop_select = QComboBox()
        self.drop_select.addItems(["Station A","Station B","Station C"])

        drop_row.addWidget(drop_label)
        drop_row.addWidget(self.drop_select)

        start_btn = QPushButton("Start Mission")
        start_btn.setMinimumHeight(45)

        start_btn.setStyleSheet("""
        QPushButton{
            background-color:#2ecc71;
            color:white;
            border-radius:8px;
            font-weight:bold;
        }
        QPushButton:hover{
            background-color:#27ae60;
        }
        """)

        # ✅ FIX: connect correct function
        start_btn.clicked.connect(self.start_mission)

        mission_layout.addLayout(rack_row)
        mission_layout.addLayout(drop_row)
        mission_layout.addStretch()
        mission_layout.addWidget(start_btn)

        mission_box.setLayout(mission_layout)

        # -------- Task Queue --------
        queue_box = QGroupBox("Task Queue")
        queue_box.setStyleSheet("""
        QGroupBox{
            background-color:white;
            border:1px solid #d0d7de;
            border-radius:10px;
            font-weight:bold;
            padding:10px;
        }
        """)

        queue_layout = QVBoxLayout()

        self.queue_list = QListWidget()

        queue_layout.addWidget(self.queue_list)
        queue_box.setLayout(queue_layout)

        content_layout.addWidget(mission_box,1)
        content_layout.addWidget(queue_box,2)

        main_layout.addLayout(content_layout)

        # -------- Complete Task Button --------
        complete_btn = QPushButton("Simulate Task Completion")
        complete_btn.clicked.connect(self.complete_task)

        main_layout.addWidget(complete_btn)

        self.setLayout(main_layout)

    # ---------------- START MISSION ----------------
    def start_mission(self):

        rack = self.rack_select.currentText()
        drop = self.drop_select.currentText()

        mission = f"{rack} → {drop}"

        # add to queue
        self.queue_list.addItem(mission)

        if self.current_task.text() == "Current Task: None":
            self.current_task.setText(f"Current Task: {mission}")

        # ✅ FIX: send to map
        self.main_window.map_page.start_mission(rack, drop)

        # ✅ FIX: switch page
        self.main_window.show_map()

    # ---------------- COMPLETE TASK ----------------
    def complete_task(self):

        if self.queue_list.count() == 0:
            return

        finished = self.queue_list.takeItem(0)

        QMessageBox.information(
            self,
            "Task Completed",
            f"Mission completed:\n{finished.text()}"
        )

        if self.queue_list.count() > 0:
            next_task = self.queue_list.item(0).text()
            self.current_task.setText(f"Current Task: {next_task}")
        else:
            self.current_task.setText("Current Task: None")
