from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout,
    QComboBox, QGroupBox, QListWidget, QMessageBox, QLineEdit
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
import json


class TaskPage(QWidget):

    def __init__(self, main_window, ros_node=None):
        super().__init__()

        self.main_window = main_window
        self.ros_node = ros_node
        self.setStyleSheet("background-color:#eef2f5;")

        # Subscribe to path results if ROS node is available
        if self.ros_node:
            self.ros_node.register_path_result_callback(self.on_path_result)

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
        self.rack_input = QLineEdit()
        self.rack_input.setPlaceholderText("Enter rack location (e.g., A1, B2, C3)")

        rack_row.addWidget(rack_label)
        rack_row.addWidget(self.rack_input)

        drop_row = QHBoxLayout()
        drop_label = QLabel("Drop Station:")
        self.drop_input = QLineEdit()
        self.drop_input.setPlaceholderText("Enter drop station (e.g., HOME-1, HOME-2, HOME-3)")

        drop_row.addWidget(drop_label)
        drop_row.addWidget(self.drop_input)

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

        rack = self.rack_input.text().strip().upper()
        drop = self.drop_input.text().strip().upper()

        if not rack or not drop:
            QMessageBox.warning(
                self,
                "Input Required",
                "Please enter both pickup rack and drop station locations."
            )
            return

        mission = f"{rack} → {drop}"

        # add to queue
        self.queue_list.addItem(mission)

        if self.current_task.text() == "Current Task: None":
            self.current_task.setText(f"Current Task: {mission}")

        # Send mission to ROS path planner
        if self.ros_node is not None:
            status = self.ros_node.get_status_snapshot()
            start_node = status.get('position', '').upper() or rack
            self.ros_node.send_path_query(start_node, drop)

        # send mission to UI map
        self.main_window.map_page.start_mission(rack, drop)

        # switch page
        self.main_window.show_map()

    # ---------------- PATH RESULT HANDLER ----------------
    def on_path_result(self, result):
        """Handle path planning results from grid_path_planner_node"""
        try:
            if result.get('status') == 'success':
                path = result.get('path', [])
                navigation = result.get('navigation', [])
                total_steps = result.get('total_steps', 0)

                # Display path information
                path_str = " → ".join(path)
                QMessageBox.information(
                    self,
                    "Path Planned Successfully",
                    f"Path found: {path_str}\n"
                    f"Total steps: {total_steps}\n\n"
                    f"Navigation steps:\n" +
                    "\n".join([f"{step['step']}. {step['node']} - {step['action']}"
                              for step in navigation])
                )

                # Update current task with path info
                current = self.current_task.text()
                if "Current Task:" in current:
                    self.current_task.setText(f"{current} (Path planned)")

            elif result.get('status') == 'error':
                error_msg = result.get('message', 'Unknown error')
                QMessageBox.warning(
                    self,
                    "Path Planning Error",
                    f"Failed to plan path: {error_msg}"
                )

        except Exception as e:
            QMessageBox.warning(
                self,
                "Path Result Error",
                f"Error processing path result: {str(e)}"
            )

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
