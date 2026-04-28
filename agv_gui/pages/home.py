from PyQt5.QtWidgets import (
    QWidget, QPushButton, QLabel, QGridLayout,
    QVBoxLayout, QHBoxLayout, QProgressBar, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer


class HomePage(QWidget):

    def __init__(self, main_window, ros_node=None):
        super().__init__()

        self.main_window = main_window
        self.ros_node = ros_node

        self.setStyleSheet("""
        QWidget{
            background:#f5f7fb;
            color:#111827;
            font-family:Segoe UI, Arial, sans-serif;
        }
        """)

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(24)

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

        self.position_card = self.create_status_card("📍 Position", "Unknown")
        self.state_card = self.create_status_card("⚡ State", "Waiting")
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
            b.setMinimumHeight(94)
            b.setFont(QFont("Arial",13,QFont.DemiBold))
            b.setStyleSheet("""
            QPushButton{
                background:white;
                color:#111827;
                border:1px solid #d8e0ea;
                border-radius:16px;
                padding:14px;
            }
            QPushButton:hover{
                background:#f3f7ff;
                border-color:#bfdbfe;
            }
            QPushButton:pressed{
                background:#e5edff;
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

        # ---------------- REAL-TIME UPDATE TIMER ----------------
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(500)  # Update every 500ms

    # ---------------- STATUS CARD ----------------
    def create_status_card(self, title, value):

        card = QFrame()
        card.setStyleSheet("""
        QFrame{
            background:white;
            border-radius:18px;
            padding:18px;
            border:1px solid #e5e7eb;
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

    # ---------------- UPDATE STATUS ----------------
    def update_status(self):
        if self.ros_node is None:
            # No ROS connection - show offline status
            self.agv_status.setText("● AGV OFFLINE")
            self.agv_status.setStyleSheet("color:#e74c3c")
            self.position_card.value_label.setText("No ROS")
            self.state_card.value_label.setText("Disconnected")
            self.task_card.value_label.setText("N/A")
            self.battery_bar.setValue(0)
            self.battery_text.setText("--")
            return

        # Get real-time status from ROS
        status = self.ros_node.get_status_snapshot()

        # Update AGV connection status
        connection = status.get('connection', 'Unknown')
        if connection == 'Topics active':
            self.agv_status.setText("● AGV ONLINE")
            self.agv_status.setStyleSheet("color:#27ae60")
        else:
            self.agv_status.setText("● AGV CONNECTING")
            self.agv_status.setStyleSheet("color:#f39c12")

        # Update position
        position = status.get('position', 'Unknown')
        self.position_card.value_label.setText(position)

        # Update state
        state = status.get('state', 'Unknown')
        self.state_card.value_label.setText(state)

        # Update task/target
        target = status.get('target', '--')
        if target and target != '--':
            self.task_card.value_label.setText(f"→ {target}")
        else:
            self.task_card.value_label.setText("None")

        # Update battery
        battery_text = status.get('battery', '--')
        self.battery_text.setText(battery_text)

        # Update battery progress bar
        try:
            if '%' in str(battery_text):
                battery_pct = int(str(battery_text).replace('%', '').strip())
            else:
                battery_pct = 0

            self.battery_bar.setValue(battery_pct)

            # Set battery bar color based on level
            if battery_pct > 50:
                color = "#27ae60"  # Green
            elif battery_pct > 20:
                color = "#f39c12"  # Orange
            else:
                color = "#e74c3c"  # Red

            self.battery_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #ddd;
                border-radius: 5px;
                text-align: center;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 4px;
            }}
            """)

        except (ValueError, AttributeError):
            self.battery_bar.setValue(0)
            self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #ddd;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #95a5a6;
                border-radius: 4px;
            }
            """)
