from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QPushButton,
    QHBoxLayout, QGridLayout, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer


class AGVStatusPage(QWidget):

    def __init__(self, main_window, ros_node=None):
        super().__init__()

        self.main_window = main_window
        self.ros_node = ros_node
        self.setStyleSheet("background-color:#eef2f5;")

        main_layout = QVBoxLayout()

        # ---------------- HEADER ----------------
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

        title = QLabel("AGV Status")
        title.setFont(QFont("Arial",20,QFont.Bold))
        title.setAlignment(Qt.AlignCenter)

        header.addWidget(back_btn)
        header.addStretch()
        header.addWidget(title)
        header.addStretch()

        main_layout.addLayout(header)

        main_layout.addSpacing(20)

        # ---------------- STATUS GRID ----------------
        grid = QGridLayout()
        grid.setSpacing(20)

        self.connection = self.create_card("Connection", "Waiting for ROS", "#2ecc71")
        self.battery = self.create_card("Battery", "--", "#27ae60")
        self.position = self.create_card("Position", "HOME-1", "#34495e")
        self.target = self.create_card("Target", "--", "#8e44ad")
        self.state = self.create_card("State", "Waiting for mission", "#f39c12")
        self.last_command = self.create_card("Last Command", "NONE", "#9b59b6")
        self.update_age = self.create_card("Update Age", "--", "#e67e22")
        self.obstacle = self.create_card("Obstacle", "Clear", "#16a085")
        self.battery_voltage = self.create_card("Battery Voltage", "--", "#f1c40f")
        self.battery_current = self.create_card("Battery Current", "--", "#e74c3c")
        self.mission_progress = self.create_card("Mission Progress", "--", "#3498db")

        grid.addWidget(self.connection,0,0)
        grid.addWidget(self.battery,0,1)
        grid.addWidget(self.position,0,2)

        grid.addWidget(self.target,1,0)
        grid.addWidget(self.state,1,1)
        grid.addWidget(self.last_command,1,2)

        grid.addWidget(self.update_age,2,0)
        grid.addWidget(self.obstacle,2,1)
        grid.addWidget(self.battery_voltage,2,2)

        grid.addWidget(self.battery_current,3,0)
        grid.addWidget(self.mission_progress,3,1)

        main_layout.addLayout(grid)

        main_layout.addStretch()

        self.setLayout(main_layout)

        # ---------------- LIVE UPDATE ----------------
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(300)

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

    # UPDATE STATUS
    def update_status(self):
        if self.ros_node is None:
            self.connection.value_label.setText('ROS node not available')
            self.connection.value_label.setStyleSheet('color:red')
            return

        status = self.ros_node.get_status_snapshot()

        self.connection.value_label.setText(status.get('connection', 'Unknown'))
        if status.get('connection') == 'Topics active':
            self.connection.value_label.setStyleSheet('color:green')
        else:
            self.connection.value_label.setStyleSheet('color:#d35400')

        battery_text = status.get('battery', '--')
        self.battery.value_label.setText(str(battery_text))
        try:
            battery_pct = int(str(battery_text).replace('%', '').strip())
            if battery_pct < 20:
                self.battery.value_label.setStyleSheet('color:red')
            elif battery_pct < 50:
                self.battery.value_label.setStyleSheet('color:#f39c12')
            else:
                self.battery.value_label.setStyleSheet('color:green')
        except ValueError:
            self.battery.value_label.setStyleSheet('color:#2c3e50')

        self.position.value_label.setText(status.get('position', '--'))
        self.target.value_label.setText(status.get('target', '--'))
        self.state.value_label.setText(status.get('state', 'Unknown'))
        self.last_command.value_label.setText(status.get('last_motor_command', 'NONE'))

        age_sec = status.get('last_update_age_sec')
        if age_sec is not None:
            if age_sec > 10:
                self.update_age.value_label.setText(f"{age_sec:.1f}s (STALE)")
                self.update_age.value_label.setStyleSheet('color:red')
            elif age_sec > 5:
                self.update_age.value_label.setText(f"{age_sec:.1f}s")
                self.update_age.value_label.setStyleSheet('color:#f39c12')
            else:
                self.update_age.value_label.setText(f"{age_sec:.1f}s")
                self.update_age.value_label.setStyleSheet('color:green')
        else:
            self.update_age.value_label.setText('--')
            self.update_age.value_label.setStyleSheet('color:#2c3e50')

        obstacle = status.get('obstacle_detected', False)
        if obstacle:
            self.obstacle.value_label.setText('DETECTED')
            self.obstacle.value_label.setStyleSheet('color:red')
        else:
            self.obstacle.value_label.setText('Clear')
            self.obstacle.value_label.setStyleSheet('color:green')

        self.battery_voltage.value_label.setText(status.get('battery_voltage', '--'))
        self.battery_current.value_label.setText(status.get('battery_current', '--'))
        self.mission_progress.value_label.setText(status.get('mission_progress', '--'))
