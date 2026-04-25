from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QPushButton,
    QHBoxLayout, QGridLayout, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer
import random


class AGVStatusPage(QWidget):

    def __init__(self, main_window):
        super().__init__()

        self.main_window = main_window
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

        self.connection = self.create_card("Connection","Online","#2ecc71")
        self.battery = self.create_card("Battery","100%","#27ae60")
        self.position = self.create_card("Position","A1","#34495e")
        self.target = self.create_card("Target","Station A","#8e44ad")
        self.state = self.create_card("State","Idle","#f39c12")

        grid.addWidget(self.connection,0,0)
        grid.addWidget(self.battery,0,1)
        grid.addWidget(self.position,0,2)

        grid.addWidget(self.target,1,0)
        grid.addWidget(self.state,1,1)

        main_layout.addLayout(grid)

        main_layout.addStretch()

        self.setLayout(main_layout)

        # ---------------- LIVE UPDATE ----------------
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(3000)

        self.battery_level = 100

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

        positions = ["A1","A2","A3","B1","B2"]
        targets = ["Station A","Station B","Station C"]
        states = ["Idle","Navigating","Picking","Delivering"]

        self.battery_level -= random.randint(0,2)

        if self.battery_level < 20:
            self.battery.value_label.setStyleSheet("color:red")
        else:
            self.battery.value_label.setStyleSheet("color:green")

        self.battery.value_label.setText(str(self.battery_level)+"%")

        self.position.value_label.setText(random.choice(positions))
        self.target.value_label.setText(random.choice(targets))
        self.state.value_label.setText(random.choice(states))
