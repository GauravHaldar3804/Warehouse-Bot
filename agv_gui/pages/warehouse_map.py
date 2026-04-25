from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer


class WarehouseMapPage(QWidget):

    def __init__(self, main_window):
        super().__init__()

        self.main_window = main_window
        self.setStyleSheet("background-color:#eef2f5;")

        # ---------- MAIN LAYOUT ----------
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 10, 20, 10)
        main_layout.setSpacing(10)

        # ---------- HEADER ----------
        header = QHBoxLayout()

        back_btn = QPushButton("← Back")
        back_btn.setFixedSize(100, 35)
        back_btn.setStyleSheet("""
            QPushButton{
                background-color:#3498db;
                color:white;
                border-radius:8px;
                font-weight:bold;
            }
            QPushButton:hover{
                background-color:#2980b9;
            }
        """)
        back_btn.clicked.connect(self.main_window.show_home)

        title = QLabel("Warehouse Map")
        title.setFont(QFont("Arial", 22, QFont.Bold))
        title.setStyleSheet("color:#2c3e50;")

        header.addWidget(back_btn)
        header.addSpacing(20)
        header.addWidget(title)
        header.addStretch()

        main_layout.addLayout(header)

        # ---------- GRID CONTAINER ----------
        container = QFrame()
        container.setStyleSheet("""
            QFrame{
                background:white;
                border-radius:15px;
                border:1px solid #d0d7de;
            }
        """)

        container_layout = QVBoxLayout()
        container_layout.setContentsMargins(20, 20, 20, 20)

        # ---------- GRID ----------
        self.grid = QGridLayout()
        self.grid.setSpacing(6)   # 👈 controlled spacing

        self.cells = {}

        rows = 6
        cols = 5

        for r in range(rows):
            for c in range(cols):
                label = QLabel("")
                label.setAlignment(Qt.AlignCenter)
                label.setFixedSize(110, 85)   # 👈 bigger cells

                label.setStyleSheet("""
                    border:1px solid #d0d7de;
                    background:#f4f7fa;
                    border-radius:10px;
                    font-weight:bold;
                """)

                self.grid.addWidget(label, r, c)
                self.cells[(r, c)] = label

        container_layout.addLayout(self.grid)
        container.setLayout(container_layout)

        main_layout.addWidget(container, alignment=Qt.AlignCenter)

        self.setLayout(main_layout)

        # ---------- AGV STATE ----------
        self.agv_pos = (0, 4)
        self.path = []
        self.index = 0
        self.target = None

        # ---------- TIMER ----------
        self.timer = QTimer()
        self.timer.timeout.connect(self.move_agv)
        self.timer.start(600)

        self.draw_map()

    # =========================================================
    # 🎯 START MISSION (called from Task Page)
    # =========================================================
    def start_mission(self, pickup, drop):

        position_map = {
            "A1": (2,1), "A2": (2,2), "A3": (2,3),
            "B1": (3,1), "B2": (3,2), "B3": (3,3),
            "C1": (4,1), "C2": (4,2), "C3": (4,3),
            "Station A": (0,0),
            "Station B": (0,1),
            "Station C": (0,2)
        }

        pickup_pos = position_map[pickup]
        drop_pos = position_map[drop]

        # generate path
        path1 = self.generate_path(self.agv_pos, pickup_pos)
        path2 = self.generate_path(pickup_pos, drop_pos)

        self.path = path1 + path2
        self.index = 0
        self.target = drop_pos

    # =========================================================
    # 🔄 PATH GENERATION
    # =========================================================
    def generate_path(self, start, end):

        path = []
        r, c = start
        tr, tc = end

        # move rows
        while r != tr:
            r += 1 if tr > r else -1
            path.append((r, c))

        # move columns
        while c != tc:
            c += 1 if tc > c else -1
            path.append((r, c))

        return path

    # =========================================================
    # 🎨 DRAW MAP
    # =========================================================
    def draw_map(self):

        # clear all cells
        for cell in self.cells.values():
            cell.setText("")
            cell.setStyleSheet("""
                border:1px solid #d0d7de;
                background:#f4f7fa;
                border-radius:10px;
                font-weight:bold;
            """)

        # ---------- STATIONS ----------
        self.set_cell(0,0,"Station A","#e74c3c")
        self.set_cell(0,1,"Station B","#f1c40f")
        self.set_cell(0,2,"Station C","#2ecc71")

        # ---------- RACKS ----------
        self.set_cell(2,1,"A1")
        self.set_cell(2,2,"A2")
        self.set_cell(2,3,"A3")

        self.set_cell(3,1,"B1")
        self.set_cell(3,2,"B2")
        self.set_cell(3,3,"B3")

        self.set_cell(4,1,"C1")
        self.set_cell(4,2,"C2")
        self.set_cell(4,3,"C3")

        # ---------- TARGET HIGHLIGHT ----------
        if self.target:
            r, c = self.target
            self.cells[(r, c)].setStyleSheet("""
                background:#f39c12;
                border:3px solid #d68910;
                border-radius:10px;
                font-weight:bold;
                color:white;
            """)

        # ---------- AGV ----------
        r, c = self.agv_pos
        cell = self.cells[(r, c)]

        cell.setText("🤖")
        cell.setStyleSheet("""
            background:#2980b9;
            color:white;
            font-size:20px;
            border-radius:12px;
            border:3px solid #1f4f75;
        """)

    # =========================================================
    # 📦 SET CELL
    # =========================================================
    def set_cell(self, r, c, text, color=None):

        cell = self.cells[(r, c)]
        cell.setText(text)

        if color:
            cell.setStyleSheet(f"""
                background:{color};
                color:white;
                border-radius:10px;
                font-weight:bold;
            """)

    # =========================================================
    # 🚗 MOVE AGV
    # =========================================================
    def move_agv(self):

        if not self.path:
            return

        if self.index < len(self.path):
            self.agv_pos = self.path[self.index]
            self.index += 1

        self.draw_map()
