import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget
from pages.home import HomePage
from pages.task_management import TaskPage
from pages.agv_status import AGVStatusPage
from pages.system_status import SystemStatusPage
from pages.activity_log import ActivityLogPage
from pages.warehouse_map import WarehouseMapPage


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("AGV Control System")
        self.setGeometry(100,100,1000,700)

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.home = HomePage(self)
        self.task = TaskPage(self)
        self.agv_status = AGVStatusPage(self)
        self.system_status = SystemStatusPage(self)
        self.activity_log = ActivityLogPage(self)
        self.map_page = WarehouseMapPage(self)

        self.stack.addWidget(self.home)
        self.stack.addWidget(self.task)
        self.stack.addWidget(self.agv_status)
        self.stack.addWidget(self.system_status)
        self.stack.addWidget(self.activity_log)
        self.stack.addWidget(self.map_page)

    def show_home(self):
        self.stack.setCurrentIndex(0)

    def show_task(self):
        self.stack.setCurrentIndex(1)
        
    def show_agv_status(self):
        self.stack.setCurrentWidget(self.agv_status)
        
    def show_system_status(self):
        self.stack.setCurrentWidget(self.system_status)
        
    def show_activity_log(self):
        self.stack.setCurrentWidget(self.activity_log)
        
    def show_map(self):
        self.stack.setCurrentWidget(self.map_page)


app = QApplication(sys.argv)

window = MainWindow()
window.show()

sys.exit(app.exec_())
