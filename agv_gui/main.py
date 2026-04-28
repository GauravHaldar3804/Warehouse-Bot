import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget
from PyQt5.QtCore import QTimer
import rclpy

from pages.home import HomePage
from pages.task_management import TaskPage
from pages.agv_status import AGVStatusPage
from pages.system_status import SystemStatusPage
from pages.activity_log import ActivityLogPage
from pages.warehouse_map import WarehouseMapPage
from ros_interface.ros_node import DashboardRosNode


class MainWindow(QMainWindow):

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("AGV Control System")
        self.setGeometry(100,100,1000,700)

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.home = HomePage(self)
        self.task = TaskPage(self, self.ros_node)
        self.agv_status = AGVStatusPage(self, self.ros_node)
        self.system_status = SystemStatusPage(self, self.ros_node)
        self.activity_log = ActivityLogPage(self, self.ros_node)
        self.map_page = WarehouseMapPage(self, self.ros_node)

        self.stack.addWidget(self.home)
        self.stack.addWidget(self.task)
        self.stack.addWidget(self.agv_status)
        self.stack.addWidget(self.system_status)
        self.stack.addWidget(self.activity_log)
        self.stack.addWidget(self.map_page)

        self.ros_node.activity_log = self.activity_log
        self.show_home()

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


def main():
    rclpy.init(args=None)
    ros_node = DashboardRosNode()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    spin_timer.start(30)

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        spin_timer.stop()
        ros_node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
