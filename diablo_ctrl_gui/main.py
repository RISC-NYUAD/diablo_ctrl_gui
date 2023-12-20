import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor

from PyQt5 import QtWidgets
from threading import Thread

from .diablo_ctrl_gui import Ui_MainWindow
from .gui_ros_node import MyGuiNode


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)

    ctrl_gui_node = MyGuiNode(ui)

    # Now we create a thread for the app, and let the ROS
    # node spin on the main thread
    executor = MultiThreadedExecutor()
    executor.add_node(ctrl_gui_node)

    thread = Thread(target=executor.spin)
    thread.start()


    try:
        MainWindow.show()
        sys.exit(app.exec_())

    finally:
        ctrl_gui_node.get_logger().info("Shutting down ROS node.")
        ctrl_gui_node.destroy_node()
        executor.shutdown()

if __name__ == '__main__':
    main()

