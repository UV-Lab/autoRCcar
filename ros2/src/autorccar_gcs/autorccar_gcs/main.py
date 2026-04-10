import sys

import rclpy
from threading import Thread
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from autorccar_gcs.ros2_node import Ros2Node
from autorccar_gcs.gui import GcsGui


def window_style(app):
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)


class GcsController:
    def __init__(self, ros_node, gui):
        self.ros_node = ros_node
        self.gui = gui
        self.ros_node.nav_status_callback_fn = self.on_nav_status_received

    def on_nav_status_received(self, msg):
        self.gui.update_nav_status_signal.emit(msg)

    def send_command(self, cmd):
        self.ros_node.publish_command(cmd)

    def send_set_yaw(self, yaw):
        self.ros_node.publish_set_yaw(yaw)

    def send_global_path(self, path_list):
        self.ros_node.publish_global_path(path_list)


def main(args=None):
    rclpy.init(args=args)
    ros_node = Ros2Node()

    app = QApplication(sys.argv)
    window_style(app)
    gui = GcsGui(controller=None)  # Controller will be set next

    controller = GcsController(ros_node, gui)
    gui.controller = controller  # Set controller in GUI

    ros_thread = Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
