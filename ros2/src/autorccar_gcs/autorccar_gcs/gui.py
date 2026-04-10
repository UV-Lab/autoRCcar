import os
import math

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from .submodules.user_geometry import *
from .submodules.cubic_spline import *


class GcsGui(QMainWindow):
    update_nav_status_signal = pyqtSignal(object)

    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.update_nav_status_signal.connect(self.update_nav_status)
        self.setWindowTitle("GCS")
        self.window_width, self.window_height = 1028, 720
        self.setMinimumSize(self.window_width, self.window_height)
        self.init_ui()
        self.reset_parameters()

    def init_ui(self):
        tabs = QTabWidget()
        tab1 = QWidget()
        tab2 = QWidget()
        tab3 = QWidget()

        tabs.addTab(tab1, "main")
        tabs.addTab(tab2, "details")
        tabs.addTab(tab3, "param")

        self.setCentralWidget(tabs)

        grid_main = QGridLayout()

        tab1.setLayout(grid_main)

        self.position_graph = self.set_widget_position_graph()
        self.position_graph.plotItem.setMenuEnabled(False)
        self.position_graph.scene().sigMouseClicked.connect(
            self.position_graph_mouse_clicked_callback
        )

        labelfont_1 = QLabel().font()
        labelfont_1.setPointSize(12)
        labelfont_1.setBold(True)

        grid_main_1 = QGridLayout()

        grid1_lab = QLabel("Graph")
        grid1_lab.setFont(labelfont_1)
        grid1_lab.setAlignment(Qt.AlignCenter)
        grid_main_1.addWidget(self.position_graph, 0, 0)
        grid_main_1.addWidget(QProgressBar(self), 1, 0)

        self.layout_pos = self.set_nav_vlayout("Pos", ["E", "N", "U"])
        self.layout_vel = self.set_nav_vlayout("Vel", ["E", "N", "U"])
        self.layout_att = self.set_nav_vlayout("Att", ["Roll", "Pitch", "Yaw"])

        grid_main_2 = QGridLayout()

        grid2_lab = QLabel("Nav")
        grid2_lab.setFont(labelfont_1)
        grid2_lab.setAlignment(Qt.AlignCenter)

        grid_main_2_1 = QGridLayout()
        grid_main_2_2 = QGridLayout()

        grid_main_2_1.addLayout(self.layout_pos, 0, 0, alignment=Qt.AlignTop)
        grid_main_2_1.addLayout(self.layout_vel, 1, 0, alignment=Qt.AlignTop)
        grid_main_2_1.addLayout(self.layout_att, 2, 0, alignment=Qt.AlignTop)

        btn_clear = QPushButton("Clear All", self)
        btn_import_path = QPushButton("Import Path", self)
        btn_export_path = QPushButton("Export Path", self)
        btn_send_path = QPushButton("Send Path", self)
        btn_set_yaw = QPushButton("Set Yaw", self)
        btn_start = QPushButton("Start", self)
        btn_stop = QPushButton("Stop", self)

        btn_clear.clicked.connect(self.clear_all)
        btn_import_path.clicked.connect(self.import_path)
        btn_export_path.clicked.connect(self.export_path)
        btn_send_path.clicked.connect(self.send_path)
        btn_set_yaw.clicked.connect(self.send_set_yaw)
        btn_start.clicked.connect(self.send_start_command)
        btn_stop.clicked.connect(self.send_stop_command)

        grid_main_2_2.addWidget(btn_clear, 1, 0)
        grid_main_2_2.addWidget(btn_import_path, 2, 0)
        grid_main_2_2.addWidget(btn_export_path, 3, 0)
        grid_main_2_2.addWidget(btn_send_path, 4, 0)
        grid_main_2_2.addWidget(btn_set_yaw, 5, 0)
        grid_main_2_2.addWidget(btn_start, 6, 0)
        grid_main_2_2.addWidget(btn_stop, 7, 0)

        grid_main_2.addLayout(grid_main_2_1, 0, 0)
        grid_main_2.addLayout(grid_main_2_2, 1, 0)
        grid_main_2.setRowStretch(0, 1)
        grid_main_2.setRowStretch(1, 1)

        grid_main.addWidget(grid1_lab, 0, 0)
        grid_main.addWidget(grid2_lab, 0, 1)
        grid_main.addLayout(grid_main_1, 1, 0)
        grid_main.addLayout(grid_main_2, 1, 1)
        grid_main.setColumnStretch(0, 7)
        grid_main.setColumnStretch(1, 3)

    def reset_parameters(self):
        self.origin_filename_ = "test.txt"
        self.plot_points_ = []
        self.x_points_ = []
        self.y_points_ = []
        self.num_points_ = 0
        self.is_point_moving_ = False
        self.close_point_idx_ = 0
        self.res_x_points_ = []
        self.res_y_points_ = []
        self.res_headings_ = []
        self.res_curvatures_ = []
        self.res_distances = []
        self.plot_spline_ = []
        self.pos_traj_e = []
        self.pos_traj_n = []
        self.plot_current_pos = self.position_graph.plot(
            symbol="o", symbolSize=20, symbolBrush="y", name="current_pos"
        )
        self.plot_pos_traj = self.position_graph.plot(
            pen=pg.mkPen(width=2, color="y"), name="pos"
        )

    def set_widget_position_graph(self):
        graph = pg.PlotWidget()

        graph.setTitle("Position")
        graph.setLabel("left", "North[m]")
        graph.setLabel("bottom", "East[m]")
        graph.showGrid(x=True, y=True)
        graph.addLegend()

        graph.setRange(rect=None, xRange=(-10, 10), yRange=(-10, 10))

        return graph

    def set_nav_vlayout(self, title, str):
        titlefont = QLabel().font()
        titlefont.setBold(True)

        title_lab = QLabel(title)
        title_lab.setFont(titlefont)

        layout = QGridLayout()
        layout.addWidget(title_lab, 0, 0)
        layout.addWidget(QLabel(str[0]), 1, 0)
        layout.addWidget(QLineEdit("0.0000"), 1, 1)
        layout.addWidget(QLabel(str[1]), 2, 0)
        layout.addWidget(QLineEdit("0.0000"), 2, 1)
        layout.addWidget(QLabel(str[2]), 3, 0)
        layout.addWidget(QLineEdit("0.0000"), 3, 1)

        return layout

    def position_graph_mouse_clicked_callback(self, evt):
        vb = self.position_graph.plotItem.vb
        scene_coords = evt.scenePos()
        if self.position_graph.sceneBoundingRect().contains(scene_coords):
            mouse_point = vb.mapSceneToView(scene_coords)
            if evt.button() == 1:
                if self.is_point_moving_:
                    close_point_idx = self.close_point_idx_
                    self.position_graph.removeItem(self.plot_points_[close_point_idx])
                    del self.plot_points_[close_point_idx]
                    plot_point = self.position_graph.plot(
                        x=[mouse_point.x()],
                        y=[mouse_point.y()],
                        symbol="o",
                        symbolSize=20,
                        symbolBrush="w",
                    )
                    self.plot_points_.insert(close_point_idx, plot_point)
                    self.x_points_[close_point_idx] = mouse_point.x()
                    self.y_points_[close_point_idx] = mouse_point.y()
                    self.is_point_moving_ = False
                    self.update_path()
                else:
                    close_point_idx = self.is_close_point(mouse_point)
                    if close_point_idx != []:
                        self.position_graph.removeItem(
                            self.plot_points_[close_point_idx]
                        )
                        del self.plot_points_[close_point_idx]
                        plot_point = self.position_graph.plot(
                            x=[self.x_points_[close_point_idx]],
                            y=[self.y_points_[close_point_idx]],
                            symbol="o",
                            symbolSize=20,
                            symbolBrush="r",
                        )
                        self.plot_points_.insert(close_point_idx, plot_point)
                        self.is_point_moving_ = True
                        self.close_point_idx_ = close_point_idx
                    else:
                        print("add point")
                        self.x_points_.append(mouse_point.x())
                        self.y_points_.append(mouse_point.y())
                        plot_point = self.position_graph.plot(
                            x=[mouse_point.x()],
                            y=[mouse_point.y()],
                            symbol="o",
                            symbolSize=20,
                            symbolBrush="w",
                        )
                        self.plot_points_.append(plot_point)
                        self.num_points_ += 1
                        self.update_path()
            elif evt.button() == 2:
                self.position_graph.removeItem(self.plot_points_[-1])
                del self.plot_points_[-1]
                del self.x_points_[-1]
                del self.y_points_[-1]
                self.num_points_ -= 1
                self.update_path()

    def update_path(self):
        if self.num_points_ > 1:
            [
                self.res_x_points_,
                self.res_y_points_,
                self.res_headings_,
                self.res_curvatures_,
                self.res_distances,
            ] = CalculateCubicSplinePath(self.x_points_, self.y_points_, 0.5)
            self.position_graph.removeItem(self.plot_spline_)
            self.plot_spline_ = self.position_graph.plot(
                x=self.res_x_points_,
                y=self.res_y_points_,
                pen=pg.mkPen(width=2, color="w", style=QtCore.Qt.DashLine),
            )

            self.pos_traj_e = []
            self.pos_traj_n = []

    def is_close_point(self, mouse_point):
        for idx in range(self.num_points_):
            d = math.sqrt(
                (mouse_point.x() - self.x_points_[idx]) ** 2
                + (mouse_point.y() - self.y_points_[idx]) ** 2
            )
            if d < 0.1:
                return idx
        return []

    def clear_plot_points(self):
        self.plot_points_ = []
        self.position_graph.clear()

    def clear_all(self):
        self.clear_plot_points()
        self.reset_parameters()
        self.position_graph.setRange(rect=None, xRange=(-10, 10), yRange=(-10, 10))

    def send_path(self):
        path_list = []
        for idx in range(len(self.x_points_)):
            path_list.append([self.x_points_[idx], self.y_points_[idx]])

        print("send path")
        self.controller.send_global_path(path_list)

    def send_set_yaw(self):
        val, ok = QInputDialog.getDouble(
            self, "Set Yaw", "Enter the yaw angle relative to true north:"
        )
        if ok:
            yaw = val
            while yaw > 180:
                yaw = yaw - 360
            while yaw <= -180:
                yaw = yaw + 360
            print("set yaw")
            self.controller.send_set_yaw(yaw)

    def send_start_command(self):
        print("start")
        self.controller.send_command(1)

    def send_stop_command(self):
        print("stop")
        self.controller.send_command(0)

    def update_nav_status(self, msg):
        pos_e = msg.position.x
        pos_n = msg.position.y
        pos_u = msg.position.z

        vel_e = msg.velocity.x
        vel_n = msg.velocity.y
        vel_u = msg.velocity.z

        qx = msg.quaternion.x
        qy = msg.quaternion.y
        qz = msg.quaternion.z
        qw = msg.quaternion.w

        eulr = quat2eulr([qw, qx, qy, qz])

        roll = eulr[0] * 180 / math.pi
        pitch = eulr[1] * 180 / math.pi
        yaw = eulr[2] * 180 / math.pi

        self.layout_pos.itemAt(2).widget().setText(str(pos_e))
        self.layout_pos.itemAt(4).widget().setText(str(pos_n))
        self.layout_pos.itemAt(6).widget().setText(str(pos_u))
        self.layout_vel.itemAt(2).widget().setText(str(vel_e))
        self.layout_vel.itemAt(4).widget().setText(str(vel_n))
        self.layout_vel.itemAt(6).widget().setText(str(vel_u))
        self.layout_att.itemAt(2).widget().setText(str(roll))
        self.layout_att.itemAt(4).widget().setText(str(pitch))
        self.layout_att.itemAt(6).widget().setText(str(yaw))
        self.update_graph_with_current_position(pos_e, pos_n)

    def update_graph_with_current_position(self, pos_e, pos_n):
        self.pos_traj_e.append(pos_e)
        self.pos_traj_n.append(pos_n)
        self.plot_current_pos.setData(x=[pos_e], y=[pos_n])
        self.plot_pos_traj.setData(x=self.pos_traj_e, y=self.pos_traj_n)

    def import_path(self):
        filename = QtWidgets.QFileDialog.getOpenFileName(self, "Import Path File")
        self.origin_filename_ = filename[0]
        with open(self.origin_filename_, "r") as f:
            lines = f.readlines()
            self.x_points_ = []
            self.y_points_ = []
            self.clear_plot_points()
            for line in lines:
                line.strip()
                splited = line.split()
                listfloat = list(map(float, splited))
                self.x_points_.append(listfloat[0])
                self.y_points_.append(listfloat[1])
                plot_point = self.position_graph.plot(
                    x=[listfloat[0]],
                    y=[listfloat[1]],
                    symbol="o",
                    symbolSize=20,
                    symbolBrush="y",
                )
                self.plot_points_.append(plot_point)
            self.num_points_ = len(self.x_points_)
            x_max = max(self.x_points_)
            x_min = min(self.x_points_)
            x_cen = (x_max + x_min) / 2.0
            y_max = max(self.y_points_)
            y_min = min(self.y_points_)
            y_cen = (y_max + y_min) / 2.0
            x_half_range = (x_max - x_min) / 2.0 + 2
            y_half_range = (y_max - y_min) / 2.0 + 2
            if x_half_range > y_half_range:
                x_range = (x_cen - x_half_range, x_cen + x_half_range)
                y_range = (y_cen - x_half_range, y_cen + x_half_range)
            else:
                x_range = (x_cen - y_half_range, x_cen + y_half_range)
                y_range = (y_cen - y_half_range, y_cen + y_half_range)
            self.position_graph.setRange(rect=None, xRange=x_range, yRange=y_range)
        self.update_path()
        print("path is imported")

    def export_path(self):
        filename = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export Path File",
            os.path.join("", self.origin_filename_),
            "Text files (*.txt)",
        )
        filename_text = filename[0]
        export_list = []
        for idx in range(len(self.x_points_)):
            export_list.append([self.x_points_[idx], self.y_points_[idx]])

        with open(filename_text, "w") as file:
            file.writelines(" ".join(str(j) for j in i) + "\n" for i in export_list)
        print("points are exported")
