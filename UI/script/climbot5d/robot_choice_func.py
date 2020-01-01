#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append("./src/birl_module_robot/UI/scripts/climbot5d")
sys.path.append("./src/birl_module_robot/UI/scripts/wheel_bipedal_7d")
from PyQt5.QtWidgets import QMainWindow,QDesktopWidget
from PyQt5.QtCore import pyqtSignal
from climbot5d_mode_set_func import climbot5d_Mode_set_func
# from arm5d_mode_set_func import Arm5d_Mode_set_func
from robot_choice import Ui_MainWindow
from rosnode import kill_nodes

class Robot_choice_func(QMainWindow,Ui_MainWindow):

    def __init__(self,parent=None):
        super(Robot_choice_func,self).__init__(parent)
        self.setupUi(self)
        self.simulation = False
        self.center()

    def climbot5d(self):
        self.windows_climbot5d_mode_set = climbot5d_Mode_set_func(self.simulation)
        self.windows_climbot5d_mode_set.sin_close.connect(self.close_climbot5d_mode_set)
        self.form.hide()
        self.windows_climbot5d_mode_set.show()
        pass

    def robot_arm5d(self):
        # self.windows_arm5d_mode_set = Arm5d_Mode_set_func(self.simulation)
        # self.windows_arm5d_mode_set.sin_close.connect(self.close_arm5d_mode_set)
        # self.form.hide()
        # self.windows_arm5d_mode_set.show()
        pass

    def wheel_bipedal_7d(self):
        self.windows_wheel_bipedal_7d_mode_set = 
        pass

    def close_climbot5d_mode_set(self):
        self.windows_climbot5d_mode_set.close()
        self.form.show()
        pass

    def close_arm5d_mode_set(self):
        self.windows_arm5d_mode_set.close()
        self.form.show()

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def if_simulation(self,pressed):
        if pressed:
            self.simulation = True
        else:
            self.simulation = False


""""
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    widows = Robot_choice_func()
    widows.show()
    sys.exit(app.exec_())
"""