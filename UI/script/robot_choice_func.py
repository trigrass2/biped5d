#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from rospkg import RosPack
rp = RosPack()
sys.path.append(rp.get_path('ui') + "/script/climbot5d")

# from os import getcwd
# print getcwd()

from PyQt5.QtWidgets import QMainWindow,QDesktopWidget
from PyQt5.QtCore import pyqtSignal
from climbot5d_mode_set_func import climbot5d_Mode_set_func
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

    

    def close_climbot5d_mode_set(self):
        self.windows_climbot5d_mode_set.close()
        del self.windows_climbot5d_mode_set
        self.form.show()
        pass

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