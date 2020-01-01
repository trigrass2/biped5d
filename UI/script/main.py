#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QApplication
from robot_choice_func import Robot_choice_func
import sys
sys.path.append("./src/birl_module_robot/UI/scripts/climbot5d")

if __name__ == "__main__":

    app = QApplication(sys.argv)
    widows = Robot_choice_func() 
    widows.show()
    sys.exit(app.exec_())