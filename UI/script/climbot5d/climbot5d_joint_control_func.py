#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
# import os
# print os.getcwd()
sys.path.append("./src/birl_module_robot/UI/scripts/climbot5d")
import time
from  math import fabs,degrees

from PyQt5.QtWidgets import QWidget,QDesktopWidget
from PyQt5.QtCore import QThread,pyqtSignal
from climbot5d_joint_control import Ui_Climbot5d_joint_control

class Climbot5d_joint_control_func(QWidget,Ui_Climbot5d_joint_control):

    # 各关节启动运行信号
    sin_I1_data = pyqtSignal(list)  
    sin_T2_data = pyqtSignal(list)
    sin_T3_data = pyqtSignal(list)
    sin_T4_data = pyqtSignal(list)
    sin_I5_data = pyqtSignal(list)

    sin_G0_data = pyqtSignal(int)
    sin_G6_data = pyqtSignal(int)

    # 设置零点信号
    sin_set_zero = pyqtSignal()

    # 回零信号
    sin_return_zero = pyqtSignal()

    # 返回上一界面
    sin_return_last_ui = pyqtSignal()
    # 急停
    sin_quick_stop = pyqtSignal()

    def __init__(self,parent=None):
        super(Climbot5d_joint_control_func,self).__init__(parent)
        self.setupUi(self)
        self.center()
        self.velocity = 10 * 0.01745  # 10 deg
        self.__open_torque = 600 # mN.n


    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    
    def I1_add_position(self,pressed):
        '''
        push button to add I1 displacement.
        :param pressed:
        :return:
        '''

        if pressed:
            self.pushButton_2.setEnabled(False)
            self.sin_I1_data.emit([1,self.velocity])

        else:
            self.sin_I1_data.emit([1,0])
            self.pushButton_2.setEnabled(True)

    def I1_sub_position(self,pressed):
        '''
        push button to sub I1 displacement.
        :param pressed:
        :return:
        '''

        if pressed:
            self.pushButton_1.setEnabled(False)
            self.sin_I1_data.emit([1, -self.velocity])
            pass

        else:
            self.sin_I1_data.emit([1,0])
            self.pushButton_1.setEnabled(True)
        pass

    def T2_add_position(self,pressed):
        if pressed:
            self.pushButton_7.setEnabled(False)
            self.sin_T2_data.emit([2,self.velocity])

        else:
            self.sin_T2_data.emit([2,0])
            self.pushButton_7.setEnabled(True)
        pass

    def T2_sub_position(self,pressed):
        if pressed:
            self.pushButton_3.setEnabled(False)
            self.sin_T2_data.emit([2,-self.velocity])
        else:
            self.sin_T2_data.emit([2,0])
            self.pushButton_3.setEnabled(True)

    def T3_add_position(self,pressed):
        if pressed:
            self.pushButton_8.setEnabled(False)
            self.sin_T3_data.emit([3,self.velocity])
            pass

        else:
            self.sin_T3_data.emit([3,0])
            self.pushButton_8.setEnabled(True)
        pass

    def T3_sub_position(self,pressed):
        if pressed:
            self.pushButton_4.setEnabled(False)
            self.sin_T3_data.emit([3,-self.velocity])
            pass

        else:
            self.sin_T3_data.emit([3,0])
            self.pushButton_4.setEnabled(True)
        pass

    def T4_add_position(self,pressed):
        if pressed:
            self.pushButton_9.setEnabled(False)
            self.sin_T4_data.emit([4,self.velocity])
            pass

        else:
            self.sin_T4_data.emit([4,0])
            self.pushButton_9.setEnabled(True)
        pass

    def T4_sub_position(self,pressed):
        if pressed:
            self.pushButton_5.setEnabled(False)
            self.sin_T4_data.emit([4,-self.velocity])
            pass

        else:
            self.sin_T4_data.emit([4,0])
            self.pushButton_5.setEnabled(True)
        pass

    def I5_add_position(self,pressed):

        if pressed:
            self.pushButton_10.setEnabled(False)
            self.sin_I5_data.emit([5,self.velocity])
            pass

        else:
            self.sin_I5_data.emit([5,0])
            self.pushButton_10.setEnabled(True)

    def I5_sub_position(self,pressed):

        if pressed:
            self.pushButton_6.setEnabled(False)
            self.sin_I5_data.emit([5,-self.velocity])

            pass

        else:
            self.sin_I5_data.emit([5,0])

            self.pushButton_6.setEnabled(True)

    def return_last_ui(self):
        '''
        return last ui and sent real robot current data.
        :return:
        '''
        self.sin_return_last_ui.emit()

    def change_velocity(self):
        '''
        move the slider to change the velocity.
        :param data:
        :return:
        '''
        try:
            velocity = float(self.lineEdit.text())
        except:
            velocity = 10

        if(0 < velocity < 60):
            self.velocity = velocity * 0.01745
        else:
            self.velocity = 10 * 0.01745

    def stop(self):
        self.sin_quick_stop.emit()


    def G6_open(self,pressed):
        if pressed:
            self.pushButton_15.setEnabled(False)  
            self.pushButton_13.setEnabled(False)
            self.sin_G6_data.emit(self.__open_torque)
        else:
            self.sin_G6_data.emit(0)


    def G6_close(self,pressed):
        if pressed:
            self.pushButton_15.setEnabled(True)  
            self.pushButton_13.setEnabled(True)
            self.sin_G6_data.emit(- self.__open_torque)
        else:
            self.sin_G6_data.emit(0)
        pass

    def G0_open(self,pressed):
        if pressed:
            self.pushButton_16.setEnabled(False)  
            self.pushButton_14.setEnabled(False)
            self.sin_G0_data.emit(self.__open_torque)
        else:
            self.sin_G0_data.emit(0)
        pass

    def G0_close(self,pressed):
        if pressed:
            self.pushButton_16.setEnabled(True)  
            self.pushButton_14.setEnabled(True)
            self.sin_G0_data.emit(- self.__open_torque)
        else:
            self.sin_G0_data.emit(0)
 


