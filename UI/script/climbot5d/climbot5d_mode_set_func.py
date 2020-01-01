#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
# from os.path import exists
from rospkg import RosPack

import time
import traceback
from math import degrees, radians

import rospy
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QDesktopWidget, QMessageBox, QWidget
from rospkg import rospack

from climbot5d_joint_control_func import Climbot5d_joint_control_func
from climbot5d_mode_set import Ui_climbot5d_Mode_Set
from climbot5d_transmit_jonit_data import Thread_transmit_joint_data

sys.path.append(RosPack().get_path('canopen_communication') + "/modular")
sys.path.append(RosPack().get_path('ui') + "/script/climbot5d")

# from os import getcwd,path
# print getcwd()

class climbot5d_Mode_set_func(QWidget,Ui_climbot5d_Mode_Set):
    # 关闭当前界面信号
    sin_close =pyqtSignal()
    # 控制运行模式信号
    sin_open_velocity_mode = pyqtSignal()
    # 控制数据信号
    sin_joint_data = pyqtSignal(list)

    sin_stop_robot_command = pyqtSignal()
    # 急停信号
    sin_quick_stop = pyqtSignal()
    


    # 回零位使用
    sin_return_zero = pyqtSignal()


    def __init__(self,simulation=False):
        super(climbot5d_Mode_set_func,self).__init__()
        self.setupUi(self)
        self.center()
        self.simulation = simulation

        if not self.simulation:

            # 底层通信类
            self.sent_joint_data = Thread_transmit_joint_data()
            # 初始化成功,错误信号
            self.sent_joint_data.sin_init_error.connect(self.init_error)
            self.sent_joint_data.sin_init_success.connect(self.init_success)
            # 连接反馈
            self.sent_joint_data.sin_joint_control_actual_joint_data.connect(self.joint_control_feedback_data)

            # 各窗口发送数据连接
            self.sin_joint_data.connect(self.sent_joint_data.joint_sent_data)

            # 停止连接
            self.sin_stop_robot_command.connect(self.sent_joint_data.if_stop)
            self.sin_quick_stop.connect(self.sent_joint_data.if_quick_stop)

            self.sent_joint_data.start()
            pass

        self.I1_value = 0
        self.T2_value = 0
        self.T3_value = 0
        self.T4_value = 0
        self.I5_value = 0


    # 关节空间控制窗口
    def joint_control(self):
        """
        open windows of joint control.
        """

        # Initialize the windows of joint control.
        self.windows_joint_control = Climbot5d_joint_control_func()
        self.windows_joint_control.sin_return_last_ui.connect(self.close_joint_control)
        self.windows_joint_control.sin_I1_data.connect(self.sent_I1_data)
        self.windows_joint_control.sin_T2_data.connect(self.sent_T2_data)
        self.windows_joint_control.sin_T3_data.connect(self.sent_T3_data)
        self.windows_joint_control.sin_T4_data.connect(self.sent_T4_data)
        self.windows_joint_control.sin_I5_data.connect(self.sent_I5_data)
        self.windows_joint_control.sin_quick_stop.connect(self.quick_stop)

        self.windows_joint_control.lineEdit_2.setText("{0}".format(round(degrees(self.I1_value),3)))
        self.windows_joint_control.lineEdit_3.setText("{0}".format(round(degrees(self.T2_value),3)))
        self.windows_joint_control.lineEdit_5.setText("{0}".format(round(degrees(self.T3_value),3)))
        self.windows_joint_control.lineEdit_4.setText("{0}".format(round(degrees(self.T4_value),3)))
        self.windows_joint_control.lineEdit_6.setText("{0}".format(round(degrees(self.I5_value),3)))

        self.form.hide()
        self.windows_joint_control.show()
        
        if not self.simulation:
            self.sin_open_velocity_mode.emit()
    
    
    # 关闭关节空间控制窗口
    def close_joint_control(self):
        '''
        close windows of joint control.
        :param data: all joint data.
        :return:
        '''
        self.windows_joint_control.close()
        self.form.show()
        del self.windows_joint_control       

    # 返回上一窗口
    def return_last_ui(self):
        '''
        return last ui.
        :return:
        '''
        if not self.simulation:
            self.sin_stop_robot_command.emit()
        self.sin_close.emit()
        pass
    
    # 窗口设置为屏幕中心
    def center(self):
        """
        put the ui in the center of current window.
        :return:
        """
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
    
    # 关节空间,发送关节数据
    def sent_I1_data(self,data):
        
        if not self.simulation:
            self.sin_joint_data.emit(data)

    def sent_T2_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)

    def sent_T3_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)

    def sent_T4_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)

    def sent_I5_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
    

           
    # 接收关节数据反馈
    def joint_control_feedback_data(self,data):
        try:
            self.I1_value = data[0]
            self.T2_value = data[1]
            self.T3_value = data[2] 
            self.T4_value = data[3]
            self.I5_value = data[4]
            self.windows_joint_control.lineEdit_2.setText("{0}".format(round(degrees(self.I1_value),3)))
            self.windows_joint_control.lineEdit_3.setText("{0}".format(round(degrees(self.T2_value),3)))
            self.windows_joint_control.lineEdit_5.setText("{0}".format(round(degrees(self.T3_value),3)))
            self.windows_joint_control.lineEdit_4.setText("{0}".format(round(degrees(self.T4_value),3)))
            self.windows_joint_control.lineEdit_6.setText("{0}".format(round(degrees(self.I5_value),3)))
        except:
            pass

    # 初始化成功
    def init_success(self):
        self.__box_1 = QMessageBox(QMessageBox.Warning, "提示", "canopen通信初始化成功！！")
        self.__box_1.addButton(self.tr("确定"), QMessageBox.YesRole)
        self.__box_1.exec_()

    # 初始化错误
    def init_error(self):
        self.__box_1 = QMessageBox(QMessageBox.Warning, "错误", "canopen通信初始化错误！！\n无法读取can信息．\n请检查是否连接正确．")
        self.__box_1.addButton(self.tr("确定"), QMessageBox.YesRole)
        self.__box_1.exec_()
        self.return_last_ui()
    
    # 槽函数 急停
    def quick_stop(self):
        self.sin_quick_stop.emit()

    # 回零位 
    def return_zero(self):
        if not self.simulation:
            self.sin_return_zero.emit()
        pass