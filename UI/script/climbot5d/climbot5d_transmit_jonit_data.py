#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from rospkg import RosPack
rp = RosPack()
sys.path.append(rp.get_path('canopen_communication') + "/modular/")
# sys.path.append(rp.get_path('canopen_communication') + "/file/")

from modular_I100 import I100
from modular_T100 import T100
from  math import fabs,degrees,radians
from PyQt5.QtWidgets import QWidget,QDesktopWidget,QMessageBox
from PyQt5.QtCore import QThread,pyqtSignal
import time
import traceback
import rospy
from std_msgs.msg import Float64MultiArray

class Thread_transmit_joint_data(QThread):

    # 发送通信是否成功
    sin_init_success = pyqtSignal()
    sin_init_error = pyqtSignal()
    # 各窗口反馈数据信号
    sin_joint_control_actual_joint_data = pyqtSignal(list)

    def __init__(self,parent=None):
        
        super(Thread_transmit_joint_data, self).__init__(parent)
        self.eds_file = rp.get_path('canopen_communication') + "/file/Copley.eds"       
        
        # 关节速度
        self.__I1_velocity = 0
        self.__T2_velocity = 0
        self.__T3_velocity = 0
        self.__T4_velocity = 0
        self.__I5_velocity = 0
        # 用于判断关节速度是否为新值
        self.__I1_old_velocity = 0
        self.__T2_old_velocity = 0 
        self.__T3_old_velocity = 0
        self.__T4_old_velocity = 0
        self.__I5_old_velocity = 0
        # 运行模式
        self.__velocity_mode = False
        # 停止方式
        self.stop = False
        self.quick_stop = False
        # 判断采用哪种反馈
        self.__feedback_joint = False
  
    def run(self):
        try:
            self.__start_communication()
            # print "communication success"
            pass
        except Exception as e:
            self.sin_init_error.emit()
            traceback.print_exc()
            # print e
        else:
            self.sin_init_success.emit()

            while not (self.quick_stop or self.stop):

                if self.__velocity_mode:

                    if self.__T2_old_velocity != self.__T2_velocity:
                        self.__T2.sent_velocity(self.__T2_velocity)
                        self.__T2_old_velocity = self.__T2_velocity

                    if self.__T3_old_velocity != self.__T3_velocity:
                        self.__T3.sent_velocity(self.__T3_velocity)
                        self.__T3_old_velocity = self.__T3_velocity

                    if self.__T4_old_velocity != self.__T4_velocity:
                        self.__T4.sent_velocity(self.__T4_velocity)
                        self.__T4_old_velocity = self.__T4_velocity

                    if self.__I1_old_velocity != self.__I1_velocity:
                        self.__I1.sent_velocity(self.__I1_velocity)
                        self.__I1_old_velocity = self.__I1_velocity         

                    if self.__I5_old_velocity != self.__I5_velocity:
                        self.__I5.sent_velocity(self.__I5_velocity)
                        self.__I5_old_velocity = self.__I5_velocity
                    self.__velocity_mode = False                                                    

                try:
                    self.__feedback = [self.__I1.get_position(),self.__T2.get_position(),self.__T3.get_position(),\
                                    self.__T4.get_position(),self.__I5.get_position()]
                    # self.__feedback = [self.__I1.get_position(),0,0,0,0]
                    self.sin_joint_control_actual_joint_data.emit(self.__feedback)
                    time.sleep(0.01) # 10ms

                except Exception as e:
                    traceback.print_exc()
                    # print e

                if self.quick_stop:
                    self.__I1.quick_stop()
                    self.__T2.quick_stop()
                    self.__T3.quick_stop()
                    self.__T4.quick_stop()
                    self.__I5.quick_stop()
                    pass
                    
                if self.stop:
                    self.__I1.stop()
                    self.__T2.stop()
                    self.__T3.stop()
                    self.__T4.stop()
                    self.__I5.stop()
                    pass            

    def if_stop(self):
        self.stop = True

    def if_quick_stop(self):
        self.quick_stop = True

    # 开始通信
    def __start_communication(self):

            self.__I1 = I100(1,self.eds_file)
            self.__T2 = T100(2,self.eds_file)
            self.__T3 = T100(3,self.eds_file)
            self.__T4 = T100(4,self.eds_file)
            self.__I5 = I100(5,self.eds_file)

            self.__I1.start()
            self.__T2.start()
            self.__T3.start()
            self.__T4.start()
            self.__I5.start()

            # set velocity mode.
            self.set_velocity_mode()

    # 关节空间,速度模式发送关节数据
    def joint_sent_data(self,velocity):
        
        if velocity[0] == 1:
            self.__I1_velocity = velocity[1]
        elif velocity[0]  == 2:
            self.__T2_velocity = velocity[1]
        elif velocity[0]  == 3:
            self.__T3_velocity = velocity[1]
        elif velocity[0]  == 4:
            self.__T4_velocity = velocity[1]
        elif velocity[0]  == 5:
            self.__I5_velocity = velocity[1]

        self.__velocity_mode = True

        self.__feedback_joint = True
        pass
   
    # 设置速度模式
    def set_velocity_mode(self):

        self.__I1.opmode_set('PROFILED VELOCITY')
        self.__T2.opmode_set('PROFILED VELOCITY')
        self.__T3.opmode_set('PROFILED VELOCITY')
        self.__T4.opmode_set('PROFILED VELOCITY')
        self.__I5.opmode_set('PROFILED VELOCITY')
        # print 'success'
        pass
    