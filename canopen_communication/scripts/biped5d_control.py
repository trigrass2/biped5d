#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from rospkg import RosPack
rp = RosPack()
sys.path.append(rp.get_path('canopen_communication') + "/modular/")

from modular_T100 import T100
from modular_I100 import I100
from modular_G100 import G100

from math import fabs, degrees, radians
from std_msgs.msg import Float64MultiArray
import threading

class Biped5d_control():

    mutex = threading.Lock()

    def __init__(self):

        self.thread_command = threading.thread(target = Biped5d_control.task_command,arg=(None,))
        self.thread_feedback = threading.thread(target = Biped5d_control.task_feedback,arg=(None,))

        self.task_command.start()
        self.task_feedback.start()

        self.task_command.join()
        self.task_feedback.join()

    
    @staticmethod
    def task_command():
        Biped5d_control.mutex.acquire()
        eds_file = rp.get_path('canopen_communication') + "/file/Copley.eds"
        rospy.loginfo("task command start")

        Biped5d_control.mutex.release()

        pass
    
    @staticmethod
    def task_feedback():
        Biped5d_control.mutex.acquire()
        rospy.loginfo("task feedback start")

        Biped5d_control.mutex.release()
       
if __name__ == "__init__":
    a = Biped5d_control()