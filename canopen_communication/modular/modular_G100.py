#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-
import sys
sys.path.append("./src/birl_modular_robot/canopen_communication/modular")
from  canopen_control_init import Canopen_control_init
import time
from math import radians

class G100(Canopen_control_init):
    """
    G100 control base on the canopen
    """
    def __init__(self,id,eds_file):
        """
        add node to the can
        :param id: node id
        :param eds_file: the location of the eds file
        :return:
        """
        super(G100, self).__init__()
        self.id = id
        self.eds_file = eds_file
        self.node = self.network.add_node(self.id, self.eds_file)
        self.__mode = 4

    def start(self):
        """
        start communication
        """
        self.node.nmt.state = 'RESET'
        self.node.nmt.wait_for_bootup(10)
        print('node {1} state 1) = {0}'.format(self.node.nmt.state, self.id))

        error_log = self.node.sdo[0x1003]
        for error in error_log.values():
            print("Error {0} was found in the log".format(error.raw))
        print('node {1} state 2) = {0}'.format(self.node.nmt.state, self.id))

        try:
            self._eds_configure()
        except:
            print("sdo configure error!!!")

        print('node {1} state 3) = {0}'.format(self.node.nmt.state, self.id))

        self.node.nmt.state = "OPERATIONAL"
        print('node {1} state 4) = {0}'.format(self.node.nmt.state, self.id))

        self.node.sdo[0x6060].raw = self.__mode

        self.__rate_torque = self.node.sdo[0x6076]

    def stop(self):
        """
        stop communication
        """
        self.node.nmt.state = 'PRE-OPERATIONAL'
        print('node {1} state 5) = {0}'.format(self.node.nmt.state, self.id))
        self.network.sync.stop()
        self.network.disconnect()

    def sent_torque(self,torque):
        """
        In the profile torque mode this function sent some control message to motor.
        :param torque: motor torque()
        :return:
        """
        if self.__mode == 4:  # Profiled Torque
            # enable operation
            self.node.sdo[0x6040].bits[0] = 1
            self.node.sdo[0x6040].bits[1] = 1
            self.node.sdo[0x6040].bits[2] = 1
            self.node.sdo[0x6040].bits[3] = 1
            self.node.sdo.download(0x6071, 0x0,self._decTohex(torque))  # torque

    def get_position(self):
        """
        get the motor actual value
        :return position(rad)
        """
        return self._G100_msg_from_device(self.node.sdo[0x6064].phys)  # rad

    def get_velocity(self):
        """
        get the motor actual value
        :return velocity(rad/s)
        """
        return (self._G100_msg_from_device(self.node.sdo[0x606c].phys)) / 10 # rad/s

    def get_torque(self):
        """
        get the motor actual value
        :return torque(rate torque(mN.m) /1000)
        """
        return self.node.sdo[0x6077].phys  # rate torque(mN.m) /1000

    def get_current(self):
        """
        get the motor actual value
        :return current(mA)
        """
        return self.node.sdo[0x221c].phys  # mA

    def quick_stop(self):
        self.node.sdo[0x6040].bits[2] = 0
        self.node.nmt.state = 'PRE-OPERATIONAL'
        print('node {1} state 5) = {0}'.format(self.node.nmt.state, self.id))
        self.network.sync.stop()
        self.network.disconnect()

    def _decTohex(self,number):
        """decimal to hexadecimal"""
        if (number < 0):
            number = hex(number & 0xffff)  # negetive decimal number
        else:
            number = hex(number)  # positive decimal number
        # print number
        if (len(number) == 3):
            number = '0' + number[2] + '00'
        elif (len(number) == 4):
            number = number[2:4] + '00'
        elif (len(number) == 5):
            number = number[3:5] + '0' + number[2]
        elif (len(number) == 6):
            number = number[4:6] + number[2:4]
        number = number.decode('hex')
        return number

    def _G100_msg_from_device(self, position):
        """
        actual position [count] -> rad
        :return:
        """
        position = radians(position) * 360 / 1600 / 4096
        return position

    def _eds_configure(self):
        """

        :return:
        """
        
        self.node.sdo[0x6040].bits[0] = 1
        self.node.sdo[0x6040].bits[1] = 1
        self.node.sdo[0x6040].bits[2] = 1
        self.node.sdo[0x6040].bits[3] = 1
        self.node.sdo[0x6040].bits[8] = 0

