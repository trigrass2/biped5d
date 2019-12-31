#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-
import sys
sys.path.append("./src/birl_modular_robot/canopen_communication/modular")
from  canopen_control_init import Canopen_control_init
import time
from math import radians,degrees

class T85(Canopen_control_init):
    """
    T85 control base on the canopen
    """
    def __init__(self,id,eds_file):
        """
        add node to the can
        :param id: node id
        :param eds_file: the location of the eds file
        :return:
        """
        super(T85, self).__init__()
        self.id = id
        self.eds_file = eds_file
        self.node = self.network.add_node(self.id, self.eds_file)

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

    def stop(self):
        """
        stop communication
        """
        self.node.nmt.state = 'PRE-OPERATIONAL'
        print('node {1} state 5) = {0}'.format(self.node.nmt.state, self.id))
        self.network.sync.stop()
        self.network.disconnect()

    def set_mode(self, mode):
        """
        :param mode: motor operation mode(1,3,4)
        1: profiled position
        3: Profiled Velocity
        4: Profiled Torque
        """
        self.mode = mode
        if (self.mode == 1 or self.mode == 3 or self.mode == 4):
            try:
                self.node.sdo[0x6060].raw = self.mode
            except:
                print "set mode error"

    def sent_position(self,position,velocity = 0.03):
        """
        In the profile position mode this function sent some control message to motor.
        :param position: motor position(rad)
        :param velocity: default motor velocity(rad/s)
        """
        if(self.mode == 1):

            self.node.sdo[0x6081].raw = 10 * self._T85_msg_to_device(velocity)  
            self.node.sdo[0x6060].raw = 1  # profile position mode
            self.node.sdo[0x6086].raw = 0
            self.node.sdo.download(0x607a, 0x0, self._decTohex_32(self._T85_msg_to_device(position)))
            self.node.sdo[0x6040].bits[4] = 0
            self.node.sdo[0x6040].bits[4] = 1
            self.node.sdo[0x6040].bits[5] = 1
            self.node.sdo[0x6040].bits[6] = 0
            self.node.sdo[0x6040].bits[4] = 0


    def sent_velocity(self,velocity):
        """
        In the profile velocity mode this function sent some control message to motor.
        :param velocity: motor velocity(rad/s)
        :return:
        """
        if self.mode == 3:  # Profiled Velocity
            self.node.sdo[0x6040].bits[0] = 1
            self.node.sdo[0x6040].bits[1] = 1
            self.node.sdo[0x6040].bits[2] = 1
            self.node.sdo[0x6040].bits[3] = 1
            # self.node.sdo[0x6040].bits[7] = 0
            velocity = 10 * self._T85_msg_to_device(velocity)
            self.node.sdo.download(0x60ff, 0x0, self._decTohex_32(velocity))  # velocity


    def sent_torque(self,torque):
        """
        In the profile torque mode this function sent some control message to motor.
        :param torque: motor torque()
        :return:
        """
        if self.mode == 4:  # Profiled Torque
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
        return self._T85_msg_from_device(self.node.sdo[0x6064].phys)  # rad

    def get_velocity(self):
        """
        get the motor actual value
        :return velocity(rad/s)
        """
        return (self._T85_msg_from_device(self.node.sdo[0x606c].phys)) / 10 # rad/s

    def get_torque(self):
        """
        get the motor actual value
        :return torque(rate torque(mN.m) /1000)
        """
        return self.node.sdo[0x6077].phys   # rate torque(mN.m) /1000

    def get_current(self):
        """
        get the motor actual value
        :return current(mA)
        """
        return self.node.sdo[0x221c].phys  # mA

    def get_operation_mode(self):
        return self.node.sdo[0x6061].phys
        pass
        
    def quick_stop(self):
        self.node.sdo[0x6040].bits[2] = 0
        self.node.nmt.state = 'PRE-OPERATIONAL'
        print('node {1} state 5) = {0}'.format(self.node.nmt.state, self.id))
        self.network.sync.stop()
        self.network.disconnect()

    def pause_run(self):
        self.node.sdo[0x6040].bits[8] = 1

    def continue_run(self):

        val = self.node.sdo[0x6040].raw
        val |= 0xF
        val &= 0x7F
        self.node.sdo[0x6040].raw = val

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

    def _decTohex_32(self,number):
        """decimal to hexadecimal"""
        if (number < 0):
            number = hex(number & 0xffffffff)  # negetive decimal number
            number = number[8:10] + number[6:8] + number[4:6] + number[2:4]
            return number.decode('hex')

        else:
            number = hex(number)  # positive decimal number
            # print number
            if (len(number) == 10):
                number = number[8:10] + number[6:8] + number[4:6] + number[2:4]
            elif (len(number) == 9):
                number = number[7:9] + number[5:7] + number[3:5] + '0' + number[2]
            elif (len(number) == 8):
                number = number[6:8] + number[4:6] + number[2:4] + '00'
            elif (len(number) == 7):
                number = number[5:7] + number[3:5] + '0' + number[2] + '00'
            elif (len(number) == 3):
                number = '0' + number[2] + '000000'
            elif (len(number) == 4):
                number = number[2:4] + '000000'
            elif (len(number) == 5):
                number = number[3:5] + '0' + number[2] + '0000'
            elif (len(number) == 6):
                number = number[4:6] + number[2:4] + '0000'

            return number.decode('hex')

    def _T85_msg_to_device(self,position):
        """
        rad -> mdeg * resolution of encoder * reduction ratio / 360 degree
        :return:
        """
        position = degrees(position) * 4096 * 200 / 360
        return int(position)

    def _T85_msg_from_device(self,position):
        """
        actual position [count] -> rad
        :return:
        """
        position = radians(position) * 360 / 200 / 4096
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
        self.node.sdo[0x605d].raw = 1 # halt options
