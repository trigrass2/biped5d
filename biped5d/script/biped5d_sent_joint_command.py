#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rospkg import RosPack
import string
from math import radians
import rospy
from std_msgs.msg import Float64MultiArray
import signal
import time

max_vel = 10 #模块最大速度是60度每秒，一般运行速度为30度每秒
joint_pos = [] #存储[P1 24.636 ,61.687 ,-10.713 ,180.000 ,50.974]



def readfile(path):

    global joint_pos

    f = open(path)   # 返回一个文件对象  
    line = f.readline()

    while line: 
        if line.startswith('P'):    #判断字符串首字母是否为P

            s = line.split("=")

            s = s[1].split(',')    #字符串按照空格分开，split()默认以空格分开 

            s[0] = string.atof(s[0])    #字符串转化为浮点数

            s[1] = string.atof(s[1])    #字符串转化为浮点数

            s[2] = string.atof(s[2])    #字符串转浮点数

            s[3] = string.atof(s[3])

            s[4] = string.atof(s[4])
            
            joint_pos.append(s[0:5])

        line = f.readline()  
    f.close()

def get_joint_position():

    global joint_pos
    temp = []
    temp.append(joint_pos[0])

    deg_interval = 5
    data_inteval = 5

    i = 0
    while(i != (len(joint_pos))):
        try:
            for j in range(len(joint_pos[i])):

                if(abs(joint_pos[i][j] - joint_pos[i+1][j]) > deg_interval ):
                    temp.append(joint_pos[i+1])
                    i = i+1
                    break

                if(abs(joint_pos[i][j] - joint_pos[i+2][j]) > deg_interval ):
                    temp.append(joint_pos[i+2])
                    i = i+2
                    break

                if(abs(joint_pos[i][j] - joint_pos[i+3][j]) > deg_interval ):
                    temp.append(joint_pos[i+3])
                    i = i+3
                    break

                if(abs(joint_pos[i][j] - joint_pos[i+4][j]) > deg_interval ):
                    temp.append(joint_pos[i+4])
                    i = i+4 
                    break

                if(abs(joint_pos[i][j] - joint_pos[i+5][j]) > deg_interval ):
                    temp.append(joint_pos[i+5]) 
                    i = i+5
                    break
        except IndexError:
            pass
        i+=1
        
    if(temp[-1] != joint_pos[-1]):
        temp.append(joint_pos[-1])
            
    joint_pos = temp

    # for i in range(len(joint_pos)):
    #     print joint_pos[i]

def get_joint_velocity():

    global joint_pos
    global max_vel
    max_pos = 0
    temp_time = 0
    temp_vel = []
    joint_vel = []


    for i in range(len(joint_pos) - 1):

        for j in range(len(joint_pos[i])):
            temp = abs(joint_pos[i][j] - joint_pos[i+1][j])
            if(temp > max_pos):
                max_pos = temp

        temp_time = max_pos / max_vel
        # print temp_time

        for j in range(len(joint_pos[i])):
            temp = abs(joint_pos[i][j] - joint_pos[i+1][j])
            temp_vel.append(temp / temp_time)
        
        joint_vel.append(temp_vel)

        temp_vel = []
        max_pos = 0

    # print len(joint_vel)
    # for i in range(len(joint_vel)):
    #     print joint_vel[i]

    del joint_pos[0]
    if(len(joint_pos) == len(joint_vel)):
        for i in range(len(joint_pos)):

            for j in range(len(joint_vel[i])):
                joint_pos[i].append(joint_vel[i][j])

    # convert from deg to rad
    for i in range(len(joint_pos)):
        for j in range(len(joint_pos[i])):
            joint_pos[i][j] = round(radians(joint_pos[i][j]), 4)

    # print len(joint_pos)
    # for i in range(len(joint_pos)):
    #     print joint_pos[i]

def main():

    readfile(RosPack().get_path('birl_module_robot') + "/data/demo_data.txt")
    get_joint_position()
    get_joint_velocity()

    rospy.init_node("biped5d_sent_joint_command", anonymous=True)
    publisher = rospy.Publisher("/low_level/biped5d_joint_command",Float64MultiArray,queue_size = 10)
    joint_command = Float64MultiArray()
    joint_command.data = []

    for i in range(len(joint_pos)):
        joint_command.data.append(joint_pos[i])

    publisher.publish(joint_command)


    signal.pause()


if __name__ == '__main__':
    main()

