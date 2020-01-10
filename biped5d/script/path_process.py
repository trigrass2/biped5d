#!/usr/bin/env python
# -*- coding: utf-8 -*-
import string
from math import radians

class Path_process():
    
    def __init__(self):

        self.__joint_pos = []
        self.__max_vel = 10  

    def __readfile(self,path):

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
                
                self.__joint_pos.append(s[0:5])

            line = f.readline()  
        f.close()


    def __get_joint_position(self):

        temp = []
        temp.append(self.__joint_pos[0])

        deg_interval = self.__max_vel
        data_inteval = 5

        i = 0
        while(i != (len(self.__joint_pos))):
            try:
                for j in range(len(self.__joint_pos[i])):

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+1])
                        i = i+1
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+2][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+2])
                        i = i+2
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+3][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+3])
                        i = i+3
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+4][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+4])
                        i = i+4 
                        break

                    if(abs(self.__joint_pos[i][j] - self.__joint_pos[i+5][j]) > deg_interval ):
                        temp.append(self.__joint_pos[i+5]) 
                        i = i+5
                        break
            except IndexError:
                pass
            i+=1
            
        if(temp[-1] != self.__joint_pos[-1]):
            temp.append(self.__joint_pos[-1])
                
        self.__joint_pos = temp

        # for i in range(len(joint_pos)):
        #     print joint_pos[i]

    def __get_joint_velocity(self):

        max_pos = 0
        temp_time = 0
        temp_vel = []
        joint_vel = []


        for i in range(len(self.__joint_pos) - 1):

            for j in range(len(self.__joint_pos[i])):
                temp = abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j])
                if(temp > max_pos):
                    max_pos = temp

            temp_time = max_pos / self.__max_vel
            # print temp_time

            for j in range(len(self.__joint_pos[i])):
                temp = abs(self.__joint_pos[i][j] - self.__joint_pos[i+1][j])
                temp_vel.append(temp / temp_time)
            
            joint_vel.append(temp_vel)

            temp_vel = []
            max_pos = 0

        # print len(joint_vel)
        # for i in range(len(joint_vel)):
        #     print joint_vel[i]

        del self.__joint_pos[0]
        if(len(self.__joint_pos) == len(joint_vel)):
            for i in range(len(self.__joint_pos)):

                for j in range(len(joint_vel[i])):
                    self.__joint_pos[i].append(joint_vel[i][j])

        # convert from deg to rad
        for i in range(len(self.__joint_pos)):
            for j in range(len(self.__joint_pos[i])):
                self.__joint_pos[i][j] = round(radians(self.__joint_pos[i][j]), 4)

        temp = []
        # convert joint data to one list
        for i in range(len(self.__joint_pos)):
            for j in range(len(self.__joint_pos[i])):
                temp.append(self.__joint_pos[i][j])
        self.__joint_pos = temp
        # print joint_pos

    def get_trajectory(self,data):
        self.__readfile(data)
        self.__get_joint_position()
        self.__get_joint_velocity()
        return self.__joint_pos


        