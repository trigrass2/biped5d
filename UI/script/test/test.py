#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy 
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import numpy as np

def main():
    rospy.init_node('a_123')
    offline_data = JointTrajectory()
    trajectory_point = JointTrajectoryPoint()
    joint_value = [[1,2,3,4,5],[2,3,4,5,6],[4,5,6,7,8]]

    print joint_value[0]
    print joint_value[1]
    print joint_value[2]
    row = np.array(joint_value).shape[0]

    offline_data.points = [trajectory_point] * row
    
    
main()
