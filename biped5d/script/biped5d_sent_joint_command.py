#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from rospkg import RosPack
sys.path.append(RosPack().get_path('birl_module_robot') + "/script/")

import rospy
import signal
import time
from birl_module_robot.msg import joint_point
from path_process import Path_process

def main():

    process = Path_process()
    trajectory = process.get_trajectory(RosPack().get_path('birl_module_robot') + "/data/demo_data.txt")

    rospy.init_node("biped5d_sent_joint_command", anonymous=True)
    publisher = rospy.Publisher("/low_level/biped5d_joint_command",joint_point,queue_size = 10)
    joint_command = joint_point()
    joint_command.data.data = trajectory

    while(not rospy.is_shutdown()):
        publisher.publish(joint_command)
        time.sleep(3)
    rospy.spin()


if __name__ == '__main__':
    main()
