#!/usr/bin/env python3

import rospy
from dy_custom.easydy import *

rospy.init_node('tester')
ab = easy_gripper_and_camera_function()
while not rospy.is_shutdown():
    command = input().split()
    print("You chose "+str(command))
    if command[0] == "gripper":
        if command[1] == "open":
            ab.open_gripper()
        elif command[1] == "close":
            ab.close_gripper()
        else : pass
    elif command[0] == "camera":
        ab.analog_camera(int(command[1]))
    else : pass
rospy.spin()