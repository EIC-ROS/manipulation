#!/usr/bin/env python3

import rospy
from pickplace_include.joint_command import *


if __name__ == "__main__":
    rospy.init_node('testeradfjil')
    a = walkie_cr3()
    while not rospy.is_shutdown():
        b = str(input())
        print("Your input = ", b)
        if b == "home":
            rospy.logwarn("home")
            a.go_home()
        elif b == "bag":
            rospy.logwarn("bag")
            a.get_the_bag()
        elif b == "hold":
            rospy.logwarn("hold")
            a.holding_object()
        else:
            pass
    rospy.spin()