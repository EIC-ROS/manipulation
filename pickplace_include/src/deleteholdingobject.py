#!/usr/bin/env python3

import moveit_commander
import rospy

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

def node():
    rospy.init_node("delete", anonymous = True)
    scene.remove_attached_object("robotiq_85_base_link", "box1")
    rospy.sleep(0.5)
    scene.remove_world_object("box1")
    rospy.signal_shutdown('Task completed')

if __name__ == "__main__":
    node()
    rospy.spin()