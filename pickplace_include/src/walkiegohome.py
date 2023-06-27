#!/usr/bin/env python3

import rospy
import moveit_commander

class cr3pick:
    def __init__(self):
        rospy.init_node("walkiegohome", anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.Home_success = False
        self.set_home_walkie(0,0,0,0,0,0)
    
    def set_home_walkie(self,a,b,c,d,e,f):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = a
        joint_goal[1] = b
        joint_goal[2] = c
        joint_goal[3] = d
        joint_goal[4] = e
        joint_goal[5] = f
        self.Home_success = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        rospy.logwarn("Walkie is back to HOME")
        return


if __name__ == "__main__":
    cr3pick = cr3pick()
    rospy.spin()