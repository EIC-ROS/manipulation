#!/usr/bin/env python3

import moveit_commander
import rospy

class walkie_cr3:
    def __init__(self):
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.Home_success = False
        
    def go_home(self):
        rospy.logwarn("Home")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        self.Home_success = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return
    
    def get_the_bag(self):
        rospy.logwarn("Bag")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 2.058
        joint_goal[1] = -0.837
        joint_goal[2] = 1.779
        joint_goal[3] = 1.256
        joint_goal[4] = 0.768
        joint_goal[5] = 1.587
        self.Home_success = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return
    
    def holding_object(self):
        rospy.logwarn("Holding")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.244
        joint_goal[2] = -2.269
        joint_goal[3] = -0.68
        joint_goal[4] = -1.57
        joint_goal[5] = -0.785
        self.Home_success = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return
    
if __name__ == "__main__":
    k = walkie_cr3()