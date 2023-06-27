#!/usr/bin/env python3

from sympy import false, true
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
import tf
import copy
import math
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from pickplace_include.srv import tfcount, tfcountRequest, tfcountResponse
from moveit_msgs.msg import Constraints, OrientationConstraint

class cr3pick:
    def __init__(self):
        rospy.init_node("picknode", anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.tf_listener1 = tf.TransformListener()
        self.tf_listener2 = tf.TransformListener()
        self.tf_listener3 = tf.TransformListener()
        self.cartesian1 = False
        self.cartesian2 = False
        self.s = rospy.Service("pick", Trigger, self.callback)
        self.success = False
        
        self.subframe1 = 'obj_approach_pose'
        self.subframe2 = 'obj_grasp_pose'
        self.subframe3 = 'obj_lift_pose'
        self.count = 1
        
        self.secondchance = False
        
        self.Approach_success = False
        self.Pick_success = False
        self.Home_success = False
        self.Retreat_success = False
        
        self.grasping_group = "hand"
        self.touch_links = self.robot.get_link_names(group=self.grasping_group)
        self.client = rospy.ServiceProxy('setplace', tfcount)
        self.srv = tfcountRequest()
    
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
        if self.Home_success:
            rospy.logwarn("Home")
            rospy.sleep(0.5)
        if f == 2.355:
            self.secondchance = True
        return

    def callback(self, data):
        # constraints = Constraints()
        # constraints.orientation_constraints = []
        # self.move_group.set_path_constraints(constraints)
        
        self.set_home_walkie(0,0,0,0,0,0)
        self.Home_success = False
        while (not self.Approach_success):
            ap = Pose()

            self.tf_listener1.waitForTransform("base_link", self.subframe1, rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener1.lookupTransform("base_link", self.subframe1, rospy.Time(0))
            
            ap.position.x = trans[0]
            ap.position.y = trans[1]
            ap.position.z = trans[2]
            ap.orientation.x = rot[0]
            ap.orientation.y = rot[1]
            ap.orientation.z = rot[2]
            ap.orientation.w = rot[3]
        
            self.move_group.set_pose_target(ap)
            self.Approach_success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            if self.Approach_success:
                rospy.logwarn("Approach")
                rospy.sleep(0.5)
            
            rospy.loginfo("Attempt "+str(self.count)+": "+str(self.success))
            
            if self.Approach_success:
                break
            else:
                self.count += 1
                if self.count == 5 and self.secondchance == False:
                    rospy.loginfo("Try Again")
                    self.success = False
                    self.subframe1 = 'obj_approach_pose'
                    self.subframe2 = 'obj_grasp_pose'
                    self.subframe3 = 'obj_lift_pose'
                    self.count = 1
                    self.set_home_walkie(0,0,0,0,0,2.355)
                    
                elif self.count == 5 and self.secondchance == True:
                    rospy.loginfo("Failed 4 Attemps")
                    self.success = False
                    self.subframe1 = 'obj_approach_pose'
                    self.subframe2 = 'obj_grasp_pose'
                    self.subframe3 = 'obj_lift_pose'
                    self.count = 1
                    break
                # if self.count == 5:
                #     rospy.loginfo("Failed 4 Attemps")
                #     self.success = False
                #     self.subframe1 = 'obj_approach_pose'
                #     self.subframe2 = 'obj_grasp_pose'
                #     self.subframe3 = 'obj_lift_pose'
                #     self.count = 1
                #     break

                else:
                    self.subframe1+= 'a'
                    self.subframe2+= 'a'
                    self.subframe3+= 'a'
        
        gp = Pose()
        rp = Pose()
        
        if self.Approach_success:
            self.tf_listener2.waitForTransform("base_link", self.subframe2, rospy.Time(), rospy.Duration(1.0))
            (trans2, rot2) = self.tf_listener2.lookupTransform("base_link", self.subframe2, rospy.Time(0))
            
            gp.position.x = trans2[0]
            gp.position.y = trans2[1]
            gp.position.z = trans2[2]
            
            gp.orientation.x = rot2[0]
            gp.orientation.y = rot2[1]
            gp.orientation.z = rot2[2]
            gp.orientation.w = rot2[3]
            
            cartesianpath = []
            cartesianpath.append(copy.deepcopy(gp))
            (plan,fraction) = self.move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
            self.Pick_success = self.move_group.execute(plan, wait=True)
            
            if self.Pick_success:
                rospy.logwarn("Grasped")
                rospy.sleep(0.5)
                
            self.scene.attach_box("robotiq_85_base_link", "box1", touch_links=self.touch_links)
            # self.move_group.set_planner_id("TRRT")
            
            # constraints = Constraints()
            # orientation_constraint = OrientationConstraint()
            # orientation_constraint.header.frame_id = "base_link"
            # orientation_constraint.link_name = "Link7"
            # orientation_constraint.orientation.x = 0
            # orientation_constraint.orientation.y = 0
            # orientation_constraint.orientation.z = 0
            # orientation_constraint.orientation.w = 1

            # orientation_constraint.absolute_x_axis_tolerance = 0.1
            # orientation_constraint.absolute_y_axis_tolerance = 0.1
            # orientation_constraint.absolute_z_axis_tolerance = 2 * math.pi

            # orientation_constraint.weight = 100000
            # constraints.orientation_constraints.append(orientation_constraint)
            # self.move_group.set_path_constraints(constraints)
            
            try:
                rospy.wait_for_service('setplace')
                self.srv.frame = self.subframe2
                response = self.client(self.srv)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call service: %s", str(e))
                exit(1)
            rospy.sleep(0.5)
            
            self.tf_listener3.waitForTransform("base_link", self.subframe3, rospy.Time(), rospy.Duration(1.0))
            (trans3, rot3) = self.tf_listener3.lookupTransform("base_link", self.subframe3, rospy.Time(0))

            rp.position.x = trans3[0]
            rp.position.y = trans3[1]
            rp.position.z = trans3[2]
            
            rp.orientation.x = rot3[0]
            rp.orientation.y = rot3[1]
            rp.orientation.z = rot3[2]
            rp.orientation.w = rot3[3]
            
            cartesianpath[0] = copy.deepcopy(rp)
            (plan,fraction) = self.move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
            self.Retreat_success = (self.move_group.execute(plan, wait=True))
            
            if self.Retreat_success:
                rospy.logwarn("Retreat")
                rospy.sleep(0.5)
            
            # hp = Pose()
            # hp.position.x = 0.154
            # hp.position.y = -0.229
            # hp.position.z = 1.256
            
            # hp.orientation.x = -0.001
            # hp.orientation.y = -0.001
            # hp.orientation.z = -0.707
            # hp.orientation.w = 0.707
            
            # cartesianpath[0] = copy.deepcopy(hp)
            # (plan,fraction) = self.move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
            # self.Retreat_success = (self.move_group.execute(plan, wait=True))
            # self.set_home_walkie(0,0.55,-2.5,2.3,3.14,1)
            self.set_home_walkie(0,0,0,0,0,0)
            # self.set_home_walkie(0,0.55,-2.5,2.3,3.14,1)
            # self.set_home_walkie(3.14, 1.5708, -1.5708, 0, 2.4, 0)
            cartesianpath = []
            
        x = TriggerResponse()
        if self.Pick_success and self.Home_success:
            x.success = True
            x.message = "Pick success"
        elif self.Pick_success and not self.Home_success:
            # self.set_home_walkie(0,0.55,-2.5,2.3,3.14,1)
            self.set_home_walkie(0,0,0,0,0,0)
            # self.set_home_walkie(0,0.55,-2.5,2.3,3.14,1)
            x.success = False
            x.message = "Pick success, Home fail"
        else:
            # self.set_home_walkie(0,0.55,-2.5,2.3,3.14,1)
            self.set_home_walkie(0,0,0,0,0,0)
            # self.set_home_walkie(0,0.55,-2.5,2.3,3.14,1)
            x.success = False
            x.message = "Pick Fail"
            
        self.Approach_success = False
        self.Pick_success = False
        self.Retreat_success = False
        self.Home_success = False
        self.secondchance = False
        self.count = 1
        
        self.subframe1 = 'obj_approach_pose'
        self.subframe2 = 'obj_grasp_pose'
        self.subframe3 = 'obj_lift_pose'
        
        return x

if __name__ == "__main__":
    cr3pick = cr3pick()
    rospy.spin()