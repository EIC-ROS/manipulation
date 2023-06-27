#!/usr/bin/env python3

from sympy import false, true
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
import tf
import copy
from std_srvs.srv import Trigger, TriggerResponse

class cr3pick:
    def __init__(self):
        rospy.init_node("placenode", anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.tf_listener4 = tf.TransformListener()
        self.tf_listener5 = tf.TransformListener()
        self.tf_listener6 = tf.TransformListener()
        self.cartesian1 = False
        self.cartesian2 = False
        self.s = rospy.Service("place", Trigger, self.callback)
        self.set_home_walkie(0,0,0,0,0,0)
        self.setsuccess = False
        self.placesuccess = False
        self.grasping_group = "hand"
        self.touch_links = self.robot.get_link_names(group=self.grasping_group)
    
    def set_home_walkie(self,a,b,c,d,e,f):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = a
        joint_goal[1] = b
        joint_goal[2] = c
        joint_goal[3] = d
        joint_goal[4] = e
        joint_goal[5] = f
        self.setsuccess = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return
    

    def callback(self, data2):
        self.setsuccess = False
        self.tf_listener4.waitForTransform("base_link", "place_approach", rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = self.tf_listener4.lookupTransform("base_link", "place_approach", rospy.Time(0))
        ap = Pose()
        ap.position.x = trans[0]
        ap.position.y = trans[1]
        ap.position.z = trans[2]
        ap.orientation.x = rot[0]
        ap.orientation.y = rot[1]
        ap.orientation.z = rot[2]
        ap.orientation.w = rot[3]
    
        self.move_group.set_pose_target(ap)
        self.placesuccess = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        gp = Pose()
        
        if self.placesuccess == True:
            self.tf_listener5.waitForTransform("base_link", "place_place", rospy.Time(), rospy.Duration(1.0))
            (trans2, rot2) = self.tf_listener5.lookupTransform("base_link", "place_place", rospy.Time(0))
            
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
            self.cartesian1 = self.move_group.execute(plan, wait=True)
            cartesianpath = []
            
            self.scene.remove_attached_object("robotiq_85_base_link", "box1")
            self.set_home_walkie(0,0,0,0,0,0)
            
            if not self.setsuccess:
                self.tf_listener6.waitForTransform("base_link", "place_safe_retreat", rospy.Time(), rospy.Duration(1.0))
                (trans3, rot3) = self.tf_listener6.lookupTransform("base_link", "place_safe_retreat", rospy.Time(0))
                
                gp.position.x = trans3[0]
                gp.position.y = trans3[1]
                gp.position.z = trans3[2]
                gp.orientation.x = rot3[0]
                gp.orientation.y = rot3[1]
                gp.orientation.z = rot3[2]
                gp.orientation.w = rot3[3]
                
                cartesianpath = []
                cartesianpath.append(copy.deepcopy(gp))
                (plan,fraction) = self.move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
                self.cartesian2 = self.move_group.execute(plan, wait=True)
                cartesianpath = []
                
                self.set_home_walkie(0,0,0,0,0,0)
                x = TriggerResponse()
                x.success = True
                x.message = "Successfully Place"
                self.cartesian1 = False
                self.placesuccess = False
                self.scene.remove_world_object("box1")
                return x
            
            if(self.cartesian1):
                x = TriggerResponse()
                x.success = True
                x.message = "Successfully Place"
                self.cartesian1 = False
                self.placesuccess = False
                self.scene.remove_world_object("box1")
                return x
            else:
                x = TriggerResponse()
                x.success = False
                x.message = "Place Cartesian Failed"
                self.cartesian1 = False
                self.placesuccess = False
                return x
 
        else:
            self.placesuccess = False
            self.set_home_walkie(0,0,0,0,0,0)
            x = TriggerResponse()
            x.success = False
            x.message = "Place failed : No valid plan"
            return x
        
if __name__=="__main__":
    cr3pick()
    rospy.spin()