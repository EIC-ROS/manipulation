#!/usr/bin/env python3

import rospy
import math
from dy_custom.srv import SetDegree, SetDegreeRequest, SetDigitalGripper, SetDigitalGripperRequest, SetGripper, SetGripperRequest, GetPosition, GetPositionRequest
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState

class easy_gripper_and_camera_function:
    def __init__(self):
        self.DigitalGripperClient = rospy.ServiceProxy('/dy_custom/gripper/set_digital', SetDigitalGripper)
        self.position_client = rospy.ServiceProxy('dy_custom/get_position', GetPosition)
        
        self.camera_state_publisher = rospy.Publisher('/camera_state_publisher', JointState, queue_size=2)
        self.gripper_state_publisher = rospy.Publisher('/gripper_state_publisher', JointState, queue_size=2)
        
        self.pub_joint_states = rospy.Service('/loop_joint_states', Trigger, self.callback)
        
        self.analog_cam_srv = SetDegreeRequest()
        self.analog_grip_srv = SetGripperRequest()
        self.digital_grip_srv = SetDigitalGripperRequest()
        self.position_srv = GetPositionRequest()
        
        self.gripper_max_pos = 0
        self.gripper_min_pos = 0
        # ---- Gripper Position Value ----
        # Closed state of the gripper usually has higher position compared the opened state
        # So self.gripper_max_pos is for closed state,
        # and self.gripper_min_pos is for opened state
        
        self.gripper_sep = (self.gripper_max_pos - self.gripper_min_pos)/2
        # Just the main formula, the real case will be updated by the self.initiate_gripper() function
        
        # ----- IMPORTANT -----
        # Assume that every time this node starts, the gripper state should be closed
        self.initiate_gripper()
        
    def callback(self, data):
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/dy_custom/get_position')
                self.position_srv.id = 1
                res = self.position_client(self.position_srv)
                gripper_current_position = ((res.position)/180)*math.pi
                print(gripper_current_position)
                
                gripper_joint_state = JointState()
                print(self.gripper_sep)
                if gripper_current_position > self.gripper_sep:
                    gripper_joint_state.position = [-0.01]
                    print("Close", gripper_joint_state.position)
                else:
                    gripper_joint_state.position = [0]
                    print("Open", gripper_joint_state.position)
                gripper_joint_state.name = ['Slider_5']
                print("Final gripper joint state's position = ",gripper_joint_state.position)
                self.gripper_state_publisher.publish(gripper_joint_state)
                
                self.position_srv.id = 2
                res2 = self.position_client(self.position_srv)
                camera_current_position = [-((res2.position)/180)*math.pi]
                
                camera_joint_state = JointState()
                camera_joint_state.position = camera_current_position
                camera_joint_state.name = ['camera_servo_joint']
                self.camera_state_publisher.publish(camera_joint_state)
                
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call service: %s", str(e))
                exit(1)
        x = TriggerResponse()
        x.success = True
        x.message = ''
        return x
        
    def initiate_gripper(self):
        try:
            rospy.wait_for_service('/dy_custom/get_position')
            self.position_srv.id = 1
            res = self.position_client(self.position_srv)
            gripper_current_position = ((res.position)/180)*math.pi
        except rospy.ServiceException as e:
                rospy.logerr("Failed to call service: %s", str(e))
                exit(1)
        self.gripper_max_pos = gripper_current_position
        # Initiate the closed state gripper
        
        try:
            rospy.wait_for_service('/dy_custom/gripper/set_digital')
            self.digital_grip_srv.id = -1
            response = self.DigitalGripperClient(self.digital_grip_srv)
        except rospy.ServiceException as e:
                rospy.logerr("Failed to call service: %s", str(e))
                exit(1)
        # Set the gripper to open state
        try:
            rospy.wait_for_service('/dy_custom/get_position')
            self.position_srv.id = 1
            res = self.position_client(self.position_srv)
            gripper_current_position = ((res.position)/180)*math.pi
        except rospy.ServiceException as e:
                rospy.logerr("Failed to call service: %s", str(e))
                exit(1)
        # Initiate the opened state gripper
        self.gripper_min_pos = gripper_current_position
        self.gripper_sep = ((self.gripper_max_pos - self.gripper_min_pos)/2)+ self.gripper_min_pos
        print(self.gripper_sep)
        # Caluculate the gripper seperation value
        return      
    
if __name__ == '__main__':
    rospy.init_node('pub_joint_states')
    k = easy_gripper_and_camera_function()
    rospy.spin()