#!/usr/bin/env python3

import rospy
import math
from dy_custom.srv import SetDegree, SetDegreeRequest, SetDigitalGripper, SetDigitalGripperRequest, SetGripper, SetGripperRequest, GetPosition, GetPositionRequest
from sensor_msgs.msg import JointState

class easy_gripper_and_camera_function:
    def __init__(self):
        self.DigitalGripperClient = rospy.ServiceProxy('/dy_custom/gripper/set_digital', SetDigitalGripper)
        self.AnalogGripperClient = rospy.ServiceProxy('/dy_custom/gripper/set_analog', SetGripper)
        self.SetCamera = rospy.ServiceProxy('/dy_custom/camera/set_camera', SetDegree)
        self.position_client = rospy.ServiceProxy('dy_custom/get_position', GetPosition)
        
        self.camera_state_publisher = rospy.Publisher('/camera_state_publisher', JointState, queue_size=2)
        self.gripper_state_publisher = rospy.Publisher('/gripper_state_publisher', JointState, queue_size=2)
        
        self.analog_cam_srv = SetDegreeRequest()
        self.analog_grip_srv = SetGripperRequest()
        self.digital_grip_srv = SetDigitalGripperRequest()
        self.position_srv = GetPositionRequest()
        
        self.gripper_max_pos = 0
        self.gripper_min_pos = 0
        # ---- Gripper Position Value ----
        # Closed state of the gripper usually has higher position compared to the opened state
        # So self.gripper_max_pos is for closed state,
        # and self.gripper_min_pos is for opened state
        
        self.gripper_sep = (self.gripper_max_pos - self.gripper_min_pos)/2
        # Just the main formula, the real case will be updated by the self.initiate_gripper() function
        
        # ----- IMPORTANT -----
        # Assume that every time this node starts, the gripper state should be closed
        if not '/gripper_state_publisher' in rospy.get_published_topics():
            self.initiate_gripper()
        
    def pub_gripper(self):
        try:
            rospy.wait_for_service('/dy_custom/get_position')
            
            self.position_srv.id = 1
            res = self.position_client(self.position_srv)
            gripper_current_position = ((res.position)/180)*math.pi
            
            gripper_joint_state = JointState()
            if gripper_current_position > self.gripper_sep:
                gripper_joint_state.position = [-0.01]
                # print("Close", gripper_joint_state.position)
            else:
                gripper_joint_state.position = [0]
                # print("Open", gripper_joint_state.position)
            gripper_joint_state.name = ['Slider_5']
            # print("Final gripper joint state's position = ",gripper_joint_state.position)
            self.gripper_state_publisher.publish(gripper_joint_state)
            
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service: %s", str(e))
            exit(1)
        
    def pub_camera(self):
        try:
            rospy.sleep(1.3)
            rospy.wait_for_service('/dy_custom/get_position')
            
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
        # print(self.gripper_sep)
        
        self.pub_gripper()
        self.pub_camera()
        # Caluculate the gripper seperation value
        return
    
    def open_gripper(self):
        try:
            rospy.wait_for_service('/dy_custom/get_position')
            self.position_srv.id = 1
            res = self.position_client(self.position_srv)
            # Get the current position of the gripper (the service returns the degree)
            gripper_current_position = ((res.position)/180)*math.pi
            # Transform from degree to rad

            if (gripper_current_position <= self.gripper_sep):
                # The current gripper state is open
                # and the command is open, it does nothing
                pass
            else:
                # Normal operation, the current gripper state is close, and the command is open
                # or the current gripper state is open, and the command is close
                rospy.wait_for_service('/dy_custom/gripper/set_digital')
                self.digital_grip_srv.id = -1
                response = self.DigitalGripperClient(self.digital_grip_srv)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service: %s", str(e))
            exit(1)
        self.pub_gripper()
    
    def close_gripper(self):
        try:
            rospy.wait_for_service('/dy_custom/get_position')
            self.position_srv.id = 1
            res = self.position_client(self.position_srv)
            # Get the current position of the gripper (the service returns the degree)
            gripper_current_position = ((res.position)/180)*math.pi
            # Transform from degree to rad
            
            # -1 = open
            # 0,1 = close
            
            if (gripper_current_position >= self.gripper_sep):
                # The current gripper state is close
                # and the command is close, it does nothing
                pass
            else:
                # Normal operation, the current gripper state is close, and the command is open
                # or the current gripper state is open, and the command is close
                rospy.wait_for_service('/dy_custom/gripper/set_digital')
                self.digital_grip_srv.id = 0
                response = self.DigitalGripperClient(self.digital_grip_srv)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service: %s", str(e))
            exit(1)
        self.pub_gripper()
        
    def digital_gripper(self, abcd):
        abc = round(int(abcd))
        check = (abc == -1) or (abc == 1) or (abc == 0)
        if check:
            try:
                rospy.wait_for_service('/dy_custom/get_position')
                self.position_srv.id = 1
                res = self.position_client(self.position_srv)
                # Get the current position of the gripper (the service returns the degree)
                gripper_current_position = ((res.position)/180)*math.pi
                # Transform from degree to rad
                
                # -1 = open
                # 0,1 = close
                
                if (gripper_current_position >= self.gripper_sep) and (abc == 0 or abc == 1):
                    # The current gripper state is close
                    # and the command is close, it does nothing
                    pass
                elif (gripper_current_position <= self.gripper_sep) and (abc == -1):
                    # The current gripper state is open
                    # and the command is open, it does nothing
                    pass
                else:
                    # Normal operation, the current gripper state is close, and the command is open
                    # or the current gripper state is open, and the command is close
                    rospy.wait_for_service('/dy_custom/gripper/set_digital')
                    self.digital_grip_srv.id = abc
                    response = self.DigitalGripperClient(self.digital_grip_srv)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call service: %s", str(e))
                exit(1)
        else : rospy.logwarn('The digital gripper only function with the number : -1 , 0, or 1.')
        self.pub_gripper()
        return
                
    def analog_gripper(self, deg, vel):
        if vel == 0:
            rospy.logwarn('Velocity cannot be 0')
            return
        else:
            try:
                rospy.wait_for_service('/dy_custom/gripper/set_analog')
                self.analog_grip_srv.degree = deg
                self.analog_grip_srv.velocity = vel
                response = self.AnalogGripperClient(self.analog_grip_srv)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call service: %s", str(e))
                exit(1)
        self.pub_gripper()
        return
        
    def analog_camera(self, deg):
        if deg < 0 and deg > 360:
            rospy.logwarn('Out of working space')
            return
        else:
            try:
                rospy.wait_for_service('/dy_custom/camera/set_camera')
                self.analog_cam_srv.degree = deg
                response = self.SetCamera(self.analog_cam_srv)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call service: %s", str(e))
                exit(1)
        self.pub_camera()
        return
    
if __name__ == '__main__':
    k = easy_gripper_and_camera_function()
    # rospy.spin()