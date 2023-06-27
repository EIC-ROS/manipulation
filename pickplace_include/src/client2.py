#!/usr/bin/env python3

import rospy
from pickplace_include.srv import pickplace, pickplaceRequest, pickplaceResponse
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

rospy.init_node('placecaller')

client = rospy.ServiceProxy('placeit', pickplace)
srv = pickplaceRequest()

srv.posercam.header.frame_id = "base_link"
srv.posercam.header.stamp = rospy.Time.now()

# #water bottle
srv.posercam.pose.position.x = 0.8
srv.posercam.pose.position.y = 0
srv.posercam.pose.position.z = 0.507

# #rockstar
# srv.posercam.pose.position.x = 0.6
# srv.posercam.pose.position.y = 0
# srv.posercam.pose.position.z = 0.8015

#pringles
# srv.posercam.pose.position.x = 0.6
# srv.posercam.pose.position.y = 0
# srv.posercam.pose.position.z = 0.785

srv.check = False

roll = 0
pitch = 0
yaw = 0
quat = quaternion_from_euler(roll, pitch, yaw)

srv.posercam.pose.orientation.x = quat[0]
srv.posercam.pose.orientation.y = quat[1]
srv.posercam.pose.orientation.z = quat[2]
srv.posercam.pose.orientation.w = quat[3]

srv.dimension.x = 0
srv.dimension.y = 0
srv.dimension.z = 0

try:
    rospy.wait_for_service('placeit')
    response = client(srv)
    rospy.loginfo("Pick Success: %s", response.graspsuccess)
except rospy.ServiceException as e:
    rospy.logerr("Failed to call service: %s", str(e))
    exit(1)