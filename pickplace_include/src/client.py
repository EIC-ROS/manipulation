#!/usr/bin/env python3

import rospy
from pickplace_include.srv import pickplace, pickplaceRequest, pickplaceResponse
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

rospy.init_node('pickcaller')

client = rospy.ServiceProxy('pickit', pickplace)
srv = pickplaceRequest()

srv.posercam.header.frame_id = "base_link"
srv.posercam.header.stamp = rospy.Time.now()

roll = 0
pitch = 0
yaw = 1.3
quat = quaternion_from_euler(roll, pitch, yaw)

srv.posercam.pose.orientation.x = quat[0]
srv.posercam.pose.orientation.y = quat[1]
srv.posercam.pose.orientation.z = quat[2]
srv.posercam.pose.orientation.w = quat[3]

srv.check = True
#water bottle
srv.dimension.x = 0.12
srv.dimension.y = 0.06
srv.dimension.z = 0.24

srv.posercam.pose.position.x = 0.62
srv.posercam.pose.position.y = 0
srv.posercam.pose.position.z = 0.627

# #rockstar
# srv.dimension.x = 0.15286
# srv.dimension.y = 0.05286
# srv.dimension.z = 0.133

# srv.posercam.pose.position.x = 0.65
# srv.posercam.pose.position.y = 0
# srv.posercam.pose.position.z = 0.574

#pringles
# srv.dimension.x = 0.0667
# srv.dimension.y = 0.0667
# srv.dimension.z = 0.1

# srv.posercam.pose.position.x = 0.65
# srv.posercam.pose.position.y = 0
# srv.posercam.pose.position.z = 0.557

# srv.dimension.x = 0.04
# srv.dimension.y = 0.1
# srv.dimension.z = 0.23



try:
    rospy.wait_for_service('pickit')
    response = client(srv)
    rospy.loginfo("Pick Success: %s", response.graspsuccess)
except rospy.ServiceException as e:
    rospy.logerr("Failed to call service: %s", str(e))
    exit(1)