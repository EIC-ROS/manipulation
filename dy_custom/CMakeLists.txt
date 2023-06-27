cmake_minimum_required(VERSION 3.0.2)
project(dy_custom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    dynamixel_sdk
    roslaunch
)
roslaunch_add_file_check(launch)
################################################################################
# Declare ROS messages, services and actions
################################################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
    SetPosition.msg
    SetVelocity.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    GetPosition.srv
    GetLoad.srv
    SetDegree.srv
    SetGripper.srv
    SetDigitalGripper.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    dynamixel_sdk
)

###########
## Build ##
###########

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(read_write_node src/read_write_node.cpp)
target_link_libraries(read_write_node ${catkin_LIBRARIES})

add_executable(camera src/camera.cpp)
target_link_libraries(camera ${catkin_LIBRARIES})

add_executable(state src/state.cpp)
target_link_libraries(state ${catkin_LIBRARIES})

add_executable(gripper src/gripper.cpp)
target_link_libraries(gripper ${catkin_LIBRARIES})

add_executable(camegrip src/camegrip.cpp)
target_link_libraries(camegrip ${catkin_LIBRARIES})


#############
## Install ##
#############

 install(TARGETS read_write_node 
 		  camera 
 	          state 
 	          gripper
 	          camegrip
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


