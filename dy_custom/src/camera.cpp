/*******************************************************************************
 * This example is written for DYNAMIXEL X series with U2D2
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dy_custom gripper
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rosservice call /get_position "id: 2"
 * 
 * $ rosservice call /set_degree "degree: 0"
 * $ rosservice call /set_degree "degree: 360"
 *
 * Author: DDechatorn
*******************************************************************************/
#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dy_custom/GetPosition.h"
#include "dy_custom/SetPosition.h"
#include "dy_custom/SetDegree.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_OPERATING        11
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.
// Default setting
#define DXL1_ID               2              // DXL1 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;

bool getPresentPositionCallback(
  dy_custom::GetPosition::Request & req,
  dy_custom::GetPosition::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  int32_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d] -> [Degree:%d]", req.id, position,int(position/11.375));
    res.position = position/11.375;
    return true;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}

bool setDegree(
  dy_custom::SetDegree::Request & req,
  dy_custom::SetDegree::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int position =0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, DXL1_ID, ADDR_GOAL_POSITION, (uint32_t)req.degree*11.375, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d] -> [Degree:%d]", DXL1_ID, position,int(position/11.375));
    res.success = true;
    return true;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    res.success = false;
    return false;
  }
}


int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: Camera %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_OPERATING , (int)3, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("Set Operating Control: Camera");
  } else {
    ROS_ERROR("Failed to set operating Control: Camera");
  }

  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION , 1023, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("Set Zero Camera");
  } else {
    ROS_ERROR("Failed to set Zero Camera Please check Operation Mode");
  }

	
  ros::init(argc, argv, "cameara_dynamixel");
  ros::NodeHandle nh;
  ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::ServiceServer set_degree_srv = nh.advertiseService("/set_degree", setDegree);
  ros::spin();

  portHandler->closePort();
  return 0;
}
