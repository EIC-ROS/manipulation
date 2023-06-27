/*******************************************************************************
 * This example is written for DYNAMIXEL X series with U2D2.
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dy_custom gripper
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rosservice call /get_load "id: 2
 * 
 * $ rosservice call /set_degree "degree: 0"
 * $ rosservice call /set_degree "degree: 360"
 * 
 * $ rosservice call /set_gripper "degree: 20000
 *                                velocity: 30"
 * 
 * Author: Dechatorn D
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dy_custom/GetPosition.h"
#include "dy_custom/GetLoad.h"
#include "dy_custom/SetPosition.h"
#include "dy_custom/SetGripper.h"
#include "dy_custom/SetDegree.h"
#include "dy_custom/SetDigitalGripper.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_OPERATING        11
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_VELOCITY    104
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_LOAD     126
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.
// Default setting
#define DXL1_ID               1               // DXL1 ID
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

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d] -> [Degree:%f]", req.id, position,position/11.375);
    res.position = position;
    return true;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getPresentLoadCallback(
  dy_custom::GetLoad::Request & req,
  dy_custom::GetLoad::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t load = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_LOAD, (uint32_t *)&load, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getLoad : [ID:%d] -> [LOAD:%d]", req.id, load);
    res.load= load;
    return true;
  } else {
    ROS_INFO("Failed to get load! Result: %d", dxl_comm_result);
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
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d] -> [Degree:%d]", DXL1_ID, position, int(position/11.375));
    res.success = true;
    return true;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    res.success = false;
    return false;
  }
}

bool setDigitalGripper(
  dy_custom::SetDigitalGripper::Request & req,
  dy_custom::SetDigitalGripper::Response & res)
{
  // Load Value of X series is 4 byte data. For AX & MX(1.0)
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int position;
  int firstposition;
  int laterposition;
  int diffpos;
  int count=0;
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&firstposition, &dxl_error);
  position = firstposition;
  bool flag = false;
  int temp;

  int abs_velocity = 40;
  int diffposition = 8000;

  if (req.id == -1)
  {
    abs_velocity *= -1;
  }
  
  while (abs(position - firstposition) < diffposition)
  {
    if (!ros::ok()){
      break;
    }
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&laterposition, &dxl_error);
    // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
    int32_t velocity = (abs_velocity)*4.63;

    // Write Goal Position (length : 4 bytes)[ERROR] [1686740126.322803717]: 
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", DXL1_ID, abs_velocity);
    } else {
      ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
      res.success = false;
      return false;
    }
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
    ROS_INFO("[Position: %d]", position);
    int diff_position = abs(position - laterposition-int(abs_velocity)+int(abs_velocity/10));
    ROS_INFO("[Diff Position: %d]", diff_position);
    
    count++;
    
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
  res.success = true;
  return true;
}

bool setGripper(dy_custom::SetGripper::Request & req,
  dy_custom::SetGripper::Response & res)
{
  // Load Value of X series is 4 byte data. For AX & MX(1.0)
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int position;
  int firstposition;
  int laterposition;
  int diffpos;
  int count=0;
  int diffposition = req.degree;
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&firstposition, &dxl_error);
  position = firstposition;
  bool flag = false;
  int temp;

  
  while (abs(position - firstposition) < diffposition)
  {
    if (!ros::ok()){
      break;
    }
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&laterposition, &dxl_error);
    // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
    int32_t velocity = ((signed int)req.velocity)*4.63;

    // Write Goal Position (length : 4 bytes)[ERROR] [1686740126.322803717]: 
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", DXL1_ID, req.velocity);
    } else {
      ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
      res.success = false;
      return false;
    }
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
    ROS_INFO("[Position: %d]", position);
    int diff_position = abs(position - laterposition-int(req.velocity)+int(req.velocity/10));
    ROS_INFO("[Diff Position: %d]", diff_position);
    
    count++;


    if (diff_position<=5  && count>20)
    {
      ROS_ERROR("It's tight");
      
      if (!flag)
      {
        flag = true;
        temp = count;
      }
      if (flag && count - temp <= 3)
      {
        ROS_ERROR("It's Force");
        res.success = true;
        break;
      }
      else
      {
        flag = false;
      }
    }
    
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
  res.success = true;
  return true;
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
    ROS_ERROR("Failed to enable torque for Dynamixel ID: Gripper %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_OPERATING , 1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("Set Operating Control: Gripper");
  } else {
    ROS_ERROR("Failed to set operating Control: Gripper");
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set zero position for Dynamixel ID %d", DXL1_ID);
    return -1;
  }


  ROS_INFO("Ready to use");
  ros::init(argc, argv, "gripper_dynamixel");
  ros::NodeHandle nh;
  ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::ServiceServer get_load_srv = nh.advertiseService("/get_load", getPresentLoadCallback);
  ros::ServiceServer set_gripper_srv = nh.advertiseService("/set_gripper", setGripper);
  ros::ServiceServer set_digital_gripper_srv = nh.advertiseService("/set_digital_gripper", setDigitalGripper);
  ros::ServiceServer set_degree_srv = nh.advertiseService("/set_degree", setDegree);
  ros::spin();

  portHandler->closePort();
  return 0;
}
