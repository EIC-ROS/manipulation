// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dy_custom state
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dy_custom/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dy_custom/SetPosition "{id: 1, position: 360}"
 * $ rosservice call /get_position "id: 1"
 * $ rostopic pub -1 /set_position dy_custom/SetPosition "{id: 2, position: 0}"
 * $ rostopic pub -1 /set_position dy_custom/SetPosition "{id: 2, position: 360}"
 * $ rosservice call /get_position "id: 2
 * $ rostopic pub -1 /set_velocity dy_custom/SetVelocity "{id: 2, velocity: 0}"
 * $ rostopic pub -1 /set_velocity dy_custom/SetVelocity "{id: 2, velocity: 55}"
 * $ rosservice call /get_load "id: 2"
 * $ rosservice call /set2ah "id: 1" Set to Ah
 * $ rosservice call /set2ah "id: -1" Ah to Set
 *
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dy_custom/GetPosition.h"
#include "dy_custom/GetLoad.h"
#include "dy_custom/SetPosition.h"
#include "dy_custom/SetVelocity.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_VELOCITY    104
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_LOAD     126
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.
// Default setting
#define DXL1_ID               1               // DXL1 Up-Down
#define DXL2_ID               2               // DXL2 Left
#define DXL3_ID               3               // DXL3 Right
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
    portHandler, (int8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
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
    portHandler, (int8_t)req.id, ADDR_PRESENT_LOAD, (uint32_t *)&load, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getLoad : [ID:%d] -> [LOAD:%d]", req.id, load);
    res.load= load;
    return true;
  } else {
    ROS_INFO("Failed to get load! Result: %d", dxl_comm_result);
    return false;
  }
}

void setPositionCallback(const dy_custom::SetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = ((unsigned int)msg->position )*11.375; // Convert int32 -> uint32 with degree op

  // Write Goal Position (length : 4 bytes)
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, (int8_t)msg->id, ADDR_GOAL_POSITION, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id, msg->position);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}

 //UsE WITH POSITION LAG
void setVelocityCallback(const dy_custom::SetVelocity::ConstPtr & msg)
{
  // Load Value of X series is 4 byte data. For AX & MX(1.0)
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int position;
  int firstposition;
  int laterposition;
  int diffpos;
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, (int8_t)msg->id, ADDR_PRESENT_POSITION, (uint32_t *)&firstposition, &dxl_error);
  position = firstposition;
  ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", msg->id, msg->velocity);
  
  while (abs(position - firstposition) <20000)
  //while ((position - firstposition <20000))
  {
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, (int8_t)msg->id, ADDR_PRESENT_POSITION, (uint32_t *)&laterposition, &dxl_error);
    // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
    int32_t velocity = ((signed int)msg->velocity)*4.63;

    // Write Goal Position (length : 4 bytes)[ERROR] [1686740126.322803717]: 
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, (int8_t)msg->id, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      //ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", msg->id, msg->velocity);
    } else {
      ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
    }
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, (int8_t)msg->id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
    //ROS_INFO("[Position: %d]", position);
    //ROS_INFO("[Diff Position: %d]", position - laterposition-abs(msg->velocity));
    if (position - laterposition-abs(msg->velocity)<=-5)
    {
      ROS_ERROR("It's Broke");
    }
  }
  
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, (int8_t)msg->id, ADDR_GOAL_VELOCITY, 0, &dxl_error);
}

bool set2ah(dy_custom::GetLoad::Request & req,
  dy_custom::GetLoad::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int position;
  int firstposition;
  int ccw = 1;

  if ((int)req.id == -1)
  {
    ccw = -1;
  }

  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID , ADDR_PRESENT_POSITION, (uint32_t *)&firstposition, &dxl_error);
  position = firstposition;

  //Up
  while (abs(position - firstposition) <34000 && ccw==1)
  {
    // Write Goal Position
    ROS_INFO("Diff Position ID1 UP: %d",position - firstposition);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID , ADDR_GOAL_VELOCITY, 55*4.63*-ccw, &dxl_error);
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID , ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID , ADDR_GOAL_VELOCITY, 0, &dxl_error);

  //Set Fing 2 UP
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID , ADDR_PRESENT_POSITION, (uint32_t *)&firstposition, &dxl_error);
  position = firstposition;
  while (abs(position - firstposition) <2750 && ccw==1)
  {
    // Write Goal Position 
    ROS_INFO("Diff Position ID2 UP: %d",position - firstposition);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID , ADDR_GOAL_VELOCITY, 8*4.63*ccw, &dxl_error);
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID , ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID , ADDR_GOAL_VELOCITY, 0, &dxl_error);

  //Set Fing 3
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL3_ID , ADDR_PRESENT_POSITION, (uint32_t *)&firstposition, &dxl_error);
  position = firstposition;
  while (abs(position - firstposition) <4750)
  {
    // Write Goal Position 
    ROS_INFO("Diff Position ID2: %d",position - firstposition);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID , ADDR_GOAL_VELOCITY, 8*4.63*ccw, &dxl_error);
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL3_ID , ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID , ADDR_GOAL_VELOCITY, 0, &dxl_error);

  //Set Fing 2 DOWN
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID , ADDR_PRESENT_POSITION, (uint32_t *)&firstposition, &dxl_error);
  position = firstposition;
  while (abs(position - firstposition) <4000 && ccw==-1)
  {
    // Write Goal Position 
    ROS_INFO("Diff Position ID2 Down: %d",position - firstposition);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID , ADDR_GOAL_VELOCITY, 8*4.63*ccw, &dxl_error);
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID , ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID , ADDR_GOAL_VELOCITY, 0, &dxl_error);

  //down
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID , ADDR_PRESENT_POSITION, (uint32_t *)&firstposition, &dxl_error);
  position = firstposition;
  while (abs(position - firstposition) <34000 && ccw==-1)
  {
    ROS_INFO("Diff Position ID1 Down: %d",position - firstposition);
    // Write Goal Position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID , ADDR_GOAL_VELOCITY, 55*4.63*-ccw, &dxl_error);
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID , ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID , ADDR_GOAL_VELOCITY, 0, &dxl_error);

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
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set zero position for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set zero position for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

    dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_ID);
    return -1;
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set zero position for Dynamixel ID %d", DXL3_ID);
    return -1;
  }

  ROS_INFO("Ready to use");
  ros::init(argc, argv, "read_write_node_copy");
  ros::NodeHandle nh;
  ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::ServiceServer Set2Ah = nh.advertiseService("/set2ah", set2ah);
  ros::ServiceServer get_load_srv = nh.advertiseService("/get_load", getPresentLoadCallback);
  ros::Subscriber set_position_sub = nh.subscribe("/set_position", 10, setPositionCallback);
  ros::Subscriber set_velocity_sub = nh.subscribe("/set_velocity", 10, setVelocityCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
