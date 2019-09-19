/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef XEL_NETWORK_COMMON_H_
#define XEL_NETWORK_COMMON_H_
// This file is shared by the XELNetworkSlave and XELNetworkMaster libraries.
// Therefore, both libraries must have the same contents in this file.

#include <Dynamixel2Arduino.h>

const uint16_t XEL_NETWORK_SLAVE_MODEL_NUM = 0x5040;
const uint16_t SLAVE_TOPIC_ITEM_FIRST_ADDR = 10;
const uint8_t XEL_TOPIC_NAME_SIZE = 16;
const uint8_t SLAVE_TOPIC_ITEM_MAX = 4;

typedef struct TopicItemHeader{
  char name[XEL_TOPIC_NAME_SIZE];
  uint8_t id; // defined in ros2arduino
  uint8_t mode; // write/read (Pub/Sub)
  uint16_t pub_interval_ms;
  uint16_t data_addr;
  uint16_t next_item_addr;
}TopicItemHeader_t;

enum TopicMode{
  SUB,
  PUB
};

// Variable data is not supported. (ex. string, vector, etc)
enum ROS2XELTopicIdNum{
  /* std_msgs */
  STD_MSGS_BOOL_TOPIC_ID,
  STD_MSGS_CHAR_TOPIC_ID,
  STD_MSGS_INT8_TOPIC_ID,
  STD_MSGS_INT16_TOPIC_ID,
  STD_MSGS_INT32_TOPIC_ID,
  STD_MSGS_INT64_TOPIC_ID,
  STD_MSGS_UINT8_TOPIC_ID,
  STD_MSGS_UINT16_TOPIC_ID,
  STD_MSGS_UINT32_TOPIC_ID,
  STD_MSGS_UINT64_TOPIC_ID,
  STD_MSGS_FLOAT32_TOPIC_ID,
  STD_MSGS_FLOAT64_TOPIC_ID,

  /* geometry_msgs */
  GEOMETRY_MSGS_POINT_TOPIC_ID,
  GEOMETRY_MSGS_VECTOR3_TOPIC_ID,
  GEOMETRY_MSGS_QUATERNION_TOPIC_ID,
  GEOMETRY_MSGS_TWIST_TOPIC_ID,
  GEOMETRY_MSGS_POSE_TOPIC_ID
};

typedef struct geometry_msgs_Point{
  double x;
  double y;
  double z;
}geometry_msgs_Point_t;

typedef struct geometry_msgs_Vector3{
  double x;
  double y;
  double z;
}geometry_msgs_Vector3_t;

typedef struct geometry_msgs_Quaternion{
  double x;
  double y;
  double z;
  double w;
}geometry_msgs_Quaternion_t;

typedef struct geometry_msgs_Twist{
  geometry_msgs_Vector3_t linear;
  geometry_msgs_Vector3_t angular;
}geometry_msgs_Twist_t;

typedef struct geometry_msgs_Pose{
  geometry_msgs_Point_t position;
  geometry_msgs_Quaternion_t orientation;
}geometry_msgs_Pose_t;


uint16_t getSizeOfTopicType(uint8_t topic_id);
uint32_t getBaudrateValueFromIndex(uint8_t baud_index);
uint8_t getBaudrateIndexFromValue(uint32_t baudrate);

#endif /* XEL_NETWORK_COMMON_H_ */