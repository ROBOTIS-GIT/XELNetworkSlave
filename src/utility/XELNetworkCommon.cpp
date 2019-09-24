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

#include "XELNetworkCommon.h"
// This file is shared by the XELNetworkSlave and XELNetworkMaster libraries.
// Therefore, both libraries must have the same contents in this file.

uint16_t getSizeOfTopicType(uint8_t topic_id)
{
  uint16_t ret_size = 0;

  switch(topic_id)
  {
    case STD_MSGS_BOOL_TOPIC_ID:
    case STD_MSGS_CHAR_TOPIC_ID:
    case STD_MSGS_INT8_TOPIC_ID:
    case STD_MSGS_UINT8_TOPIC_ID:
      ret_size = 1;
      break;

    case STD_MSGS_INT16_TOPIC_ID:
    case STD_MSGS_UINT16_TOPIC_ID:
      ret_size = 2;
      break;

    case STD_MSGS_INT32_TOPIC_ID:
    case STD_MSGS_UINT32_TOPIC_ID:
    case STD_MSGS_FLOAT32_TOPIC_ID:
      ret_size = 4;
      break;

    case STD_MSGS_INT64_TOPIC_ID:
    case STD_MSGS_UINT64_TOPIC_ID:
    case STD_MSGS_FLOAT64_TOPIC_ID:
      ret_size = 8;
      break;

    case GEOMETRY_MSGS_VECTOR3_TOPIC_ID:
      ret_size = sizeof(geometry_msgs_Point_t);
      break;

    case GEOMETRY_MSGS_POINT_TOPIC_ID:
      ret_size = sizeof(geometry_msgs_Vector3_t);
      break;

    case GEOMETRY_MSGS_QUATERNION_TOPIC_ID:
      ret_size = sizeof(geometry_msgs_Quaternion_t);
      break;

    case GEOMETRY_MSGS_TWIST_TOPIC_ID:
      ret_size = sizeof(geometry_msgs_Twist_t);
      break;

    case GEOMETRY_MSGS_POSE_TOPIC_ID:
      ret_size = sizeof(geometry_msgs_Pose_t);
      break;
  }

  return ret_size;
}

uint32_t getBaudrateValueFromIndex(uint8_t baud_index)
{
  uint32_t baudrate = 0;

  switch(baud_index)
  {
    case 0:
      baudrate = 9600;
      break;
    case 1:
      baudrate = 57600;
      break;
    case 2:
      baudrate = 115200;
      break;
    case 3:
      baudrate = 1000000;
      break;
    case 4:
      baudrate = 2000000;
      break;
    case 5:
      baudrate = 3000000;
      break;
    case 6:
      baudrate = 4000000;
      break;
    case 7:
      baudrate = 4500000;
      break;
  }

  return baudrate;
}

uint8_t getBaudrateIndexFromValue(uint32_t baudrate)
{
  uint8_t baud_idx = 0xFF;

  switch(baudrate)
  {
    case 9600:
      baud_idx = 0;
      break;
    case 57600:
      baud_idx = 1;
      break;
    case 115200:
      baud_idx = 2;
      break;
    case 1000000:
      baud_idx = 3;
      break;
    case 2000000:
      baud_idx = 4;
      break;
    case 3000000:
      baud_idx = 5;
      break;
    case 4000000:
      baud_idx = 6;
      break;
    case 4500000:
      baud_idx = 7;
      break;
  }

  return baud_idx;
}