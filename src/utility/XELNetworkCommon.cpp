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