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

#include "XELNetworkSlave.h"


/* XELNetworkSlave */
XELNetworkSlave::XELNetworkSlave(HardwareSerial& dxl_port_serial, const int dir_pin)
: dxl_port_(dxl_port_serial, dir_pin), dxl_(XEL_NETWORK_SLAVE_MODEL_NUM),
  registered_item_cnt_(0)
{
  memset(topic_item_table_, 0, sizeof(TopicItemInfo_t));
  dxl_.setPort(dxl_port_);
}

void XELNetworkSlave::begin(uint8_t id, uint32_t baud, const float dxl_port_protocol_ver)
{
  dxl_port_.begin(baud);

  if(dxl_.setPortProtocolVersion(dxl_port_protocol_ver) == false)
    dxl_.setPortProtocolVersion(2.0);
    
  if(dxl_.setID(id) == false)
    dxl_.setID(1);
}

void XELNetworkSlave::run()
{
  dxl_.processPacket();
}


bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, uint8_t topic_id, uint8_t *p_data, uint16_t publish_interval_ms)
{
  static uint16_t available_start_addr = SLAVE_TOPIC_ITEM_FIRST_ADDR;

  if(p_data == nullptr)
    return false;

  if(registered_item_cnt_ >= SLAVE_TOPIC_ITEM_MAX)
    return false;

  if(dxl_.getNumCanBeRegistered() < (1+1)) //header(1) + data(1)
    return false;

  uint16_t data_size = (uint16_t)getSizeOfTopicType(topic_id);
  uint16_t header_size = sizeof(TopicItemHeader_t);
  uint16_t addr_length = header_size + data_size;

  if(dxl_.isEnoughSpaceInControlTable(available_start_addr, addr_length) == false)
    return false;

  strncpy(topic_item_table_[registered_item_cnt_].topic.name, p_name, XEL_TOPIC_NAME_SIZE);
  topic_item_table_[registered_item_cnt_].topic.id = topic_id;
  topic_item_table_[registered_item_cnt_].topic.mode = mode;
  topic_item_table_[registered_item_cnt_].topic.pub_interval_ms = publish_interval_ms;
  topic_item_table_[registered_item_cnt_].topic.data_addr = available_start_addr + header_size;
  if(registered_item_cnt_ > 0)
    topic_item_table_[registered_item_cnt_-1].topic.next_item_addr = available_start_addr;
  topic_item_table_[registered_item_cnt_].topic.next_item_addr = 0;
  topic_item_table_[registered_item_cnt_].p_data = p_data;

  // Add header
  dxl_.addControlItem(available_start_addr, (uint8_t*)&topic_item_table_[registered_item_cnt_].topic, header_size);
  available_start_addr += header_size;
  
  // Add data
  dxl_.addControlItem(available_start_addr, (uint8_t*)&topic_item_table_[registered_item_cnt_].p_data[0], data_size);
  available_start_addr += data_size;

  registered_item_cnt_++;

  return true;
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, bool &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_BOOL_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, uint8_t &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_UINT8_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, uint16_t &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_UINT16_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, uint32_t &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_UINT32_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, uint64_t &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_UINT64_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, int8_t &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_INT8_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, int16_t &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_INT16_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, int32_t &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_INT32_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, int64_t &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_INT64_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, float &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_FLOAT32_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}

bool XELNetworkSlave::addTopicItem(const char* p_name, uint8_t mode, double &data, uint16_t publish_interval_ms)
{
  return addTopicItem(p_name, mode, (uint8_t)STD_MSGS_FLOAT64_TOPIC_ID, (uint8_t*)&data, publish_interval_ms);
}
