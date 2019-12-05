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


enum XELNetworkSlaveDefaultItemAddr{
  // ADDR_ITEM_MODEL_NUMBER    = 0, //2byte, Default setting in the Slave class.
  // ADDR_ITEM_FIRMWARE_VER    = 6, //1byte, Default setting in the Slave class.
  // ADDR_ITEM_ID              = 7, //1byte, Default setting in the Slave class.    
  ADDR_ITEM_BAUDRATE           = 8, //1byte
  // ADDR_ITEM_PROTOCOL_VER    = 9, //1byte, Default setting in the Slave class.
};

//static void read_callback_func_default(uint16_t item_addr, uint8_t &dxl_err_code, void* arg);
static void write_callback_func_default(uint16_t item_addr, uint8_t &dxl_err_code, void* arg);


/* XELNetworkSlave */
XELNetworkSlave::XELNetworkSlave(HardwareSerial& dxl_port_serial, const int dir_pin)
: dxl_port_(dxl_port_serial, dir_pin), dxl_(XEL_NETWORK_SLAVE_MODEL_NUM),
  registered_item_cnt_(0), item_port_baud_idx_(1)
{
  memset(topic_item_table_, 0, sizeof(TopicItemInfo_t));
  dxl_.setPort(dxl_port_);

//  dxl_.setReadCallbackFunc(read_callback_func_default, this);
  dxl_.setWriteCallbackFunc(write_callback_func_default, this);

  dxl_.addControlItem(ADDR_ITEM_BAUDRATE, item_port_baud_idx_);
}

void XELNetworkSlave::begin(uint8_t id, uint32_t baud, const float dxl_port_protocol_ver)
{
  uint8_t baud_idx = getBaudrateIndexFromValue(baud);
  if(baud_idx == 0xFF){
    baud = 57600;
    baud_idx = getBaudrateIndexFromValue(baud);
  }
  item_port_baud_idx_ = baud_idx;
  dxl_port_.begin(baud);

  if(dxl_.setPortProtocolVersion(dxl_port_protocol_ver) == false)
    dxl_.setPortProtocolVersion((float)2.0);
    
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
  if(data_size == 0)
    return false;

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


uint8_t XELNetworkSlave::getID() const
{
  return dxl_.getID();
}

uint8_t XELNetworkSlave::getBaudrateIndex() const
{
  return item_port_baud_idx_;
}

uint8_t XELNetworkSlave::getProtocolVersionIndex() const
{
  return dxl_.getPortProtocolVersionIndex();
}

void XELNetworkSlave::setBaudrateIndex(uint8_t baud_idx)
{
  item_port_baud_idx_ = baud_idx;
}

DYNAMIXEL::SerialPortHandler& XELNetworkSlave::getPort() const
{
  return (DYNAMIXEL::SerialPortHandler&)dxl_port_;
}


// static void read_callback_func_default(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
// {
//   dxl_err_code = 0;

//   XELNetworkSlave* p_slave = (XELNetworkSlave*)arg;
// }

static void write_callback_func_default(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  dxl_err_code = 0;

  XELNetworkSlave* p_slave = (XELNetworkSlave*)arg;

  if(item_addr == ADDR_ITEM_BAUDRATE){
    DYNAMIXEL::SerialPortHandler& port = (DYNAMIXEL::SerialPortHandler&)p_slave->getPort();
    uint32_t baudrate = getBaudrateValueFromIndex(p_slave->getBaudrateIndex());
    if(baudrate != 0){
      port.begin(baudrate);
    }else{
      p_slave->setBaudrateIndex(getBaudrateIndexFromValue(port.getBaud()));
      if(p_slave->getProtocolVersionIndex() == 2){
        dxl_err_code = DXL2_0_ERR_DATA_RANGE;
      }else if(p_slave->getProtocolVersionIndex() == 1){
        dxl_err_code |= 1<<DXL1_0_ERR_RANGE_BIT;
      }
    }
  }
}