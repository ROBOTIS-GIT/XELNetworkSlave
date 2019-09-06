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

#ifndef XEL_NETWORK_SLAVE_H_
#define XEL_NETWORK_SLAVE_H_

#include "utility/XELNetworkCommon.h"


class XELNetworkSlave
{
  public:
    XELNetworkSlave(HardwareSerial& dxl_port_serial, const int dir_pin);
    
    void begin(uint8_t id, uint32_t baud = 57600, const float dxl_port_protocol_ver = 2.0);
    void run();

    bool addTopicItem(const char* p_name, uint8_t mode, uint8_t topic_id, uint8_t *p_data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, bool &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, uint8_t &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, uint16_t &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, uint32_t &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, uint64_t &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, int8_t &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, int16_t &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, int32_t &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, int64_t &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, float &data, uint16_t publish_interval_ms = 1000);
    bool addTopicItem(const char* p_name, uint8_t mode, double &data, uint16_t publish_interval_ms = 1000);

  private:
    DYNAMIXEL::SerialPortHandler dxl_port_;
    DYNAMIXEL::Slave dxl_;
   
    typedef struct TopicItemInfo{
      TopicItemHeader_t topic;
      uint8_t *p_data; // data size = data size of topic type.
    }TopicItemInfo_t;

    TopicItemInfo_t topic_item_table_[SLAVE_TOPIC_ITEM_MAX];
    uint8_t registered_item_cnt_;
};


#endif /* XEL_NETWORK_SLAVE_H_ */