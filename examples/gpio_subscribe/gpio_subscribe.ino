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

#include <XELNetworkSlave.h>

HardwareSerial& DXL_SERIAL = Serial1;
const int DXL_DIR_PIN = 2;
const float DXL_PROTOCOL_VER_1_0 = 1.0;
const float DXL_PROTOCOL_VER_2_0 = 2.0;
uint32_t DXL_BAUD_RATE = 57600;
uint8_t DXL_ID = 1;

XELNetworkSlave SensorXEL(DXL_SERIAL, DXL_DIR_PIN);

// note: 'int' is not supported because its size varies by system architecture.
uint8_t topic_item_led;

void setup() {
  // put your setup code here, to run once:
  SensorXEL.begin(DXL_ID, DXL_BAUD_RATE, DXL_PROTOCOL_VER_2_0);

  SensorXEL.addTopicItem("XEL_LED", TopicMode::SUB, topic_item_led);
}

void loop() {
  // put your main code here, to run repeatedly:
  SensorXEL.run();

  // Update or use variables for each topic item.
  digitalWrite(LED_BUILTIN, topic_item_led);
}