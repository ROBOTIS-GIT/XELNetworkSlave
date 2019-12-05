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

#define DEBUG_SERIAL Serial //Internally, Serial is defined as Serial1.
const float DXL_PROTOCOL_VER_1_0 = 1.0;
const float DXL_PROTOCOL_VER_2_0 = 2.0;
uint32_t DXL_BAUD_RATE = 1000000;
uint8_t DXL_ID = 0;

XELNetworkSlave PowerXEL(DEFAULT_DXL_SERIAL, DEFAULT_DXL_PIN);

// note: 'int' is not supported because its size varies by system architecture.
int32_t topic_item_voltage;
int32_t topic_item_current;

const int ITEM_VOLTAGE_PIN = A0;
const int ITEM_CURRENT_PIN = A1;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  PowerXEL.begin(DXL_ID, DXL_BAUD_RATE, DXL_PROTOCOL_VER_2_0);

  // The maximum size of a topic name is defined in "XELNetworkCommon.h" as XEL_TOPIC_NAME_SIZE.
  PowerXEL.addTopicItem("Power_Voltage", TopicMode::PUB, topic_item_voltage);
  PowerXEL.addTopicItem("Power_Current", TopicMode::PUB, topic_item_current);

  setupItems();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t pre_time;

  // Update or use variables for each topic item.
  updateItems(1000);

  // If there is a packet from the master, process it.
  PowerXEL.run();

  /* LED */
  if(millis() - pre_time >= 1000){
    pre_time = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }  
}


void setupItems()
{
  // For topic_item_voltage, topic_item_current
  analogReadResolution(12); //0~4095

  // To power the DYNAMIXEL port.
  pinMode(DEFAULT_DXL_PWR_EN_PIN, OUTPUT);
  digitalWrite(DEFAULT_DXL_PWR_EN_PIN, HIGH);
}

void updateItems(uint32_t interval_ms)
{
  static uint32_t pre_time;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    // For topic_item_voltage
    topic_item_voltage = getVoltage();
    // For topic_item_current
    topic_item_current = getCurrent();
  }  
}


int32_t getVoltage()
{
  return (int32_t)(analogRead(ITEM_VOLTAGE_PIN)*3300/4096*85/10);
}

//https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Exponential%20Moving%20Average/C++Implementation.html
template <uint8_t K, class uint_t = uint16_t>
class EMA {
  public:
    uint_t operator()(uint_t x) {
        z += x;
        uint_t y = (z + (1 << (K - 1))) >> K;
        z -= y;
        return y;
    }

    static_assert(
        uint_t(0) < uint_t(-1),  // Check that `uint_t` is an unsigned type
        "Error: the uint_t type should be an unsigned integer, otherwise, "
        "the division using bit shifts is invalid.");

  private:
    uint_t z = 0;
};

#define ZERO_CURRENT_mV 330 //410
int32_t getCurrent()
{
  static EMA<2> filter;
  int32_t average_analog = 0;
  float voltage, current;

  average_analog = filter(analogRead(ITEM_CURRENT_PIN));
//  for(int i=0; i<10; i++){
//    average_analog += (int32_t)analogRead(A1);
//  }
//  average_analog /= 10;

  voltage = (float)average_analog*0.8;
  current = (voltage - ZERO_CURRENT_mV)*1000/264;

  return (int32_t)current;
}