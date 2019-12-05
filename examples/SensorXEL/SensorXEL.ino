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
uint8_t DXL_ID = 1;

XELNetworkSlave SensorXEL(DEFAULT_DXL_SERIAL, DEFAULT_DXL_PIN);

// note: 'int' is not supported because its size varies by system architecture.
int16_t topic_item_analog = 0;
bool topic_item_gpio_in = 0;
bool topic_item_gpio_out = 0;
int16_t topic_item_pwm = 0;

const int ITEM_ANALOG_PIN = A0; //0
const int ITEM_GPIO_IN_PIN = 1;
const int ITEM_GPIO_OUT_PIN = 2;
const int ITEM_PWM_PIN = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  SensorXEL.begin(DXL_ID, DXL_BAUD_RATE, DXL_PROTOCOL_VER_2_0);

  // The maximum size of a topic name is defined in "XELNetworkCommon.h" as XEL_TOPIC_NAME_SIZE.
  char topic_name[XEL_TOPIC_NAME_SIZE];
  snprintf(topic_name, XEL_TOPIC_NAME_SIZE, "XEL%03d_analog", DXL_ID);
  SensorXEL.addTopicItem(topic_name, TopicMode::PUB, topic_item_analog);
  snprintf(topic_name, XEL_TOPIC_NAME_SIZE, "XEL%03d_gpio_in", DXL_ID);
  SensorXEL.addTopicItem(topic_name, TopicMode::PUB, topic_item_gpio_in);
  snprintf(topic_name, XEL_TOPIC_NAME_SIZE, "XEL%03d_gpio_out", DXL_ID);
  SensorXEL.addTopicItem(topic_name, TopicMode::SUB, topic_item_gpio_out);
  snprintf(topic_name, XEL_TOPIC_NAME_SIZE, "XEL%03d_pwm", DXL_ID);
  SensorXEL.addTopicItem(topic_name, TopicMode::SUB, topic_item_pwm);

  setupItems();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t pre_time;

  // Update or use variables for each topic item.
  updateItems(1000);

  // If there is a packet from the master, process it.
  SensorXEL.run();

  /* LED */
  if(millis() - pre_time >= 1000){
    pre_time = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }  
}


void setupItems()
{
  // For topic_item_analog
  analogReadResolution(12); //0~4095
  // For topic_item_gpio_in
  pinMode(ITEM_GPIO_IN_PIN, INPUT_PULLUP);
  // For topic_item_gpio_out
  pinMode(ITEM_GPIO_OUT_PIN, OUTPUT);
  // For topic_item_pwm
  analogWriteResolution(12); //0~4095
  pinMode(ITEM_PWM_PIN, OUTPUT);
}

void updateItems(uint32_t interval_ms)
{
  static uint32_t pre_time;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    // For topic_item_analog
    topic_item_analog = analogRead(ITEM_ANALOG_PIN);
    // For topic_item_gpio_in
    topic_item_gpio_in = digitalRead(ITEM_GPIO_IN_PIN);
    // For topic_item_gpio_out
    digitalWrite(ITEM_GPIO_OUT_PIN, topic_item_gpio_out);
    // For topic_item_pwm
    analogWrite(ITEM_PWM_PIN, topic_item_pwm);
  }  
}