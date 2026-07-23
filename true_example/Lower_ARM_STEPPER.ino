//实现步进上下往复运动
#include <Arduino.h>
#include "TTL_STEPPER.h"

//串口步进配置
#define STEPPER_TX PA2
#define STEPPER_RX PA3
#define ARM_STEPPER_ID 6

//             串口步进
HardwareSerial Serial_Stepper(STEPPER_RX, STEPPER_TX);

/**步进电机***************************************************/
// 创建步进电机的通信协议对象,函数内已经默认串口波特率为115200
TTL_Protocol Stepper_protocol(&Serial_Stepper,115200);
TTL_Stepper armStepper(ARM_STEPPER_ID,&Stepper_protocol);

void setup() {
  delay(1000);//等待步进启动
  Stepper_protocol.init(&Serial_Stepper, 115200);  // 舵机通信协议初始化
  armStepper.init();
}

void loop() {
   armStepper.runToNewPosition(800);//默认3200一圈
   delay(1000);
   armStepper.runToNewPosition(0);//默认3200一圈
   delay(1000);
}
