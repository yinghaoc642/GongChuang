#include <Arduino.h>
#include <AccelStepper.h>              //https://github.com/waspinator/AccelStepper


// 脉冲控制步进引脚
#define M1_4_EN_PIN  PE13
#define M1_DIR_PIN  PD6
#define M1_STP_PIN  PD4
#define M2_DIR_PIN  PE9
#define M2_STP_PIN  PE11
#define M3_DIR_PIN  PD14
#define M3_STP_PIN  PD15
#define M4_DIR_PIN  PC3_C
#define M4_STP_PIN  PA1
#define M5_EN_PIN  PE10
#define M5_DIR_PIN  PE15
#define M5_STP_PIN  PB11
#define motorInterfaceType 1  //< Stepper Driver, 2 driver pins required

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, M1_STP_PIN, M1_DIR_PIN);


void setup() {
  digitalWrite(M1_4_EN_PIN, LOW);    // 使能步进电机 低电平有效
  stepper.setMaxSpeed(10000);  //最高转速1200rpm 每秒20转 *3200脉冲=64000脉冲/秒 最大不要超过60000
  stepper.setAcceleration(10000);

  delay(100);
  
}

void loop() {
//阻塞式的会影响数据更新性能
  stepper.runToNewPosition(-6400);  //3200一圈  正负号控制方向
  stepper.runToNewPosition(0);
  stepper.runToNewPosition(6400);  //3200一圈  正负号控制方向
  stepper.runToNewPosition(0);
}


