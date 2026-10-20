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

// Create 4 new instance of the AccelStepper class:
AccelStepper M1_stepper = AccelStepper(motorInterfaceType, M1_STP_PIN, M1_DIR_PIN);
AccelStepper M2_stepper = AccelStepper(motorInterfaceType, M2_STP_PIN, M2_DIR_PIN);
AccelStepper M3_stepper = AccelStepper(motorInterfaceType, M3_STP_PIN, M3_DIR_PIN);
AccelStepper M4_stepper = AccelStepper(motorInterfaceType, M4_STP_PIN, M4_DIR_PIN);

void setup() {
   digitalWrite(M5_EN_PIN, HIGH);    // 使能步进电机 高电平失能
  digitalWrite(M1_4_EN_PIN, LOW);    // 使能步进电机 低电平使能
  M1_stepper.setMaxSpeed(10000);  //最高转速1200rpm 每秒20转 *3200脉冲=64000脉冲/秒 最大不要超过60000
  M1_stepper.setAcceleration(2000);
  M1_stepper.moveTo(-6400);

  M2_stepper.setMaxSpeed(10000);  //最高转速1200rpm 每秒20转 *3200脉冲=64000脉冲/秒 最大不要超过60000
  M2_stepper.setAcceleration(2000);
  M2_stepper.moveTo(6400);

  M3_stepper.setMaxSpeed(10000);  //最高转速1200rpm 每秒20转 *3200脉冲=64000脉冲/秒 最大不要超过60000
  M3_stepper.setAcceleration(2000);
  M3_stepper.moveTo(-6400);

  M4_stepper.setMaxSpeed(10000);  //最高转速1200rpm 每秒20转 *3200脉冲=64000脉冲/秒 最大不要超过60000
  M4_stepper.setAcceleration(2000);
  M4_stepper.moveTo(6400);
}

void loop() {

M1_stepper.run();
M2_stepper.run();
M3_stepper.run();
M4_stepper.run();

}


