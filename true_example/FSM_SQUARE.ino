#include <Arduino.h>
#include <AccelStepper.h>

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
#define motorInterfaceType 1  // Stepper Driver, 2 driver pins required

// 所需的脉冲数
const long PULSES_PER_METER = 10000;

// 4个电机极性参数，1 表示正常， -1 表示反转
int motorPolarity[4] = {-1, 1, -1, 1}; 

// 创建 4 个 AccelStepper 类的实例
AccelStepper M1_stepper = AccelStepper(motorInterfaceType, M1_STP_PIN, M1_DIR_PIN);
AccelStepper M2_stepper = AccelStepper(motorInterfaceType, M2_STP_PIN, M2_DIR_PIN);
AccelStepper M3_stepper = AccelStepper(motorInterfaceType, M3_STP_PIN, M3_DIR_PIN);
AccelStepper M4_stepper = AccelStepper(motorInterfaceType, M4_STP_PIN, M4_DIR_PIN);

// 有限状态机状态枚举
enum State {
  STATE_FORWARD,
  STATE_LEFT,
  STATE_BACKWARD,
  STATE_RIGHT,
  STATE_STOP
};
bool isFirst[4]= {true, true, true, true};

State currentState = STATE_FORWARD;

void setup() {
  digitalWrite(M5_EN_PIN, HIGH);    // 使能步进电机 高电平失能
  digitalWrite(M1_4_EN_PIN, LOW);    // 使能步进电机 低电平使能
  
  M1_stepper.setMaxSpeed(5000);  // 最高转速 1200rpm 每秒 20 转 *3200 脉冲=64000 脉冲/秒 最大不要超过 60000
  M1_stepper.setAcceleration(2000);
  
  M2_stepper.setMaxSpeed(5000);
  M2_stepper.setAcceleration(2000);
  
  M3_stepper.setMaxSpeed(5000);
  M3_stepper.setAcceleration(2000);
  
  M4_stepper.setMaxSpeed(5000);
  M4_stepper.setAcceleration(2000);
}

void loop() {
  switch (currentState) {
    case STATE_FORWARD:
    if(isFirst[0]){
      M1_stepper.move(PULSES_PER_METER * motorPolarity[0]);
      M2_stepper.move(PULSES_PER_METER * motorPolarity[1]);
      M3_stepper.move(PULSES_PER_METER * motorPolarity[2]);
      M4_stepper.move(PULSES_PER_METER * motorPolarity[3]);
      isFirst[0]=false;
    }

      if (M1_stepper.distanceToGo() == 0 && M2_stepper.distanceToGo() == 0 &&
          M3_stepper.distanceToGo() == 0 && M4_stepper.distanceToGo() == 0) {
        currentState = STATE_LEFT;
      }
      break;
    case STATE_LEFT:
    if(isFirst[1]){
      M1_stepper.move(-PULSES_PER_METER * motorPolarity[0]);
      M2_stepper.move(PULSES_PER_METER * motorPolarity[1]);
      M3_stepper.move(PULSES_PER_METER * motorPolarity[2]);
      M4_stepper.move(-PULSES_PER_METER * motorPolarity[3]);
      isFirst[1]=false;
    }
      if (M1_stepper.distanceToGo() == 0 && M2_stepper.distanceToGo() == 0 &&
          M3_stepper.distanceToGo() == 0 && M4_stepper.distanceToGo() == 0) {
        currentState = STATE_BACKWARD;
      }
      break;
    case STATE_BACKWARD:
    if(isFirst[2]){
      M1_stepper.move(-PULSES_PER_METER * motorPolarity[0]);
      M2_stepper.move(-PULSES_PER_METER * motorPolarity[1]);
      M3_stepper.move(-PULSES_PER_METER * motorPolarity[2]);
      M4_stepper.move(-PULSES_PER_METER * motorPolarity[3]);
      isFirst[2]=false;
    }
      if (M1_stepper.distanceToGo() == 0 && M2_stepper.distanceToGo() == 0 &&
          M3_stepper.distanceToGo() == 0 && M4_stepper.distanceToGo() == 0) {
        currentState = STATE_RIGHT;
      }
      break;
    case STATE_RIGHT:
    if(isFirst[3]){

      M1_stepper.move(PULSES_PER_METER * motorPolarity[0]);
      M2_stepper.move(-PULSES_PER_METER * motorPolarity[1]);
      M3_stepper.move(-PULSES_PER_METER * motorPolarity[2]);
      M4_stepper.move(PULSES_PER_METER * motorPolarity[3]);
      isFirst[3]=false;
    }

 
      if (M1_stepper.distanceToGo() == 0 && M2_stepper.distanceToGo() == 0 &&
          M3_stepper.distanceToGo() == 0 && M4_stepper.distanceToGo() == 0) {
        currentState = STATE_STOP;
      }
      break;
        case STATE_STOP:
      // 停止所有电机
      M1_stepper.stop();
      M2_stepper.stop();
      M3_stepper.stop();
      M4_stepper.stop();
      break;
  }
  
  M1_stepper.run();
  M2_stepper.run();
  M3_stepper.run();
  M4_stepper.run();
}