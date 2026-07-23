// 圆柱坐标机械臂控制程序
// 机械臂底部旋转轴使用编号5的张大头42步进电机，采用AccelStepper库进行脉冲方向控制。旋转轴减速比4:1。
// 机械臂丝杠升降轴使用编号7的张大头28步进电机，使用丝杠传动，丝杠参数T6，导程12mm，采用自己编写的TTL_STEPPER库串口控制
// 机械臂悬臂轴使用编号6的张大头28步进电机，使用齿轮齿条传动，齿轮参数模数1，齿数36，采用自己编写的TTL_STEPPER库串口控制
// 末端手爪采用深圳市华馨京科技有限公司的25KG总线伺服舵机，型号RA8-U25(H)-M，使用SDK for Arduino v2.0控制。

#include <Arduino.h>
#include <AccelStepper.h>
#include "TTL_STEPPER.h" //自己编写的TTL_STEPPER库5.12版
#include <FashionStar_SmartGripper.h> 

// 脉冲控制步进引脚（底部旋转轴）
#define M5_EN_PIN  PE10
#define M5_DIR_PIN  PE15
#define M5_STP_PIN  PB11
#define motorInterfaceType 1  // Stepper Driver, 2 driver pins required

// 串口控制步进电机控制串口
#define STEPPER_TX PA2
#define STEPPER_RX PA3
#define SMALL_ARM_STEPPER_ID 6  // 悬臂轴步进电机ID
#define BIG_ARM_STEPPER_ID 7    // 丝杠升降轴步进电机ID

// 串口舵机控制串口
#define SERVO_BAUDRATE 115200   // 串口舵机波特率
#define SERVO_RX PC7
#define SERVO_TX PC6

// 舵机编号常量符号化
#define GRIPPER_SERVO_ID 4   // 手爪舵机ID号

// 爪子的配置
#define GRIPPER_OPEN_ANGLE 0.0   // 爪子张开时的角度
#define GRIPPER_CLOSE_ANGLE -90.0  // 爪子闭合时的角度

// 硬件串口对象
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);
HardwareSerial Serial_Stepper(STEPPER_RX, STEPPER_TX);

// 舵机相关对象
FSUS_Protocol protocol(&Serial_SERVO, SERVO_BAUDRATE);  // 协议V2版本新增
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol);  // 手爪
FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);

// 步进电机相关对象
TTL_Protocol Stepper_protocol(&Serial_Stepper, 115200);
TTL_Stepper small_armStepper(SMALL_ARM_STEPPER_ID, &Stepper_protocol);  // 悬臂轴步进电机
TTL_Stepper big_armStepper(BIG_ARM_STEPPER_ID, &Stepper_protocol);    // 丝杠升降轴步进电机

AccelStepper rotationStepper(motorInterfaceType, M5_STP_PIN, M5_DIR_PIN);  // 底部旋转轴步进电机

// 运动参数
const float PULSES_PER_REV = 200;  // 步进电机每转脉冲数
const float MICROSTEPS = 16;       // 细分倍数
// 底部旋转轴减速比
const float ROTATION_GEAR_RATIO = 4; // 底部旋转轴减速比 4:1
// 计算底部旋转轴每度脉冲数时考虑减速比
const float ROTATION_PULSES_PER_DEG = (PULSES_PER_REV * MICROSTEPS * ROTATION_GEAR_RATIO) / 360.0;  // 底部旋转轴每度脉冲数
const float LEAD_SCREW_PITCH = 12; // 丝杠导程
const float GEAR_MODULE = 1;       // 齿轮模数
const float GEAR_TEETH = 36;       // 齿轮齿数
const float RACK_PITCH = PI * GEAR_MODULE; // 齿条齿距

void setup() {
 delay(100);//等待步进上电启动 
  // 初始化串口通信
  Serial.begin(115200);
  Serial_SERVO.begin(SERVO_BAUDRATE);
  Serial_Stepper.begin(115200);

  // 初始化步进电机
  rotationStepper.setMaxSpeed(1000);  // 设置底部旋转轴最大速度
  rotationStepper.setAcceleration(500); // 设置底部旋转轴加速度
  Stepper_protocol.init(&Serial_Stepper, 115200);
  small_armStepper.init();  // 悬臂轴步进电机初始化
  big_armStepper.init();    // 丝杠升降轴步进电机初始化

  // 初始化舵机
  protocol.init(&Serial_SERVO, SERVO_BAUDRATE);  // 舵机通信协议初始化
  gripper.init();                                // 手爪舵机初始化
  gripper.setMaxPower(400);    // 设置最大功率，单位mW

}

void loop() {
  // 示例运动控制
  // 底部旋转轴旋转 30 度
  rotateBase(30);
  delay(2000);

  // 悬臂轴伸出 10mm
  moveSmallArm(10);
  delay(2000);

  // 丝杠升降轴上升 10mm
  moveBigArm(10);
  delay(2000);

  // 手爪闭合
  closeGripper();
  delay(1000);

  // 手爪张开
  openGripper();
  delay(1000);

  // 回到初始位置
  rotateBase(0);
  moveSmallArm(0);
  moveBigArm(0);
  delay(2000);

  while (1); // 停止循环
}

// 底部旋转轴旋转函数
void rotateBase(float degrees) {
  long targetPulses = degrees * ROTATION_PULSES_PER_DEG;
  rotationStepper.moveTo(targetPulses);
  while (rotationStepper.distanceToGo() != 0) {
    rotationStepper.run();
  }
}

// 悬臂轴移动函数
void moveSmallArm(float distance) {
  long steps = (distance / RACK_PITCH) * PULSES_PER_REV * MICROSTEPS;
  small_armStepper.move(steps);
  while (small_armStepper.distanceToGo() != 0) {
    // 等待悬臂轴移动完成
  }
}

// 丝杠升降轴移动函数
void moveBigArm(float distance) {
  long steps = (distance / LEAD_SCREW_PITCH) * PULSES_PER_REV * MICROSTEPS;
  big_armStepper.move(steps);
  while (big_armStepper.distanceToGo() != 0) {
    // 等待丝杠升降轴移动完成
  }
}

// 手爪闭合函数
void closeGripper() {
  gripper.setAngle(GRIPPER_CLOSE_ANGLE);
  delay(1000); // 等待手爪闭合
}

// 手爪张开函数
void openGripper() {
  gripper.setAngle(GRIPPER_OPEN_ANGLE);
  delay(1000); // 等待手爪张开
}