#include <Arduino.h>
#include <PulseMecanumChassis.h>

using namespace mecanum;

// yyq5/main.cpp 中 M1～M4 的 STEP/DIR 和公共低有效使能脚。
const PulseMecanumPins motorPins(
    PD4, PD6,       // M1：前左
    PE11, PE9,      // M2：前右
    PD15, PD14,     // M3：后左
    PA1, PC3_C,     // M4：后右
    PE13, true);    // 公共 EN：低电平有效

// 当前 GongChuang 运动学文档中的实测几何尺寸。
MecanumKinematics geometry =
    MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);

PulseMecanumChassis chassis(
    motorPins,
    geometry,
    WheelDirections(-1, +1, -1, +1),
    3200.0f,   // 16 细分、1.8°电机：3200 pulse/rev
    10000.0f,  // yyq5 直线标定：10000 pulse/m
    30000.0f); // yyq5 最大脉冲频率：30000 pulse/s

// 架空并确认四轮方向前保持 false，避免示例一烧录就让底盘运动。
const bool ENABLE_MOTION = false;

void setup() {
  Serial.begin(115200);

  if (!chassis.begin(ENABLE_MOTION)) {
    Serial.println("Pulse chassis configuration invalid");
    return;
  }

  if (ENABLE_MOTION) {
    // vx=0.20 m/s 向前，vy=0，wz=0；设置一次后由 loop() 持续发脉冲。
    chassis.drive(0.20f, 0.0f, 0.0f);

    // 其他调用示例：
    // chassis.drive(0.0f, 0.20f, 0.0f); // 向左 0.20 m/s
    // chassis.drive(0.0f, 0.0f, 0.50f); // 逆时针 0.50 rad/s
    // chassis.drive(0.20f, 0.10f, 0.30f); // 平移与旋转复合运动
  }
}

void loop() {
  /*
   * 必须尽可能频繁且无条件调用。不要在底盘运动期间使用 delay()、wait()
   * 或长时间阻塞循环，否则软件无法按时产生 STEP 脉冲。
   */
  chassis.run();
}
