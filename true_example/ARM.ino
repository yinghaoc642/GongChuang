#include <Arduino.h>
#include <AccelStepper.h>

/*
 * 机械臂底部旋转轴独立测试例程
 *
 * 动作：
 *   1. 上电等待 1.5 秒；
 *   2. 底部旋转轴从开机位置转到 90°；
 *   3. 停留 2 秒；
 *   4. 反向转回开机位置 0°；
 *   5. 全程只执行一次。
 *
 * 水平伸缩轴说明：
 *   PA2/PA3 串口完全不初始化；
 *   不发送使能、回零或位置命令；
 *   因此 STM32 不会主动让水平伸缩轴运动。
 *
 * 底部旋转轴没有零点传感器，本例的 0°是本次开机位置。
 */

constexpr uint8_t BASE_ENABLE_PIN = PE10;
constexpr uint8_t BASE_DIRECTION_PIN = PE15;
constexpr uint8_t BASE_STEP_PIN = PB11;

constexpr float BASE_TEST_ANGLE_DEG = 90.0f;
constexpr uint32_t BASE_TURNAROUND_PAUSE_MS = 2000UL;
constexpr uint32_t POWER_ON_SETTLE_TIME_MS = 1500UL;

// 电机每圈200整步，驱动器16细分，机械减速比1:5。
constexpr float BASE_MOTOR_STEPS_PER_REV = 200.0f;
constexpr float BASE_MICROSTEPS = 16.0f;
constexpr float BASE_GEAR_RATIO = 5.0f;
constexpr float BASE_PULSES_PER_DEG =
    BASE_MOTOR_STEPS_PER_REV *
    BASE_MICROSTEPS *
    BASE_GEAR_RATIO / 360.0f;

constexpr float BASE_MAX_SPEED_PULSES_PER_SEC = 1000.0f;
constexpr float BASE_ACCELERATION_PULSES_PER_SEC2 = 500.0f;

AccelStepper baseStepper(
    AccelStepper::DRIVER,
    BASE_STEP_PIN,
    BASE_DIRECTION_PIN);

void moveBaseToBootAngle(float angleDeg) {
  const long targetPulses =
      lroundf(angleDeg * BASE_PULSES_PER_DEG);

  baseStepper.moveTo(targetPulses);
  baseStepper.runToPosition();
}

void setup() {
  delay(POWER_ON_SETTLE_TIME_MS);

  pinMode(BASE_ENABLE_PIN, OUTPUT);
  digitalWrite(BASE_ENABLE_PIN, LOW);  // 底部驱动器低电平使能

  baseStepper.setMaxSpeed(BASE_MAX_SPEED_PULSES_PER_SEC);
  baseStepper.setAcceleration(
      BASE_ACCELERATION_PULSES_PER_SEC2);
  baseStepper.setCurrentPosition(0);

  moveBaseToBootAngle(BASE_TEST_ANGLE_DEG);
  delay(BASE_TURNAROUND_PAUSE_MS);
  moveBaseToBootAngle(0.0f);
}

void loop() {
  // 往返动作只在setup中执行一次。
}
