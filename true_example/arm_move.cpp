#include <Arduino.h>
#include <ArmMotorController.h>

/*
 * M5/M6/M7 独立角度控制库演示
 *
 * 正方向来自实机测试：
 *   M5：顺时针；
 *   M6：收缩；
 *   M7：上升。
 *
 * M1～M4 的原引脚定义保持不变，本程序不会驱动它们。
 */

namespace {

// 原 M1～M4 定义：保持不变。
const uint8_t DRIVE_ENABLE_PIN = PE13;
const uint8_t M1_STEP_PIN = PD4;
const uint8_t M1_DIRECTION_PIN = PD6;
const uint8_t M2_STEP_PIN = PE11;
const uint8_t M2_DIRECTION_PIN = PE9;
const uint8_t M3_STEP_PIN = PD15;
const uint8_t M3_DIRECTION_PIN = PD14;
const uint8_t M4_STEP_PIN = PA1;
const uint8_t M4_DIRECTION_PIN = PC3_C;

const uint8_t DEBUG_RX_PIN = PB12;
const uint8_t DEBUG_TX_PIN = PB13;
const uint32_t DEBUG_BAUDRATE = 115200UL;
HardwareSerial SerialDebug(DEBUG_RX_PIN, DEBUG_TX_PIN);

const float TEST_ANGLE_DEGREES = 90.0f;
const uint32_t POWER_ON_SETTLE_TIME_MS = 1500UL;
const uint32_t TURNAROUND_PAUSE_MS = 1500UL;

ArmMotorController armMotors;

} // namespace

void setup() {
  SerialDebug.begin(DEBUG_BAUDRATE);

  // M1～M4 公共使能为低有效；保持高电平，确保四个底盘电机不动。
  pinMode(DRIVE_ENABLE_PIN, OUTPUT);
  digitalWrite(DRIVE_ENABLE_PIN, HIGH);

  delay(POWER_ON_SETTLE_TIME_MS);
  armMotors.begin();

  SerialDebug.println("ArmMotorController cycle test ready");
  SerialDebug.print("M5 pulses/90deg=");
  SerialDebug.println(
      armMotors.m5PulsesForDegrees(TEST_ANGLE_DEGREES));
  SerialDebug.print("M6 pulses/90deg=");
  SerialDebug.println(
      armMotors.m6PulsesForDegrees(TEST_ANGLE_DEGREES));
  SerialDebug.print("M7 pulses/90deg=");
  SerialDebug.println(
      armMotors.m7PulsesForDegrees(TEST_ANGLE_DEGREES));
}

void loop() {
  SerialDebug.println(
      "M5 clockwise, M6 retract, M7 raise: 90 degrees");

  // M6/M7 命令发送后立即返回；M5 阻塞运行期间三轴同时运动。
  armMotors.retractM6ByDegrees(TEST_ANGLE_DEGREES);
  armMotors.raiseM7ByDegrees(TEST_ANGLE_DEGREES);
  armMotors.rotateM5ClockwiseByDegrees(
      TEST_ANGLE_DEGREES);

  delay(TURNAROUND_PAUSE_MS);

  SerialDebug.println(
      "M5 counter-clockwise, M6 extend, M7 lower: 90 degrees");

  armMotors.extendM6ByDegrees(TEST_ANGLE_DEGREES);
  armMotors.lowerM7ByDegrees(TEST_ANGLE_DEGREES);
  armMotors.rotateM5CounterClockwiseByDegrees(
      TEST_ANGLE_DEGREES);

  delay(TURNAROUND_PAUSE_MS);
}
