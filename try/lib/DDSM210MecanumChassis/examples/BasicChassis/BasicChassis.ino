#include <Arduino.h>
#include <DDSM210MecanumChassis.h>

using namespace mecanum;

#define M0603C_RX PA3
#define M0603C_TX PA2

HardwareSerial motorSerial(M0603C_RX, M0603C_TX);

MecanumKinematics geometry =
    MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);

DDSM210MecanumChassis chassis(
    motorSerial,
    geometry,
    WheelDirections(-1, +1, -1, +1));

// 安全起见默认不让实体底盘运动；完成架空方向测试后再改为 true。
const bool ENABLE_MOTION = false;

uint32_t lastDriveMs = 0;
uint32_t lastOdometryMs = 0;
uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(115200);

  if (!chassis.begin()) {
    Serial.println("Chassis configuration invalid");
    return;
  }

  // 50 Hz 更新时，单周期跳变不应接近 1 圈；用于拒绝异常里程突变。
  chassis.odometry().setMaximumWheelDeltaTurns(1.0);

  // 上电后先发送一次零速度。
  chassis.stop();
}

void loop() {
  const uint32_t now = millis();

  if (static_cast<uint32_t>(now - lastDriveMs) >= 20U) {
    lastDriveMs = now;

    if (ENABLE_MOTION) {
      // 一个函数完成运动学换算、方向修正、限幅和四电机串口发送。
      chassis.drive(0.20f, 0.0f, 0.0f);
    } else {
      chassis.stop();
    }
  }

  if (static_cast<uint32_t>(now - lastOdometryMs) >= 50U) {
    lastOdometryMs = now;

    // 一个函数依次读取四个电机并更新 x/y/航向。
    chassis.updateOdometry();

    /*
     * 接入 HWT101 后，可用下面这一行替代上面的 updateOdometry()：
     *
     * chassis.updateOdometryWithImuHeadingDegrees(hwt101YawDegrees, 0.2);
     *
     * hwt101YawDegrees 即项目原例程中的：
     * JY901.stcAngle.Angle[2] / 32768.0f * 180.0f
     */
  }

  if (static_cast<uint32_t>(now - lastPrintMs) >= 200U) {
    lastPrintMs = now;
    const Pose2D &pose = chassis.pose();

    Serial.print("x=");
    Serial.print(pose.xMeters, 4);
    Serial.print(" m, y=");
    Serial.print(pose.yMeters, 4);
    Serial.print(" m, heading=");
    Serial.print(pose.headingRadians, 4);
    Serial.print(" rad, odom=");
    Serial.println(chassis.lastOdometryUpdateSucceeded() ? "OK" : "ERROR");
  }
}
