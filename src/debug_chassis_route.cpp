/*
 * 独立完整底盘路线调试固件：仅用 debug_chassis_route 环境编译/烧录。
 *
 * 直接复用正式 route_chassis_main.inc 的19条底盘命令、启停区1路线状态机
 * 和全部底盘参数，
 * 不包含正式 main.cpp，因此不会初始化或运行M5/M6/M7、夹爪、物料台、
 * MaixCAM及六个工位动作。每条底盘命令正常完成后立即进入下一条命令；
 * 默认启停区1普通出发：1->4->5...19，第4段保留扫码条件停车。
 * 正式路线模式设为1时，才启用前进150 mm、第4段横移200 mm后逆时针360°、
 * 中心左移扫码和等距归位。两种出发模式均使用21->20的分段返程，
 * 最后补回150 mm后结束。所有平移的
 * 行程中IMU纠偏均关闭，IMU新鲜度保护和到位后的静态航向闭环继续执行。
 *
 * 下列修正量必须与正式main一致，都是实车修正量，不是理论路线量：
 *   - 第4段：车头+20 mm；第6段：只保留车头+10 mm；
 *   - 第7/15段：全段车头+80 mm（原70 mm再加10 mm朝扫码区）；
 *   - 第7段超过50%：只对剩余运动再加车头+15 mm。
 *   - 第15段超过50%：只对剩余运动再加车头+40 mm。
 */
#ifndef GONGCHUANG_STEP04_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_STEP04_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_STEP04_FORWARD_CORRECTION_MM
#define GONGCHUANG_STEP04_FORWARD_CORRECTION_MM 20.0f
#endif
#ifndef GONGCHUANG_STEP06_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_STEP06_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_STEP06_FORWARD_CORRECTION_MM
#define GONGCHUANG_STEP06_FORWARD_CORRECTION_MM 10.0f
#endif
#ifndef GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_MM
#define GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_MM 80.0f
#endif
#ifndef GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM
#define GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM 15.0f
#endif
#ifndef GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM
#define GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM 40.0f
#endif

#include <AccelStepper.h>
#include <Arduino.h>
#include <Stm32SerialCompat.h>
#include <MecanumKinematics.h>
#include <OneButton.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static_assert(
    GONGCHUANG_STEP04_FORWARD_CORRECTION_MM == 20.0f &&
        GONGCHUANG_STEP06_FORWARD_CORRECTION_MM == 10.0f,
    "Chassis-only step-4/6 corrections must match production");
static_assert(
    GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_MM == 80.0f &&
        GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM == 15.0f &&
        GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM == 40.0f,
    "Chassis-only step-7/15 corrections must match production");

namespace route_chassis {
#include "route_chassis_main.inc"
}

namespace {
namespace rc = route_chassis;
constexpr uint32_t CHASSIS_ROUTE_START_MINIMUM_MS = 500UL;
}

/*
 * 使用共享底盘初始化，不调用正式main.cpp的机械臂、舵机或视觉初始化。
 */
void setup() {
  rc::setup();
  rc::SerialDebug.println(
      "[DEBUG CHASSIS ROUTE] full 19-command route; no workstation actions");
  rc::SerialDebug.println(
      "[DEBUG CHASSIS ROUTE] start minimum=500 ms; production route mode");
}

/*
 * 该循环与共享底盘调度顺序一致，只补两项正式main已经具备的边界：
 *   1. PB9启动请求确认后不足500 ms时非阻塞补足；
 *   2. 只在路线正式运行后接收并锁定二维码。
 *
 * 不在这里复制路线表，也不插入工位暂停或固定段间等待。
 */
void loop() {
  rc::runAllMotors();
  rc::startButton.tick();
  rc::receiveHmiStartCommand();
  rc::receiveImuData();
  if (rc::programState == rc::PROGRAM_RUNNING) {
    // 独立底盘固件没有外层TaskPlan；消费握手即可开放中心点新扫码窗口。
    (void)rc::consumeZone1FreshQrResetRequest();
    rc::receiveQrData();
  }
  rc::updateHmiYaw();
  rc::serviceBatteryVoltage();

  if (rc::abortRequested) {
    if (rc::programState == rc::PROGRAM_RUNNING ||
        (rc::programState == rc::PROGRAM_WAITING &&
         rc::startRequested)) {
      rc::abortRoute();
      return;
    }
    rc::abortRequested = false;
  }

  if (rc::startRequested &&
      static_cast<uint32_t>(
          millis() - rc::startRequestAcceptedMs) >=
          CHASSIS_ROUTE_START_MINIMUM_MS) {
    rc::beginRoute();
  }

  rc::updateRoute();
  rc::serviceMotionTelemetry();
}
