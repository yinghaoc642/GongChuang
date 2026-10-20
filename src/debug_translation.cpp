// 独立单段平移固件：仅用 debug_translation 环境编译/烧录。
// 修改下面的规范段号（不是 route[] 下标），默认第4段。
// PB9单击后执行一次；结束后保持结束状态，复位才能再次执行。
// 复用正式路线的方向、理论距离、距离倍率、速度曲线和段末IMU检查。
// 起始朝向取按键后的实时IMU，不执行前序路线、工位或机械臂动作。
// 所有平移的行程中IMU纠偏均关闭；IMU新鲜度保护和到位后的静态航向闭环保留。
// 选择第4段时复用正式扫码停车：未扫码则在原终点前190 mm停稳等待，扫码
// 后继续到原终点；提前扫到则不停车。扫码串口和任务码校验也复用正式实现。
// 第4段叠加车头+20 mm；第6段只叠加车头+10 mm；
// 选择第7/15段时叠加向车头+70 mm（含本轮新增10 mm）。
// 越过50%后只对剩余运动追加车头修正：第7段+15 mm，第15段+40 mm。
// 这些都是实测修正量，不参加横移理论量的motionScale换算。第7段仍是
// 一次连续运动，不停车、不重启；第4段扫码停车点沿完整二维向量计算。
#ifndef GONGCHUANG_DEBUG_TRANSLATION_STEP
#define GONGCHUANG_DEBUG_TRANSLATION_STEP 4
#endif
#define GONGCHUANG_DEBUG_TRANSLATION_STEP04_FORWARD_CORRECTION_ENABLED 1
#define GONGCHUANG_DEBUG_TRANSLATION_STEP04_FORWARD_CORRECTION_MM 20.0f
#define GONGCHUANG_DEBUG_TRANSLATION_STEP06_FORWARD_CORRECTION_ENABLED 1
#define GONGCHUANG_DEBUG_TRANSLATION_STEP06_FORWARD_CORRECTION_MM 10.0f
#define GONGCHUANG_DEBUG_TRANSLATION_LONG_LATERAL_FORWARD_CORRECTION_ENABLED 1
#define GONGCHUANG_DEBUG_TRANSLATION_LONG_LATERAL_FORWARD_CORRECTION_MM 70.0f
#define GONGCHUANG_DEBUG_TRANSLATION_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1
#define GONGCHUANG_DEBUG_TRANSLATION_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM 15.0f
#define GONGCHUANG_DEBUG_TRANSLATION_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1
#define GONGCHUANG_DEBUG_TRANSLATION_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM 40.0f

#include <AccelStepper.h>
#include <Arduino.h>
#include <MecanumKinematics.h>
#include <OneButton.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static_assert(
    GONGCHUANG_DEBUG_TRANSLATION_STEP >= 1 &&
        GONGCHUANG_DEBUG_TRANSLATION_STEP <= 21,
    "GONGCHUANG_DEBUG_TRANSLATION_STEP must be a specification step 1..21");
static_assert(
    GONGCHUANG_DEBUG_TRANSLATION_STEP04_FORWARD_CORRECTION_MM == 20.0f,
    "Step-4 measured correction must remain +20 mm user-forward");
static_assert(
    GONGCHUANG_DEBUG_TRANSLATION_STEP06_FORWARD_CORRECTION_MM == 10.0f,
    "Step-6 measured correction must remain +10 mm user-forward");
static_assert(
    GONGCHUANG_DEBUG_TRANSLATION_LONG_LATERAL_FORWARD_CORRECTION_MM == 70.0f,
    "Long-lateral measured correction must remain +70 mm user-forward");
static_assert(
    GONGCHUANG_DEBUG_TRANSLATION_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM == 15.0f,
    "Step-7 second-half measured correction must remain +15 mm user-forward");
static_assert(
    GONGCHUANG_DEBUG_TRANSLATION_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM == 40.0f,
    "Step-15 second-half measured correction must remain +40 mm user-forward");

namespace route_chassis {
#include "route_chassis_main.inc"
}

namespace {
namespace rc = route_chassis;
constexpr uint8_t DEBUG_TRANSLATION_STEP =
    GONGCHUANG_DEBUG_TRANSLATION_STEP;
constexpr uint32_t DEBUG_START_MINIMUM_MS = 500UL;
size_t selectedTranslationIndex = rc::ROUTE_COMMAND_COUNT;
bool debugTranslationStarted = false;
bool debugTranslationCompletionReported = false;

bool isDebugTranslationType(rc::CommandType type) {
  return type == rc::COMMAND_ZONE_LONGITUDINAL_FAST ||
         type == rc::COMMAND_FORWARD_FAST ||
         type == rc::COMMAND_BACKWARD_FAST ||
         type == rc::COMMAND_RIGHT_FAST ||
         type == rc::COMMAND_LEFT_FAST;
}

bool findDebugTranslation() {
  for (size_t index = 0U; index < rc::ROUTE_COMMAND_COUNT; ++index) {
    const rc::RouteCommand &command = rc::route[index];
    if (command.specificationStep != DEBUG_TRANSLATION_STEP) {
      continue;
    }
    if (!isDebugTranslationType(command.type)) {
      return false;
    }
    selectedTranslationIndex = index;
    return true;
  }
  return false;
}

void startDebugTranslation() {
  rc::beginRoute();
  if (rc::programState != rc::PROGRAM_RUNNING) {
    return;  // Fresh IMU is still required; waiting remains nonblocking.
  }

  rc::routeIndex = selectedTranslationIndex;
  // Do not force scanFlag. Step 4 must enter the same QR checkpoint used by
  // the formal route; other translation steps ignore the flag.

  // Production step 14/15 checks the heading remembered from step 7.
  // This isolated firmware has no preceding route: its freshly established
  // start heading is the reference. Never remove the production interlock.
  if (DEBUG_TRANSLATION_STEP == 14U || DEBUG_TRANSLATION_STEP == 15U) {
    rc::step7RawAreaHeadingReferenceDegrees =
        rc::targetCounterClockwiseHeadingDegrees;
    rc::step7RawAreaHeadingReferenceValid = true;
    rc::step15HeadingMatchAppliedAtSecondRawArea =
        DEBUG_TRANSLATION_STEP == 15U;
  }

  debugTranslationStarted = true;
  rc::SerialDebug.print("[DEBUG TRANSLATION] start step=");
  rc::SerialDebug.println(DEBUG_TRANSLATION_STEP);
  rc::startCurrentCommand();
}

void serviceDebugTranslation() {
  if (!debugTranslationStarted) {
    return;
  }

  if (rc::programState == rc::PROGRAM_RUNNING) {
    // Guard both before and after the shared scheduler. advanceRoute() may
    // increment its index on completion, but the next command must never run.
    if (rc::routeIndex == selectedTranslationIndex) {
      rc::updateRoute();
    }
    if (rc::programState == rc::PROGRAM_RUNNING &&
        rc::routeIndex != selectedTranslationIndex) {
      rc::finishProgram(true, DEBUG_TRANSLATION_STEP);
    }
  }

  if (rc::programState == rc::PROGRAM_FINISHED &&
      !debugTranslationCompletionReported) {
    debugTranslationCompletionReported = true;
    rc::SerialDebug.println(
        "[DEBUG TRANSLATION] complete; reset to run again");
  }
}
}  // namespace

void setup() {
  rc::setup();
  if (!findDebugTranslation()) {
    rc::routeFault("Debug step is missing or is not a translation");
    return;
  }

  const rc::RouteCommand &command = rc::route[selectedTranslationIndex];
  rc::SerialDebug.print("[DEBUG TRANSLATION] step/nominal-mm/scale=");
  rc::SerialDebug.print(DEBUG_TRANSLATION_STEP);
  rc::SerialDebug.print('/');
  rc::SerialDebug.print(command.value, 1);
  rc::SerialDebug.print('/');
  rc::SerialDebug.println(command.motionScale, 3);
  rc::SerialDebug.println(
      DEBUG_TRANSLATION_STEP == 4U
          ? "[DEBUG TRANSLATION] PB9 click: step4 with QR wait/resume to endpoint"
          : "[DEBUG TRANSLATION] PB9 click: one move; running hold: stop");
}

void loop() {
  rc::runAllMotors();
  rc::startButton.tick();
  rc::receiveHmiStartCommand();
  rc::receiveImuData();
  if (debugTranslationStarted &&
      rc::programState == rc::PROGRAM_RUNNING) {
    rc::receiveQrData();
  }
  rc::updateHmiYaw();
  rc::serviceBatteryVoltage();

  if (rc::abortRequested) {
    rc::abortRoute();
    return;
  }

  if (!debugTranslationStarted && rc::startRequested &&
      static_cast<uint32_t>(millis() - rc::startRequestAcceptedMs) >=
          DEBUG_START_MINIMUM_MS) {
    startDebugTranslation();
  }
  serviceDebugTranslation();
  rc::serviceMotionTelemetry();
}
