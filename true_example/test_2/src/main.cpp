#include <ArmMotorController.h>
#include <Arduino.h>
#include <FashionStar_UartServo.h>
#include <OneButton.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*
 * 暂存区“霍夫圆定位 + 从载物盘取料 + 三圈放置”独立测试。
 *
 * 代码参数和动作顺序来自正式 src/main.cpp，底盘、二维码、IMU、原料区及
 * 粗加工区均未包含。测试中PE13始终为高，保证M1~M4公共驱动保持失能。
 *
 * 上电前必须人工确认：
 *   M5 = 旧坐标0°行驶姿态，M6 = 完全缩回，M7 = 最高点。
 * 本机没有M6/M7限位开关，本程序只建立软件零点，不执行机械寻零。
 */

namespace {

// ---------------------------------------------------------------------------
// 用户测试配置
// ---------------------------------------------------------------------------

// 1：暂存区第一轮；2：暂存区第二轮。
constexpr uint8_t TEST_STORAGE_ROUND = 1U;

// 三个载物盘槽位依次放到哪个圈。需要模拟任务码时只修改这里。
constexpr uint8_t TEST_RING_ORDER[3] = {1U, 2U, 3U};

static_assert(
    TEST_STORAGE_ROUND == 1U || TEST_STORAGE_ROUND == 2U,
    "TEST_STORAGE_ROUND must be 1 or 2");

// ---------------------------------------------------------------------------
// 引脚、串口与执行器
// ---------------------------------------------------------------------------

constexpr uint8_t DRIVE_ENABLE_PIN = PE13;  // M1~M4公共使能，低有效
constexpr uint8_t START_BUTTON_PIN = PB9;
constexpr uint8_t DEBUG_RX_PIN = PB12;
constexpr uint8_t DEBUG_TX_PIN = PB13;
constexpr uint8_t MAIXCAM_RX_PIN = PE7;
constexpr uint8_t MAIXCAM_TX_PIN = PE8;
constexpr uint8_t ARM_LINEAR_RX_PIN = PA3;
constexpr uint8_t ARM_LINEAR_TX_PIN = PA2;
constexpr uint8_t SERVO_RX_PIN = PC7;
constexpr uint8_t SERVO_TX_PIN = PC6;

constexpr uint32_t DEBUG_BAUDRATE = 115200UL;
constexpr uint32_t MAIXCAM_BAUDRATE = 115200UL;
constexpr uint32_t ARM_LINEAR_BAUDRATE = 115200UL;
constexpr uint32_t SERVO_BAUDRATE = 115200UL;

HardwareSerial SerialDebug(DEBUG_RX_PIN, DEBUG_TX_PIN);
HardwareSerial SerialMaixcam(MAIXCAM_RX_PIN, MAIXCAM_TX_PIN);
HardwareSerial SerialArmLinear(
    ARM_LINEAR_RX_PIN, ARM_LINEAR_TX_PIN);
HardwareSerial SerialServo(SERVO_RX_PIN, SERVO_TX_PIN);

constexpr uint8_t GRIPPER_SERVO_ID = 4U;
constexpr uint8_t STORAGE_SERVO_ID = 5U;
FSUS_Protocol servoProtocol;
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &servoProtocol);
FSUS_Servo storageServo(STORAGE_SERVO_ID, &servoProtocol);
OneButton startButton(START_BUTTON_PIN, true, true);
ArmMotorController armMotors;

// ---------------------------------------------------------------------------
// 与正式main一致的机械臂、视觉及舵机参数
// ---------------------------------------------------------------------------

constexpr float PI_F = 3.14159265358979323846f;

constexpr int32_t ARM_BASE_STANDARD_OLD_FRAME_DEGREES = -90;
constexpr float ARM_BASE_TRAVEL_STANDARD_FRAME_DEGREES = 90.0f;
constexpr float ARM_CONTAINER_CLOCKWISE_DEGREES = -100.0f;
constexpr float ARM_BASE_MAXIMUM_STEP_RATE = 60000.0f;
constexpr float ARM_BASE_STEP_ACCELERATION = 17500.0f;
constexpr uint32_t ARM_BASE_SETTLE_MS = 50UL;

constexpr uint8_t ARM_EXTENSION_ADDRESS = 6U;
constexpr uint8_t ARM_LIFT_ADDRESS = 7U;
constexpr float FULL_STEPS_PER_REVOLUTION = 200.0f;
constexpr float ARM_LINEAR_MICROSTEPS = 256.0f;
const float M6_TRAVEL_PER_REVOLUTION_MM = PI_F * 35.0f;
constexpr float M7_TRAVEL_PER_REVOLUTION_MM = 12.0f;
const float M6_PULSES_PER_MM =
    FULL_STEPS_PER_REVOLUTION * ARM_LINEAR_MICROSTEPS /
    M6_TRAVEL_PER_REVOLUTION_MM;
constexpr float M7_PULSES_PER_MM =
    FULL_STEPS_PER_REVOLUTION * ARM_LINEAR_MICROSTEPS /
    M7_TRAVEL_PER_REVOLUTION_MM;

constexpr uint8_t M6_EXTEND_DIRECTION = 0U;
constexpr uint8_t M6_RETRACT_DIRECTION = 1U;
constexpr uint8_t M7_RAISE_DIRECTION = 1U;
constexpr uint8_t M7_LOWER_DIRECTION = 0U;
constexpr uint16_t M6_SPEED_RPM = 290U;
constexpr uint8_t M6_ACCELERATION = 118U;
constexpr uint16_t M7_SPEED_RPM = 2496U;
constexpr uint8_t M7_ACCELERATION = 125U;
constexpr uint32_t ARM_LINEAR_POWER_ON_SETTLE_MS = 1500UL;
constexpr uint32_t ARM_AXIS_COMMAND_GUARD_MS = 30UL;
constexpr uint32_t ARM_AXIS_STATUS_INTERVAL_MS = 30UL;
constexpr uint32_t ARM_AXIS_MINIMUM_ON_POSITION_MS = 120UL;
constexpr uint32_t ARM_AXIS_MINIMUM_TIMEOUT_MS = 5000UL;
constexpr uint32_t ARM_AXIS_MAXIMUM_TIMEOUT_MS = 30000UL;
constexpr uint32_t ARM_AXIS_STARTUP_RESPONSE_TIMEOUT_MS = 150UL;
constexpr uint8_t ARM_AXIS_STALL_CONFIRMATION_SAMPLES = 3U;

constexpr float M6_STANDARD_EXTENSION_MM = 0.0f;
constexpr float M7_STANDARD_HEIGHT_MM = 0.0f;
constexpr float M6_MAXIMUM_EXTENSION_MM = 150.0f;
constexpr float M7_MINIMUM_HEIGHT_MM = -160.0f;
constexpr float ARM_AXIS_POSITION_TOLERANCE_MM = 0.05f;

constexpr float HOUGH_VISION_LOWER_MM = 90.0f;
constexpr float HOUGH_VISION_HEIGHT_MM =
    M7_STANDARD_HEIGHT_MM - HOUGH_VISION_LOWER_MM;
constexpr float CONTAINER_EXTENSION_MM = 10.0f;
constexpr float CONTAINER_PICK_LOWER_MM = 40.0f;
constexpr float STORAGE_ROUND1_PLACE_LOWER_MM = 150.0f;
constexpr float STORAGE_ROUND2_PLACE_LOWER_MM = 90.0f;
constexpr float RING_RELEASE_LOWER_REDUCTION_MM = 5.0f;
// 暂存区三个圈都在同类动作中再少下降5 mm。
constexpr float STORAGE_RING_LOWER_REDUCTION_MM = 5.0f;

constexpr float GRIPPER_CLOSE_ANGLE_DEGREES = 99.0f;
constexpr float GRIPPER_OPEN_ANGLE_DEGREES = 37.0f;
constexpr uint16_t GRIPPER_INTERVAL_MS = 100U;
constexpr uint16_t GRIPPER_OPEN_POWER_MW = 0U;
constexpr uint16_t GRIPPER_CLOSE_POWER_MW = 2000U;
constexpr uint32_t GRIPPER_OPEN_SETTLE_MS = 175UL;
constexpr uint32_t GRIPPER_CLOSE_SETTLE_MS = 225UL;
constexpr uint16_t STORAGE_SERVO_INTERVAL_MS = 200U;
constexpr uint32_t STORAGE_SERVO_SETTLE_MS = 300UL;
constexpr float STORAGE_SERVO_PARK_ANGLE_DEGREES = 165.0f;
constexpr float STORAGE_SERVO_POSITIONS_DEGREES[3] = {
    -5.0f, -95.0f, -185.0f};

constexpr uint8_t MAIXCAM_FRAME_HEADER = 0xAAU;
constexpr uint8_t MAIXCAM_FRAME_TAIL = 0xBBU;
constexpr uint8_t MAIXCAM_STOP_REQUEST = 0x00U;
constexpr uint8_t MAIXCAM_HOUGH_CIRCLE_REQUEST = 0x09U;
constexpr uint32_t MAIXCAM_REQUEST_REPEAT_MS = 1000UL;
constexpr uint32_t MAIXCAM_MODE_SWITCH_GUARD_MS = 100UL;
constexpr uint32_t MAIXCAM_COORDINATE_STALE_MS = 1500UL;
constexpr size_t MAIXCAM_LINE_CAPACITY = 32U;

constexpr int16_t IMAGE_CENTER_X = 160;
constexpr int16_t IMAGE_CENTER_Y = 120;
constexpr int16_t IMAGE_MAX_X = 319;
constexpr int16_t IMAGE_MAX_Y = 239;
constexpr float CIRCLE_MM_PER_PIXEL = 35.0f / 72.0f;
constexpr float ARM_PIVOT_TO_CAMERA_CENTER_MM = 135.0f;
constexpr float SIDE_RING_CENTER_OFFSET_MM = 150.0f;
constexpr float IMAGE_Y_TO_ARM_OUTWARD_SIGN = -1.0f;
constexpr float CIRCLE_CENTER_TOLERANCE_PIXELS = 3.0f;
constexpr uint32_t CIRCLE_CENTER_STABILITY_MS = 500UL;
constexpr uint8_t CIRCLE_STABILITY_MINIMUM_SAMPLES = 2U;
constexpr uint32_t VISION_STABILITY_MAXIMUM_SAMPLE_GAP_MS =
    1200UL;

static_assert(
    ARM_AXIS_STALL_CONFIRMATION_SAMPLES >= 3U,
    "M6/M7 stall shutdown requires 3 consecutive samples");

enum TestState {
  TEST_READY,
  TEST_PREPARING,
  TEST_CALIBRATING,
  TEST_POST_CALIBRATION_STANDARD,
  TEST_PLACING,
  TEST_WAIT_STORAGE_SERVO,
  TEST_FINAL_STANDARD,
  TEST_FINAL_TRAVEL,
  TEST_COMPLETE,
  TEST_FAULT
};

TestState testState = TEST_READY;
bool abortRequested = false;
uint8_t testItemIndex = 0U;
uint32_t storageServoDeadlineMs = 0UL;
uint32_t finalTravelDeadlineMs = 0UL;

const char *testStateName(TestState state) {
  switch (state) {
    case TEST_READY:
      return "READY";
    case TEST_PREPARING:
      return "PREPARING";
    case TEST_CALIBRATING:
      return "CALIBRATING";
    case TEST_POST_CALIBRATION_STANDARD:
      return "POST_CAL_STANDARD";
    case TEST_PLACING:
      return "PLACING";
    case TEST_WAIT_STORAGE_SERVO:
      return "WAIT_STORAGE_SERVO";
    case TEST_FINAL_STANDARD:
      return "FINAL_STANDARD";
    case TEST_FINAL_TRAVEL:
      return "FINAL_TRAVEL";
    case TEST_COMPLETE:
      return "COMPLETE";
    case TEST_FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

void setTestState(TestState nextState) {
  if (testState == nextState) {
    return;
  }
  SerialDebug.print("[STATE] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, TEST: ");
  SerialDebug.print(testStateName(testState));
  SerialDebug.print(" -> ");
  SerialDebug.println(testStateName(nextState));
  testState = nextState;
}

bool deadlineReached(uint32_t deadlineMs) {
  return static_cast<int32_t>(millis() - deadlineMs) >= 0;
}

bool testIsRunning() {
  return testState != TEST_READY &&
         testState != TEST_COMPLETE &&
         testState != TEST_FAULT;
}

void stopMaixRequest();
void emergencyStopArmLinearAxes();
void invalidateArmLinearReference();

void testFault(const char *reason) {
  if (testState == TEST_FAULT) {
    return;
  }

  const TestState beforeFault = testState;
  stopMaixRequest();
  armMotors.stopM5Immediately();
  armMotors.disableM5();
  emergencyStopArmLinearAxes();
  invalidateArmLinearReference();
  digitalWrite(DRIVE_ENABLE_PIN, HIGH);
  setTestState(TEST_FAULT);

  SerialDebug.print("[FAULT] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, previous=");
  SerialDebug.print(testStateName(beforeFault));
  SerialDebug.print(", reason=");
  SerialDebug.println(reason);
  SerialDebug.println(
      "[FAULT] M5/M6/M7 software coordinates are no longer trusted.");
  SerialDebug.println(
      "[FAULT] Power-cycle and manually restore M5 old 0 / M6 retract / M7 top.");
}

bool validateTestConfiguration() {
  bool seen[4] = {false, false, false, false};
  for (uint8_t i = 0U; i < 3U; ++i) {
    const uint8_t ring = TEST_RING_ORDER[i];
    if (ring < 1U || ring > 3U || seen[ring]) {
      SerialDebug.println(
          "[CONFIG ERROR] TEST_RING_ORDER must be a permutation of 1,2,3");
      return false;
    }
    seen[ring] = true;
  }
  return true;
}

// ---------------------------------------------------------------------------
// M5
// ---------------------------------------------------------------------------

void startArmBaseStandardFrameDegrees(float angleDegrees) {
  const float libraryDegrees =
      static_cast<float>(ARM_BASE_STANDARD_OLD_FRAME_DEGREES) +
      angleDegrees;
  armMotors.moveM5ToDegrees(libraryDegrees, false);

  SerialDebug.print("[M5 CMD] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, standard/library=");
  SerialDebug.print(angleDegrees, 3);
  SerialDebug.print("/");
  SerialDebug.print(libraryDegrees, 3);
  SerialDebug.print(" deg, pulses=");
  SerialDebug.println(
      armMotors.m5PulsesForDegrees(libraryDegrees));
}

// ---------------------------------------------------------------------------
// M6/M7 EMM V5非阻塞控制
// ---------------------------------------------------------------------------

struct LinearAxisMotion {
  uint8_t address;
  float currentMm;
  float targetMm;
  bool active;
  bool fault;
  bool commandAcknowledged;
  bool motionObserved;
  uint8_t stallSampleCount;
  uint32_t startMs;
  uint32_t lastStatusRequestMs;
  uint32_t timeoutMs;

  explicit LinearAxisMotion(uint8_t axisAddress)
      : address(axisAddress),
        currentMm(0.0f),
        targetMm(0.0f),
        active(false),
        fault(false),
        commandAcknowledged(false),
        motionObserved(false),
        stallSampleCount(0U),
        startMs(0UL),
        lastStatusRequestMs(0UL),
        timeoutMs(ARM_AXIS_MINIMUM_TIMEOUT_MS) {}
};

LinearAxisMotion extensionAxis(ARM_EXTENSION_ADDRESS);
LinearAxisMotion liftAxis(ARM_LIFT_ADDRESS);
bool armLinearReferenceValid = false;
bool armLinearSerialInitialized = false;
uint8_t armLinearReceiveWindow[4] = {0U, 0U, 0U, 0U};
uint8_t armLinearReceiveCount = 0U;

LinearAxisMotion *linearAxisForAddress(uint8_t address) {
  if (address == extensionAxis.address) {
    return &extensionAxis;
  }
  if (address == liftAxis.address) {
    return &liftAxis;
  }
  return nullptr;
}

void writeArmLinearEnable(uint8_t address, bool enable) {
  const uint8_t frame[6] = {
      address, 0xF3U, 0xABU,
      static_cast<uint8_t>(enable ? 1U : 0U),
      0x00U, 0x6BU};
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearPosition(
    uint8_t address,
    uint8_t direction,
    uint16_t speedRpm,
    uint8_t acceleration,
    uint32_t pulses) {
  const uint8_t frame[13] = {
      address,
      0xFDU,
      direction,
      static_cast<uint8_t>(speedRpm >> 8),
      static_cast<uint8_t>(speedRpm),
      acceleration,
      static_cast<uint8_t>(pulses >> 24),
      static_cast<uint8_t>(pulses >> 16),
      static_cast<uint8_t>(pulses >> 8),
      static_cast<uint8_t>(pulses),
      0x00U,  // relative motion
      0x00U,  // no multi-axis synchronization
      0x6BU};
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearStatusRequest(uint8_t address) {
  const uint8_t frame[3] = {address, 0x3AU, 0x6BU};
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearClearStallProtection(uint8_t address) {
  const uint8_t frame[4] = {
      address, 0x0EU, 0x52U, 0x6BU};
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearStop(uint8_t address) {
  const uint8_t frame[5] = {
      address, 0xFEU, 0x98U, 0x00U, 0x6BU};
  SerialArmLinear.write(frame, sizeof(frame));
}

void clearArmLinearReceiveBuffer() {
  while (SerialArmLinear.available()) {
    SerialArmLinear.read();
  }
  armLinearReceiveCount = 0U;
}

void printArmLinearFrame(
    const char *label,
    const uint8_t *frame) {
  SerialDebug.print("[EMM RX] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, ");
  SerialDebug.print(label);
  SerialDebug.print(": ");
  for (uint8_t i = 0U; i < 4U; ++i) {
    if (frame[i] < 0x10U) {
      SerialDebug.print('0');
    }
    SerialDebug.print(
        static_cast<unsigned int>(frame[i]), HEX);
    if (i + 1U < 4U) {
      SerialDebug.print(' ');
    }
  }
  SerialDebug.println();
}

bool waitForArmLinearFrame(
    uint8_t address,
    uint8_t functionCode,
    uint8_t *frame,
    uint32_t timeoutMs) {
  uint8_t window[4] = {0U, 0U, 0U, 0U};
  uint8_t count = 0U;
  const uint32_t startMs = millis();

  do {
    while (SerialArmLinear.available()) {
      const uint8_t incoming =
          static_cast<uint8_t>(SerialArmLinear.read());
      if (count < 4U) {
        window[count++] = incoming;
      } else {
        window[0] = window[1];
        window[1] = window[2];
        window[2] = window[3];
        window[3] = incoming;
      }

      if (count != 4U || incoming != 0x6BU) {
        continue;
      }
      if (window[0] == address &&
          window[1] == functionCode) {
        memcpy(frame, window, sizeof(window));
        armLinearReceiveCount = 0U;
        return true;
      }
      count = 0U;
    }
    delay(1);
  } while (millis() - startMs < timeoutMs);

  armLinearReceiveCount = 0U;
  return false;
}

bool recoverAndVerifyArmLinearAxis(
    LinearAxisMotion &axis) {
  uint8_t response[4] = {0U, 0U, 0U, 0U};

  SerialDebug.print("[EMM BOOT] M");
  SerialDebug.print(axis.address);
  SerialDebug.println(" stop, clear latched stall protection");
  clearArmLinearReceiveBuffer();
  writeArmLinearStop(axis.address);
  SerialArmLinear.flush();
  delay(ARM_AXIS_COMMAND_GUARD_MS);
  clearArmLinearReceiveBuffer();

  writeArmLinearClearStallProtection(axis.address);
  SerialArmLinear.flush();
  if (waitForArmLinearFrame(
          axis.address,
          0x0EU,
          response,
          ARM_AXIS_STARTUP_RESPONSE_TIMEOUT_MS)) {
    printArmLinearFrame("startup clear response", response);
  } else {
    SerialDebug.print("[EMM BOOT] M");
    SerialDebug.print(axis.address);
    SerialDebug.println(" no clear ACK; verify with 3A");
  }

  clearArmLinearReceiveBuffer();
  writeArmLinearEnable(axis.address, true);
  SerialArmLinear.flush();
  if (waitForArmLinearFrame(
          axis.address,
          0xF3U,
          response,
          ARM_AXIS_STARTUP_RESPONSE_TIMEOUT_MS)) {
    printArmLinearFrame("startup enable response", response);
  } else {
    SerialDebug.print("[EMM BOOT] M");
    SerialDebug.print(axis.address);
    SerialDebug.println(" no enable ACK; verify with 3A");
  }

  axis.stallSampleCount = 0U;
  for (uint8_t attempt = 0U;
       attempt < ARM_AXIS_STALL_CONFIRMATION_SAMPLES;
       ++attempt) {
    clearArmLinearReceiveBuffer();
    writeArmLinearStatusRequest(axis.address);
    SerialArmLinear.flush();
    if (!waitForArmLinearFrame(
            axis.address,
            0x3AU,
            response,
            ARM_AXIS_STARTUP_RESPONSE_TIMEOUT_MS)) {
      SerialDebug.print("[EMM BOOT ERROR] M");
      SerialDebug.print(axis.address);
      SerialDebug.println(" no 3A status response");
      axis.fault = true;
      return false;
    }

    printArmLinearFrame("startup status", response);
    const uint8_t flags = response[2];
    SerialDebug.print("[EMM BOOT] M");
    SerialDebug.print(axis.address);
    SerialDebug.print(" enabled/on-position/stall/protect=");
    SerialDebug.print((flags & 0x01U) != 0U ? 1 : 0);
    SerialDebug.print("/");
    SerialDebug.print((flags & 0x02U) != 0U ? 1 : 0);
    SerialDebug.print("/");
    SerialDebug.print((flags & 0x04U) != 0U ? 1 : 0);
    SerialDebug.print("/");
    SerialDebug.println((flags & 0x08U) != 0U ? 1 : 0);

    if ((flags & 0x01U) == 0U) {
      axis.fault = true;
      return false;
    }
    if ((flags & 0x0CU) == 0U) {
      axis.stallSampleCount = 0U;
      return true;
    }

    ++axis.stallSampleCount;
    SerialDebug.print("[EMM BOOT STALL] M");
    SerialDebug.print(axis.address);
    SerialDebug.print(", sample=");
    SerialDebug.print(axis.stallSampleCount);
    SerialDebug.print("/");
    SerialDebug.println(ARM_AXIS_STALL_CONFIRMATION_SAMPLES);
    if (axis.stallSampleCount >=
        ARM_AXIS_STALL_CONFIRMATION_SAMPLES) {
      axis.fault = true;
      axis.stallSampleCount = 0U;
      return false;
    }
    delay(ARM_AXIS_STATUS_INTERVAL_MS);
  }

  axis.fault = true;
  axis.stallSampleCount = 0U;
  return false;
}

void markLinearAxisArrived(LinearAxisMotion &axis) {
  axis.active = false;
  axis.currentMm = axis.targetMm;
  axis.stallSampleCount = 0U;
  SerialDebug.print("[EMM ARRIVE] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, M");
  SerialDebug.print(axis.address);
  SerialDebug.print("=");
  SerialDebug.print(axis.currentMm, 2);
  SerialDebug.println(" mm");
}

void faultLinearAxis(
    LinearAxisMotion &axis,
    const char *reason) {
  if (axis.fault) {
    return;
  }
  axis.active = false;
  axis.fault = true;
  axis.stallSampleCount = 0U;
  testFault(reason);
}

bool consumeArmLinearEnableResponse(
    LinearAxisMotion &axis) {
  uint8_t window[4] = {0U, 0U, 0U, 0U};
  uint8_t count = 0U;
  bool enableResponseSeen = false;

  while (SerialArmLinear.available()) {
    const uint8_t incoming =
        static_cast<uint8_t>(SerialArmLinear.read());
    if (count < 4U) {
      window[count++] = incoming;
    } else {
      window[0] = window[1];
      window[1] = window[2];
      window[2] = window[3];
      window[3] = incoming;
    }

    if (count != 4U || incoming != 0x6BU) {
      continue;
    }

    if (window[0] == axis.address &&
        window[1] == 0xF3U) {
      enableResponseSeen = true;
      printArmLinearFrame(
          window[2] == 0x02U
              ? "enable accepted"
              : "enable condition not met",
          window);
    } else if (
        window[0] == axis.address &&
        window[1] == 0x00U &&
        window[2] == 0xEEU) {
      printArmLinearFrame("enable wrong command", window);
      faultLinearAxis(axis, "EMM enable wrong command");
      armLinearReceiveCount = 0U;
      return false;
    }
    count = 0U;
  }

  armLinearReceiveCount = 0U;
  if (!enableResponseSeen) {
    SerialDebug.print("[EMM ENABLE] t=");
    SerialDebug.print(millis());
    SerialDebug.print(" ms, M");
    SerialDebug.print(axis.address);
    SerialDebug.println(" no F3 ACK");
  }
  return true;
}

void handleArmLinearFrame(const uint8_t *frame) {
  if (frame[3] != 0x6BU) {
    return;
  }

  LinearAxisMotion *axis =
      linearAxisForAddress(frame[0]);
  if (axis == nullptr || !axis->active) {
    return;
  }

  const uint32_t elapsedMs = millis() - axis->startMs;
  if (frame[1] == 0x00U && frame[2] == 0xEEU) {
    printArmLinearFrame("wrong command", frame);
    faultLinearAxis(*axis, "EMM wrong command");
    return;
  }
  if (frame[1] == 0xFDU && frame[2] == 0xE2U) {
    printArmLinearFrame("position condition not met", frame);
    faultLinearAxis(
        *axis, "EMM position condition not met");
    return;
  }
  if (frame[1] == 0xFDU && frame[2] == 0x02U) {
    axis->commandAcknowledged = true;
    SerialDebug.print("[EMM ACK] t=");
    SerialDebug.print(millis());
    SerialDebug.print(" ms, M");
    SerialDebug.print(axis->address);
    SerialDebug.println(" position accepted");
    return;
  }

  if (frame[1] == 0x3AU) {
    const uint8_t flags = frame[2];
    if ((flags & 0x01U) == 0U) {
      printArmLinearFrame("axis disabled", frame);
      faultLinearAxis(*axis, "EMM axis is not enabled");
      return;
    }

    const bool stallReported = (flags & 0x04U) != 0U;
    const bool stallProtectionActive =
        (flags & 0x08U) != 0U;
    if (stallReported || stallProtectionActive) {
      if (axis->stallSampleCount < UINT8_MAX) {
        ++axis->stallSampleCount;
      }
      SerialDebug.print("[EMM STALL] t=");
      SerialDebug.print(millis());
      SerialDebug.print(" ms, M");
      SerialDebug.print(axis->address);
      SerialDebug.print(", sample=");
      SerialDebug.print(axis->stallSampleCount);
      SerialDebug.print("/");
      SerialDebug.print(ARM_AXIS_STALL_CONFIRMATION_SAMPLES);
      SerialDebug.print(", stall/protect=");
      SerialDebug.print(stallReported ? 1 : 0);
      SerialDebug.print("/");
      SerialDebug.println(stallProtectionActive ? 1 : 0);
      if (axis->stallSampleCount >=
          ARM_AXIS_STALL_CONFIRMATION_SAMPLES) {
        printArmLinearFrame(
            "persistent stall indication", frame);
        faultLinearAxis(
            *axis, "EMM persistent stall indication");
      }
      return;
    }

    if (axis->stallSampleCount != 0U) {
      SerialDebug.print("[EMM STALL] M");
      SerialDebug.print(axis->address);
      SerialDebug.print(" recovered after ");
      SerialDebug.print(axis->stallSampleCount);
      SerialDebug.println(" sample(s)");
      axis->stallSampleCount = 0U;
    }
    if ((flags & 0x02U) == 0U) {
      axis->motionObserved = true;
      return;
    }
    if (elapsedMs >= ARM_AXIS_MINIMUM_ON_POSITION_MS &&
        (axis->commandAcknowledged ||
         axis->motionObserved)) {
      markLinearAxisArrived(*axis);
    }
    return;
  }

  if (frame[1] == 0xFDU && frame[2] == 0x9FU) {
    markLinearAxisArrived(*axis);
  }
}

void serviceArmLinearAxes() {
  while (SerialArmLinear.available()) {
    const uint8_t incoming =
        static_cast<uint8_t>(SerialArmLinear.read());
    if (armLinearReceiveCount < 4U) {
      armLinearReceiveWindow[armLinearReceiveCount++] =
          incoming;
    } else {
      armLinearReceiveWindow[0] = armLinearReceiveWindow[1];
      armLinearReceiveWindow[1] = armLinearReceiveWindow[2];
      armLinearReceiveWindow[2] = armLinearReceiveWindow[3];
      armLinearReceiveWindow[3] = incoming;
    }

    if (armLinearReceiveCount == 4U) {
      handleArmLinearFrame(armLinearReceiveWindow);
      if (incoming == 0x6BU) {
        armLinearReceiveCount = 0U;
      }
    }
  }

  LinearAxisMotion *const axes[2] = {
      &extensionAxis, &liftAxis};
  const uint32_t nowMs = millis();
  for (uint8_t i = 0U; i < 2U; ++i) {
    LinearAxisMotion &axis = *axes[i];
    if (!axis.active) {
      continue;
    }
    if (nowMs - axis.startMs >= axis.timeoutMs) {
      faultLinearAxis(axis, "EMM motion timeout");
      continue;
    }
    if (nowMs - axis.lastStatusRequestMs >=
        ARM_AXIS_STATUS_INTERVAL_MS) {
      axis.lastStatusRequestMs = nowMs;
      writeArmLinearStatusRequest(axis.address);
    }
  }
}

bool startLinearAxisMove(
    LinearAxisMotion &axis,
    float targetMm,
    float minimumMm,
    float maximumMm,
    float pulsesPerMm,
    uint8_t positiveDirection,
    uint8_t negativeDirection,
    uint16_t speedRpm,
    uint8_t acceleration) {
  if (!armLinearReferenceValid || axis.fault) {
    testFault("EMM axis reference invalid or faulted");
    return false;
  }
  if (targetMm < minimumMm - ARM_AXIS_POSITION_TOLERANCE_MM ||
      targetMm > maximumMm + ARM_AXIS_POSITION_TOLERANCE_MM) {
    testFault("Arm target outside calibrated travel");
    return false;
  }

  if (targetMm < minimumMm) {
    targetMm = minimumMm;
  } else if (targetMm > maximumMm) {
    targetMm = maximumMm;
  }

  const float deltaMm = targetMm - axis.currentMm;
  axis.targetMm = targetMm;
  if (fabsf(deltaMm) <= ARM_AXIS_POSITION_TOLERANCE_MM) {
    axis.active = false;
    axis.currentMm = targetMm;
    return true;
  }

  const uint32_t pulses = static_cast<uint32_t>(
      lroundf(fabsf(deltaMm) * pulsesPerMm));
  if (pulses == 0U) {
    axis.active = false;
    axis.currentMm = targetMm;
    return true;
  }

  const float pulsesPerSecond =
      static_cast<float>(speedRpm) *
      FULL_STEPS_PER_REVOLUTION *
      ARM_LINEAR_MICROSTEPS / 60.0f;
  uint32_t estimatedMs = static_cast<uint32_t>(
      static_cast<float>(pulses) /
          pulsesPerSecond * 3000.0f) +
      2000UL;
  if (estimatedMs < ARM_AXIS_MINIMUM_TIMEOUT_MS) {
    estimatedMs = ARM_AXIS_MINIMUM_TIMEOUT_MS;
  } else if (estimatedMs > ARM_AXIS_MAXIMUM_TIMEOUT_MS) {
    estimatedMs = ARM_AXIS_MAXIMUM_TIMEOUT_MS;
  }
  axis.timeoutMs = estimatedMs;

  clearArmLinearReceiveBuffer();
  writeArmLinearEnable(axis.address, true);
  SerialArmLinear.flush();
  delay(ARM_AXIS_COMMAND_GUARD_MS);
  if (!consumeArmLinearEnableResponse(axis)) {
    return false;
  }
  clearArmLinearReceiveBuffer();

  axis.active = true;
  axis.commandAcknowledged = false;
  axis.motionObserved = false;
  axis.stallSampleCount = 0U;
  axis.startMs = millis();
  axis.lastStatusRequestMs = axis.startMs;
  const uint8_t direction =
      deltaMm > 0.0f
          ? positiveDirection
          : negativeDirection;
  writeArmLinearPosition(
      axis.address,
      direction,
      speedRpm,
      acceleration,
      pulses);
  SerialArmLinear.flush();

  SerialDebug.print("[EMM CMD] t=");
  SerialDebug.print(axis.startMs);
  SerialDebug.print(" ms, M");
  SerialDebug.print(axis.address);
  SerialDebug.print(": ");
  SerialDebug.print(axis.currentMm, 2);
  SerialDebug.print(" -> ");
  SerialDebug.print(axis.targetMm, 2);
  SerialDebug.print(" mm, pulses/dir/rpm/acc=");
  SerialDebug.print(pulses);
  SerialDebug.print("/");
  SerialDebug.print(direction);
  SerialDebug.print("/");
  SerialDebug.print(speedRpm);
  SerialDebug.print("/");
  SerialDebug.println(acceleration);
  return true;
}

bool startExtensionToMm(float extensionMm) {
  return startLinearAxisMove(
      extensionAxis,
      extensionMm,
      M6_STANDARD_EXTENSION_MM,
      M6_MAXIMUM_EXTENSION_MM,
      M6_PULSES_PER_MM,
      M6_EXTEND_DIRECTION,
      M6_RETRACT_DIRECTION,
      M6_SPEED_RPM,
      M6_ACCELERATION);
}

bool startLiftToHeightMm(float heightMm) {
  return startLinearAxisMove(
      liftAxis,
      heightMm,
      M7_MINIMUM_HEIGHT_MM,
      M7_STANDARD_HEIGHT_MM,
      M7_PULSES_PER_MM,
      M7_RAISE_DIRECTION,
      M7_LOWER_DIRECTION,
      M7_SPEED_RPM,
      M7_ACCELERATION);
}

bool extensionMoveFinished() {
  return !extensionAxis.active && !extensionAxis.fault;
}

bool liftMoveFinished() {
  return !liftAxis.active && !liftAxis.fault;
}

void emergencyStopArmLinearAxes() {
  if (armLinearSerialInitialized) {
    writeArmLinearStop(extensionAxis.address);
    writeArmLinearStop(liftAxis.address);
    SerialArmLinear.flush();
  }
  extensionAxis.active = false;
  liftAxis.active = false;
}

void resetArmLinearSoftwareOrigin() {
  extensionAxis.currentMm = M6_STANDARD_EXTENSION_MM;
  extensionAxis.targetMm = M6_STANDARD_EXTENSION_MM;
  extensionAxis.active = false;
  extensionAxis.fault = false;
  extensionAxis.commandAcknowledged = false;
  extensionAxis.motionObserved = false;
  extensionAxis.stallSampleCount = 0U;

  liftAxis.currentMm = M7_STANDARD_HEIGHT_MM;
  liftAxis.targetMm = M7_STANDARD_HEIGHT_MM;
  liftAxis.active = false;
  liftAxis.fault = false;
  liftAxis.commandAcknowledged = false;
  liftAxis.motionObserved = false;
  liftAxis.stallSampleCount = 0U;
  armLinearReferenceValid = true;
}

void invalidateArmLinearReference() {
  armLinearReferenceValid = false;
}

// ---------------------------------------------------------------------------
// 舵机
// ---------------------------------------------------------------------------

void commandGripperOpen() {
  gripperServo.setRawAngle(
      GRIPPER_OPEN_ANGLE_DEGREES,
      GRIPPER_INTERVAL_MS,
      GRIPPER_OPEN_POWER_MW);
  SerialDebug.print("[SERVO] gripper(ID4)=OPEN, angle=");
  SerialDebug.println(GRIPPER_OPEN_ANGLE_DEGREES, 1);
}

void commandGripperClose() {
  gripperServo.setRawAngle(
      GRIPPER_CLOSE_ANGLE_DEGREES,
      GRIPPER_INTERVAL_MS,
      GRIPPER_CLOSE_POWER_MW);
  SerialDebug.print("[SERVO] gripper(ID4)=CLOSE, angle=");
  SerialDebug.println(GRIPPER_CLOSE_ANGLE_DEGREES, 1);
}

void commandStorageServoPosition(uint8_t positionIndex) {
  if (positionIndex >= 3U) {
    testFault("Invalid storage servo position");
    return;
  }
  storageServo.setRawAngleMTurn(
      STORAGE_SERVO_POSITIONS_DEGREES[positionIndex],
      STORAGE_SERVO_INTERVAL_MS);
  SerialDebug.print("[SERVO] storage(ID5) slot=");
  SerialDebug.print(positionIndex);
  SerialDebug.print(", angle=");
  SerialDebug.println(
      STORAGE_SERVO_POSITIONS_DEGREES[positionIndex], 1);
}

void commandStorageServoParkingPosition() {
  storageServo.setRawAngleMTurn(
      STORAGE_SERVO_PARK_ANGLE_DEGREES,
      STORAGE_SERVO_INTERVAL_MS);
  SerialDebug.print("[SERVO] storage(ID5)=PARK, angle=");
  SerialDebug.println(STORAGE_SERVO_PARK_ANGLE_DEGREES, 1);
}

bool verifyManipulationServosOnline() {
  const bool gripperOnline = gripperServo.ping();
  const bool storageOnline = storageServo.ping();
  SerialDebug.print("[SERVO PING] gripper/storage=");
  SerialDebug.print(gripperOnline ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.println(storageOnline ? 1 : 0);
  return gripperOnline && storageOnline;
}

// ---------------------------------------------------------------------------
// MaixCAM：只保留AA 09 BB霍夫圆模式
// ---------------------------------------------------------------------------

struct MaixCoordinate {
  uint8_t resultId;
  int16_t x;
  int16_t y;
  uint32_t sequence;
  uint32_t receivedMs;

  MaixCoordinate()
      : resultId(0U),
        x(0),
        y(0),
        sequence(0UL),
        receivedMs(0UL) {}
};

bool maixcamSerialInitialized = false;
uint8_t maixRequestedMode = MAIXCAM_STOP_REQUEST;
bool maixModeCommandSent = false;
uint32_t maixModeSwitchStartMs = 0UL;
uint32_t maixLastRequestMs = 0UL;
char maixReceiveLine[MAIXCAM_LINE_CAPACITY] = {0};
size_t maixReceiveLength = 0U;
MaixCoordinate latestMaixCoordinate;
uint32_t calibrationLastMaixSequence = 0UL;

void writeMaixRequestFrame(uint8_t request) {
  const uint8_t frame[3] = {
      MAIXCAM_FRAME_HEADER, request, MAIXCAM_FRAME_TAIL};
  const size_t queuedBytes =
      SerialMaixcam.write(frame, sizeof(frame));
  SerialDebug.print("[MAIX TX] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, frame=AA ");
  if (request < 0x10U) {
    SerialDebug.print('0');
  }
  SerialDebug.print(request, HEX);
  SerialDebug.print(" BB, queued=");
  SerialDebug.println(
      static_cast<unsigned int>(queuedBytes));
}

void stopMaixRequest() {
  if (maixcamSerialInitialized) {
    writeMaixRequestFrame(MAIXCAM_STOP_REQUEST);
  }
  maixRequestedMode = MAIXCAM_STOP_REQUEST;
  maixModeCommandSent = false;
  maixReceiveLength = 0U;
}

void beginCircleVision() {
  stopMaixRequest();
  while (SerialMaixcam.available()) {
    SerialMaixcam.read();
  }
  maixRequestedMode = MAIXCAM_HOUGH_CIRCLE_REQUEST;
  maixModeSwitchStartMs = millis();
  maixLastRequestMs = 0UL;
  maixModeCommandSent = false;
  calibrationLastMaixSequence =
      latestMaixCoordinate.sequence;

  SerialDebug.print("[MAIX PLAN] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, mode=9, TX after ");
  SerialDebug.print(MAIXCAM_MODE_SWITCH_GUARD_MS);
  SerialDebug.println(" ms");
}

bool parseMaixCoordinateLine(
    const char *line,
    uint8_t &resultId,
    int16_t &x,
    int16_t &y) {
  if (line == nullptr || line[0] == '\0') {
    return false;
  }

  char *idEnd = nullptr;
  const long parsedId = strtol(line, &idEnd, 10);
  if (idEnd == line || idEnd == nullptr || *idEnd != ',') {
    return false;
  }
  const char *xStart = idEnd + 1;
  char *xEnd = nullptr;
  const long parsedX = strtol(xStart, &xEnd, 10);
  if (xEnd == xStart || xEnd == nullptr || *xEnd != ',') {
    return false;
  }
  const char *yStart = xEnd + 1;
  char *yEnd = nullptr;
  const long parsedY = strtol(yStart, &yEnd, 10);
  if (yEnd == yStart || yEnd == nullptr || *yEnd != '\0') {
    return false;
  }
  if (parsedId != MAIXCAM_HOUGH_CIRCLE_REQUEST ||
      parsedX < 0L || parsedX > IMAGE_MAX_X ||
      parsedY < 0L || parsedY > IMAGE_MAX_Y) {
    return false;
  }

  resultId = static_cast<uint8_t>(parsedId);
  x = static_cast<int16_t>(parsedX);
  y = static_cast<int16_t>(parsedY);
  return true;
}

void finishMaixCoordinateLine() {
  maixReceiveLine[maixReceiveLength] = '\0';
  uint8_t resultId = 0U;
  int16_t x = 0;
  int16_t y = 0;
  if (maixModeCommandSent &&
      maixRequestedMode == MAIXCAM_HOUGH_CIRCLE_REQUEST &&
      parseMaixCoordinateLine(
          maixReceiveLine, resultId, x, y)) {
    latestMaixCoordinate.resultId = resultId;
    latestMaixCoordinate.x = x;
    latestMaixCoordinate.y = y;
    ++latestMaixCoordinate.sequence;
    latestMaixCoordinate.receivedMs = millis();
    SerialDebug.print("[MAIX RX] result=");
    SerialDebug.print(resultId);
    SerialDebug.print(", x/y=");
    SerialDebug.print(x);
    SerialDebug.print("/");
    SerialDebug.println(y);
  }
  maixReceiveLength = 0U;
}

void serviceMaixcam() {
  while (SerialMaixcam.available()) {
    const char incoming =
        static_cast<char>(SerialMaixcam.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      finishMaixCoordinateLine();
      continue;
    }
    if (maixReceiveLength < MAIXCAM_LINE_CAPACITY - 1U) {
      maixReceiveLine[maixReceiveLength++] = incoming;
    } else {
      maixReceiveLength = 0U;
    }
  }

  if (maixRequestedMode == MAIXCAM_STOP_REQUEST) {
    return;
  }
  const uint32_t nowMs = millis();
  if (!maixModeCommandSent) {
    if (nowMs - maixModeSwitchStartMs <
        MAIXCAM_MODE_SWITCH_GUARD_MS) {
      return;
    }
    writeMaixRequestFrame(maixRequestedMode);
    maixModeCommandSent = true;
    maixLastRequestMs = nowMs;
    return;
  }
  if (nowMs - maixLastRequestMs >=
      MAIXCAM_REQUEST_REPEAT_MS) {
    writeMaixRequestFrame(maixRequestedMode);
    maixLastRequestMs = nowMs;
  }
}

bool readNewMaixCoordinate(
    uint8_t &resultId,
    int16_t &x,
    int16_t &y) {
  if (latestMaixCoordinate.sequence ==
          calibrationLastMaixSequence ||
      millis() - latestMaixCoordinate.receivedMs >
          MAIXCAM_COORDINATE_STALE_MS) {
    return false;
  }
  calibrationLastMaixSequence =
      latestMaixCoordinate.sequence;
  resultId = latestMaixCoordinate.resultId;
  x = latestMaixCoordinate.x;
  y = latestMaixCoordinate.y;
  return true;
}

// ---------------------------------------------------------------------------
// 机械臂姿态、标准化和完整取放动作
// ---------------------------------------------------------------------------

struct ArmPose {
  float standardFrameAngleDegrees;
  float extensionMm;
  float heightMm;

  ArmPose(
      float angleDegrees = 0.0f,
      float extension = 0.0f,
      float height = 0.0f)
      : standardFrameAngleDegrees(angleDegrees),
        extensionMm(extension),
        heightMm(height) {}
};

bool armPoseIsValid(const ArmPose &pose) {
  return pose.extensionMm >=
             M6_STANDARD_EXTENSION_MM -
                 ARM_AXIS_POSITION_TOLERANCE_MM &&
         pose.extensionMm <=
             M6_MAXIMUM_EXTENSION_MM +
                 ARM_AXIS_POSITION_TOLERANCE_MM &&
         pose.heightMm >=
             M7_MINIMUM_HEIGHT_MM -
                 ARM_AXIS_POSITION_TOLERANCE_MM &&
         pose.heightMm <=
             M7_STANDARD_HEIGHT_MM +
                 ARM_AXIS_POSITION_TOLERANCE_MM;
}

enum ArmStandardPhase {
  ARM_STANDARD_IDLE,
  ARM_STANDARD_WAIT_OPEN,
  ARM_STANDARD_WAIT_LIFT,
  ARM_STANDARD_WAIT_RETRACT,
  ARM_STANDARD_WAIT_BASE,
  ARM_STANDARD_WAIT_BASE_SETTLE,
  ARM_STANDARD_COMPLETE
};

ArmStandardPhase armStandardPhase = ARM_STANDARD_IDLE;
uint32_t armStandardDeadlineMs = 0UL;

const char *armStandardPhaseName(ArmStandardPhase phase) {
  switch (phase) {
    case ARM_STANDARD_IDLE:
      return "IDLE";
    case ARM_STANDARD_WAIT_OPEN:
      return "WAIT_OPEN";
    case ARM_STANDARD_WAIT_LIFT:
      return "WAIT_M7_TOP";
    case ARM_STANDARD_WAIT_RETRACT:
      return "WAIT_M6_RETRACT";
    case ARM_STANDARD_WAIT_BASE:
      return "WAIT_M5_STANDARD";
    case ARM_STANDARD_WAIT_BASE_SETTLE:
      return "WAIT_M5_SETTLE";
    case ARM_STANDARD_COMPLETE:
      return "COMPLETE";
  }
  return "UNKNOWN";
}

void setArmStandardPhase(ArmStandardPhase nextPhase) {
  if (armStandardPhase == nextPhase) {
    return;
  }
  SerialDebug.print("[STATE] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, ARM_STD: ");
  SerialDebug.print(armStandardPhaseName(armStandardPhase));
  SerialDebug.print(" -> ");
  SerialDebug.println(armStandardPhaseName(nextPhase));
  armStandardPhase = nextPhase;
}

void beginArmStandardization() {
  SerialDebug.print("[ARM STD] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, M6/M7 current=");
  SerialDebug.print(extensionAxis.currentMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(liftAxis.currentMm, 2);
  SerialDebug.println(" mm");
  commandGripperOpen();
  armStandardDeadlineMs =
      millis() + GRIPPER_OPEN_SETTLE_MS;
  setArmStandardPhase(ARM_STANDARD_WAIT_OPEN);
}

bool serviceArmStandardization() {
  switch (armStandardPhase) {
    case ARM_STANDARD_IDLE:
      return false;

    case ARM_STANDARD_WAIT_OPEN:
      if (deadlineReached(armStandardDeadlineMs) &&
          startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)) {
        setArmStandardPhase(ARM_STANDARD_WAIT_LIFT);
      }
      break;

    case ARM_STANDARD_WAIT_LIFT:
      if (liftMoveFinished() &&
          startExtensionToMm(M6_STANDARD_EXTENSION_MM)) {
        setArmStandardPhase(ARM_STANDARD_WAIT_RETRACT);
      }
      break;

    case ARM_STANDARD_WAIT_RETRACT:
      if (extensionMoveFinished()) {
        startArmBaseStandardFrameDegrees(0.0f);
        setArmStandardPhase(ARM_STANDARD_WAIT_BASE);
      }
      break;

    case ARM_STANDARD_WAIT_BASE:
      if (!armMotors.isM5Running()) {
        armStandardDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        setArmStandardPhase(
            ARM_STANDARD_WAIT_BASE_SETTLE);
      }
      break;

    case ARM_STANDARD_WAIT_BASE_SETTLE:
      if (deadlineReached(armStandardDeadlineMs)) {
        setArmStandardPhase(ARM_STANDARD_COMPLETE);
      }
      break;

    case ARM_STANDARD_COMPLETE:
      return true;
  }
  return armStandardPhase == ARM_STANDARD_COMPLETE;
}

enum ArmTransferPhase {
  ARM_TRANSFER_IDLE,
  ARM_TRANSFER_WAIT_SOURCE_OPEN,
  ARM_TRANSFER_WAIT_PREPARE_LIFT,
  ARM_TRANSFER_WAIT_PREPARE_RETRACT,
  ARM_TRANSFER_WAIT_SOURCE_ROTATION,
  ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE,
  ARM_TRANSFER_WAIT_SOURCE_EXTENSION,
  ARM_TRANSFER_WAIT_SOURCE_LOWER,
  ARM_TRANSFER_WAIT_GRIP_CLOSE,
  ARM_TRANSFER_WAIT_LOADED_LIFT,
  ARM_TRANSFER_WAIT_LOADED_RETRACT,
  ARM_TRANSFER_WAIT_DESTINATION_ROTATION,
  ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE,
  ARM_TRANSFER_WAIT_DESTINATION_EXTENSION,
  ARM_TRANSFER_WAIT_DESTINATION_LOWER,
  ARM_TRANSFER_WAIT_DESTINATION_OPEN,
  ARM_TRANSFER_WAIT_FINAL_LIFT,
  ARM_TRANSFER_WAIT_FINAL_RETRACT,
  ARM_TRANSFER_WAIT_STANDARD_ROTATION,
  ARM_TRANSFER_WAIT_STANDARD_SETTLE,
  ARM_TRANSFER_COMPLETE
};

ArmTransferPhase armTransferPhase = ARM_TRANSFER_IDLE;
ArmPose armTransferSourcePose;
ArmPose armTransferDestinationPose;
uint32_t armTransferDeadlineMs = 0UL;
uint32_t armTransferStartMs = 0UL;

const char *armTransferPhaseName(ArmTransferPhase phase) {
  switch (phase) {
    case ARM_TRANSFER_IDLE:
      return "IDLE";
    case ARM_TRANSFER_WAIT_SOURCE_OPEN:
      return "WAIT_SOURCE_OPEN";
    case ARM_TRANSFER_WAIT_PREPARE_LIFT:
      return "WAIT_PREPARE_LIFT";
    case ARM_TRANSFER_WAIT_PREPARE_RETRACT:
      return "WAIT_PREPARE_RETRACT";
    case ARM_TRANSFER_WAIT_SOURCE_ROTATION:
      return "WAIT_SOURCE_ROTATION";
    case ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE:
      return "WAIT_SOURCE_ROTATION_SETTLE";
    case ARM_TRANSFER_WAIT_SOURCE_EXTENSION:
      return "WAIT_SOURCE_EXTENSION";
    case ARM_TRANSFER_WAIT_SOURCE_LOWER:
      return "WAIT_SOURCE_LOWER";
    case ARM_TRANSFER_WAIT_GRIP_CLOSE:
      return "WAIT_GRIP_CLOSE";
    case ARM_TRANSFER_WAIT_LOADED_LIFT:
      return "WAIT_LOADED_LIFT";
    case ARM_TRANSFER_WAIT_LOADED_RETRACT:
      return "WAIT_LOADED_RETRACT";
    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION:
      return "WAIT_DEST_ROTATION";
    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE:
      return "WAIT_DEST_ROTATION_SETTLE";
    case ARM_TRANSFER_WAIT_DESTINATION_EXTENSION:
      return "WAIT_DEST_EXTENSION";
    case ARM_TRANSFER_WAIT_DESTINATION_LOWER:
      return "WAIT_DEST_LOWER";
    case ARM_TRANSFER_WAIT_DESTINATION_OPEN:
      return "WAIT_DEST_OPEN";
    case ARM_TRANSFER_WAIT_FINAL_LIFT:
      return "WAIT_FINAL_LIFT";
    case ARM_TRANSFER_WAIT_FINAL_RETRACT:
      return "WAIT_FINAL_RETRACT";
    case ARM_TRANSFER_WAIT_STANDARD_ROTATION:
      return "WAIT_STANDARD_ROTATION";
    case ARM_TRANSFER_WAIT_STANDARD_SETTLE:
      return "WAIT_STANDARD_SETTLE";
    case ARM_TRANSFER_COMPLETE:
      return "COMPLETE";
  }
  return "UNKNOWN";
}

void setArmTransferPhase(ArmTransferPhase nextPhase) {
  if (armTransferPhase == nextPhase) {
    return;
  }
  SerialDebug.print("[STATE] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, ARM_TRANSFER: ");
  SerialDebug.print(armTransferPhaseName(armTransferPhase));
  SerialDebug.print(" -> ");
  SerialDebug.println(armTransferPhaseName(nextPhase));
  armTransferPhase = nextPhase;
}

void beginArmTransfer(
    const ArmPose &source,
    const ArmPose &destination) {
  if (!armPoseIsValid(source) ||
      !armPoseIsValid(destination)) {
    testFault("Transfer pose outside arm travel");
    return;
  }

  armTransferSourcePose = source;
  armTransferDestinationPose = destination;
  armTransferStartMs = millis();
  commandGripperOpen();
  armTransferDeadlineMs =
      millis() + GRIPPER_OPEN_SETTLE_MS;

  SerialDebug.print("[ARM TRANSFER] item=");
  SerialDebug.print(
      static_cast<unsigned int>(testItemIndex + 1U));
  SerialDebug.print(", source angle/ext/height=");
  SerialDebug.print(source.standardFrameAngleDegrees, 3);
  SerialDebug.print("/");
  SerialDebug.print(source.extensionMm, 3);
  SerialDebug.print("/");
  SerialDebug.println(source.heightMm, 3);
  SerialDebug.print(
      "[ARM TRANSFER] destination angle/ext/height=");
  SerialDebug.print(destination.standardFrameAngleDegrees, 3);
  SerialDebug.print("/");
  SerialDebug.print(destination.extensionMm, 3);
  SerialDebug.print("/");
  SerialDebug.println(destination.heightMm, 3);

  setArmTransferPhase(ARM_TRANSFER_WAIT_SOURCE_OPEN);
}

void serviceArmTransfer() {
  switch (armTransferPhase) {
    case ARM_TRANSFER_IDLE:
    case ARM_TRANSFER_COMPLETE:
      return;

    case ARM_TRANSFER_WAIT_SOURCE_OPEN:
      if (deadlineReached(armTransferDeadlineMs) &&
          startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_PREPARE_LIFT);
      }
      break;

    case ARM_TRANSFER_WAIT_PREPARE_LIFT:
      if (liftMoveFinished() &&
          startExtensionToMm(M6_STANDARD_EXTENSION_MM)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_PREPARE_RETRACT);
      }
      break;

    case ARM_TRANSFER_WAIT_PREPARE_RETRACT:
      if (extensionMoveFinished()) {
        startArmBaseStandardFrameDegrees(
            armTransferSourcePose
                .standardFrameAngleDegrees);
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_SOURCE_ROTATION);
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_ROTATION:
      if (!armMotors.isM5Running()) {
        armTransferDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE);
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE:
      if (deadlineReached(armTransferDeadlineMs) &&
          startExtensionToMm(
              armTransferSourcePose.extensionMm)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_SOURCE_EXTENSION);
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_EXTENSION:
      if (extensionMoveFinished() &&
          startLiftToHeightMm(
              armTransferSourcePose.heightMm)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_SOURCE_LOWER);
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_LOWER:
      if (liftMoveFinished()) {
        commandGripperClose();
        armTransferDeadlineMs =
            millis() + GRIPPER_CLOSE_SETTLE_MS;
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_GRIP_CLOSE);
      }
      break;

    case ARM_TRANSFER_WAIT_GRIP_CLOSE:
      if (deadlineReached(armTransferDeadlineMs) &&
          startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_LOADED_LIFT);
      }
      break;

    case ARM_TRANSFER_WAIT_LOADED_LIFT:
      if (liftMoveFinished() &&
          startExtensionToMm(M6_STANDARD_EXTENSION_MM)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_LOADED_RETRACT);
      }
      break;

    case ARM_TRANSFER_WAIT_LOADED_RETRACT:
      if (extensionMoveFinished()) {
        startArmBaseStandardFrameDegrees(
            armTransferDestinationPose
                .standardFrameAngleDegrees);
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_DESTINATION_ROTATION);
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION:
      if (!armMotors.isM5Running()) {
        armTransferDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE);
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE:
      if (deadlineReached(armTransferDeadlineMs) &&
          startExtensionToMm(
              armTransferDestinationPose.extensionMm)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_DESTINATION_EXTENSION);
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_EXTENSION:
      if (extensionMoveFinished() &&
          startLiftToHeightMm(
              armTransferDestinationPose.heightMm)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_DESTINATION_LOWER);
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_LOWER:
      if (liftMoveFinished()) {
        commandGripperOpen();
        armTransferDeadlineMs =
            millis() + GRIPPER_OPEN_SETTLE_MS;
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_DESTINATION_OPEN);
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_OPEN:
      if (deadlineReached(armTransferDeadlineMs) &&
          startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_FINAL_LIFT);
      }
      break;

    case ARM_TRANSFER_WAIT_FINAL_LIFT:
      if (liftMoveFinished() &&
          startExtensionToMm(M6_STANDARD_EXTENSION_MM)) {
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_FINAL_RETRACT);
      }
      break;

    case ARM_TRANSFER_WAIT_FINAL_RETRACT:
      if (extensionMoveFinished()) {
        startArmBaseStandardFrameDegrees(0.0f);
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_STANDARD_ROTATION);
      }
      break;

    case ARM_TRANSFER_WAIT_STANDARD_ROTATION:
      if (!armMotors.isM5Running()) {
        armTransferDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        setArmTransferPhase(
            ARM_TRANSFER_WAIT_STANDARD_SETTLE);
      }
      break;

    case ARM_TRANSFER_WAIT_STANDARD_SETTLE:
      if (deadlineReached(armTransferDeadlineMs)) {
        SerialDebug.print("[ARM TRANSFER] COMPLETE, elapsed=");
        SerialDebug.print(millis() - armTransferStartMs);
        SerialDebug.println(" ms");
        setArmTransferPhase(ARM_TRANSFER_COMPLETE);
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// 暂存区1/3号圈闭环校准与2号圈坐标计算
// ---------------------------------------------------------------------------

ArmPose calibratedRingHorizontalPoses[3];
bool calibratedRingPosesValid = false;

enum RingCalibrationPhase {
  RING_CALIBRATION_IDLE,
  RING_CALIBRATION_WAIT_BASE,
  RING_CALIBRATION_WAIT_BASE_SETTLE,
  RING_CALIBRATION_WAIT_EXTENSION,
  RING_CALIBRATION_WAIT_LOWER,
  RING_CALIBRATION_WAIT_COORDINATE,
  RING_CALIBRATION_WAIT_CORRECTION_BASE,
  RING_CALIBRATION_WAIT_CORRECTION_BASE_SETTLE,
  RING_CALIBRATION_WAIT_CORRECTION_EXTENSION,
  RING_CALIBRATION_WAIT_SWITCH_LIFT,
  RING_CALIBRATION_WAIT_SWITCH_RETRACT,
  RING_CALIBRATION_COMPLETE
};

RingCalibrationPhase ringCalibrationPhase =
    RING_CALIBRATION_IDLE;
uint8_t calibratingRingPosition = 0U;
ArmPose ringCalibrationCurrentPose;
ArmPose ringCalibrationTargetPose;
uint32_t ringCalibrationDeadlineMs = 0UL;
uint32_t circleStableStartMs = 0UL;
uint32_t circleLastStableSampleMs = 0UL;
uint8_t circleStableSampleCount = 0U;

const char *ringCalibrationPhaseName(
    RingCalibrationPhase phase) {
  switch (phase) {
    case RING_CALIBRATION_IDLE:
      return "IDLE";
    case RING_CALIBRATION_WAIT_BASE:
      return "WAIT_BASE";
    case RING_CALIBRATION_WAIT_BASE_SETTLE:
      return "WAIT_BASE_SETTLE";
    case RING_CALIBRATION_WAIT_EXTENSION:
      return "WAIT_EXTENSION";
    case RING_CALIBRATION_WAIT_LOWER:
      return "WAIT_LOWER";
    case RING_CALIBRATION_WAIT_COORDINATE:
      return "WAIT_COORDINATE";
    case RING_CALIBRATION_WAIT_CORRECTION_BASE:
      return "WAIT_CORRECTION_BASE";
    case RING_CALIBRATION_WAIT_CORRECTION_BASE_SETTLE:
      return "WAIT_CORRECTION_BASE_SETTLE";
    case RING_CALIBRATION_WAIT_CORRECTION_EXTENSION:
      return "WAIT_CORRECTION_EXTENSION";
    case RING_CALIBRATION_WAIT_SWITCH_LIFT:
      return "WAIT_SWITCH_LIFT";
    case RING_CALIBRATION_WAIT_SWITCH_RETRACT:
      return "WAIT_SWITCH_RETRACT";
    case RING_CALIBRATION_COMPLETE:
      return "COMPLETE";
  }
  return "UNKNOWN";
}

void setRingCalibrationPhase(
    RingCalibrationPhase nextPhase) {
  if (ringCalibrationPhase == nextPhase) {
    return;
  }
  SerialDebug.print("[STATE] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, RING_CAL: ");
  SerialDebug.print(
      ringCalibrationPhaseName(ringCalibrationPhase));
  SerialDebug.print(" -> ");
  SerialDebug.println(
      ringCalibrationPhaseName(nextPhase));
  ringCalibrationPhase = nextPhase;
}

void resetCircleStabilityWindow() {
  circleStableStartMs = 0UL;
  circleLastStableSampleMs = 0UL;
  circleStableSampleCount = 0U;
}

void resetRingCalibration() {
  stopMaixRequest();
  setRingCalibrationPhase(RING_CALIBRATION_IDLE);
  calibratingRingPosition = 0U;
  ringCalibrationCurrentPose = ArmPose();
  ringCalibrationTargetPose = ArmPose();
  calibratedRingPosesValid = false;
  for (uint8_t i = 0U; i < 3U; ++i) {
    calibratedRingHorizontalPoses[i] = ArmPose();
  }
  resetCircleStabilityWindow();
}

bool nominalRingHorizontalPose(
    uint8_t ringPosition,
    ArmPose &pose) {
  if (ringPosition < 1U || ringPosition > 3U) {
    testFault("Invalid ring position");
    return false;
  }

  float imageLeftMm = 0.0f;
  if (ringPosition == 1U) {
    imageLeftMm = SIDE_RING_CENTER_OFFSET_MM;
  } else if (ringPosition == 3U) {
    imageLeftMm = -SIDE_RING_CENTER_OFFSET_MM;
  }
  const float radialDistanceMm =
      hypotf(
          ARM_PIVOT_TO_CAMERA_CENTER_MM,
          imageLeftMm);
  pose.standardFrameAngleDegrees =
      atan2f(
          imageLeftMm,
          ARM_PIVOT_TO_CAMERA_CENTER_MM) *
      180.0f / PI_F;
  pose.extensionMm =
      radialDistanceMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;
  pose.heightMm = M7_STANDARD_HEIGHT_MM;
  return true;
}

bool startRingCalibrationApproach(
    uint8_t ringPosition) {
  ArmPose nominalPose;
  if (!nominalRingHorizontalPose(
          ringPosition, nominalPose)) {
    return false;
  }

  calibratingRingPosition = ringPosition;
  ringCalibrationTargetPose = nominalPose;
  SerialDebug.print("[RING CAL] approach ring ");
  SerialDebug.print(
      static_cast<unsigned int>(ringPosition));
  SerialDebug.print(", nominal angle/ext=");
  SerialDebug.print(
      nominalPose.standardFrameAngleDegrees, 4);
  SerialDebug.print("/");
  SerialDebug.println(nominalPose.extensionMm, 4);
  startArmBaseStandardFrameDegrees(
      nominalPose.standardFrameAngleDegrees);
  setRingCalibrationPhase(
      RING_CALIBRATION_WAIT_BASE);
  return true;
}

bool beginRingCalibration() {
  resetRingCalibration();
  SerialDebug.println(
      "[RING CAL] chassis disabled/fixed; calibrate ring 1 then ring 3");
  return startRingCalibrationApproach(1U);
}

bool calculateArmCircleCorrection(
    int16_t imageX,
    int16_t imageY,
    ArmPose &targetPose) {
  const float pixelDeltaX =
      static_cast<float>(imageX - IMAGE_CENTER_X);
  const float outwardCorrectionMm =
      static_cast<float>(imageY - IMAGE_CENTER_Y) *
      CIRCLE_MM_PER_PIXEL *
      IMAGE_Y_TO_ARM_OUTWARD_SIGN;
  const float currentRadiusMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      ringCalibrationCurrentPose.extensionMm;
  const float correctedLocalRadialMm =
      currentRadiusMm + outwardCorrectionMm;
  const float correctedLocalClockwiseMm =
      pixelDeltaX * CIRCLE_MM_PER_PIXEL;

  if (correctedLocalRadialMm <= 0.0f) {
    testFault("Circle correction crossed arm pivot");
    return false;
  }

  const float targetRadiusMm =
      hypotf(
          correctedLocalRadialMm,
          correctedLocalClockwiseMm);
  targetPose.standardFrameAngleDegrees =
      ringCalibrationCurrentPose
          .standardFrameAngleDegrees -
      atan2f(
          correctedLocalClockwiseMm,
          correctedLocalRadialMm) *
          180.0f / PI_F;
  targetPose.extensionMm =
      targetRadiusMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;
  targetPose.heightMm = HOUGH_VISION_HEIGHT_MM;

  if (!armPoseIsValid(targetPose)) {
    testFault("Circle correction outside arm reach");
    return false;
  }
  return true;
}

void recordCalibratedSideRing() {
  ArmPose recordedPose = ringCalibrationCurrentPose;
  recordedPose.heightMm = M7_STANDARD_HEIGHT_MM;
  calibratedRingHorizontalPoses[
      calibratingRingPosition - 1U] = recordedPose;
  SerialDebug.print("[RING CAL] ring ");
  SerialDebug.print(
      static_cast<unsigned int>(calibratingRingPosition));
  SerialDebug.print(
      " centered; recorded angle/ext=");
  SerialDebug.print(
      recordedPose.standardFrameAngleDegrees, 4);
  SerialDebug.print("/");
  SerialDebug.println(recordedPose.extensionMm, 4);
}

bool calculateCalibratedMiddleRing() {
  const ArmPose &ring1 =
      calibratedRingHorizontalPoses[0];
  const ArmPose &ring3 =
      calibratedRingHorizontalPoses[2];
  const float ring1AngleRadians =
      ring1.standardFrameAngleDegrees * PI_F / 180.0f;
  const float ring3AngleRadians =
      ring3.standardFrameAngleDegrees * PI_F / 180.0f;
  const float ring1RadiusMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      ring1.extensionMm;
  const float ring3RadiusMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      ring3.extensionMm;

  const float middleLeftMm =
      (ring1RadiusMm * sinf(ring1AngleRadians) +
       ring3RadiusMm * sinf(ring3AngleRadians)) *
      0.5f;
  const float middleOutwardMm =
      (ring1RadiusMm * cosf(ring1AngleRadians) +
       ring3RadiusMm * cosf(ring3AngleRadians)) *
      0.5f;
  const float middleRadiusMm =
      hypotf(middleLeftMm, middleOutwardMm);

  ArmPose middlePose;
  middlePose.standardFrameAngleDegrees =
      atan2f(middleLeftMm, middleOutwardMm) *
      180.0f / PI_F;
  middlePose.extensionMm =
      middleRadiusMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;
  middlePose.heightMm = M7_STANDARD_HEIGHT_MM;
  if (!armPoseIsValid(middlePose)) {
    testFault(
        "Calculated middle ring outside arm reach");
    return false;
  }

  calibratedRingHorizontalPoses[1] = middlePose;
  calibratedRingPosesValid = true;
  SerialDebug.print(
      "[RING CAL] ring 2 cylindrical midpoint angle/ext=");
  SerialDebug.print(
      middlePose.standardFrameAngleDegrees, 4);
  SerialDebug.print("/");
  SerialDebug.println(middlePose.extensionMm, 4);
  return true;
}

bool finishCenteredRingCalibration() {
  stopMaixRequest();
  resetCircleStabilityWindow();
  recordCalibratedSideRing();

  if (calibratingRingPosition == 1U) {
    SerialDebug.println(
        "[RING CAL] ring 1 done -> M7 top, M6 retract, then ring 3");
    if (startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)) {
      setRingCalibrationPhase(
          RING_CALIBRATION_WAIT_SWITCH_LIFT);
      return true;
    }
    return false;
  }

  if (calibratingRingPosition != 3U) {
    testFault("Unexpected calibrated ring number");
    return false;
  }
  if (!calculateCalibratedMiddleRing()) {
    return false;
  }
  setRingCalibrationPhase(RING_CALIBRATION_COMPLETE);
  return true;
}

bool serviceRingCalibration() {
  switch (ringCalibrationPhase) {
    case RING_CALIBRATION_IDLE:
      return false;

    case RING_CALIBRATION_WAIT_BASE:
      if (!armMotors.isM5Running()) {
        ringCalibrationDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_BASE_SETTLE);
      }
      break;

    case RING_CALIBRATION_WAIT_BASE_SETTLE:
      if (deadlineReached(ringCalibrationDeadlineMs) &&
          startExtensionToMm(
              ringCalibrationTargetPose.extensionMm)) {
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_EXTENSION);
      }
      break;

    case RING_CALIBRATION_WAIT_EXTENSION:
      if (extensionMoveFinished()) {
        ringCalibrationCurrentPose =
            ringCalibrationTargetPose;
        ringCalibrationCurrentPose.heightMm =
            M7_STANDARD_HEIGHT_MM;
        if (startLiftToHeightMm(
                HOUGH_VISION_HEIGHT_MM)) {
          setRingCalibrationPhase(
              RING_CALIBRATION_WAIT_LOWER);
        }
      }
      break;

    case RING_CALIBRATION_WAIT_LOWER:
      if (liftMoveFinished()) {
        ringCalibrationCurrentPose.heightMm =
            HOUGH_VISION_HEIGHT_MM;
        SerialDebug.print("[RING CAL] ring ");
        SerialDebug.print(
            static_cast<unsigned int>(
                calibratingRingPosition));
        SerialDebug.print(" M7=");
        SerialDebug.print(HOUGH_VISION_HEIGHT_MM, 2);
        SerialDebug.println(
            " mm reached; now request AA 09 BB");
        beginCircleVision();
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_COORDINATE);
      }
      break;

    case RING_CALIBRATION_WAIT_COORDINATE: {
      uint8_t resultId = 0U;
      int16_t x = 0;
      int16_t y = 0;
      if (!readNewMaixCoordinate(resultId, x, y)) {
        break;
      }
      (void)resultId;

      const int32_t dx =
          static_cast<int32_t>(x - IMAGE_CENTER_X);
      const int32_t dy =
          static_cast<int32_t>(y - IMAGE_CENTER_Y);
      const float pixelRadius =
          hypotf(
              static_cast<float>(dx),
              static_cast<float>(dy));
      SerialDebug.print("[RING CAL] ring/circle/error=");
      SerialDebug.print(
          static_cast<unsigned int>(
              calibratingRingPosition));
      SerialDebug.print("/");
      SerialDebug.print(x);
      SerialDebug.print(",");
      SerialDebug.print(y);
      SerialDebug.print("/");
      SerialDebug.print(pixelRadius, 3);
      SerialDebug.println(" px");

      if (pixelRadius >
          CIRCLE_CENTER_TOLERANCE_PIXELS) {
        resetCircleStabilityWindow();
        stopMaixRequest();
        if (!calculateArmCircleCorrection(
                x, y, ringCalibrationTargetPose)) {
          break;
        }
        SerialDebug.print(
            "[RING CAL] correction target M5/M6=");
        SerialDebug.print(
            ringCalibrationTargetPose
                .standardFrameAngleDegrees,
            4);
        SerialDebug.print("/");
        SerialDebug.println(
            ringCalibrationTargetPose.extensionMm, 4);
        startArmBaseStandardFrameDegrees(
            ringCalibrationTargetPose
                .standardFrameAngleDegrees);
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_CORRECTION_BASE);
        break;
      }

      if (circleStableSampleCount == 0U) {
        circleStableStartMs = millis();
        circleLastStableSampleMs =
            circleStableStartMs;
      } else if (
          millis() - circleLastStableSampleMs >
          VISION_STABILITY_MAXIMUM_SAMPLE_GAP_MS) {
        circleStableStartMs = millis();
        circleStableSampleCount = 0U;
      }
      circleLastStableSampleMs = millis();
      if (circleStableSampleCount < UINT8_MAX) {
        ++circleStableSampleCount;
      }
      SerialDebug.print("[RING CAL] stable sample/time=");
      SerialDebug.print(circleStableSampleCount);
      SerialDebug.print("/");
      SerialDebug.print(millis() - circleStableStartMs);
      SerialDebug.println(" ms");
      if (circleStableSampleCount >=
              CIRCLE_STABILITY_MINIMUM_SAMPLES &&
          millis() - circleStableStartMs >=
              CIRCLE_CENTER_STABILITY_MS) {
        finishCenteredRingCalibration();
      }
      break;
    }

    case RING_CALIBRATION_WAIT_CORRECTION_BASE:
      if (!armMotors.isM5Running()) {
        ringCalibrationDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_CORRECTION_BASE_SETTLE);
      }
      break;

    case RING_CALIBRATION_WAIT_CORRECTION_BASE_SETTLE:
      if (deadlineReached(ringCalibrationDeadlineMs) &&
          startExtensionToMm(
              ringCalibrationTargetPose.extensionMm)) {
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_CORRECTION_EXTENSION);
      }
      break;

    case RING_CALIBRATION_WAIT_CORRECTION_EXTENSION:
      if (extensionMoveFinished()) {
        ringCalibrationCurrentPose =
            ringCalibrationTargetPose;
        beginCircleVision();
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_COORDINATE);
      }
      break;

    case RING_CALIBRATION_WAIT_SWITCH_LIFT:
      if (liftMoveFinished() &&
          startExtensionToMm(M6_STANDARD_EXTENSION_MM)) {
        ringCalibrationCurrentPose.heightMm =
            M7_STANDARD_HEIGHT_MM;
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_SWITCH_RETRACT);
      }
      break;

    case RING_CALIBRATION_WAIT_SWITCH_RETRACT:
      if (extensionMoveFinished()) {
        ringCalibrationCurrentPose.extensionMm =
            M6_STANDARD_EXTENSION_MM;
        startRingCalibrationApproach(3U);
      }
      break;

    case RING_CALIBRATION_COMPLETE:
      return true;
  }
  return ringCalibrationPhase == RING_CALIBRATION_COMPLETE;
}

float ringWorkingHeightMm(
    uint8_t ringPosition,
    float lowerMm) {
  (void)ringPosition;
  return -(
      lowerMm - STORAGE_RING_LOWER_REDUCTION_MM);
}

bool calibratedRingPose(
    uint8_t ringPosition,
    float lowerMm,
    ArmPose &pose) {
  if (!calibratedRingPosesValid ||
      ringPosition < 1U ||
      ringPosition > 3U) {
    testFault("Ring pose requested before calibration");
    return false;
  }
  pose = calibratedRingHorizontalPoses[
      ringPosition - 1U];
  pose.heightMm =
      ringWorkingHeightMm(ringPosition, lowerMm);
  if (!armPoseIsValid(pose)) {
    testFault("Calibrated ring pose outside arm travel");
    return false;
  }
  return true;
}

ArmPose containerPose() {
  return ArmPose(
      ARM_CONTAINER_CLOCKWISE_DEGREES,
      CONTAINER_EXTENSION_MM,
      -CONTAINER_PICK_LOWER_MM);
}

// ---------------------------------------------------------------------------
// 独立暂存区测试总状态机
// ---------------------------------------------------------------------------

bool beginCurrentPlacement() {
  if (testItemIndex >= 3U) {
    testFault("Placement item index overflow");
    return false;
  }

  const uint8_t ringPosition =
      TEST_RING_ORDER[testItemIndex];
  const float normalLowerMm =
      TEST_STORAGE_ROUND == 1U
          ? STORAGE_ROUND1_PLACE_LOWER_MM
          : STORAGE_ROUND2_PLACE_LOWER_MM;
  const float releaseLowerMm =
      normalLowerMm -
      RING_RELEASE_LOWER_REDUCTION_MM;
  ArmPose destination;
  if (!calibratedRingPose(
          ringPosition,
          releaseLowerMm,
          destination)) {
    return false;
  }

  SerialDebug.print("[PLACE START] slot/ring/normal/release/actual-height=");
  SerialDebug.print(
      static_cast<unsigned int>(testItemIndex));
  SerialDebug.print("/");
  SerialDebug.print(
      static_cast<unsigned int>(ringPosition));
  SerialDebug.print("/");
  SerialDebug.print(normalLowerMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(releaseLowerMm, 2);
  SerialDebug.print("/");
  SerialDebug.println(destination.heightMm, 2);
  beginArmTransfer(containerPose(), destination);
  setTestState(TEST_PLACING);
  return testState != TEST_FAULT;
}

void beginStorageTest() {
  if (!validateTestConfiguration()) {
    testFault("Invalid test configuration");
    return;
  }
  if (!armLinearReferenceValid) {
    testFault("M6/M7 software origin invalid");
    return;
  }
  if (!verifyManipulationServosOnline()) {
    testFault("Gripper/storage servo offline");
    return;
  }

  abortRequested = false;
  testItemIndex = 0U;
  armTransferPhase = ARM_TRANSFER_IDLE;
  armStandardPhase = ARM_STANDARD_IDLE;
  resetRingCalibration();
  commandStorageServoPosition(0U);
  storageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  beginArmStandardization();
  setTestState(TEST_PREPARING);

  SerialDebug.print("[TEST START] storage round=");
  SerialDebug.print(
      static_cast<unsigned int>(TEST_STORAGE_ROUND));
  SerialDebug.print(", ring order=");
  SerialDebug.print(TEST_RING_ORDER[0]);
  SerialDebug.print("/");
  SerialDebug.print(TEST_RING_ORDER[1]);
  SerialDebug.print("/");
  SerialDebug.println(TEST_RING_ORDER[2]);
  SerialDebug.println(
      "[SAFETY] chassis PE13 HIGH; M1-M4 stay disabled");
}

void abortStorageTest() {
  if (!testIsRunning()) {
    abortRequested = false;
    return;
  }
  abortRequested = false;
  testFault("PB9 long-press emergency stop");
}

void serviceStorageTest() {
  switch (testState) {
    case TEST_READY:
    case TEST_COMPLETE:
    case TEST_FAULT:
      return;

    case TEST_PREPARING:
      if (serviceArmStandardization() &&
          deadlineReached(storageServoDeadlineMs)) {
        armStandardPhase = ARM_STANDARD_IDLE;
        if (beginRingCalibration()) {
          setTestState(TEST_CALIBRATING);
        }
      }
      break;

    case TEST_CALIBRATING:
      if (serviceRingCalibration()) {
        SerialDebug.println(
            "[RING CAL] all coordinates ready; restore standard pose");
        beginArmStandardization();
        setTestState(TEST_POST_CALIBRATION_STANDARD);
      }
      break;

    case TEST_POST_CALIBRATION_STANDARD:
      if (serviceArmStandardization()) {
        armStandardPhase = ARM_STANDARD_IDLE;
        beginCurrentPlacement();
      }
      break;

    case TEST_PLACING:
      serviceArmTransfer();
      if (armTransferPhase == ARM_TRANSFER_COMPLETE) {
        armTransferPhase = ARM_TRANSFER_IDLE;
        ++testItemIndex;
        SerialDebug.print("[PLACE DONE] completed=");
        SerialDebug.print(
            static_cast<unsigned int>(testItemIndex));
        SerialDebug.println("/3");

        if (testItemIndex < 3U) {
          commandStorageServoPosition(testItemIndex);
        } else {
          commandStorageServoParkingPosition();
        }
        storageServoDeadlineMs =
            millis() + STORAGE_SERVO_SETTLE_MS;
        setTestState(TEST_WAIT_STORAGE_SERVO);
      }
      break;

    case TEST_WAIT_STORAGE_SERVO:
      if (!deadlineReached(storageServoDeadlineMs)) {
        break;
      }
      if (testItemIndex < 3U) {
        beginCurrentPlacement();
      } else {
        beginArmStandardization();
        setTestState(TEST_FINAL_STANDARD);
      }
      break;

    case TEST_FINAL_STANDARD:
      if (serviceArmStandardization() &&
          deadlineReached(storageServoDeadlineMs)) {
        armStandardPhase = ARM_STANDARD_IDLE;
        SerialDebug.println(
            "[FINAL] standard pose confirmed; M5 CCW 90 deg to old-frame 0");
        startArmBaseStandardFrameDegrees(
            ARM_BASE_TRAVEL_STANDARD_FRAME_DEGREES);
        setTestState(TEST_FINAL_TRAVEL);
      }
      break;

    case TEST_FINAL_TRAVEL:
      if (!armMotors.isM5Running() &&
          finalTravelDeadlineMs == 0UL) {
        finalTravelDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
      }
      if (finalTravelDeadlineMs != 0UL &&
          deadlineReached(finalTravelDeadlineMs)) {
        finalTravelDeadlineMs = 0UL;
        armMotors.disableM5();
        commandGripperClose();
        stopMaixRequest();
        setTestState(TEST_COMPLETE);
        SerialDebug.println(
            "[TEST COMPLETE] M5 old 0, M6 retracted, M7 top, storage 165 deg");
        SerialDebug.println(
            "[READY] short-press PB9 to run the same test again");
      }
      break;
  }
}

void onStartButtonClick() {
  SerialDebug.print("[INPUT] PB9 click, state=");
  SerialDebug.println(testStateName(testState));
  if (testState == TEST_READY ||
      testState == TEST_COMPLETE) {
    beginStorageTest();
  } else {
    SerialDebug.println("[INPUT] click ignored");
  }
}

void onStartButtonLongPress() {
  SerialDebug.print("[INPUT] PB9 long press, state=");
  SerialDebug.println(testStateName(testState));
  if (testIsRunning()) {
    abortRequested = true;
  }
}

void initializeHardware() {
  pinMode(DRIVE_ENABLE_PIN, OUTPUT);
  digitalWrite(DRIVE_ENABLE_PIN, HIGH);
  SerialDebug.println(
      "[BOOT] chassis disabled: PE13=HIGH");

  SerialMaixcam.begin(MAIXCAM_BAUDRATE);
  maixcamSerialInitialized = true;
  while (SerialMaixcam.available()) {
    SerialMaixcam.read();
  }
  stopMaixRequest();

  const uint32_t startupMs = millis();
  if (startupMs < ARM_LINEAR_POWER_ON_SETTLE_MS) {
    delay(ARM_LINEAR_POWER_ON_SETTLE_MS - startupMs);
  }
  SerialArmLinear.begin(ARM_LINEAR_BAUDRATE);
  armLinearSerialInitialized = true;
  clearArmLinearReceiveBuffer();
  resetArmLinearSoftwareOrigin();
  const bool m6Ready =
      recoverAndVerifyArmLinearAxis(extensionAxis);
  const bool m7Ready =
      recoverAndVerifyArmLinearAxis(liftAxis);
  if (!m6Ready || !m7Ready) {
    invalidateArmLinearReference();
  }
  clearArmLinearReceiveBuffer();

  servoProtocol.init(&SerialServo, SERVO_BAUDRATE);
  delay(100);
  gripperServo.init();
  storageServo.init();
  SerialDebug.print("[BOOT] servo online ID4/ID5=");
  SerialDebug.print(gripperServo.isOnline ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.println(storageServo.isOnline ? 1 : 0);
  commandGripperClose();
  commandStorageServoParkingPosition();

  armMotors.beginM5();
  armMotors.setM5MotionProfile(
      ARM_BASE_MAXIMUM_STEP_RATE,
      ARM_BASE_STEP_ACCELERATION);
  armMotors.setM5CurrentAngle(0.0f);
  armMotors.disableM5();

  SerialDebug.print("[BOOT] M6/M7 microsteps=");
  SerialDebug.println(ARM_LINEAR_MICROSTEPS, 0);
  SerialDebug.print("[BOOT] M6 pulses/mm=");
  SerialDebug.println(M6_PULSES_PER_MM, 4);
  SerialDebug.print("[BOOT] M7 pulses/mm=");
  SerialDebug.println(M7_PULSES_PER_MM, 4);

  if (!m6Ready || !m7Ready) {
    testFault("M6/M7 startup recovery failed");
  }
}

}  // namespace

void setup() {
  SerialDebug.begin(DEBUG_BAUDRATE);
  delay(50);
  SerialDebug.println();
  SerialDebug.println(
      "=== Storage Hough calibration and placement test ===");
  initializeHardware();

  startButton.reset();
  startButton.attachClick(onStartButtonClick);
  startButton.attachLongPressStart(onStartButtonLongPress);

  if (testState != TEST_FAULT) {
    SerialDebug.print("[CONFIG] round/order=");
    SerialDebug.print(
        static_cast<unsigned int>(TEST_STORAGE_ROUND));
    SerialDebug.print("/");
    SerialDebug.print(TEST_RING_ORDER[0]);
    SerialDebug.print(",");
    SerialDebug.print(TEST_RING_ORDER[1]);
    SerialDebug.print(",");
    SerialDebug.println(TEST_RING_ORDER[2]);
    SerialDebug.println(
        "[READY] short-press PB9 to start; long-press to stop");
  }
}

void loop() {
  armMotors.serviceM5();
  startButton.tick();
  serviceArmLinearAxes();
  serviceMaixcam();

  if (abortRequested) {
    abortStorageTest();
  }
  serviceStorageTest();
}
