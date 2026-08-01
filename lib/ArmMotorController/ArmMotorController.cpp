#include "ArmMotorController.h"

#include <RobotConfig.h>
#include <math.h>

namespace {

const uint8_t M5_ENABLE_PIN = PE10;
const uint8_t M5_DIRECTION_PIN = PE15;
const uint8_t M5_STEP_PIN = PB11;

const uint8_t M6M7_TX_PIN = PA2;
const uint8_t M6M7_RX_PIN = PA3;
const uint32_t M6M7_BAUDRATE = 115200UL;

const uint8_t M6_ADDRESS = 6U;
const uint8_t M7_ADDRESS = 7U;

constexpr uint16_t FULL_STEPS_PER_REVOLUTION =
    gongchuang::config::arm_hardware::
        FULL_STEPS_PER_REVOLUTION;
constexpr uint16_t M5_MICROSTEPS =
    gongchuang::config::arm_hardware::M5_MICROSTEPS;
constexpr uint16_t M6_MICROSTEPS =
    gongchuang::config::arm_hardware::M6_MICROSTEPS;
constexpr uint16_t M7_MICROSTEPS =
    gongchuang::config::arm_hardware::M7_MICROSTEPS;
constexpr float M5_GEAR_RATIO =
    gongchuang::config::arm_hardware::M5_GEAR_RATIO;

const float M6_RACK_MODULE_MM = 1.0f;
constexpr float M6_PINION_PITCH_DIAMETER_MM =
    gongchuang::config::arm_hardware::
        M6_PINION_PITCH_DIAMETER_MM;
const float M6_PINION_TEETH =
    M6_PINION_PITCH_DIAMETER_MM / M6_RACK_MODULE_MM;
const float M6_TRAVEL_PER_REVOLUTION_MM =
    PI * M6_RACK_MODULE_MM * M6_PINION_TEETH;

constexpr float DEFAULT_M7_LEAD_MM_PER_REVOLUTION =
    gongchuang::config::arm_hardware::
        M7_LEAD_MM_PER_REVOLUTION;

const float DEFAULT_M5_MAXIMUM_STEP_RATE = 1800.0f;
const float DEFAULT_M5_STEP_ACCELERATION = 750.0f;
const uint16_t M5_MINIMUM_STEP_PULSE_WIDTH_US = 2U;

const uint16_t DEFAULT_SERIAL_SPEED_RPM = 108U;
const uint8_t DEFAULT_SERIAL_ACCELERATION = 119U;
const uint32_t SERIAL_RESPONSE_SETTLE_TIME_MS = 30UL;

const uint8_t M6_POSITIVE_DIRECTION = 1U;
const uint8_t M7_POSITIVE_DIRECTION = 0U;

uint8_t oppositeDirection(uint8_t direction) {
  return direction == 0U ? 1U : 0U;
}

}

ArmMotorController::ArmMotorController()
    : motorM5_(
          AccelStepper::DRIVER,
          M5_STEP_PIN,
          M5_DIRECTION_PIN),
      serialM6M7_(M6M7_RX_PIN, M6M7_TX_PIN),
      serialProtocol_(),
      serialSpeedRpm_(DEFAULT_SERIAL_SPEED_RPM),
      serialAcceleration_(DEFAULT_SERIAL_ACCELERATION),
      m7LeadMmPerRevolution_(
          DEFAULT_M7_LEAD_MM_PER_REVOLUTION) {}

void ArmMotorController::begin() {
  beginM5();
  serialProtocol_.init(&serialM6M7_, M6M7_BAUDRATE);
  enableAll();
}

void ArmMotorController::beginM5() {
  pinMode(M5_ENABLE_PIN, OUTPUT);
  digitalWrite(M5_ENABLE_PIN, HIGH);

  motorM5_.enableOutputs();

  motorM5_.setPinsInverted(false, false, false);
  motorM5_.setMinPulseWidth(
      M5_MINIMUM_STEP_PULSE_WIDTH_US);
  setM5MotionProfile(
      DEFAULT_M5_MAXIMUM_STEP_RATE,
      DEFAULT_M5_STEP_ACCELERATION);
  motorM5_.setCurrentPosition(0L);
}

void ArmMotorController::enableAll() {
  enableM5();
  enableM6();
  enableM7();
}

void ArmMotorController::disableAll() {
  disableM5();
  disableM6();
  disableM7();
}

void ArmMotorController::enableM5() {
  digitalWrite(M5_ENABLE_PIN, LOW);
}

void ArmMotorController::enableM6() {
  setSerialMotorEnabled(M6_ADDRESS, true);
}

void ArmMotorController::enableM7() {
  setSerialMotorEnabled(M7_ADDRESS, true);
}

void ArmMotorController::disableM5() {
  digitalWrite(M5_ENABLE_PIN, HIGH);
}

void ArmMotorController::disableM6() {
  setSerialMotorEnabled(M6_ADDRESS, false);
}

void ArmMotorController::disableM7() {
  setSerialMotorEnabled(M7_ADDRESS, false);
}

void ArmMotorController::moveM5ByDegrees(
    float outputDegrees, bool waitUntilDone) {
  const long pulses = m5PulsesForDegrees(outputDegrees);
  if (pulses == 0L) {
    return;
  }

  enableM5();
  motorM5_.move(pulses);
  if (waitUntilDone) {
    motorM5_.runToPosition();
  }
}

void ArmMotorController::moveM5ToDegrees(
    float outputDegrees, bool waitUntilDone) {
  enableM5();
  motorM5_.moveTo(m5PulsesForDegrees(outputDegrees));
  if (waitUntilDone) {
    motorM5_.runToPosition();
  }
}

void ArmMotorController::rotateM5ClockwiseByDegrees(
    float outputDegrees, bool waitUntilDone) {
  moveM5ByDegrees(
      -fabsf(outputDegrees), waitUntilDone);
}

void ArmMotorController::rotateM5CounterClockwiseByDegrees(
    float outputDegrees, bool waitUntilDone) {
  moveM5ByDegrees(
      fabsf(outputDegrees), waitUntilDone);
}

void ArmMotorController::setM5CurrentAngle(
    float outputDegrees) {
  motorM5_.setCurrentPosition(
      m5PulsesForDegrees(outputDegrees));
}

void ArmMotorController::serviceM5() {
  motorM5_.run();
}

bool ArmMotorController::isM5Running() {
  return motorM5_.distanceToGo() != 0L;
}

void ArmMotorController::stopM5Immediately() {
  const long currentPosition = motorM5_.currentPosition();
  motorM5_.setCurrentPosition(currentPosition);
}

void ArmMotorController::moveM6ByMillimeters(
    float extensionMillimeters) {
  const uint32_t pulses =
      m6PulsesForMillimeters(extensionMillimeters);
  if (pulses == 0UL) {
    return;
  }

  const uint8_t direction =
      extensionMillimeters > 0.0f
          ? oppositeDirection(M6_POSITIVE_DIRECTION)
          : M6_POSITIVE_DIRECTION;
  moveSerialMotorByPulses(
      M6_ADDRESS, direction, pulses);
}

void ArmMotorController::extendM6ByMillimeters(
    float distanceMillimeters) {
  moveM6ByMillimeters(fabsf(distanceMillimeters));
}

void ArmMotorController::retractM6ByMillimeters(
    float distanceMillimeters) {
  moveM6ByMillimeters(-fabsf(distanceMillimeters));
}

void ArmMotorController::moveM7ByMillimeters(
    float verticalMillimeters) {
  const uint32_t pulses =
      m7PulsesForMillimeters(verticalMillimeters);
  if (pulses == 0UL) {
    return;
  }

  const uint8_t direction =
      verticalMillimeters > 0.0f
          ? oppositeDirection(M7_POSITIVE_DIRECTION)
          : M7_POSITIVE_DIRECTION;
  moveSerialMotorByPulses(
      M7_ADDRESS, direction, pulses);
}

void ArmMotorController::raiseM7ByMillimeters(
    float distanceMillimeters) {
  moveM7ByMillimeters(fabsf(distanceMillimeters));
}

void ArmMotorController::lowerM7ByMillimeters(
    float distanceMillimeters) {
  moveM7ByMillimeters(-fabsf(distanceMillimeters));
}

void ArmMotorController::setM7LeadMillimetersPerRevolution(
    float leadMillimeters) {
  if (leadMillimeters > 0.0f) {
    m7LeadMmPerRevolution_ = leadMillimeters;
  }
}

void ArmMotorController::stopM6() {
  clearSerialResponses();
  serialProtocol_.Emm_V5_Stop_Now(M6_ADDRESS, false);
  delay(SERIAL_RESPONSE_SETTLE_TIME_MS);
  clearSerialResponses();
}

void ArmMotorController::stopM7() {
  clearSerialResponses();
  serialProtocol_.Emm_V5_Stop_Now(M7_ADDRESS, false);
  delay(SERIAL_RESPONSE_SETTLE_TIME_MS);
  clearSerialResponses();
}

void ArmMotorController::setM5MotionProfile(
    float maximumStepRate, float stepAcceleration) {
  motorM5_.setMaxSpeed(maximumStepRate);
  motorM5_.setAcceleration(stepAcceleration);
}

void ArmMotorController::setSerialMotionProfile(
    uint16_t speedRpm, uint8_t acceleration) {
  serialSpeedRpm_ = speedRpm;
  serialAcceleration_ = acceleration;
}

long ArmMotorController::m5PulsesForDegrees(
    float outputDegrees) const {
  return lroundf(
      outputDegrees *
      static_cast<float>(FULL_STEPS_PER_REVOLUTION) *
      static_cast<float>(M5_MICROSTEPS) *
      M5_GEAR_RATIO / 360.0f);
}

uint32_t ArmMotorController::m6PulsesForMillimeters(
    float extensionMillimeters) const {
  const float pulsesPerRevolution =
      static_cast<float>(FULL_STEPS_PER_REVOLUTION) *
      static_cast<float>(M6_MICROSTEPS);
  return static_cast<uint32_t>(
      lroundf(
          fabsf(extensionMillimeters) *
          pulsesPerRevolution /
          M6_TRAVEL_PER_REVOLUTION_MM));
}

uint32_t ArmMotorController::m7PulsesForMillimeters(
    float verticalMillimeters) const {
  const float pulsesPerRevolution =
      static_cast<float>(FULL_STEPS_PER_REVOLUTION) *
      static_cast<float>(M7_MICROSTEPS);
  return static_cast<uint32_t>(
      lroundf(
          fabsf(verticalMillimeters) *
          pulsesPerRevolution /
          m7LeadMmPerRevolution_));
}

void ArmMotorController::clearSerialResponses() {
  while (serialM6M7_.available() > 0) {
    serialM6M7_.read();
  }
}

void ArmMotorController::setSerialMotorEnabled(
    uint8_t address, bool enabled) {
  clearSerialResponses();
  serialProtocol_.Emm_V5_En_Control(
      address, enabled, false);
  delay(SERIAL_RESPONSE_SETTLE_TIME_MS);
  clearSerialResponses();
}

void ArmMotorController::moveSerialMotorByPulses(
    uint8_t address,
    uint8_t direction,
    uint32_t pulses) {
  if (pulses == 0UL) {
    return;
  }

  setSerialMotorEnabled(address, true);
  clearSerialResponses();
  serialProtocol_.Emm_V5_Pos_Control(
      address,
      direction,
      serialSpeedRpm_,
      serialAcceleration_,
      pulses,
      false,
      false);
  delay(SERIAL_RESPONSE_SETTLE_TIME_MS);
  clearSerialResponses();
}
