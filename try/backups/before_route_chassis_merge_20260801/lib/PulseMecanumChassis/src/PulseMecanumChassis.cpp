#include "PulseMecanumChassis.h"

#include <math.h>

namespace mecanum {
namespace {

const float kTwoPi = 6.28318530717958647692f;
const float kMicrosecondsPerSecond = 1000000.0f;

bool directionIsValid(int8_t direction) {
  return direction == 1 || direction == -1;
}

}

PulseMecanumChassis::PulseMecanumChassis(
    const PulseMecanumPins &pins,
    const MecanumKinematics &kinematics,
    const WheelDirections &motorDirections,
    float pulsesPerWheelRevolution,
    float linearPulsesPerMeter,
    float maximumPulseRate,
    uint16_t minimumPulseWidthMicros)
    : pins_(pins), kinematics_(kinematics),
      motorDirections_(motorDirections),
      pulsesPerWheelRevolution_(pulsesPerWheelRevolution),
      linearPulsesPerMeter_(linearPulsesPerMeter),
      maximumPulseRate_(maximumPulseRate),
      minimumPulseWidthMicros_(minimumPulseWidthMicros),

      motor1_(AccelStepper::DRIVER, pins.motor1StepPin,
              pins.motor1DirectionPin, 0, 0, false),
      motor2_(AccelStepper::DRIVER, pins.motor2StepPin,
              pins.motor2DirectionPin, 0, 0, false),
      motor3_(AccelStepper::DRIVER, pins.motor3StepPin,
              pins.motor3DirectionPin, 0, 0, false),
      motor4_(AccelStepper::DRIVER, pins.motor4StepPin,
              pins.motor4DirectionPin, 0, 0, false),
      begun_(false), enabled_(false), lastPulseRates_(),
      lastLimitScale_(1.0f), lastRequestedVelocity_() {}

bool PulseMecanumChassis::pinsAreValid() const {
  return digitalPinIsValid(pins_.motor1StepPin) &&
         digitalPinIsValid(pins_.motor1DirectionPin) &&
         digitalPinIsValid(pins_.motor2StepPin) &&
         digitalPinIsValid(pins_.motor2DirectionPin) &&
         digitalPinIsValid(pins_.motor3StepPin) &&
         digitalPinIsValid(pins_.motor3DirectionPin) &&
         digitalPinIsValid(pins_.motor4StepPin) &&
         digitalPinIsValid(pins_.motor4DirectionPin) &&
         digitalPinIsValid(pins_.enablePin);
}

bool PulseMecanumChassis::pinsAreUnique() const {
  const uint8_t pins[] = {
      pins_.motor1StepPin,      pins_.motor1DirectionPin,
      pins_.motor2StepPin,      pins_.motor2DirectionPin,
      pins_.motor3StepPin,      pins_.motor3DirectionPin,
      pins_.motor4StepPin,      pins_.motor4DirectionPin,
      pins_.enablePin};

  const uint8_t pinCount = sizeof(pins) / sizeof(pins[0]);
  for (uint8_t first = 0; first < pinCount; ++first) {
    for (uint8_t second = first + 1U; second < pinCount; ++second) {
      if (pins[first] == pins[second]) {
        return false;
      }
    }
  }
  return true;
}

bool PulseMecanumChassis::configurationIsValid() const {
  return kinematics_.isValid() && pinsAreValid() && pinsAreUnique() &&
         directionIsValid(motorDirections_.frontLeft) &&
         directionIsValid(motorDirections_.frontRight) &&
         directionIsValid(motorDirections_.rearLeft) &&
         directionIsValid(motorDirections_.rearRight) &&
         isfinite(pulsesPerWheelRevolution_) &&
         pulsesPerWheelRevolution_ > 0.0f &&
         isfinite(linearPulsesPerMeter_) &&
         linearPulsesPerMeter_ > 0.0f &&
         isfinite(maximumPulseRate_) && maximumPulseRate_ > 0.0f &&
         minimumPulseWidthMicros_ > 0U &&
         maximumPulseRate_ <=
             kMicrosecondsPerSecond /
                 static_cast<float>(minimumPulseWidthMicros_);
}

bool PulseMecanumChassis::begin(bool enableMotors) {
  begun_ = false;
  enabled_ = false;
  stop();

  if (!digitalPinIsValid(pins_.enablePin)) {
    return false;
  }

  digitalWrite(pins_.enablePin, enableLevel(false));
  pinMode(pins_.enablePin, OUTPUT);

  if (!configurationIsValid()) {
    return false;
  }

  motor1_.enableOutputs();
  motor2_.enableOutputs();
  motor3_.enableOutputs();
  motor4_.enableOutputs();

  motor1_.setMaxSpeed(maximumPulseRate_);
  motor2_.setMaxSpeed(maximumPulseRate_);
  motor3_.setMaxSpeed(maximumPulseRate_);
  motor4_.setMaxSpeed(maximumPulseRate_);

  motor1_.setMinPulseWidth(minimumPulseWidthMicros_);
  motor2_.setMinPulseWidth(minimumPulseWidthMicros_);
  motor3_.setMinPulseWidth(minimumPulseWidthMicros_);
  motor4_.setMinPulseWidth(minimumPulseWidthMicros_);

  begun_ = true;
  return setEnabled(enableMotors);
}

bool PulseMecanumChassis::velocityIsValid(
    const ChassisVelocity &velocity) const {
  return isfinite(velocity.vx) && isfinite(velocity.vy) &&
         isfinite(velocity.wz);
}

bool PulseMecanumChassis::calculatePulseRates(
    const ChassisVelocity &velocity,
    WheelSpeeds &pulseRates,
    float &limitScale) const {
  if (!configurationIsValid() || !velocityIsValid(velocity)) {
    pulseRates = WheelSpeeds();
    limitScale = 0.0f;
    return false;
  }

  const WheelSpeeds translationWheelRadPerSecond =
      kinematics_.inverse(ChassisVelocity(velocity.vx, velocity.vy, 0.0f));
  const WheelSpeeds rotationWheelRadPerSecond =
      kinematics_.inverse(ChassisVelocity(0.0f, 0.0f, velocity.wz));

  const float translationPulseFactor =
      kinematics_.wheelRadiusMeters() * linearPulsesPerMeter_;
  const float rotationPulseFactor =
      pulsesPerWheelRevolution_ / kTwoPi;

  pulseRates = WheelSpeeds(
      (translationWheelRadPerSecond.frontLeft * translationPulseFactor +
       rotationWheelRadPerSecond.frontLeft * rotationPulseFactor) *
          motorDirections_.frontLeft,
      (translationWheelRadPerSecond.frontRight * translationPulseFactor +
       rotationWheelRadPerSecond.frontRight * rotationPulseFactor) *
          motorDirections_.frontRight,
      (translationWheelRadPerSecond.rearLeft * translationPulseFactor +
       rotationWheelRadPerSecond.rearLeft * rotationPulseFactor) *
          motorDirections_.rearLeft,
      (translationWheelRadPerSecond.rearRight * translationPulseFactor +
       rotationWheelRadPerSecond.rearRight * rotationPulseFactor) *
          motorDirections_.rearRight);

  limitScale = MecanumKinematics::limitPreservingRatio(
      pulseRates, maximumPulseRate_);
  return limitScale > 0.0f;
}

void PulseMecanumChassis::applyPulseRates(
    const WheelSpeeds &pulseRates) {
  motor1_.setSpeed(pulseRates.frontLeft);
  motor2_.setSpeed(pulseRates.frontRight);
  motor3_.setSpeed(pulseRates.rearLeft);
  motor4_.setSpeed(pulseRates.rearRight);
}

bool PulseMecanumChassis::drive(const ChassisVelocity &velocity) {
  WheelSpeeds pulseRates;
  float limitScale = 0.0f;

  if (!begun_ || !enabled_ ||
      !calculatePulseRates(velocity, pulseRates, limitScale)) {
    stop();
    return false;
  }

  applyPulseRates(pulseRates);
  lastPulseRates_ = pulseRates;
  lastLimitScale_ = limitScale;
  lastRequestedVelocity_ = velocity;
  return true;
}

bool PulseMecanumChassis::drive(
    float vxMetersPerSecond, float vyMetersPerSecond,
    float wzRadiansPerSecond) {
  return drive(ChassisVelocity(vxMetersPerSecond, vyMetersPerSecond,
                               wzRadiansPerSecond));
}

bool PulseMecanumChassis::drive(
    float forwardMetersPerSecond, float wzRadiansPerSecond) {
  return drive(forwardMetersPerSecond, 0.0f, wzRadiansPerSecond);
}

bool PulseMecanumChassis::setVelocity(
    const ChassisVelocity &velocity) {
  return drive(velocity);
}

bool PulseMecanumChassis::setVelocity(
    float vxMetersPerSecond, float vyMetersPerSecond,
    float wzRadiansPerSecond) {
  return drive(vxMetersPerSecond, vyMetersPerSecond,
               wzRadiansPerSecond);
}

bool PulseMecanumChassis::setVelocity(
    float forwardMetersPerSecond, float wzRadiansPerSecond) {
  return drive(forwardMetersPerSecond, wzRadiansPerSecond);
}

bool PulseMecanumChassis::run() {
  if (!begun_ || !enabled_) {
    return false;
  }

  const bool motor1Stepped = motor1_.runSpeed();
  const bool motor2Stepped = motor2_.runSpeed();
  const bool motor3Stepped = motor3_.runSpeed();
  const bool motor4Stepped = motor4_.runSpeed();

  return motor1Stepped || motor2Stepped ||
         motor3Stepped || motor4Stepped;
}

void PulseMecanumChassis::stop() {
  lastRequestedVelocity_ = ChassisVelocity();
  lastPulseRates_ = WheelSpeeds();
  lastLimitScale_ = 1.0f;
  applyPulseRates(lastPulseRates_);
}

uint8_t PulseMecanumChassis::enableLevel(bool enabled) const {
  if (pins_.enableActiveLow) {
    return enabled ? LOW : HIGH;
  }
  return enabled ? HIGH : LOW;
}

bool PulseMecanumChassis::setEnabled(bool enabled) {
  if (!begun_) {
    return false;
  }

  if (!enabled) {
    stop();
  }
  digitalWrite(pins_.enablePin, enableLevel(enabled));
  enabled_ = enabled;
  return true;
}

bool PulseMecanumChassis::enable() {
  return setEnabled(true);
}

void PulseMecanumChassis::disable() {
  setEnabled(false);
}

bool PulseMecanumChassis::isBegun() const {
  return begun_;
}

bool PulseMecanumChassis::isEnabled() const {
  return enabled_;
}

const WheelSpeeds &PulseMecanumChassis::lastPulseRates() const {
  return lastPulseRates_;
}

float PulseMecanumChassis::lastLimitScale() const {
  return lastLimitScale_;
}

const ChassisVelocity &
PulseMecanumChassis::lastRequestedVelocity() const {
  return lastRequestedVelocity_;
}

const MecanumKinematics &PulseMecanumChassis::kinematics() const {
  return kinematics_;
}

}

