#include "MecanumKinematics.h"

#include <math.h>

namespace mecanum {
namespace {

const float kPi = 3.14159265358979323846f;

const float kRadPerSecToRpm = 60.0f / (2.0f * kPi);

const float kRpmToRadPerSec = (2.0f * kPi) / 60.0f;

float absoluteValue(float value) { return value < 0.0f ? -value : value; }

float largestMagnitude(const WheelSpeeds &values) {
  float largest = absoluteValue(values.frontLeft);
  const float frontRight = absoluteValue(values.frontRight);
  const float rearLeft = absoluteValue(values.rearLeft);
  const float rearRight = absoluteValue(values.rearRight);

  if (frontRight > largest) {
    largest = frontRight;
  }
  if (rearLeft > largest) {
    largest = rearLeft;
  }
  if (rearRight > largest) {
    largest = rearRight;
  }
  return largest;
}

int16_t roundedCommand(float value, int16_t maximumCommand) {
  float limited = value;
  if (limited > maximumCommand) {
    limited = maximumCommand;
  } else if (limited < -maximumCommand) {
    limited = -maximumCommand;
  }

  return static_cast<int16_t>(limited >= 0.0f ? limited + 0.5f
                                               : limited - 0.5f);
}

}

MecanumKinematics::MecanumKinematics(float wheelbaseMeters,
                                     float trackWidthMeters,
                                     float wheelRadiusMeters)
    : wheelbaseMeters_(wheelbaseMeters),
      trackWidthMeters_(trackWidthMeters),
      wheelRadiusMeters_(wheelRadiusMeters),
      rotationLeverArmMeters_((wheelbaseMeters + trackWidthMeters) * 0.5f) {}

MecanumKinematics
MecanumKinematics::fromMillimeters(float wheelbaseMm, float trackWidthMm,
                                   float wheelDiameterMm) {
  return MecanumKinematics(wheelbaseMm * 0.001f, trackWidthMm * 0.001f,
                           wheelDiameterMm * 0.0005f);
}

bool MecanumKinematics::isValid() const {
  return isfinite(wheelbaseMeters_) && isfinite(trackWidthMeters_) &&
         isfinite(wheelRadiusMeters_) && wheelbaseMeters_ > 0.0f &&
         trackWidthMeters_ > 0.0f && wheelRadiusMeters_ > 0.0f;
}

float MecanumKinematics::wheelbaseMeters() const {
  return wheelbaseMeters_;
}

float MecanumKinematics::trackWidthMeters() const {
  return trackWidthMeters_;
}

float MecanumKinematics::wheelRadiusMeters() const {
  return wheelRadiusMeters_;
}

float MecanumKinematics::rotationLeverArmMeters() const {
  return rotationLeverArmMeters_;
}

WheelSpeeds
MecanumKinematics::inverse(const ChassisVelocity &velocity) const {

  if (!isValid()) {
    return WheelSpeeds();
  }

  const float rotation = rotationLeverArmMeters_ * velocity.wz;

  const float inverseRadius = 1.0f / wheelRadiusMeters_;

  return WheelSpeeds(
      (velocity.vx - velocity.vy - rotation) * inverseRadius,
      (velocity.vx + velocity.vy + rotation) * inverseRadius,
      (velocity.vx + velocity.vy - rotation) * inverseRadius,
      (velocity.vx - velocity.vy + rotation) * inverseRadius);
}

ChassisVelocity
MecanumKinematics::forward(const WheelSpeeds &wheelRadPerSec) const {

  if (!isValid()) {
    return ChassisVelocity();
  }

  const float linearFactor = wheelRadiusMeters_ * 0.25f;

  const float angularFactor =
      wheelRadiusMeters_ / (4.0f * rotationLeverArmMeters_);

  return ChassisVelocity(
      linearFactor *
          (wheelRadPerSec.frontLeft + wheelRadPerSec.frontRight +
           wheelRadPerSec.rearLeft + wheelRadPerSec.rearRight),

      linearFactor *
          (-wheelRadPerSec.frontLeft + wheelRadPerSec.frontRight +
           wheelRadPerSec.rearLeft - wheelRadPerSec.rearRight),

      angularFactor *
          (-wheelRadPerSec.frontLeft + wheelRadPerSec.frontRight -
           wheelRadPerSec.rearLeft + wheelRadPerSec.rearRight));
}

WheelSpeeds
MecanumKinematics::radPerSecToRpm(const WheelSpeeds &wheelRadPerSec) {
  return WheelSpeeds(
      wheelRadPerSec.frontLeft * kRadPerSecToRpm,
      wheelRadPerSec.frontRight * kRadPerSecToRpm,
      wheelRadPerSec.rearLeft * kRadPerSecToRpm,
      wheelRadPerSec.rearRight * kRadPerSecToRpm);
}

WheelSpeeds
MecanumKinematics::rpmToRadPerSec(const WheelSpeeds &wheelRpm) {
  return WheelSpeeds(wheelRpm.frontLeft * kRpmToRadPerSec,
                     wheelRpm.frontRight * kRpmToRadPerSec,
                     wheelRpm.rearLeft * kRpmToRadPerSec,
                     wheelRpm.rearRight * kRpmToRadPerSec);
}

float MecanumKinematics::limitPreservingRatio(WheelSpeeds &values,
                                              float maximumMagnitude) {

  if (!isfinite(maximumMagnitude) || maximumMagnitude <= 0.0f) {
    values = WheelSpeeds();
    return 0.0f;
  }

  const float largest = largestMagnitude(values);

  if (!isfinite(largest)) {
    values = WheelSpeeds();
    return 0.0f;
  }

  if (largest <= maximumMagnitude || largest == 0.0f) {
    return 1.0f;
  }

  const float scale = maximumMagnitude / largest;
  values.frontLeft *= scale;
  values.frontRight *= scale;
  values.rearLeft *= scale;
  values.rearRight *= scale;
  return scale;
}

DDSM210Commands MecanumKinematics::toDDSM210Commands(
    const WheelSpeeds &physicalWheelRpm,
    const WheelDirections &motorDirections, int16_t maximumCommand,
    float commandUnitsPerRpm) {

  if (maximumCommand <= 0 || !isfinite(commandUnitsPerRpm) ||
      commandUnitsPerRpm <= 0.0f) {
    return DDSM210Commands();
  }

  WheelSpeeds rawCommands(
      physicalWheelRpm.frontLeft * motorDirections.frontLeft *
          commandUnitsPerRpm,
      physicalWheelRpm.frontRight * motorDirections.frontRight *
          commandUnitsPerRpm,
      physicalWheelRpm.rearLeft * motorDirections.rearLeft *
          commandUnitsPerRpm,
      physicalWheelRpm.rearRight * motorDirections.rearRight *
          commandUnitsPerRpm);

  limitPreservingRatio(rawCommands, static_cast<float>(maximumCommand));

  return DDSM210Commands(
      roundedCommand(rawCommands.frontLeft, maximumCommand),
      roundedCommand(rawCommands.frontRight, maximumCommand),
      roundedCommand(rawCommands.rearLeft, maximumCommand),
      roundedCommand(rawCommands.rearRight, maximumCommand));
}

}

