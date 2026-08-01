#include "MecanumOdometry.h"

#include <math.h>

namespace mecanum {
namespace {

const double kPi = 3.1415926535897932384626433832795;
const double kTwoPi = 2.0 * kPi;
const double kSmallAngle = 1.0e-9;

bool directionIsValid(int8_t direction) {
  return direction == 1 || direction == -1;
}

bool valueIsFinite(double value) { return isfinite(value); }

}

MecanumOdometry::MecanumOdometry(
    const MecanumKinematics &kinematics,
    const WheelDirections &motorDirections, uint32_t positionUnitsPerTurn)
    : kinematics_(kinematics), motorDirections_(motorDirections),
      positionUnitsPerTurn_(positionUnitsPerTurn),
      maximumWheelDeltaTurns_(0.0), initialized_(false),
      imuHeadingReferenceValid_(false), imuHeadingOffsetRadians_(0.0),
      previousTurns_{0.0, 0.0, 0.0, 0.0},
      pose_(), lastDisplacement_() {}

bool MecanumOdometry::isValid() const {
  return kinematics_.isValid() && positionUnitsPerTurn_ > 0U &&
         positionUnitsPerTurn_ <= 65536U &&
         directionIsValid(motorDirections_.frontLeft) &&
         directionIsValid(motorDirections_.frontRight) &&
         directionIsValid(motorDirections_.rearLeft) &&
         directionIsValid(motorDirections_.rearRight);
}

void MecanumOdometry::reset(const Pose2D &newPose) {
  if (valueIsFinite(newPose.xMeters) && valueIsFinite(newPose.yMeters) &&
      valueIsFinite(newPose.headingRadians)) {
    pose_ = newPose;
    pose_.headingRadians = normalizeAngle(pose_.headingRadians);
  } else {
    pose_ = Pose2D();
  }

  initialized_ = false;
  imuHeadingReferenceValid_ = false;
  imuHeadingOffsetRadians_ = 0.0;
  previousTurns_[0] = 0.0;
  previousTurns_[1] = 0.0;
  previousTurns_[2] = 0.0;
  previousTurns_[3] = 0.0;
  lastDisplacement_ = ChassisDisplacement();
}

double MecanumOdometry::continuousTurns(
    const WheelPositionSample &sample) const {
  return static_cast<double>(sample.wholeTurns) +
         static_cast<double>(sample.position) /
             static_cast<double>(positionUnitsPerTurn_);
}

bool MecanumOdometry::samplesAreValid(
    const WheelPositionSamples &samples) const {
  if (!isValid()) {
    return false;
  }

  return static_cast<uint32_t>(samples.frontLeft.position) <
             positionUnitsPerTurn_ &&
         static_cast<uint32_t>(samples.frontRight.position) <
             positionUnitsPerTurn_ &&
         static_cast<uint32_t>(samples.rearLeft.position) <
             positionUnitsPerTurn_ &&
         static_cast<uint32_t>(samples.rearRight.position) <
             positionUnitsPerTurn_;
}

bool MecanumOdometry::update(const WheelPositionSamples &samples) {
  return updateInternal(samples, nullptr);
}

bool MecanumOdometry::update(
    const WheelPositionSamples &samples,
    const ImuHeadingMeasurement &imuHeading) {
  return updateInternal(samples, &imuHeading);
}

bool MecanumOdometry::imuMeasurementIsValid(
    const ImuHeadingMeasurement &measurement) const {
  return valueIsFinite(measurement.yawRadians) &&
         valueIsFinite(measurement.fusionWeight) &&
         measurement.fusionWeight >= 0.0 &&
         measurement.fusionWeight <= 1.0;
}

bool MecanumOdometry::updateInternal(
    const WheelPositionSamples &samples,
    const ImuHeadingMeasurement *imuHeading) {
  if (!samplesAreValid(samples)) {
    return false;
  }
  if (imuHeading != nullptr && !imuMeasurementIsValid(*imuHeading)) {
    return false;
  }

  const double currentTurns[4] = {
      continuousTurns(samples.frontLeft),
      continuousTurns(samples.frontRight),
      continuousTurns(samples.rearLeft),
      continuousTurns(samples.rearRight)};

  if (!initialized_) {
    previousTurns_[0] = currentTurns[0];
    previousTurns_[1] = currentTurns[1];
    previousTurns_[2] = currentTurns[2];
    previousTurns_[3] = currentTurns[3];
    initialized_ = true;

    if (imuHeading != nullptr && !imuHeadingReferenceValid_) {
      setImuHeadingReference(imuHeading->yawRadians,
                             pose_.headingRadians);
    }

    lastDisplacement_ = ChassisDisplacement();
    return true;
  }

  double deltaTurns[4] = {
      currentTurns[0] - previousTurns_[0],
      currentTurns[1] - previousTurns_[1],
      currentTurns[2] - previousTurns_[2],
      currentTurns[3] - previousTurns_[3]};

  for (uint8_t i = 0; i < 4; ++i) {
    if (!valueIsFinite(deltaTurns[i])) {
      return false;
    }
    if (maximumWheelDeltaTurns_ > 0.0 &&
        fabs(deltaTurns[i]) > maximumWheelDeltaTurns_) {
      return false;
    }
  }

  const double wheelRadians[4] = {
      deltaTurns[0] * motorDirections_.frontLeft * kTwoPi,
      deltaTurns[1] * motorDirections_.frontRight * kTwoPi,
      deltaTurns[2] * motorDirections_.rearLeft * kTwoPi,
      deltaTurns[3] * motorDirections_.rearRight * kTwoPi};

  const double radius =
      static_cast<double>(kinematics_.wheelRadiusMeters());
  const double leverArm =
      static_cast<double>(kinematics_.rotationLeverArmMeters());

  const double dxBody =
      radius * 0.25 *
      (wheelRadians[0] + wheelRadians[1] + wheelRadians[2] +
       wheelRadians[3]);
  const double dyBody =
      radius * 0.25 *
      (-wheelRadians[0] + wheelRadians[1] + wheelRadians[2] -
       wheelRadians[3]);
  const double wheelDHeading =
      radius / (4.0 * leverArm) *
      (-wheelRadians[0] + wheelRadians[1] - wheelRadians[2] +
       wheelRadians[3]);

  if (!valueIsFinite(dxBody) || !valueIsFinite(dyBody) ||
      !valueIsFinite(wheelDHeading)) {
    return false;
  }

  double fusedDHeading = wheelDHeading;

  if (imuHeading != nullptr) {
    const double wheelPredictedHeading =
        normalizeAngle(pose_.headingRadians + wheelDHeading);

    if (!imuHeadingReferenceValid_ &&
        !setImuHeadingReference(imuHeading->yawRadians,
                                wheelPredictedHeading)) {
      return false;
    }

    const double imuWorldHeading =
        normalizeAngle(imuHeading->yawRadians +
                       imuHeadingOffsetRadians_);
    const double headingError =
        normalizeAngle(imuWorldHeading - wheelPredictedHeading);
    const double fusedHeading =
        normalizeAngle(wheelPredictedHeading +
                       headingError * imuHeading->fusionWeight);

    fusedDHeading =
        normalizeAngle(fusedHeading - pose_.headingRadians);
  }

  lastDisplacement_ =
      ChassisDisplacement(dxBody, dyBody, fusedDHeading);

  double sinc;
  double cosc;
  if (fabs(fusedDHeading) < kSmallAngle) {
    const double angleSquared = fusedDHeading * fusedDHeading;
    sinc = 1.0 - angleSquared / 6.0;
    cosc = fusedDHeading * 0.5;
  } else {
    sinc = sin(fusedDHeading) / fusedDHeading;
    cosc = (1.0 - cos(fusedDHeading)) / fusedDHeading;
  }

  const double localDx = sinc * dxBody - cosc * dyBody;
  const double localDy = cosc * dxBody + sinc * dyBody;
  const double cosHeading = cos(pose_.headingRadians);
  const double sinHeading = sin(pose_.headingRadians);

  pose_.xMeters += cosHeading * localDx - sinHeading * localDy;
  pose_.yMeters += sinHeading * localDx + cosHeading * localDy;
  pose_.headingRadians =
      normalizeAngle(pose_.headingRadians + fusedDHeading);

  previousTurns_[0] = currentTurns[0];
  previousTurns_[1] = currentTurns[1];
  previousTurns_[2] = currentTurns[2];
  previousTurns_[3] = currentTurns[3];
  return true;
}

bool MecanumOdometry::setImuHeadingReference(
    double sensorYawRadians, double worldHeadingRadians) {
  if (!valueIsFinite(sensorYawRadians) ||
      !valueIsFinite(worldHeadingRadians)) {
    return false;
  }

  imuHeadingOffsetRadians_ =
      normalizeAngle(worldHeadingRadians - sensorYawRadians);
  imuHeadingReferenceValid_ = true;
  return true;
}

void MecanumOdometry::clearImuHeadingReference() {
  imuHeadingReferenceValid_ = false;
  imuHeadingOffsetRadians_ = 0.0;
}

bool MecanumOdometry::hasImuHeadingReference() const {
  return imuHeadingReferenceValid_;
}

bool MecanumOdometry::setMaximumWheelDeltaTurns(double maximumTurns) {
  if (!valueIsFinite(maximumTurns) || maximumTurns < 0.0) {
    return false;
  }
  maximumWheelDeltaTurns_ = maximumTurns;
  return true;
}

double MecanumOdometry::maximumWheelDeltaTurns() const {
  return maximumWheelDeltaTurns_;
}

bool MecanumOdometry::isInitialized() const { return initialized_; }

const Pose2D &MecanumOdometry::pose() const { return pose_; }

const ChassisDisplacement &MecanumOdometry::lastDisplacement() const {
  return lastDisplacement_;
}

uint32_t MecanumOdometry::positionUnitsPerTurn() const {
  return positionUnitsPerTurn_;
}

double MecanumOdometry::normalizeAngle(double radians) {
  while (radians >= kPi) {
    radians -= kTwoPi;
  }
  while (radians < -kPi) {
    radians += kTwoPi;
  }
  return radians;
}

}

