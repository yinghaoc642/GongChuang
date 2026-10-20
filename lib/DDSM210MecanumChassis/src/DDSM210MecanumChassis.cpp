#include "DDSM210MecanumChassis.h"

#include <math.h>

namespace mecanum {
namespace {

const double kDegreesToRadians =
    3.1415926535897932384626433832795 / 180.0;

bool directionIsValid(int8_t direction) {
  return direction == 1 || direction == -1;
}

} // namespace

DDSM210MecanumChassis::DDSM210MecanumChassis(
    HardwareSerial &motorSerial, const MecanumKinematics &kinematics,
    const WheelDirections &motorDirections, uint32_t positionUnitsPerTurn,
    int16_t maximumCommand, float commandUnitsPerRpm,
    uint8_t accelerationTime)
    : motorSerial_(motorSerial), kinematics_(kinematics),
      motorDirections_(motorDirections),
      odometry_(kinematics, motorDirections, positionUnitsPerTurn),
      communication_(), maximumCommand_(maximumCommand),
      commandUnitsPerRpm_(commandUnitsPerRpm),
      accelerationTime_(accelerationTime), begun_(false),
      lastOdometryUpdateSucceeded_(false), lastCommands_(),
      lastWheelPositions_() {}

bool DDSM210MecanumChassis::configurationIsValid() const {
  return kinematics_.isValid() && odometry_.isValid() &&
         maximumCommand_ > 0 && maximumCommand_ <= 32767 &&
         isfinite(commandUnitsPerRpm_) && commandUnitsPerRpm_ > 0.0f &&
         directionIsValid(motorDirections_.frontLeft) &&
         directionIsValid(motorDirections_.frontRight) &&
         directionIsValid(motorDirections_.rearLeft) &&
         directionIsValid(motorDirections_.rearRight);
}

bool DDSM210MecanumChassis::begin(unsigned long baudrate,
                                  uint32_t startupDelayMs) {
  if (!configurationIsValid() || baudrate == 0UL) {
    begun_ = false;
    return false;
  }

  motorSerial_.begin(baudrate);
  communication_.pSerial = &motorSerial_;
  communication_.set_ddsm_type(TYPE_DDSM210);

  if (startupDelayMs > 0U) {
    delay(startupDelayMs);
  }
  communication_.clear_ddsm_buffer();

  odometry_.reset();
  lastCommands_ = DDSM210Commands();
  lastWheelPositions_ = WheelPositionSamples();
  lastOdometryUpdateSucceeded_ = false;
  begun_ = true;
  return true;
}

DDSM210Commands
DDSM210MecanumChassis::drive(const ChassisVelocity &velocity) {
  if (!begun_) {
    lastCommands_ = DDSM210Commands();
    return lastCommands_;
  }

  const WheelSpeeds wheelRadPerSecond = kinematics_.inverse(velocity);
  const WheelSpeeds wheelRpm =
      MecanumKinematics::radPerSecToRpm(wheelRadPerSecond);

  lastCommands_ = MecanumKinematics::toDDSM210Commands(
      wheelRpm, motorDirections_, maximumCommand_, commandUnitsPerRpm_);

  communication_.ddsm210_ctrl_4(
      lastCommands_.wheel1, lastCommands_.wheel2, lastCommands_.wheel3,
      lastCommands_.wheel4, accelerationTime_);
  return lastCommands_;
}

DDSM210Commands DDSM210MecanumChassis::drive(
    float vxMetersPerSecond, float vyMetersPerSecond,
    float wzRadiansPerSecond) {
  return drive(ChassisVelocity(vxMetersPerSecond, vyMetersPerSecond,
                               wzRadiansPerSecond));
}

DDSM210Commands DDSM210MecanumChassis::stop() {
  return drive(ChassisVelocity());
}

bool DDSM210MecanumChassis::updateOdometry() {
  if (!begun_) {
    lastOdometryUpdateSucceeded_ = false;
    return false;
  }

  WheelPositionSamples samples;
  if (!readWheelPositions(samples) || !odometry_.update(samples)) {
    lastOdometryUpdateSucceeded_ = false;
    return false;
  }

  lastWheelPositions_ = samples;
  lastOdometryUpdateSucceeded_ = true;
  return true;
}

bool DDSM210MecanumChassis::updateOdometryWithImuHeading(
    double yawRadians, double fusionWeight) {
  if (!begun_) {
    lastOdometryUpdateSucceeded_ = false;
    return false;
  }

  WheelPositionSamples samples;
  const ImuHeadingMeasurement imuHeading(yawRadians, fusionWeight);
  if (!readWheelPositions(samples) ||
      !odometry_.update(samples, imuHeading)) {
    lastOdometryUpdateSucceeded_ = false;
    return false;
  }

  lastWheelPositions_ = samples;
  lastOdometryUpdateSucceeded_ = true;
  return true;
}

bool DDSM210MecanumChassis::updateOdometryWithImuHeadingDegrees(
    double yawDegrees, double fusionWeight) {
  return updateOdometryWithImuHeading(
      yawDegrees * kDegreesToRadians, fusionWeight);
}

bool DDSM210MecanumChassis::readWheelPositions(
    WheelPositionSamples &samples) {
  DDSM210OdometryFeedback feedback[4];
  for (uint8_t index = 0; index < 4; ++index) {
    const uint8_t motorId = static_cast<uint8_t>(index + 1U);
    if (!communication_.ddsm210_get_odometry(motorId, feedback[index])) {
      return false;
    }
  }

  samples = WheelPositionSamples(
      WheelPositionSample(feedback[0].mileage, feedback[0].position),
      WheelPositionSample(feedback[1].mileage, feedback[1].position),
      WheelPositionSample(feedback[2].mileage, feedback[2].position),
      WheelPositionSample(feedback[3].mileage, feedback[3].position));
  return true;
}

bool DDSM210MecanumChassis::driveAndUpdateOdometry(
    const ChassisVelocity &velocity) {
  drive(velocity);
  return updateOdometry();
}

bool DDSM210MecanumChassis::driveAndUpdateOdometry(
    float vxMetersPerSecond, float vyMetersPerSecond,
    float wzRadiansPerSecond) {
  return driveAndUpdateOdometry(
      ChassisVelocity(vxMetersPerSecond, vyMetersPerSecond,
                      wzRadiansPerSecond));
}

void DDSM210MecanumChassis::resetOdometry(const Pose2D &newPose) {
  odometry_.reset(newPose);
  lastOdometryUpdateSucceeded_ = false;
}

bool DDSM210MecanumChassis::setImuHeadingReference(
    double sensorYawRadians, double worldHeadingRadians) {
  return odometry_.setImuHeadingReference(sensorYawRadians,
                                          worldHeadingRadians);
}

void DDSM210MecanumChassis::clearImuHeadingReference() {
  odometry_.clearImuHeadingReference();
}

bool DDSM210MecanumChassis::hasImuHeadingReference() const {
  return odometry_.hasImuHeadingReference();
}

const Pose2D &DDSM210MecanumChassis::pose() const {
  return odometry_.pose();
}

const WheelPositionSamples &
DDSM210MecanumChassis::lastWheelPositions() const {
  return lastWheelPositions_;
}

const DDSM210Commands &DDSM210MecanumChassis::lastCommands() const {
  return lastCommands_;
}

bool DDSM210MecanumChassis::lastOdometryUpdateSucceeded() const {
  return lastOdometryUpdateSucceeded_;
}

bool DDSM210MecanumChassis::isBegun() const { return begun_; }

const MecanumKinematics &DDSM210MecanumChassis::kinematics() const {
  return kinematics_;
}

MecanumOdometry &DDSM210MecanumChassis::odometry() {
  return odometry_;
}

const MecanumOdometry &DDSM210MecanumChassis::odometry() const {
  return odometry_;
}

DDSM_CTRL &DDSM210MecanumChassis::communication() {
  return communication_;
}

const DDSM_CTRL &DDSM210MecanumChassis::communication() const {
  return communication_;
}

} // namespace mecanum
