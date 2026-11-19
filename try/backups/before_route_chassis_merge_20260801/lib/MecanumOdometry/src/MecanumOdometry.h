#ifndef MECANUM_ODOMETRY_H
#define MECANUM_ODOMETRY_H

#include <MecanumKinematics.h>

#include <stdint.h>

namespace mecanum {

struct Pose2D {
  double xMeters;
  double yMeters;
  double headingRadians;

  Pose2D(double x = 0.0, double y = 0.0, double heading = 0.0)
      : xMeters(x), yMeters(y), headingRadians(heading) {}
};

struct WheelPositionSample {
  int32_t wholeTurns;
  uint16_t position;

  WheelPositionSample(int32_t turns = 0, uint16_t singleTurnPosition = 0)
      : wholeTurns(turns), position(singleTurnPosition) {}
};

struct WheelPositionSamples {
  WheelPositionSample frontLeft;
  WheelPositionSample frontRight;
  WheelPositionSample rearLeft;
  WheelPositionSample rearRight;

  WheelPositionSamples(
      const WheelPositionSample &wheel1 = WheelPositionSample(),
      const WheelPositionSample &wheel2 = WheelPositionSample(),
      const WheelPositionSample &wheel3 = WheelPositionSample(),
      const WheelPositionSample &wheel4 = WheelPositionSample())
      : frontLeft(wheel1), frontRight(wheel2), rearLeft(wheel3),
        rearRight(wheel4) {}
};

struct ChassisDisplacement {
  double dxBodyMeters;
  double dyBodyMeters;
  double dHeadingRadians;

  ChassisDisplacement(double dx = 0.0, double dy = 0.0,
                      double dHeading = 0.0)
      : dxBodyMeters(dx), dyBodyMeters(dy),
        dHeadingRadians(dHeading) {}
};

struct ImuHeadingMeasurement {
  double yawRadians;
  double fusionWeight;

  ImuHeadingMeasurement(double yaw = 0.0, double weight = 1.0)
      : yawRadians(yaw), fusionWeight(weight) {}
};

class MecanumOdometry {
public:

  MecanumOdometry(const MecanumKinematics &kinematics,
                  const WheelDirections &motorDirections,
                  uint32_t positionUnitsPerTurn = 65536U);

  bool isValid() const;

  void reset(const Pose2D &newPose = Pose2D());

  bool update(const WheelPositionSamples &samples);

  bool update(const WheelPositionSamples &samples,
              const ImuHeadingMeasurement &imuHeading);

  bool setImuHeadingReference(double sensorYawRadians,
                              double worldHeadingRadians = 0.0);

  void clearImuHeadingReference();

  bool hasImuHeadingReference() const;

  bool setMaximumWheelDeltaTurns(double maximumTurns);

  double maximumWheelDeltaTurns() const;

  bool isInitialized() const;

  const Pose2D &pose() const;

  const ChassisDisplacement &lastDisplacement() const;

  uint32_t positionUnitsPerTurn() const;

private:
  double continuousTurns(const WheelPositionSample &sample) const;
  bool samplesAreValid(const WheelPositionSamples &samples) const;
  bool updateInternal(const WheelPositionSamples &samples,
                      const ImuHeadingMeasurement *imuHeading);
  bool imuMeasurementIsValid(
      const ImuHeadingMeasurement &measurement) const;
  static double normalizeAngle(double radians);

  MecanumKinematics kinematics_;
  WheelDirections motorDirections_;
  uint32_t positionUnitsPerTurn_;
  double maximumWheelDeltaTurns_;

  bool initialized_;
  bool imuHeadingReferenceValid_;
  double imuHeadingOffsetRadians_;
  double previousTurns_[4];
  Pose2D pose_;
  ChassisDisplacement lastDisplacement_;
};

}

#endif

