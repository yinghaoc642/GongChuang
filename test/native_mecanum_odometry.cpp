#include <MecanumKinematics.h>
#include <MecanumOdometry.h>

#include <assert.h>
#include <math.h>
#include <iostream>

using namespace mecanum;

namespace {

const double kPi = 3.1415926535897932384626433832795;
const double kTolerance = 1.0e-6;
const WheelDirections kIdentityWheelDirections(+1, +1, +1, +1);

bool near(double actual, double expected, double tolerance = kTolerance) {
  return fabs(actual - expected) <= tolerance;
}

WheelPositionSamples allWheels(int32_t turns, uint16_t position = 0) {
  const WheelPositionSample sample(turns, position);
  return WheelPositionSamples(sample, sample, sample, sample);
}

void testForwardOneTurn() {
  const MecanumKinematics model =
      MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
  MecanumOdometry odometry(model, kIdentityWheelDirections);

  assert(odometry.update(allWheels(0)));
  assert(odometry.update(allWheels(1)));

  const Pose2D &pose = odometry.pose();
  assert(near(pose.xMeters, kPi * 0.1));
  assert(near(pose.yMeters, 0.0));
  assert(near(pose.headingRadians, 0.0));
}

void testQuarterTurnUsesSingleTurnPosition() {
  const MecanumKinematics model =
      MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
  MecanumOdometry odometry(
      model, kIdentityWheelDirections, 65536U);

  assert(odometry.update(allWheels(0)));
  assert(odometry.update(allWheels(0, 16384U)));
  assert(near(odometry.pose().xMeters, kPi * 0.025));
}

void testLeftTranslation() {
  const MecanumKinematics model =
      MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
  MecanumOdometry odometry(model, kIdentityWheelDirections);

  assert(odometry.update(allWheels(0)));
  assert(odometry.update(WheelPositionSamples(
      WheelPositionSample(-1, 0), WheelPositionSample(1, 0),
      WheelPositionSample(1, 0), WheelPositionSample(-1, 0))));

  assert(near(odometry.pose().xMeters, 0.0));
  assert(near(odometry.pose().yMeters, kPi * 0.1));
  assert(near(odometry.pose().headingRadians, 0.0));
}

void testCounterClockwiseRotation() {
  const MecanumKinematics model =
      MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
  MecanumOdometry odometry(model, kIdentityWheelDirections);

  assert(odometry.update(allWheels(0)));
  assert(odometry.update(WheelPositionSamples(
      WheelPositionSample(-1, 0), WheelPositionSample(1, 0),
      WheelPositionSample(-1, 0), WheelPositionSample(1, 0))));

  const double expectedHeading =
      (2.0 * kPi * 0.05) / 0.19125;
  assert(near(odometry.pose().xMeters, 0.0));
  assert(near(odometry.pose().yMeters, 0.0));
  assert(near(odometry.pose().headingRadians, expectedHeading));
}

void testMotorDirectionCorrection() {
  const MecanumKinematics model =
      MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
  MecanumOdometry odometry(
      model, WheelDirections(-1, +1, -1, +1));

  assert(odometry.update(allWheels(0)));
  assert(odometry.update(WheelPositionSamples(
      WheelPositionSample(-1, 0), WheelPositionSample(1, 0),
      WheelPositionSample(-1, 0), WheelPositionSample(1, 0))));

  assert(near(odometry.pose().xMeters, kPi * 0.1));
  assert(near(odometry.pose().yMeters, 0.0));
  assert(near(odometry.pose().headingRadians, 0.0));
}

void testJumpProtectionDoesNotMoveBaseline() {
  const MecanumKinematics model =
      MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
  MecanumOdometry odometry(model, kIdentityWheelDirections);

  assert(odometry.setMaximumWheelDeltaTurns(0.5));
  assert(odometry.update(allWheels(0)));
  assert(!odometry.update(allWheels(1)));
  assert(near(odometry.pose().xMeters, 0.0));

  assert(odometry.update(allWheels(0, 16384U)));
  assert(near(odometry.pose().xMeters, kPi * 0.025));
}

void testImuHeadingOverridesWheelHeading() {
  const MecanumKinematics model =
      MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
  MecanumOdometry odometry(model, kIdentityWheelDirections);

  
  assert(odometry.update(allWheels(0),
                         ImuHeadingMeasurement(kPi / 6.0, 1.0)));
  assert(odometry.hasImuHeadingReference());
  assert(near(odometry.pose().headingRadians, 0.0));

  
  assert(odometry.update(WheelPositionSamples(
                             WheelPositionSample(-1, 0),
                             WheelPositionSample(1, 0),
                             WheelPositionSample(-1, 0),
                             WheelPositionSample(1, 0)),
                         ImuHeadingMeasurement(kPi / 6.0, 1.0)));
  assert(near(odometry.pose().headingRadians, 0.0));
}

void testImuWrapUsesShortestAngle() {
  const MecanumKinematics model =
      MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
  MecanumOdometry odometry(model, kIdentityWheelDirections);

  const double degree = kPi / 180.0;
  assert(odometry.update(allWheels(0),
                         ImuHeadingMeasurement(179.0 * degree, 1.0)));

  
  assert(odometry.update(allWheels(0),
                         ImuHeadingMeasurement(-179.0 * degree, 1.0)));
  assert(near(odometry.pose().headingRadians, 2.0 * degree));
}

void testImuFusionWeight() {
  const MecanumKinematics model =
      MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
  MecanumOdometry odometry(model, kIdentityWheelDirections);

  assert(odometry.update(allWheels(0),
                         ImuHeadingMeasurement(0.0, 0.5)));
  assert(odometry.update(allWheels(0),
                         ImuHeadingMeasurement(kPi / 2.0, 0.5)));

  
  assert(near(odometry.pose().headingRadians, kPi / 4.0));
}

} 

int main() {
  testForwardOneTurn();
  testQuarterTurnUsesSingleTurnPosition();
  testLeftTranslation();
  testCounterClockwiseRotation();
  testMotorDirectionCorrection();
  testJumpProtectionDoesNotMoveBaseline();
  testImuHeadingOverridesWheelHeading();
  testImuWrapUsesShortestAngle();
  testImuFusionWeight();

  std::cout << "MecanumOdometry native tests passed" << std::endl;
  return 0;
}
