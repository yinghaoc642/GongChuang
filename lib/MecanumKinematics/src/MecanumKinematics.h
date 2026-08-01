#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#include <stdint.h>

namespace mecanum {

struct ChassisVelocity {

  float vx;

  float vy;

  float wz;

  ChassisVelocity(float vxValue = 0.0f, float vyValue = 0.0f,
                  float wzValue = 0.0f)
      : vx(vxValue), vy(vyValue), wz(wzValue) {}
};

struct WheelSpeeds {

  float frontLeft;

  float frontRight;

  float rearLeft;

  float rearRight;

  WheelSpeeds(float wheel1 = 0.0f, float wheel2 = 0.0f,
              float wheel3 = 0.0f, float wheel4 = 0.0f)
      : frontLeft(wheel1), frontRight(wheel2), rearLeft(wheel3),
        rearRight(wheel4) {}
};

struct WheelDirections {

  int8_t frontLeft;

  int8_t frontRight;

  int8_t rearLeft;

  int8_t rearRight;

  WheelDirections(int8_t wheel1 = -1, int8_t wheel2 = +1,
                  int8_t wheel3 = -1, int8_t wheel4 = +1)
      : frontLeft(wheel1), frontRight(wheel2), rearLeft(wheel3),
        rearRight(wheel4) {}
};

struct DDSM210Commands {

  int16_t wheel1;

  int16_t wheel2;

  int16_t wheel3;

  int16_t wheel4;

  DDSM210Commands(int16_t command1 = 0, int16_t command2 = 0,
                  int16_t command3 = 0, int16_t command4 = 0)
      : wheel1(command1), wheel2(command2), wheel3(command3),
        wheel4(command4) {}
};

class MecanumKinematics {
public:

  MecanumKinematics(float wheelbaseMeters, float trackWidthMeters,
                    float wheelRadiusMeters);

  static MecanumKinematics fromMillimeters(float wheelbaseMm,
                                           float trackWidthMm,
                                           float wheelDiameterMm);

  bool isValid() const;

  float wheelbaseMeters() const;

  float trackWidthMeters() const;

  float wheelRadiusMeters() const;

  float rotationLeverArmMeters() const;

  WheelSpeeds inverse(const ChassisVelocity &velocity) const;

  ChassisVelocity forward(const WheelSpeeds &wheelRadPerSec) const;

  static WheelSpeeds radPerSecToRpm(const WheelSpeeds &wheelRadPerSec);

  static WheelSpeeds rpmToRadPerSec(const WheelSpeeds &wheelRpm);

  static float limitPreservingRatio(WheelSpeeds &values,
                                    float maximumMagnitude);

  static DDSM210Commands
  toDDSM210Commands(const WheelSpeeds &physicalWheelRpm,
                    const WheelDirections &motorDirections,
                    int16_t maximumCommand = 2100,
                    float commandUnitsPerRpm = 10.0f);

private:

  float wheelbaseMeters_;

  float trackWidthMeters_;

  float wheelRadiusMeters_;

  float rotationLeverArmMeters_;
};

}

#endif
