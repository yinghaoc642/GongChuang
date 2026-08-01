#ifndef PULSE_MECANUM_CHASSIS_H
#define PULSE_MECANUM_CHASSIS_H

#include <AccelStepper.h>
#include <Arduino.h>
#include <MecanumKinematics.h>

#include <stdint.h>

namespace mecanum {

struct PulseMecanumPins {
  uint8_t motor1StepPin;
  uint8_t motor1DirectionPin;
  uint8_t motor2StepPin;
  uint8_t motor2DirectionPin;
  uint8_t motor3StepPin;
  uint8_t motor3DirectionPin;
  uint8_t motor4StepPin;
  uint8_t motor4DirectionPin;
  uint8_t enablePin;
  bool enableActiveLow;

  PulseMecanumPins(uint8_t m1Step, uint8_t m1Direction,
                   uint8_t m2Step, uint8_t m2Direction,
                   uint8_t m3Step, uint8_t m3Direction,
                   uint8_t m4Step, uint8_t m4Direction,
                   uint8_t commonEnablePin,
                   bool commonEnableActiveLow = true)
      : motor1StepPin(m1Step), motor1DirectionPin(m1Direction),
        motor2StepPin(m2Step), motor2DirectionPin(m2Direction),
        motor3StepPin(m3Step), motor3DirectionPin(m3Direction),
        motor4StepPin(m4Step), motor4DirectionPin(m4Direction),
        enablePin(commonEnablePin),
        enableActiveLow(commonEnableActiveLow) {}
};

class PulseMecanumChassis {
public:

  PulseMecanumChassis(
      const PulseMecanumPins &pins,
      const MecanumKinematics &kinematics,
      const WheelDirections &motorDirections =
          WheelDirections(-1, +1, -1, +1),
      float pulsesPerWheelRevolution = 3200.0f,
      float linearPulsesPerMeter = 10000.0f,
      float maximumPulseRate = 30000.0f,
      uint16_t minimumPulseWidthMicros = 1U);

  bool begin(bool enableMotors = true);

  bool drive(const ChassisVelocity &velocity);

  bool drive(float vxMetersPerSecond, float vyMetersPerSecond,
             float wzRadiansPerSecond);

  bool drive(float forwardMetersPerSecond, float wzRadiansPerSecond);

  bool setVelocity(const ChassisVelocity &velocity);
  bool setVelocity(float vxMetersPerSecond, float vyMetersPerSecond,
                   float wzRadiansPerSecond);
  bool setVelocity(float forwardMetersPerSecond,
                   float wzRadiansPerSecond);

  bool run();

  void stop();

  bool setEnabled(bool enabled);
  bool enable();
  void disable();

  bool isBegun() const;
  bool isEnabled() const;
  bool configurationIsValid() const;

  const WheelSpeeds &lastPulseRates() const;

  float lastLimitScale() const;

  const ChassisVelocity &lastRequestedVelocity() const;

  const MecanumKinematics &kinematics() const;

private:
  bool pinsAreValid() const;
  bool pinsAreUnique() const;
  bool velocityIsValid(const ChassisVelocity &velocity) const;
  bool calculatePulseRates(const ChassisVelocity &velocity,
                           WheelSpeeds &pulseRates,
                           float &limitScale) const;
  void applyPulseRates(const WheelSpeeds &pulseRates);
  uint8_t enableLevel(bool enabled) const;

  PulseMecanumPins pins_;
  MecanumKinematics kinematics_;
  WheelDirections motorDirections_;
  float pulsesPerWheelRevolution_;
  float linearPulsesPerMeter_;
  float maximumPulseRate_;
  uint16_t minimumPulseWidthMicros_;

  AccelStepper motor1_;
  AccelStepper motor2_;
  AccelStepper motor3_;
  AccelStepper motor4_;

  bool begun_;
  bool enabled_;
  WheelSpeeds lastPulseRates_;
  float lastLimitScale_;
  ChassisVelocity lastRequestedVelocity_;
};

}

#endif

