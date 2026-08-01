#ifndef ARM_MOTOR_CONTROLLER_H
#define ARM_MOTOR_CONTROLLER_H

#include <AccelStepper.h>
#include <Arduino.h>
#include <TTL_STEPPER.h>

class ArmMotorController {
public:
  ArmMotorController();

  void begin();

  void beginM5();

  void enableAll();
  void disableAll();
  void enableM5();
  void enableM6();
  void enableM7();
  void disableM5();
  void disableM6();
  void disableM7();

  void moveM5ByDegrees(
      float outputDegrees, bool waitUntilDone = true);
  void moveM5ToDegrees(
      float outputDegrees, bool waitUntilDone = true);
  void rotateM5ClockwiseByDegrees(
      float outputDegrees, bool waitUntilDone = true);
  void rotateM5CounterClockwiseByDegrees(
      float outputDegrees, bool waitUntilDone = true);
  void setM5CurrentAngle(float outputDegrees);
  void serviceM5();
  bool isM5Running();
  void stopM5Immediately();

  void moveM6ByMillimeters(float extensionMillimeters);
  void extendM6ByMillimeters(float distanceMillimeters);
  void retractM6ByMillimeters(float distanceMillimeters);

  void moveM7ByMillimeters(float verticalMillimeters);
  void raiseM7ByMillimeters(float distanceMillimeters);
  void lowerM7ByMillimeters(float distanceMillimeters);
  void setM7LeadMillimetersPerRevolution(
      float leadMillimeters);
  void stopM6();
  void stopM7();

  void setM5MotionProfile(
      float maximumStepRate, float stepAcceleration);
  void setSerialMotionProfile(
      uint16_t speedRpm, uint8_t acceleration);

  long m5PulsesForDegrees(float outputDegrees) const;
  uint32_t m6PulsesForMillimeters(
      float extensionMillimeters) const;
  uint32_t m7PulsesForMillimeters(
      float verticalMillimeters) const;

private:
  void clearSerialResponses();
  void setSerialMotorEnabled(uint8_t address, bool enabled);
  void moveSerialMotorByPulses(
      uint8_t address,
      uint8_t direction,
      uint32_t pulses);

  AccelStepper motorM5_;
  HardwareSerial serialM6M7_;
  TTL_Protocol serialProtocol_;
  uint16_t serialSpeedRpm_;
  uint8_t serialAcceleration_;
  float m7LeadMmPerRevolution_;
};

#endif
