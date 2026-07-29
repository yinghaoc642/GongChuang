#ifndef ARM_MOTOR_CONTROLLER_H
#define ARM_MOTOR_CONTROLLER_H

#include <AccelStepper.h>
#include <Arduino.h>
#include <TTL_STEPPER.h>

/*
 * 圆柱转台机械臂 M5/M6/M7 基础控制库
 *
 * 角度约定：
 *   M5：正角度逆时针，负角度顺时针（转台输出轴角度）；
 *
 * 直线位移约定：
 *   M6：正毫米伸长，负毫米收缩；
 *   M7：正毫米上升，负毫米下降。
 *
 * M5 的阻塞式接口会等待运动完成。
 * M6/M7 使用串口驱动器，接口在命令发送后返回，电机继续自行运动。
 */
class ArmMotorController {
public:
  ArmMotorController();

  // 初始化固定引脚、串口和运动参数，并使能 M5/M6/M7。
  void begin();
  // 只初始化 M5；不会初始化或占用 M6/M7 的 PA2/PA3 串口。
  void beginM5();

  void enableAll();
  void disableAll();
  void enableM5();
  void enableM6();
  void enableM7();
  void disableM5();
  void disableM6();
  void disableM7();

  // M5：按转台输出角运动。
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

  // M6：按齿条直线位移做相对运动，正数伸长，负数收缩。
  void moveM6ByMillimeters(float extensionMillimeters);
  void extendM6ByMillimeters(float distanceMillimeters);
  void retractM6ByMillimeters(float distanceMillimeters);

  // M7：按丝杠直线位移做相对运动，正数上升，负数下降。
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
