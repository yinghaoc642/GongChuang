#pragma once

#include <Arduino.h>

#include <RobotConfig.h>

namespace gongchuang {

class MaixCamClient {
 public:
  enum Request : uint8_t {
    STOP_REQUEST = 0x00U,
    RED_REQUEST = 0x01U,
    YELLOW_REQUEST = 0x02U,
    BLUE_REQUEST = 0x03U,
    GREEN_REQUEST = 0x04U,
    ALL_COLORS_REQUEST = 0x08U,
    HOUGH_CIRCLE_REQUEST = 0x09U,
    ENDPOINT_CIRCLE_REQUEST = 0x0AU
  };

  struct Coordinate {
    uint8_t targetId;
    uint8_t requestSequence;
    uint8_t mode;
    int16_t x;
    int16_t y;
    uint16_t metric;
    uint16_t confidence;
    uint32_t cameraTimestampMs;
    uint32_t sequence;
    uint32_t receivedMs;

    Coordinate();
  };

  using FaultHandler = void (*)(const char *reason);

  MaixCamClient(
      HardwareSerial &serial,
      Print &debug,
      FaultHandler faultHandler);

  void begin(uint32_t baudrate);
  void stopRequest();
  bool beginRequest(uint8_t request);
  void service();

  bool readNewCoordinate(
      uint32_t &lastSequence,
      uint8_t &targetId,
      int16_t &x,
      int16_t &y) const;

  const Coordinate &latest() const;
  bool initialized() const;
  uint8_t requestedMode() const;

 private:
  static constexpr size_t LINE_CAPACITY =
      config::vision_link::RECEIVE_LINE_CAPACITY;

  HardwareSerial &serial_;
  Print &debug_;
  FaultHandler faultHandler_;
  bool initialized_;
  uint8_t requestedMode_;
  uint8_t requestSequence_;
  bool modeCommandSent_;
  uint32_t modeSwitchStartMs_;
  uint32_t lastRequestMs_;
  char receiveLine_[LINE_CAPACITY];
  size_t receiveLength_;
  bool receiveOverflow_;
  Coordinate latest_;
  uint16_t telemetrySampleCount_;
  int16_t telemetryFirstX_;
  int16_t telemetryFirstY_;
  int16_t telemetryPreviousX_;
  int16_t telemetryPreviousY_;
  int16_t telemetryMinimumX_;
  int16_t telemetryMaximumX_;
  int16_t telemetryMinimumY_;
  int16_t telemetryMaximumY_;
  int32_t telemetrySumX_;
  int32_t telemetrySumY_;

  void fault(const char *reason);
  void writeRequestFrame(uint8_t request);
  void finishCoordinateLine();
  void resetCoordinateTelemetry();
  void printCoordinateTelemetry();
  static bool requestIsValid(uint8_t request);
  bool responseTargetMatchesMode(
      uint8_t target) const;
};

}
