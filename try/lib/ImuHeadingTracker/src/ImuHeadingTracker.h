#pragma once

#include <Arduino.h>

namespace gongchuang {

class ImuHeadingTracker {
 public:
  explicit ImuHeadingTracker(HardwareSerial &serial);

  void begin(uint32_t baudrate);
  void service();

  bool initialized() const;
  bool fresh() const;
  float headingDegrees() const;

  static float wrapDeltaDegrees(float degrees);

 private:
  HardwareSerial &serial_;
  bool initialized_;
  float lastSignedRawDegrees_;
  float counterClockwiseDegrees_;
  uint32_t lastReceiveMs_;
  uint8_t frame_[11];
  uint8_t frameIndex_;

  void updateContinuousHeading(int16_t rawYawValue);
};

}
