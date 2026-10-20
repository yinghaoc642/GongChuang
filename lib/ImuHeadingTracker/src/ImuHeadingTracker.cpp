#include "ImuHeadingTracker.h"

#include <JY901.h>
#include <RobotConfig.h>

namespace gongchuang {

ImuHeadingTracker::ImuHeadingTracker(
    HardwareSerial &serial)
    : serial_(serial),
      initialized_(false),
      lastSignedRawDegrees_(0.0f),
      counterClockwiseDegrees_(0.0f),
      lastReceiveMs_(0UL),
      frame_{0U},
      frameIndex_(0U) {}

void ImuHeadingTracker::begin(uint32_t baudrate) {
  serial_.begin(baudrate);
}

float ImuHeadingTracker::wrapDeltaDegrees(
    float degrees) {
  while (degrees >= 180.0f) {
    degrees -= 360.0f;
  }
  while (degrees < -180.0f) {
    degrees += 360.0f;
  }
  return degrees;
}

void ImuHeadingTracker::updateContinuousHeading(
    int16_t rawYawValue) {
  const float rawDegrees =
      static_cast<float>(rawYawValue) /
      32768.0f * 180.0f;
  const float counterClockwiseSignedRaw =
      rawDegrees *
      static_cast<float>(
          config::imu::COUNTERCLOCKWISE_SIGN);

  lastReceiveMs_ = millis();
  if (!initialized_) {
    initialized_ = true;
    lastSignedRawDegrees_ = counterClockwiseSignedRaw;
    counterClockwiseDegrees_ = 0.0f;
    return;
  }

  const float continuousDeltaDegrees =
      wrapDeltaDegrees(
          counterClockwiseSignedRaw -
          lastSignedRawDegrees_);
  counterClockwiseDegrees_ += continuousDeltaDegrees;
  lastSignedRawDegrees_ = counterClockwiseSignedRaw;
}

void ImuHeadingTracker::service() {
  while (serial_.available()) {
    const uint8_t incomingByte =
        static_cast<uint8_t>(serial_.read());
    JY901.CopeSerialData(incomingByte);

    if (frameIndex_ == 0U) {
      if (incomingByte == 0x55U) {
        frame_[frameIndex_++] = incomingByte;
      }
      continue;
    }

    frame_[frameIndex_++] = incomingByte;
    if (frameIndex_ != sizeof(frame_)) {
      continue;
    }

    uint8_t checksum = 0U;
    for (uint8_t i = 0U; i < 10U; ++i) {
      checksum =
          static_cast<uint8_t>(checksum + frame_[i]);
    }

    if (checksum == frame_[10] &&
        frame_[0] == 0x55U &&
        frame_[1] == 0x53U) {
      const uint16_t yawUnsigned =
          static_cast<uint16_t>(frame_[6]) |
          (static_cast<uint16_t>(frame_[7]) << 8);
      updateContinuousHeading(
          static_cast<int16_t>(yawUnsigned));
    }

    if (checksum != frame_[10] &&
        frame_[10] == 0x55U) {
      frame_[0] = 0x55U;
      frameIndex_ = 1U;
    } else {
      frameIndex_ = 0U;
    }
  }
}

bool ImuHeadingTracker::initialized() const {
  return initialized_;
}

bool ImuHeadingTracker::fresh() const {
  return initialized_ &&
         millis() - lastReceiveMs_ <=
             config::imu::STALE_TIMEOUT_MS;
}

float ImuHeadingTracker::headingDegrees() const {
  return counterClockwiseDegrees_;
}

}
