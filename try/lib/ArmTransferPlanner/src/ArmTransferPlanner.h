#pragma once

#include <stdint.h>

namespace gongchuang {

struct ArmPose {
  float standardFrameAngleDegrees;
  float extensionMm;
  float heightMm;

  ArmPose(
      float angleDegrees = 0.0f,
      float extension = 0.0f,
      float height = 0.0f)
      : standardFrameAngleDegrees(angleDegrees),
        extensionMm(extension),
        heightMm(height) {}
};

namespace arm_transfer {

enum MotionProfile : uint8_t {
  PROFILE_STANDARD,
  PROFILE_RAW,
  PROFILE_RING_PLACE,
  PROFILE_RING_RETURN
};

MotionProfile selectMotionProfile(
    bool rawFastProfile,
    bool mapSource,
    bool mapDestination,
    bool gentleDestinationRelease);

ArmPose containerPickPose(uint8_t placementSequenceIndex);
ArmPose containerPlacePose();
ArmPose containerReturnPlacePose();

}
}
