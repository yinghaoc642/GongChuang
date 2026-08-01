#include "ArmTransferPlanner.h"

#include <RobotConfig.h>

namespace gongchuang {
namespace arm_transfer {

MotionProfile selectMotionProfile(
    bool rawFastProfile,
    bool mapSource,
    bool mapDestination,
    bool gentleDestinationRelease) {
  if (rawFastProfile) {
    return PROFILE_RAW;
  }
  if (mapDestination && gentleDestinationRelease) {
    return PROFILE_RING_PLACE;
  }
  if (mapSource && !mapDestination) {
    return PROFILE_RING_RETURN;
  }
  return PROFILE_STANDARD;
}

ArmPose containerPickPose(uint8_t placementSequenceIndex) {
  const float lowerMm =
      config::arm_transfer::CONTAINER_PICK_PHYSICAL_LOWER_MM -
      config::arm_hardware::M7_STARTUP_WORKING_ZERO_OFFSET_MM +
      (placementSequenceIndex == 1U
           ? config::arm_transfer::SECOND_PICK_EXTRA_LOWER_MM
           : 0.0f);
  return ArmPose(
      config::arm_transfer::CONTAINER_PICK_ANGLE_DEGREES,
      config::arm_transfer::CONTAINER_PICK_EXTENSION_MM,
      -lowerMm);
}

ArmPose containerPlacePose() {
  return ArmPose(
      config::arm_transfer::CONTAINER_PLACE_ANGLE_DEGREES,
      0.0f,
      -(
          config::arm_transfer::CONTAINER_PLACE_PHYSICAL_LOWER_MM -
          config::arm_hardware::M7_STARTUP_WORKING_ZERO_OFFSET_MM));
}

ArmPose containerReturnPlacePose() {
  ArmPose pose = containerPlacePose();
  pose.standardFrameAngleDegrees =
      config::arm_transfer::CONTAINER_RETURN_PLACE_ANGLE_DEGREES;
  return pose;
}

}
}
