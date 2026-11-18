#include "ArmTransferPlanner.h"

#include <RobotConfig.h>

namespace gongchuang {
namespace arm_transfer {

namespace {

ArmPose containerPoseAtHeight(float heightMm) {
  return ArmPose(
      config::arm_transfer::CONTAINER_ANGLE_DEGREES,
      config::arm_transfer::CONTAINER_EXTENSION_MM,
      heightMm);
}

}

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
  return containerPoseAtHeight(-lowerMm);
}

ArmPose containerPlacePose() {
  return containerPoseAtHeight(
      -(
          config::arm_transfer::CONTAINER_PLACE_PHYSICAL_LOWER_MM -
          config::arm_hardware::M7_STARTUP_WORKING_ZERO_OFFSET_MM));
}

ArmPose containerReturnPlacePose() {
  ArmPose pose = containerPlacePose();
  pose.heightMm -=
      config::arm_transfer::CONTAINER_RETURN_PLACE_EXTRA_LOWER_MM;
  return pose;
}

}
}
