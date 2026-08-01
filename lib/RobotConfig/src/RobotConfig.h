#pragma once

#include <stdint.h>

#ifndef GONGCHUANG_RUN_MODE
#define GONGCHUANG_RUN_MODE 0
#endif

static_assert(
    GONGCHUANG_RUN_MODE == 0 ||
        GONGCHUANG_RUN_MODE == 1,
    "GONGCHUANG_RUN_MODE must be 0 or 1");

namespace gongchuang {
namespace config {

namespace m7_experiment {

// Experimental M7 profile. Set this single flag to false to restore every
// pre-experiment M7 speed and acceleration without touching geometry, delays,
// or transfer parallelism.
constexpr bool DOUBLE_SPEED_AND_ACCELERATION = true;
// The first experiment already doubled M7 physical acceleration. The current
// experiment doubles that effective acceleration once more while keeping the
// same one-flag rollback through DOUBLE_SPEED_AND_ACCELERATION.
constexpr bool DOUBLE_ACCELERATION_AGAIN = true;

// The EMM42 V5 position-mode documentation limits target speed to 5000 RPM.
// Requested 2x speeds are saturated here instead of sending unsupported values.
constexpr uint16_t EMM42_V5_MAXIMUM_SPEED_RPM = 5000U;

constexpr uint16_t doubledSpeedRpm(uint16_t baselineRpm) {
  return DOUBLE_SPEED_AND_ACCELERATION
             ? (static_cast<uint32_t>(baselineRpm) * 2U >
                        EMM42_V5_MAXIMUM_SPEED_RPM
                    ? EMM42_V5_MAXIMUM_SPEED_RPM
                    : static_cast<uint16_t>(
                          static_cast<uint32_t>(baselineRpm) * 2U))
             : baselineRpm;
}

// EMM acceleration is approximately proportional to 1/(256 - byte).
// Doubling physical acceleration therefore maps b to (256 + b) / 2,
// rounded upward. Byte 0 remains reserved for direct/instant start.
constexpr uint8_t doubledAccelerationStep(uint8_t baseline) {
  return baseline >= 255U
             ? 255U
             : static_cast<uint8_t>(
                   (257U +
                    static_cast<uint16_t>(baseline)) /
                   2U);
}

constexpr uint8_t experimentalAcceleration(uint8_t baseline) {
  return !DOUBLE_SPEED_AND_ACCELERATION
             ? baseline
             : (DOUBLE_ACCELERATION_AGAIN
                    ? doubledAccelerationStep(
                          doubledAccelerationStep(baseline))
                    : doubledAccelerationStep(baseline));
}

static_assert(
    !DOUBLE_SPEED_AND_ACCELERATION ||
        doubledSpeedRpm(65535U) <=
            EMM42_V5_MAXIMUM_SPEED_RPM,
    "M7 experimental speed must stay inside EMM42 V5 range");
static_assert(
    !DOUBLE_SPEED_AND_ACCELERATION ||
        experimentalAcceleration(244U) <= 255U,
    "M7 experimental acceleration byte overflow");

}

namespace arm_hardware {

constexpr uint16_t FULL_STEPS_PER_REVOLUTION = 200U;
constexpr uint16_t M5_MICROSTEPS = 16U;
constexpr uint16_t M6_MICROSTEPS = 256U;
constexpr uint16_t M7_MICROSTEPS = 256U;
constexpr float M5_GEAR_RATIO = 5.0f;
constexpr float M6_PINION_PITCH_DIAMETER_MM = 35.0f;
constexpr float M7_LEAD_MM_PER_REVOLUTION = 12.0f;

constexpr float M6_STARTUP_WORKING_ZERO_OFFSET_MM = 10.0f;
constexpr float M7_STARTUP_WORKING_ZERO_OFFSET_MM = 10.0f;

constexpr uint16_t M7_TRAVEL_SPEED_RPM = 6740U;
constexpr uint8_t M7_TRAVEL_ACCELERATION = 239U;
constexpr float M7_ZERO_SOFT_LANDING_DISTANCE_MM = 2.0f;
constexpr uint16_t M7_ZERO_SOFT_LANDING_SPEED_RPM = 2160U;
constexpr uint8_t M7_ZERO_SOFT_LANDING_ACCELERATION = 171U;

constexpr float M6_CONTACT_SOFT_LANDING_DISTANCE_MM = 2.0f;
constexpr uint8_t M6_CONTACT_SOFT_LANDING_ACCELERATION = 149U;
constexpr float M7_CONTACT_SOFT_LANDING_DISTANCE_MM = 2.0f;
constexpr uint8_t M7_CONTACT_SOFT_LANDING_ACCELERATION = 171U;

}

namespace arm_transfer {

constexpr float CONTAINER_PICK_ANGLE_DEGREES = -103.0f;
constexpr float CONTAINER_PLACE_ANGLE_DEGREES = -102.0f;
constexpr float CONTAINER_RETURN_PLACE_ANGLE_DEGREES = -102.0f;
constexpr float CONTAINER_PICK_EXTENSION_MM = 1.0f;
// Every tray pickup descends 3 mm farther than the previous 38 mm setting.
// The second slot keeps its independent extra 3 mm compensation below.
constexpr float CONTAINER_PICK_PHYSICAL_LOWER_MM = 41.0f;
constexpr float CONTAINER_PLACE_PHYSICAL_LOWER_MM = 40.0f;
constexpr float SECOND_PICK_EXTRA_LOWER_MM = 3.0f;
constexpr float RING_RETURN_PICK_EXTRA_LOWER_MM = 3.0f;
// All three target placements descend another 3 mm from the previous
// 2 mm correction. Return-pick depth remains independently calibrated.
constexpr float TARGET_PLACE_EXTRA_LOWER_MM = 5.0f;
// Apply the measured radial correction once when the visual ring coordinates
// are converted to a gripper pose. The stored pose is then shared by placing
// and picking, so the return pass targets the material that was actually put
// down instead of reintroducing the uncorrected 8 mm offset.
constexpr float MAPPED_RING_EXTENSION_REDUCTION_MM = 8.0f;

// M7 reaches and stops at these measured collision-clearance heights;
// only then do M5 and M6 start their planar moves together. M6/M7 share
// one serial bus and must never move or query concurrently.
constexpr float TRAY_ROTATION_CLEARANCE_HEIGHT_MM = -10.0f;
constexpr float RING_ROTATION_CLEARANCE_HEIGHT_MM = -10.0f;

constexpr float M5_STANDARD_MAXIMUM_STEP_RATE = 210600.0f;
constexpr float M5_STANDARD_STEP_ACCELERATION = 81000.0f;
constexpr float M5_PLACE_MAXIMUM_STEP_RATE = 181440.0f;
constexpr float M5_PLACE_STEP_ACCELERATION = 72000.0f;
// Ring-to-tray uses exactly the same planar profile as tray-to-ring. Keep
// aliases instead of duplicated literals so the two directions cannot drift.
constexpr float M5_RETURN_MAXIMUM_STEP_RATE =
    M5_PLACE_MAXIMUM_STEP_RATE;
constexpr float M5_RETURN_STEP_ACCELERATION =
    M5_PLACE_STEP_ACCELERATION;
constexpr float M5_LOADED_RETURN_MAXIMUM_STEP_RATE =
    M5_PLACE_MAXIMUM_STEP_RATE;
constexpr float M5_LOADED_RETURN_STEP_ACCELERATION =
    M5_PLACE_STEP_ACCELERATION;
constexpr float M5_RAW_MAXIMUM_STEP_RATE = 210600.0f;
constexpr float M5_RAW_STEP_ACCELERATION = 90000.0f;

constexpr uint16_t M6_STANDARD_SPEED_RPM = 702U;
constexpr uint8_t M6_STANDARD_ACCELERATION = 199U;
constexpr uint16_t M6_RAW_SPEED_RPM = 756U;
constexpr uint8_t M6_RAW_ACCELERATION = 205U;
constexpr uint16_t M6_PLACE_SPEED_RPM = 810U;
constexpr uint8_t M6_PLACE_ACCELERATION = 208U;
constexpr uint16_t M6_RETURN_SPEED_RPM = M6_PLACE_SPEED_RPM;
constexpr uint8_t M6_RETURN_ACCELERATION = M6_PLACE_ACCELERATION;
constexpr uint16_t M6_LOADED_RETURN_SPEED_RPM = M6_PLACE_SPEED_RPM;
constexpr uint8_t M6_LOADED_RETURN_ACCELERATION =
    M6_PLACE_ACCELERATION;
static_assert(
    M5_LOADED_RETURN_MAXIMUM_STEP_RATE ==
        M5_PLACE_MAXIMUM_STEP_RATE &&
        M5_RETURN_MAXIMUM_STEP_RATE ==
            M5_PLACE_MAXIMUM_STEP_RATE,
    "M5 tray/ring transfer speeds must remain identical");
static_assert(
    M5_LOADED_RETURN_STEP_ACCELERATION ==
        M5_PLACE_STEP_ACCELERATION &&
        M5_RETURN_STEP_ACCELERATION ==
            M5_PLACE_STEP_ACCELERATION,
    "M5 tray/ring transfer accelerations must remain identical");
static_assert(
    M6_RETURN_SPEED_RPM == M6_PLACE_SPEED_RPM &&
        M6_LOADED_RETURN_SPEED_RPM == M6_PLACE_SPEED_RPM,
    "M6 tray/ring transfer speeds must remain identical");
static_assert(
    M6_RETURN_ACCELERATION == M6_PLACE_ACCELERATION &&
        M6_LOADED_RETURN_ACCELERATION == M6_PLACE_ACCELERATION,
    "M6 tray/ring transfer accelerations must remain identical");

constexpr uint16_t M7_RETURN_SPEED_RPM =
    arm_hardware::M7_TRAVEL_SPEED_RPM;
constexpr uint8_t M7_RETURN_ACCELERATION =
    arm_hardware::M7_TRAVEL_ACCELERATION;
static_assert(
    M7_RETURN_SPEED_RPM == arm_hardware::M7_TRAVEL_SPEED_RPM &&
        M7_RETURN_ACCELERATION ==
            arm_hardware::M7_TRAVEL_ACCELERATION,
    "M7 tray/ring transfer profiles must remain identical");

constexpr uint16_t M7_RING_PLACE_SPEED_RPM = 2700U;
constexpr uint8_t M7_RING_PLACE_ACCELERATION = 199U;

}

namespace gripper {

constexpr float CLOSE_ANGLE_DEGREES = 87.0f;
constexpr float OPEN_ANGLE_DEGREES = 20.0f;
constexpr float MAX_OPEN_ANGLE_DEGREES = -101.0f;

}

namespace vision_link {

constexpr int16_t IMAGE_CENTER_X = 160;
constexpr int16_t IMAGE_CENTER_Y = 120;
constexpr uint32_t REQUEST_REPEAT_MS = 1000UL;
constexpr uint32_t MODE_SWITCH_GUARD_PREVIOUS_MS = 100UL;
constexpr uint32_t MODE_SWITCH_GUARD_MS = 10UL;
static_assert(
    MODE_SWITCH_GUARD_MS * 10UL ==
        MODE_SWITCH_GUARD_PREVIOUS_MS,
    "Vision mode-switch guard must remain reduced by 90 percent");
constexpr uint32_t COORDINATE_STALE_MS = 1500UL;
constexpr uint16_t RECEIVE_LINE_CAPACITY = 96U;

}

namespace imu {

constexpr int8_t COUNTERCLOCKWISE_SIGN = +1;
constexpr uint32_t STALE_TIMEOUT_MS = 600UL;

}

}
}
