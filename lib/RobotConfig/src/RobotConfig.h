#pragma once

#include <stdint.h>

#ifndef GONGCHUANG_RUN_MODE
#define GONGCHUANG_RUN_MODE 0
#endif

static_assert(
    GONGCHUANG_RUN_MODE == 0 ||
        GONGCHUANG_RUN_MODE == 1 ||
        GONGCHUANG_RUN_MODE == 2,
    "GONGCHUANG_RUN_MODE must be 0, 1 or 2");

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
// User-authorized follow-up trial: raise the already-effective M7 speed and
// physical acceleration by another 30 percent. This remains isolated from M6
// and from all chassis tuning.
constexpr bool BOOST_EFFECTIVE_PROFILE_BY_30_PERCENT = true;

// Previous software saturation was 5000 RPM. The 30-percent trial deliberately
// raises the transmitted M7 ceiling to 6500 RPM without changing the wire
// protocol (the speed field is already uint16_t).
constexpr uint16_t PREVIOUS_M7_SPEED_LIMIT_RPM = 5000U;
constexpr uint16_t M7_EXPERIMENTAL_SPEED_LIMIT_RPM = 6500U;

constexpr uint16_t doubledSpeedRpm(uint16_t baselineRpm) {
  return DOUBLE_SPEED_AND_ACCELERATION
             ? (static_cast<uint32_t>(baselineRpm) * 2U >
                        PREVIOUS_M7_SPEED_LIMIT_RPM
                    ? PREVIOUS_M7_SPEED_LIMIT_RPM
                    : static_cast<uint16_t>(
                          static_cast<uint32_t>(baselineRpm) * 2U))
             : baselineRpm;
}

constexpr uint16_t boostedSpeedRpm(uint16_t effectiveRpm) {
  return !DOUBLE_SPEED_AND_ACCELERATION ||
                 !BOOST_EFFECTIVE_PROFILE_BY_30_PERCENT
             ? effectiveRpm
             : (static_cast<uint32_t>(effectiveRpm) * 13U / 10U >
                        M7_EXPERIMENTAL_SPEED_LIMIT_RPM
                    ? M7_EXPERIMENTAL_SPEED_LIMIT_RPM
                    : static_cast<uint16_t>(
                          static_cast<uint32_t>(effectiveRpm) * 13U / 10U));
}

constexpr uint16_t experimentalSpeedRpm(uint16_t baselineRpm) {
  return boostedSpeedRpm(doubledSpeedRpm(baselineRpm));
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

constexpr uint8_t acceleratedBeforeThirtyPercentBoost(uint8_t baseline) {
  return !DOUBLE_SPEED_AND_ACCELERATION
             ? baseline
             : (DOUBLE_ACCELERATION_AGAIN
                    ? doubledAccelerationStep(
                          doubledAccelerationStep(baseline))
                    : doubledAccelerationStep(baseline));
}

// The EMM physical acceleration is approximately proportional to
// 1/(256-byte). Reducing that denominator by 10/13 produces a 30-percent
// physical increase. Round to the nearest representable byte and never wrap
// past 255. The current travel profile reaches the requested byte 255 ceiling.
constexpr uint8_t boostedPhysicalAcceleration(uint8_t effectiveByte) {
  return !DOUBLE_SPEED_AND_ACCELERATION ||
                 !BOOST_EFFECTIVE_PROFILE_BY_30_PERCENT
             ? effectiveByte
             : static_cast<uint8_t>(
                   256U -
                   (((256U - static_cast<uint16_t>(effectiveByte)) * 10U +
                     6U) /
                    13U));
}

constexpr uint8_t experimentalAcceleration(uint8_t baseline) {
  return boostedPhysicalAcceleration(
      acceleratedBeforeThirtyPercentBoost(baseline));
}

static_assert(
    !DOUBLE_SPEED_AND_ACCELERATION ||
        experimentalSpeedRpm(65535U) <=
            M7_EXPERIMENTAL_SPEED_LIMIT_RPM,
    "M7 experimental speed limit overflow");
static_assert(
    experimentalAcceleration(249U) == 255U,
    "M7 experimental acceleration mapping changed unexpectedly");
static_assert(
    experimentalSpeedRpm(6740U) == 6500U &&
        experimentalAcceleration(249U) == 255U,
    "M7 30-percent travel-profile trial changed unexpectedly");

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

// Physical check found the previous +20 mm direction reversed. Use -20 mm
// in the body-left coordinate; retain the original 20 mm distance.
// Applies only to confirmed outward M6 stalls during endpoint scanning.
constexpr float M6_SCAN_RECOVERY_LEFT_MM = -20.0f;
// If endpoint camera centering would still exceed the real M6 travel, move
// the chassis a small distance toward its front and rebuild the endpoint map.
constexpr float M6_CAMERA_REACH_RECOVERY_FORWARD_MM = 20.0f;
constexpr uint32_t M6_SCAN_RECOVERY_DELAY_MS = 10UL;
constexpr uint32_t M6_SCAN_RECOVERY_MOVE_TIMEOUT_MS = 5000UL;
constexpr uint8_t M6_SCAN_RECOVERY_MAX_MOVES = 3U;

constexpr uint16_t M7_TRAVEL_SPEED_RPM = 6740U;
// The experiment transform maps this baseline byte to the requested maximum
// driver acceleration byte 255 while preserving the centralized M7 profile.
constexpr uint8_t M7_TRAVEL_ACCELERATION = 249U;
static_assert(
    m7_experiment::experimentalAcceleration(M7_TRAVEL_ACCELERATION) == 255U,
    "M7 travel commands must transmit acceleration byte 255");
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
// Every tray pickup descends another 2 mm from the previous 41 mm setting.
// The second slot keeps its independent extra 3 mm compensation below.
constexpr float CONTAINER_PICK_PHYSICAL_LOWER_MM = 43.0f;
constexpr float CONTAINER_PLACE_PHYSICAL_LOWER_MM = 40.0f;
// A processed ring returning from the rough-processing station must enter the
// tray 2 mm deeper than the ordinary RAW-to-tray placement height.
constexpr float CONTAINER_RETURN_PLACE_EXTRA_LOWER_MM = 2.0f;
constexpr float SECOND_PICK_EXTRA_LOWER_MM = 3.0f;
constexpr float RING_RETURN_PICK_EXTRA_LOWER_MM = 3.0f;
// Both PROCESS visits now use independently named 4 mm ring-pick corrections:
// the prior 2 mm calibration plus the requested common 2 mm deeper pickup.
constexpr float PROCESS_ROUND1_RING_RETURN_PICK_ADDITIONAL_LOWER_MM = 4.0f;
constexpr float PROCESS_ROUND2_RING_RETURN_PICK_ADDITIONAL_LOWER_MM = 4.0f;
// All three target placements descend another 3 mm from the previous
// 2 mm correction. Return-pick depth remains independently calibrated.
constexpr float TARGET_PLACE_EXTRA_LOWER_MM = 5.0f;
// Ground placement after vision only: both PROCESS rounds and STORAGE1.
// Total M6 retraction by ring 1/2/3 is 6/6/9 mm. Tray targets are unchanged.
constexpr float RING1_TARGET_PLACE_EXTENSION_REDUCTION_MM = 6.0f;
constexpr float RING2_TARGET_PLACE_EXTENSION_REDUCTION_MM = 6.0f;
constexpr float RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM = 9.0f;
// STORAGE round 2 stacks onto color-guided targets. Correct the observed fan
// shape locally: ring 1 uses the visual mapping directly, ring 2 retracts
// 10 mm, and ring 3 retracts 2 mm.
// STORAGE2 survey: user frame 0 near ring1, 180 near ring3.
// Indexed by physical ring1/2/3; survey visits 3 -> 2 -> 1.
constexpr float STORAGE2_SCAN_USER_ANGLES_DEGREES[3] = {30.0f, 90.0f, 150.0f};
constexpr float STORAGE2_SCAN_EXTENSION_ADVANCE_MM = 10.0f;
constexpr uint32_t STORAGE2_SCAN_DWELL_MS = 500UL;
// Second STORAGE visit only: enlarge the existing pixel-to-millimeter mapping
// by 30 percent. RAW, PROCESS, STORAGE1 and endpoint scanning keep their
// established scales.
constexpr float STORAGE2_PIXEL_TO_MM_CORRECTION_SCALE = 1.30f;
// Apply only to ring positions 1 and 3, after every existing ring-specific
// correction and immediately before the final STORAGE2 container-to-ring
// transfer. Ring 2 deliberately has no final multiplier. This must not alter
// the stored vision map or any pickup/scan pose.
constexpr float STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE = 0.90f;
// Second STORAGE placement only: after its existing per-ring correction,
// outer-position 0.90 scale and shared workstation net have all been applied,
// extend physical positions 1 and 3 by this exact manual correction amount.
constexpr float STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM = 5.0f;
// A STORAGE2 scan already knows the bottom-layer color assigned to its ring
// from the first-round QR plan. Accept that targeted color only when its
// coordinate stays inside this center ROI, so an adjacent ring cannot supply
// the coordinate for the current ring.
constexpr uint16_t STORAGE2_TARGET_COLOR_ROI_RADIUS_PIXELS = 100U;
constexpr float STORAGE2_RING1_TARGET_PLACE_EXTENSION_REDUCTION_MM = 0.0f;
constexpr float STORAGE2_MIDDLE_TARGET_PLACE_EXTENSION_REDUCTION_MM = 10.0f;
constexpr float STORAGE2_RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM = 2.0f;
// Apply the measured radial correction once when the visual ring coordinates
// are converted to a gripper pose. The stored pose is then shared by placing
// and picking, so the return pass targets the material that was actually put
// down without applying any additional M6 retraction offset.
constexpr float MAPPED_RING_EXTENSION_REDUCTION_MM = 0.0f;

// Every ordinary transfer raises M7 fully to its 0 mm working zero before
// M5/M6 start a planar move. M6/M7 share one serial bus, so ordinary callers
// must never move or query them concurrently. The sole exception is the RAW
// pickup descent tracker in main.cpp: it explicitly opts one addressed M6
// correction into the existing alternating status arbiter while M7 descends.
// Do not broaden that exception to any other motion path.
constexpr float TRAY_ROTATION_CLEARANCE_HEIGHT_MM = 0.0f;
constexpr float RING_ROTATION_CLEARANCE_HEIGHT_MM = 0.0f;

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

// Relative to the former 95-degree clamp, relax closing by 2 degrees toward
// OPEN_ANGLE_DEGREES. Shared by production and debug_workstation builds.
constexpr float CLOSE_ANGLE_DEGREES = 93.0f;
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
