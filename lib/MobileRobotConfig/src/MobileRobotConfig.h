#pragma once

namespace gongchuang {
namespace mobile_robot {

// M1-M4 chassis tuning is intentionally isolated from RobotConfig.h,
// which owns the manipulator and vision configuration.  These values match
// the verified morning chassis profile used by route.cpp.
constexpr float ROUTE_FAST_BASE_MAXIMUM_STEP_RATE = 9295.0f;
constexpr float ROUTE_FAST_BASE_STEP_ACCELERATION = 2500.0f;
constexpr float ROUTE_SCAN_BASE_MAXIMUM_STEP_RATE = 4160.0f;
constexpr float ROUTE_SCAN_BASE_STEP_ACCELERATION = 1600.0f;
constexpr float ROUTE_TURN_BASE_MAXIMUM_STEP_RATE = 3750.0f;
constexpr float ROUTE_TURN_BASE_STEP_ACCELERATION = 1000.0f;
constexpr float ROUTE_HEADING_BASE_MAXIMUM_STEP_RATE = 1500.0f;
constexpr float ROUTE_HEADING_BASE_STEP_ACCELERATION = 450.0f;

// Chassis-only profiles used by local positioning and workstation alignment.
constexpr float MAXIMUM_STEP_RATE = 7150.0f;
constexpr float CENTRAL_CHANNEL_MAXIMUM_STEP_RATE = 9295.0f;
constexpr float STEP_ACCELERATION = 2500.0f;
constexpr float TURN_MAXIMUM_STEP_RATE = 3000.0f;
constexpr float TURN_STEP_ACCELERATION = 800.0f;
constexpr float HEADING_CORRECTION_MAXIMUM_STEP_RATE = 1500.0f;
constexpr float HEADING_CORRECTION_STEP_ACCELERATION = 450.0f;
constexpr float WORKSTATION_MAXIMUM_STEP_RATE = 2080.0f;
constexpr float WORKSTATION_STEP_ACCELERATION = 700.0f;
constexpr float FINAL_MAXIMUM_STEP_RATE = 1040.0f;
constexpr float FINAL_STEP_ACCELERATION = 400.0f;
constexpr float QR_SCAN_MAXIMUM_STEP_RATE = 1040.0f;
constexpr float QR_SCAN_STEP_ACCELERATION = 400.0f;

static_assert(
    ROUTE_FAST_BASE_MAXIMUM_STEP_RATE ==
        CENTRAL_CHANNEL_MAXIMUM_STEP_RATE,
    "Route and central-channel chassis speed must stay aligned");
static_assert(
    ROUTE_HEADING_BASE_MAXIMUM_STEP_RATE ==
            HEADING_CORRECTION_MAXIMUM_STEP_RATE &&
        ROUTE_HEADING_BASE_STEP_ACCELERATION ==
            HEADING_CORRECTION_STEP_ACCELERATION,
    "Route and local chassis heading profiles must stay aligned");

}
}
