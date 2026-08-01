#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.cpp"
MOBILE_CONFIG = (
    ROOT
    / "lib"
    / "MobileRobotConfig"
    / "src"
    / "MobileRobotConfig.h"
)
ROBOT_CONFIG = (
    ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h"
)
CHASSIS_HEADER = (
    ROOT
    / "lib"
    / "PulseMecanumChassis"
    / "src"
    / "PulseMecanumChassis.h"
)


def compact(path: Path) -> str:
    source = path.read_text(encoding="utf-8")
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    source = re.sub(r"//[^\n]*", "", source)
    return re.sub(r"\s+", "", source)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def require_all(source: str, expected: tuple[str, ...], label: str) -> None:
    missing = [entry for entry in expected if entry not in source]
    require(not missing, f"{label} missing: {', '.join(missing)}")


def main() -> int:
    main_source = compact(MAIN)
    mobile_config = compact(MOBILE_CONFIG)
    robot_config = compact(ROBOT_CONFIG)
    chassis_header = compact(CHASSIS_HEADER)

    require_all(
        mobile_config,
        (
            "ROUTE_FAST_BASE_MAXIMUM_STEP_RATE=9295.0f;",
            "ROUTE_FAST_BASE_STEP_ACCELERATION=2500.0f;",
            "ROUTE_SCAN_BASE_MAXIMUM_STEP_RATE=4160.0f;",
            "ROUTE_SCAN_BASE_STEP_ACCELERATION=1600.0f;",
            "ROUTE_TURN_BASE_MAXIMUM_STEP_RATE=3750.0f;",
            "ROUTE_TURN_BASE_STEP_ACCELERATION=1000.0f;",
            "ROUTE_HEADING_BASE_MAXIMUM_STEP_RATE=1500.0f;",
            "ROUTE_HEADING_BASE_STEP_ACCELERATION=450.0f;",
            "MAXIMUM_STEP_RATE=7150.0f;",
            "CENTRAL_CHANNEL_MAXIMUM_STEP_RATE=9295.0f;",
            "STEP_ACCELERATION=2500.0f;",
            "TURN_MAXIMUM_STEP_RATE=3000.0f;",
            "TURN_STEP_ACCELERATION=800.0f;",
            "HEADING_CORRECTION_MAXIMUM_STEP_RATE=1500.0f;",
            "HEADING_CORRECTION_STEP_ACCELERATION=450.0f;",
            "WORKSTATION_MAXIMUM_STEP_RATE=2080.0f;",
            "WORKSTATION_STEP_ACCELERATION=700.0f;",
            "FINAL_MAXIMUM_STEP_RATE=1040.0f;",
            "FINAL_STEP_ACCELERATION=400.0f;",
            "QR_SCAN_MAXIMUM_STEP_RATE=1040.0f;",
            "QR_SCAN_STEP_ACCELERATION=400.0f;",
        ),
        "morning M1-M4 chassis profile",
    )
    require_all(
        main_source,
        (
            "#include<MobileRobotConfig.h>",
            "namespacemobile_robot_config=gongchuang::mobile_robot;",
            "mobile_robot_config::ROUTE_FAST_BASE_MAXIMUM_STEP_RATE*",
            "mobile_robot_config::ROUTE_FAST_BASE_STEP_ACCELERATION*",
            "mobile_robot_config::ROUTE_SCAN_BASE_MAXIMUM_STEP_RATE*",
            "mobile_robot_config::ROUTE_SCAN_BASE_STEP_ACCELERATION*",
            "mobile_robot_config::ROUTE_TURN_BASE_MAXIMUM_STEP_RATE*",
            "mobile_robot_config::ROUTE_TURN_BASE_STEP_ACCELERATION*",
            "mobile_robot_config::ROUTE_HEADING_BASE_MAXIMUM_STEP_RATE*",
            "mobile_robot_config::ROUTE_HEADING_BASE_STEP_ACCELERATION*",
            "MAXIMUM_STEP_RATE=mobile_robot_config::MAXIMUM_STEP_RATE;",
            "STEP_ACCELERATION=mobile_robot_config::STEP_ACCELERATION;",
            "STEP_07_MOTION_SCALE=0.998f;",
        ),
        "production chassis integration",
    )
    require(
        "floatmaximumPulseRate=30000.0f," in chassis_header,
        "PulseMecanumChassis default did not return to 30000 pulse/s",
    )

    # The chassis-only rollback must not touch the current manipulator test
    # results or the complete-flow build selection.
    require_all(
        robot_config,
        (
            "#defineGONGCHUANG_RUN_MODE0",
            "M7_TRAVEL_SPEED_RPM=6740U;",
            "M7_TRAVEL_ACCELERATION=239U;",
            "M5_STANDARD_MAXIMUM_STEP_RATE=210600.0f;",
            "M6_STANDARD_SPEED_RPM=702U;",
            "M7_RING_PLACE_SPEED_RPM=2700U;",
        ),
        "unchanged manipulator/full-flow configuration",
    )
    for forbidden in ("M5_", "M6_", "M7_", "ARM_"):
        require(
            forbidden not in mobile_config,
            f"manipulator symbol leaked into chassis config: {forbidden}",
        )

    print(
        "PASS chassis profile: morning M1-M4 speed/acceleration restored, "
        "step 7 remains 0.998, manipulator tuning remains isolated"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, UnicodeError) as error:
        print(f"CHASSIS MOTION PROFILE CHECK FAILED: {error}")
        raise SystemExit(1)
