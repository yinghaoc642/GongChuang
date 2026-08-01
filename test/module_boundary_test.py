#!/usr/bin/env python3

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.cpp"
CONFIG = (
    ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h"
)
MOBILE_ROBOT_CONFIG = (
    ROOT
    / "lib"
    / "MobileRobotConfig"
    / "src"
    / "MobileRobotConfig.h"
)
ARM_MOTOR = (
    ROOT
    / "lib"
    / "ArmMotorController"
    / "ArmMotorController.cpp"
)
TRANSFER_HEADER = (
    ROOT
    / "lib"
    / "ArmTransferPlanner"
    / "src"
    / "ArmTransferPlanner.h"
)
TRANSFER_SOURCE = (
    ROOT
    / "lib"
    / "ArmTransferPlanner"
    / "src"
    / "ArmTransferPlanner.cpp"
)
MAIX_HEADER = (
    ROOT / "lib" / "MaixCamClient" / "src" / "MaixCamClient.h"
)
MAIX_SOURCE = (
    ROOT / "lib" / "MaixCamClient" / "src" / "MaixCamClient.cpp"
)
IMU_HEADER = (
    ROOT
    / "lib"
    / "ImuHeadingTracker"
    / "src"
    / "ImuHeadingTracker.h"
)
IMU_SOURCE = (
    ROOT
    / "lib"
    / "ImuHeadingTracker"
    / "src"
    / "ImuHeadingTracker.cpp"
)

def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)

def read(path: Path) -> str:
    require(path.is_file(), f"missing module file: {path}")
    return path.read_text(encoding="utf-8")

def main() -> int:
    main_source = read(MAIN)
    config_source = read(CONFIG)
    mobile_robot_config = read(MOBILE_ROBOT_CONFIG)
    arm_source = read(ARM_MOTOR)
    transfer_header = read(TRANSFER_HEADER)
    transfer_source = read(TRANSFER_SOURCE)
    maix_header = read(MAIX_HEADER)
    maix_source = read(MAIX_SOURCE)
    imu_header = read(IMU_HEADER)
    imu_source = read(IMU_SOURCE)

    require(
        "#include <RobotConfig.h>" in main_source,
        "main.cpp bypasses the shared RobotConfig",
    )
    require(
        "#include <MobileRobotConfig.h>" in main_source
        and "namespace mobile_robot_config = gongchuang::mobile_robot;"
        in main_source
        and "ROUTE_FAST_BASE_MAXIMUM_STEP_RATE = 9295.0f"
        in mobile_robot_config
        and "M5_" not in mobile_robot_config
        and "M6_" not in mobile_robot_config
        and "M7_" not in mobile_robot_config,
        "M1-M4 chassis tuning is not isolated from manipulator config",
    )
    require(
        "#include <ArmTransferPlanner.h>" in main_source
        and "struct ArmPose" not in main_source
        and "struct ArmPose" in transfer_header
        and "selectMotionProfile(" in transfer_source
        and "containerPickPose(" in transfer_source
        and "containerReturnPlacePose(" in transfer_source,
        "arm transfer pose/profile planning leaked back into main.cpp",
    )
    require(
        "#define GONGCHUANG_RUN_MODE" not in main_source,
        "run-mode switch was duplicated back into main.cpp",
    )
    require(
        "M7_MICROSTEPS = 256U" in config_source,
        "M7 hardware microsteps are not centralized at 256",
    )
    require(
        "M7_TRAVEL_SPEED_RPM = 6740U" in config_source
        and "M7_TRAVEL_ACCELERATION = 239U" in config_source
        and "M7_ZERO_SOFT_LANDING_DISTANCE_MM = 2.0f"
        in config_source
        and "M7_ZERO_SOFT_LANDING_SPEED_RPM = 2160U"
        in config_source
        and "M7_ZERO_SOFT_LANDING_ACCELERATION = 171U"
        in config_source,
        "M7 travel/soft-zero tuning is missing from RobotConfig",
    )
    require(
        "M6_CONTACT_SOFT_LANDING_DISTANCE_MM = 2.0f"
        in config_source
        and "M7_CONTACT_SOFT_LANDING_DISTANCE_MM = 2.0f"
        in config_source,
        "contact soft-landing distances are not centralized",
    )
    require(
        "CLOSE_ANGLE_DEGREES = 87.0f" in config_source
        and "OPEN_ANGLE_DEGREES = 20.0f" in config_source
        and "MAX_OPEN_ANGLE_DEGREES = -101.0f" in config_source,
        "gripper calibration is not centralized at 87/20/-101 degrees",
    )
    require(
        "config::gripper::CLOSE_ANGLE_DEGREES" in main_source
        and "config::gripper::OPEN_ANGLE_DEGREES" in main_source
        and "config::gripper::MAX_OPEN_ANGLE_DEGREES" in main_source,
        "main.cpp bypasses the shared gripper calibration",
    )
    require(
        "STORAGE_SERVO_PARK_ANGLE_DEGREES = 165.0f" in main_source
        and "STORAGE_SERVO_WORK_ZERO_ANGLE_DEGREES = -5.0f"
        in main_source
        and "STORAGE_SERVO_CLOCKWISE_STEP_DEGREES = -90.0f"
        in main_source,
        "lower storage-turntable servo calibration changed unexpectedly",
    )
    require(
        "#include <RobotConfig.h>" in arm_source
        and "config::arm_hardware::M7_MICROSTEPS"
        in arm_source,
        "ArmMotorController does not consume shared M7 hardware config",
    )

    require(
        "#include <MaixCamClient.h>" in main_source,
        "main.cpp does not use MaixCamClient",
    )
    for forbidden in (
        "SerialMaixcam.read(",
        "SerialMaixcam.write(",
        "vision_protocol::parseResponse(",
        "vision_protocol::buildRequest(",
    ):
        require(
            forbidden not in main_source,
            f"MaixCAM transport leaked back into main.cpp: {forbidden}",
        )
    require(
        "class MaixCamClient" in maix_header
        and "vision_protocol::parseResponse(" in maix_source
        and "vision_protocol::buildRequest(" in maix_source,
        "MaixCamClient does not own the complete protocol transport",
    )
    require(
        "[VISION CAL] req/mode/target/n=" in maix_source
        and "center-d=" in maix_source
        and "prev-d=" in maix_source
        and "first-d=" in maix_source
        and "span=" in maix_source
        and "mean=" in maix_source,
        "MaixCamClient does not emit repeatability telemetry",
    )

    require(
        "#include <ImuHeadingTracker.h>" in main_source,
        "main.cpp does not use ImuHeadingTracker",
    )
    require(
        "JY901.CopeSerialData(" not in main_source
        and "SerialImu.read(" not in main_source,
        "raw JY901 parsing leaked back into main.cpp",
    )
    require(
        "class ImuHeadingTracker" in imu_header
        and "JY901.CopeSerialData(" in imu_source
        and "wrapDeltaDegrees(" in imu_source,
        "ImuHeadingTracker does not own validation and angle unwrapping",
    )

    print(
        "PASS module boundaries: shared config, MaixCAM transport, "
        "and IMU parsing are isolated"
    )
    return 0

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, UnicodeError) as error:
        print(f"MODULE BOUNDARY CHECK FAILED: {error}")
        raise SystemExit(1)
