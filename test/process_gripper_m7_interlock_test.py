#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.cpp"
CONFIG = ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h"


def compact(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"\s+", "", text)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def case_body(code: str, current: str, following: str) -> str:
    start = code.index(f"case{current}:")
    end = code.index(f"case{following}:", start)
    return code[start:end]


def function_body(code: str, marker: str) -> str:
    marker_index = code.index(marker)
    start = code.index("{", marker_index)
    depth = 1
    for index in range(start + 1, len(code)):
        if code[index] == "{":
            depth += 1
        elif code[index] == "}":
            depth -= 1
            if depth == 0:
                return code[start + 1 : index]
    raise AssertionError(f"unterminated function {marker}")


def uint_constant(code: str, name: str) -> int:
    match = re.search(rf"{name}=(\d+)U?L?;", code)
    if match is None:
        raise AssertionError(f"missing integer constant {name}")
    return int(match.group(1))


def simulate_handoff(pre_ms: int, servo_ms: int, post_ms: int) -> dict[str, int]:
    """Discrete-event model from verified M7 stop to the next M7 command."""
    gripper_command_ms = pre_ms
    gripper_complete_ms = gripper_command_ms + servo_ms
    m7_restart_ms = gripper_complete_ms + post_ms
    return {
        "m7_stop": 0,
        "gripper_command": gripper_command_ms,
        "gripper_complete": gripper_complete_ms,
        "m7_restart": m7_restart_ms,
    }


def simulate_m7_driver_lag_bypass(
    physical_stop_ms: int,
    delayed_driver_terminal_ms: int,
    query_interval_ms: int,
    sample_gap_ms: int,
) -> dict[str, int]:
    """Model encoder release while the EMM terminal frame is delayed."""
    first_stable_query_ms = (
        (physical_stop_ms + query_interval_ms - 1)
        // query_interval_ms
        * query_interval_ms
    )
    encoder_release_ms = first_stable_query_ms + sample_gap_ms
    return {
        "physical_stop": physical_stop_ms,
        "encoder_release": encoder_release_ms,
        "driver_terminal": delayed_driver_terminal_ms,
        "release_lag": encoder_release_ms - physical_stop_ms,
    }


def main() -> int:
    code = compact(MAIN.read_text(encoding="utf-8"))
    tuning = compact(CONFIG.read_text(encoding="utf-8"))

    require(
        "MODE_SWITCH_GUARD_PREVIOUS_MS=100UL;" in tuning
        and "MODE_SWITCH_GUARD_MS=10UL;" in tuning
        and "MODE_SWITCH_GUARD_MS*10UL=="
        "MODE_SWITCH_GUARD_PREVIOUS_MS" in tuning,
        "vision request mode-switch guard is not reduced by exactly 90 percent",
    )

    require(
        "WORK_M7_TO_GRIPPER_GAP_MS=10UL;" in code
        and "WORK_GRIPPER_TO_M7_GAP_MS=10UL;" in code
        and "WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_MS=5UL;" in code
        and "WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS=100UL;" in code
        and "armGripperLiftIsolationEnabled=kind!=WORK_ACTION_NONE;"
        in code
        and code.count("armGripperLiftIsolationEnabled=false;") >= 3,
        "RAW/PROCESS/STORAGE gripper/M7 isolation enable/reset is incomplete",
    )
    require(
        "boolarmTransferUsesFastWorkGripperProfile(){"
        "returnarmTransferUsesPlaceProfile()||"
        "armTransferUsesReturnProfile();}" in code
        and code.count("armTransferUsesFastWorkGripperProfile()") >= 3,
        "tray pickup and ring return do not select work gripper profiles",
    )
    require(
        "ARM_STANDARD_WAIT_PRE_OPEN_GAP" in code
        and "millis()+GRIPPER_OPEN_SETTLE_MS+"
        "WORK_GRIPPER_TO_M7_GAP_MS;" in code
        and "effectiveConcurrentSourcePreparation="
        "concurrentSourcePreparation&&!armGripperLiftIsolationEnabled;"
        in code
        and "ARM_TRANSFER_WAIT_PRE_SOURCE_OPEN_GAP" in code,
        "initial/standard gripper operations can overlap M7",
    )
    open_command = function_body(
        code, "voidcommandArmTransferGripperOpen("
    )
    begin_transfer = function_body(code, "voidbeginArmTransfer(")
    release_destination = function_body(
        code, "voidreleaseArmTransferDestination("
    )
    require(
        "commandGripperDoubleSpeedOpen();return;" in open_command
        and "commandGripperTargetPlaceOpen();return;" in open_command
        and "commandGripperOpen();" in open_command
        and "commandArmTransferGripperOpen();" not in open_command
        and "armTransferPhase=" not in open_command
        and "armTransferDeadlineMs=" not in open_command
        and "ARM_TRANSFER_WAIT_PRE_SOURCE_OPEN_GAP" in begin_transfer
        and begin_transfer.index("ARM_TRANSFER_WAIT_PRE_SOURCE_OPEN_GAP")
        < begin_transfer.index("commandArmTransferGripperOpen();")
        and "commandArmTransferGripperOpen();" in release_destination
        and release_destination.index("commandArmTransferGripperOpen();")
        < release_destination.index("gripperOpenCommandMs=millis();")
        < release_destination.index(
            "armTransferPhase=ARM_TRANSFER_WAIT_DESTINATION_OPEN;"
        ),
        "gripper command/orchestration boundary can suppress a physical open command",
    )
    require(
        "ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP" in code
        and "ARM_TRANSFER_WAIT_DESTINATION_GRIPPER_GAP" in code
        and "armTransferDestinationM7ToGripperGapMs()" in code
        and "millis()+armTransferGripperCloseSettleMs()+"
        "WORK_GRIPPER_TO_M7_GAP_MS;" in code
        and "gripperOpenCommandMs+armTransferGripperOpenSettleMs()+"
        "(armGripperLiftIsolationEnabled?"
        "WORK_GRIPPER_TO_M7_GAP_MS:0UL);"
        in code,
        "pickup/release does not enforce the configured short handoffs",
    )
    source_lower = case_body(
        code,
        "ARM_TRANSFER_WAIT_SOURCE_LOWER",
        "ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP",
    )
    source_gap = case_body(
        code,
        "ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP",
        "ARM_TRANSFER_WAIT_GRIP_CLOSE",
    )
    source_close = case_body(
        code,
        "ARM_TRANSFER_WAIT_GRIP_CLOSE",
        "ARM_TRANSFER_WAIT_LOADED_CLEARANCE",
    )
    destination_lower = case_body(
        code,
        "ARM_TRANSFER_WAIT_DESTINATION_LOWER",
        "ARM_TRANSFER_WAIT_DESTINATION_GRIPPER_GAP",
    )
    destination_gap = case_body(
        code,
        "ARM_TRANSFER_WAIT_DESTINATION_GRIPPER_GAP",
        "ARM_TRANSFER_WAIT_DESTINATION_OPEN",
    )
    require(
        "armTransferPhase=ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP;"
        in source_lower
        and "commandArmTransferGripperClose();" not in source_lower.split(
            "else{", 1
        )[0]
        and "commandArmTransferGripperClose();" in source_gap
        and "startArmTransferLiftToHeightMm(" not in source_gap
        and "deadlineReached(armTransferDeadlineMs)" in source_close
        and "startArmTransferLiftToHeightMm(" in source_close
        and "armTransferPhase=ARM_TRANSFER_WAIT_DESTINATION_GRIPPER_GAP;"
        in destination_lower
        and "releaseArmTransferDestination();" in destination_gap
        and "startArmTransferLiftToHeightMm(" not in destination_gap,
        "a transfer phase can still reverse M7 before the gripper gap completes",
    )
    pre_gap_ms = uint_constant(code, "WORK_M7_TO_GRIPPER_GAP_MS")
    post_gap_ms = uint_constant(code, "WORK_GRIPPER_TO_M7_GAP_MS")
    target_place_pre_gap_ms = uint_constant(
        code, "WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_MS"
    )
    response_limit_ms = uint_constant(
        code, "WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS"
    )
    normal_close_ms = uint_constant(code, "GRIPPER_CLOSE_SETTLE_MS")
    target_close_ms = uint_constant(
        code, "GRIPPER_TARGET_PICK_CLOSE_SETTLE_MS"
    )
    tray_close_ms = uint_constant(
        code, "GRIPPER_TRAY_PICK_CLOSE_SETTLE_MS"
    )
    normal_open_ms = uint_constant(code, "GRIPPER_OPEN_SETTLE_MS")
    target_place_open_ms = uint_constant(
        code, "GRIPPER_TARGET_PLACE_OPEN_SETTLE_MS"
    )
    tray_release_open_ms = uint_constant(
        code, "GRIPPER_TRAY_RELEASE_OPEN_SETTLE_MS"
    )
    require(
        pre_gap_ms > 0
        and post_gap_ms > 0
        and target_place_pre_gap_ms > 0
        and pre_gap_ms <= response_limit_ms
        and target_place_pre_gap_ms <= response_limit_ms,
        "M7/gripper handoff was collapsed to simultaneous commands",
    )
    require(
        pre_gap_ms + target_close_ms + post_gap_ms == 140
        and target_place_pre_gap_ms
        + target_place_open_ms
        + post_gap_ms
        == 55
        and pre_gap_ms + tray_release_open_ms + post_gap_ms == 40,
        "target pickup/target release/tray release timelines are wrong",
    )
    require(
        pre_gap_ms + tray_close_ms + post_gap_ms == 180
        and "if(armTransferUsesPlaceProfile()){"
        "returnGRIPPER_TRAY_PICK_CLOSE_SETTLE_MS;}" in code,
        "tray pickup can raise M7 before loaded gripper closure finishes",
    )
    tray_timeline = simulate_handoff(
        pre_gap_ms, tray_close_ms, post_gap_ms
    )
    target_timeline = simulate_handoff(
        pre_gap_ms, target_close_ms, post_gap_ms
    )
    target_release_timeline = simulate_handoff(
        target_place_pre_gap_ms, target_place_open_ms, post_gap_ms
    )
    tray_release_timeline = simulate_handoff(
        pre_gap_ms, tray_release_open_ms, post_gap_ms
    )
    require(
        tray_timeline
        == {
            "m7_stop": 0,
            "gripper_command": 10,
            "gripper_complete": 170,
            "m7_restart": 180,
        }
        and target_timeline["m7_restart"] == 140
        and target_release_timeline["m7_restart"] == 55
        and tray_release_timeline["m7_restart"] == 40
        and all(
            timeline["m7_stop"]
            < timeline["gripper_command"]
            < timeline["gripper_complete"]
            < timeline["m7_restart"]
            for timeline in (
                tray_timeline,
                target_timeline,
                target_release_timeline,
                tray_release_timeline,
            )
        ),
        "discrete-event simulation found overlapping M7/gripper commands",
    )
    require(
        "M7_AXIS_STATUS_INTERVAL_MS=10UL;" in code
        and "M7_AXIS_MINIMUM_ON_POSITION_MS=100UL;" in code
        and "axis->address==ARM_LIFT_ADDRESS?"
        "M7_AXIS_MINIMUM_ON_POSITION_MS:"
        "ARM_AXIS_MINIMUM_ON_POSITION_MS;" in code
        and "axis.address==ARM_LIFT_ADDRESS?"
        "M7_AXIS_STATUS_INTERVAL_MS:ARM_AXIS_STATUS_INTERVAL_MS" in code
        and code.count("[WORKTIMING]") == 2,
        "M7 verified-arrival polling or gripper response telemetry is incomplete",
    )
    m7_query_interval_ms = uint_constant(
        code, "M7_FAST_ARRIVAL_QUERY_INTERVAL_MS"
    )
    m7_sample_gap_ms = uint_constant(
        code, "M7_FAST_ARRIVAL_SAMPLE_GAP_MS"
    )
    m7_response_timeout_ms = uint_constant(
        code, "M7_FAST_ARRIVAL_RESPONSE_TIMEOUT_MS"
    )
    m7_status_fresh_ms = uint_constant(
        code, "M7_FAST_ARRIVAL_STATUS_FRESH_MS"
    )
    require(
        m7_query_interval_ms == 20
        and m7_sample_gap_ms == 10
        and m7_response_timeout_ms == 20
        and m7_status_fresh_ms == 30
        and "M7_ENCODER_FAST_ARRIVAL_ENABLED=true;" in code
        and "M7_FAST_ARRIVAL_TARGET_TOLERANCE_MM=0.35f;" in code
        and "M7_FAST_ARRIVAL_STABILITY_MM=0.15f;" in code
        and "ARM_AXIS_VERIFY_M7_FAST_ARRIVAL=5U" in code
        and "requestLinearAxisTerminalVerification(axis,"
        "axis.lastStatusFlags,ARM_AXIS_VERIFY_M7_FAST_ARRIVAL);"
        in code
        and "[M7FASTARRIVAL]encoderstable;" in code
        and "bypassdelayeddriver" in code
        and "ignoredtopreventadelayedoldframecompletinganewM7move"
        in code,
        "M7 encoder fast-arrival bypass is incomplete",
    )
    driver_lag_timeline = simulate_m7_driver_lag_bypass(
        physical_stop_ms=501,
        delayed_driver_terminal_ms=1500,
        query_interval_ms=m7_query_interval_ms,
        sample_gap_ms=m7_sample_gap_ms,
    )
    require(
        driver_lag_timeline["release_lag"] <= 30
        and driver_lag_timeline["encoder_release"]
        < driver_lag_timeline["driver_terminal"],
        "M7 encoder release still waits for the delayed EMM terminal frame",
    )
    require(
        "MAPPED_RING_EXTENSION_REDUCTION_MM=8.0f;" in tuning
        and "correctedExtensionMm=rawExtensionMm-"
        "arm_config::MAPPED_RING_EXTENSION_REDUCTION_MM;" in code
        and "pose.extensionMm=commandedExtensionMm;" in code
        and "pose=measuredRingPoses[ringPosition];" in code,
        "mapped ring coordinate settlement does not apply the 8 mm M6 reduction",
    )
    require(
        "TARGET_PLACE_EXTRA_LOWER_MM=5.0f;" in tuning
        and "boolapplyTargetPlacementExtraLower(ArmPose&pose){" in code
        and code.count("applyTargetPlacementExtraLower(destination)") == 1,
        "target-only M7 placement height is not lowered by 5 mm exactly once",
    )
    require(
        "GRIPPER_DOUBLE_SPEED_INTERVAL_MS=20U;" in code
        and "GRIPPER_TARGET_PLACE_OPEN_INTERVAL_MS=40U;" in code
        and "commandGripperDoubleSpeedClose();" in code
        and "commandGripperDoubleSpeedOpen();" in code,
        "the three selected gripper operations are not doubled to 20 ms",
    )
    require(
        "uint32_tarmTransferTransitionSettleMs(){"
        "returnarmTransferUsesFastWorkGripperProfile()?"
        "ARM_TRANSFER_RETURN_SETTLE_MS:ARM_TRANSFER_BASE_SETTLE_MS;}"
        in code
        and "ARM_TRANSFER_RETURN_SETTLE_MS=0UL;" in code,
        "fast tray/ring transitions still contain an avoidable planar settle",
    )
    require(
        "sourceAlreadyPrepared||concurrentSourcePreparation,"
        "prepareFirstPlacedRingPickup);" in code,
        "first mapped-endpoint handoff can repeat an already completed gripper-open cycle",
    )
    require(
        "M5_RETURN_MAXIMUM_STEP_RATE=M5_PLACE_MAXIMUM_STEP_RATE;"
        in tuning
        and "M5_LOADED_RETURN_MAXIMUM_STEP_RATE="
        "M5_PLACE_MAXIMUM_STEP_RATE;"
        in tuning
        and "M5_LOADED_RETURN_STEP_ACCELERATION="
        "M5_PLACE_STEP_ACCELERATION;" in tuning
        and "M6_RETURN_SPEED_RPM=M6_PLACE_SPEED_RPM;" in tuning
        and "M6_LOADED_RETURN_SPEED_RPM=M6_PLACE_SPEED_RPM;"
        in tuning
        and "M6_LOADED_RETURN_ACCELERATION=M6_PLACE_ACCELERATION;"
        in tuning
        and "M7_RETURN_SPEED_RPM="
        "arm_hardware::M7_TRAVEL_SPEED_RPM;" in tuning
        and "arm_config::M5_LOADED_RETURN_MAXIMUM_STEP_RATE,"
        "arm_config::M5_LOADED_RETURN_STEP_ACCELERATION" in code
        and "loadedReturnMotion?"
        "arm_config::M6_LOADED_RETURN_SPEED_RPM:"
        "RETURN_M6_SPEED_RPM" in code
        and '"loadedclearance->destination",'
        "armTransferUsesReturnProfile())" in code,
        "tray-to-ring and ring-to-tray motor profiles are not identical",
    )
    require(
        "RING_RETURN_STORAGE_COMMAND_DELAY_MS=500UL;" in code
        and "armTransferStorageCommandDueMs=gripperOpenCommandMs+"
        "armTransferGripperOpenSettleMs()+"
        "RING_RETURN_STORAGE_COMMAND_DELAY_MS;" in code
        and "STORAGE_SERVO_INTERVAL_MS=409U;" in code
        and "STORAGE_SERVO_SETTLE_MS=510UL;" in code
        and "armTransferNextStorageDeadlineMs=millis()+"
        "STORAGE_SERVO_SETTLE_MS;" in code
        and "(!armTransferPrepareNextSource||"
        "armTransferNextStorageCommanded)" in code,
        "turntable 500 ms post-release delay or arrival gate is incomplete",
    )
    require(
        "FIRST_ENDPOINT_M7_SETTLE_MS=10UL;" in code
        and "ARM_BASE_SETTLE_MS=20UL;" in code
        and "FIRST_ENDPOINT_M7_SETTLE_MS*2UL==ARM_BASE_SETTLE_MS"
        in code
        and "millis()+FIRST_ENDPOINT_M7_SETTLE_MS;"
        in case_body(
            code,
            "ARM_STANDARD_WAIT_ENDPOINT_PARALLEL",
            "ARM_STANDARD_WAIT_BASE_SETTLE",
        )
        and "millis()+ARM_BASE_SETTLE_MS;"
        in case_body(
            code,
            "ARM_STANDARD_WAIT_BASE",
            "ARM_STANDARD_WAIT_ENDPOINT_PARALLEL",
        ),
        "vision-facing arm settles are not capped at 20 ms",
    )
    require(
        "ENDPOINT_BASE_TO_EXTENSION_SETTLE_PREVIOUS_MS=50UL;" in code
        and "ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS=15UL;" in code
        and "ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS=15UL;" in code
        and "ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS=20UL;" in code
        and "VISION_POST_MOTION_SETTLE_TIME_MS=20UL;" in code
        and "VISION_HEADING_STABLE_TIME_MS=20UL;" in code
        and "updateHeadingLock(MOTION_TIMEOUT_MS,"
        "ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS,"
        "ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS)" in code
        and code.count(
            "millis()+ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS;"
        )
        == 2,
        "endpoint and chassis vision handoffs are not capped at 20 ms",
    )

    print(
        "PASS workstation timeline simulation: tray-pick/target-pick/"
        "target-release/tray-release=180/140/55/40 ms; "
        "M7 physical-stop release<=30 ms despite delayed driver terminal; "
        "target Z=-5 mm; mapped M6=-8 mm; "
        "zero fast planar settle; matched directional motor profiles; "
        "storage speed 70% with arrival gate"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, UnicodeError) as error:
        print(f"WORKSTATION INTERLOCK CHECK FAILED: {error}")
        raise SystemExit(1)
