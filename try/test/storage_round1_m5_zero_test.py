#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.cpp"


def compact(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"\s+", "", text)


def function_body(code: str, marker: str) -> str:
    start = code.index(marker)
    brace = code.index("{", start)
    depth = 1
    for index in range(brace + 1, len(code)):
        if code[index] == "{":
            depth += 1
        elif code[index] == "}":
            depth -= 1
            if depth == 0:
                return code[brace + 1 : index]
    raise AssertionError(f"unterminated function: {marker}")


source = compact(MAIN.read_text(encoding="utf-8"))

assert "PROCESS1_M5_ZERO_HANDOFF_US=100UL;" in source
assert "STORAGE_TRAVEL_PREP_WAIT_ROUND1_M7_HANDOFF" in source
assert "STORAGE_TRAVEL_PREP_WAIT_ROUND1_PARALLEL" in source
assert (
    "PROCESS1_STORAGE_TRAVEL_M5_MAXIMUM_STEP_RATE="
    "arm_config::M5_RETURN_MAXIMUM_STEP_RATE*0.20f;" in source
)
assert (
    "PROCESS1_STORAGE_TRAVEL_M5_STEP_ACCELERATION="
    "arm_config::M5_RETURN_STEP_ACCELERATION*0.20f;" in source
)
assert "PROCESS_STORAGE_TRAVEL_M6_SPEED_RPM=162U;" in source
assert "PROCESS_STORAGE_TRAVEL_M6_ACCELERATION=42U;" in source
assert (
    "PROCESS1_STORAGE_TRAVEL_SERVO_INTERVAL_MS="
    "DEPARTURE_STORAGE_SERVO_INTERVAL_MS;" in source
)
assert "DEPARTURE_STORAGE_SERVO_INTERVAL_MS=static_cast<uint16_t>(" in source
assert "static_cast<uint32_t>(STORAGE_SERVO_INTERVAL_MS)*10UL/3UL" in source
assert "DEPARTURE_STORAGE_SERVO_SETTLE_MS=static_cast<uint32_t>(" in source

begin_travel = function_body(
    source, "voidbeginStorageTravelPreparation(uint8_troundNumber)"
)
assert "storageTravelM5ZeroHandoffDeadlineUs=0UL;" in begin_travel

service = function_body(
    source, "voidserviceStorageTravelPreparation()"
)
wait_m7 = service.split(
    "caseSTORAGE_TRAVEL_PREP_WAIT_M7_ONLY:"
)[1].split("caseSTORAGE_TRAVEL_PREP_WAIT_ROUND1_M7_HANDOFF:")[0]
assert "if(!liftMoveFinished())" in wait_m7
assert "liftAxis.currentMm-M7_STANDARD_HEIGHT_MM" in wait_m7
assert (
    "storageTravelM5ZeroHandoffDeadlineUs="
    "micros()+PROCESS1_M5_ZERO_HANDOFF_US;" in wait_m7
)
assert (
    "storageTravelPreparationPhase="
    "STORAGE_TRAVEL_PREP_WAIT_ROUND1_M7_HANDOFF;" in wait_m7
)
assert "startExtensionToMmWithProfile" not in wait_m7

handoff = service.split(
    "caseSTORAGE_TRAVEL_PREP_WAIT_ROUND1_M7_HANDOFF:"
)[1].split("caseSTORAGE_TRAVEL_PREP_WAIT_ROUND1_PARALLEL:")[0]
assert (
    "static_cast<int32_t>(micros()-"
    "storageTravelM5ZeroHandoffDeadlineUs)<0" in handoff
)
assert (
    "startExtensionToMmWithProfile(ENDPOINT_SEARCH_SEED_EXTENSION_MM,"
    "PROCESS_STORAGE_TRAVEL_M6_SPEED_RPM,"
    "PROCESS_STORAGE_TRAVEL_M6_ACCELERATION)" in handoff
)
assert (
    "armMotors.setM5MotionProfile("
    "PROCESS1_STORAGE_TRAVEL_M5_MAXIMUM_STEP_RATE,"
    "PROCESS1_STORAGE_TRAVEL_M5_STEP_ACCELERATION);" not in handoff
)
assert "startArmBaseStandardFrameDegrees(" not in service
assert (
    "commandStorageServoPosition(0U,"
    "PROCESS1_STORAGE_TRAVEL_SERVO_INTERVAL_MS);" in handoff
)
assert (
    "workStorageServoDeadlineMs=millis()+"
    "PROCESS1_STORAGE_TRAVEL_SERVO_SETTLE_MS;" in handoff
)
assert "storageTravelM5ZeroHandoffDeadlineUs=0UL;" in handoff
assert (
    "storageTravelPreparationPhase="
    "STORAGE_TRAVEL_PREP_WAIT_ROUND1_PARALLEL;"
    in handoff
)

parallel = service.split(
    "caseSTORAGE_TRAVEL_PREP_WAIT_ROUND1_PARALLEL:"
)[1].split("caseSTORAGE_TRAVEL_PREP_WAIT_M7_HEAD_START:")[0]
assert "armMotors.isM5Running()" in parallel
assert "!extensionMoveFinished()" in parallel
assert "!liftMoveFinished()" in parallel
assert "!deadlineReached(workStorageServoDeadlineMs)" in parallel
assert "ENDPOINT_SEARCH_SEED_EXTENSION_MM" in parallel
assert "storageTravelPreparationPhase=STORAGE_TRAVEL_PREP_COMPLETE;" in parallel

initial_storage = function_body(
    source, "boolstartInitialStorageServoHeadStart(){"
)
assert "activeWorkAction==WORK_ACTION_PROCESS" in initial_storage
assert (
    "activeWorkAction==WORK_ACTION_STORAGE&&workRoundIndex==0U"
    in initial_storage
)
assert "initialStorageSlot=0U;" in initial_storage
assert "commandStorageServoPosition(initialStorageSlot);" in initial_storage
assert (
    "workStorageServoDeadlineMs=millis()+STORAGE_SERVO_SETTLE_MS;"
    in initial_storage
)
assert "endpointInitialStorageCommanded=true;" in initial_storage

standard_service = function_body(source, "boolserviceArmStandardization()")
endpoint_pre_open = standard_service.split(
    "caseARM_STANDARD_WAIT_PRE_OPEN_GAP:"
)[1].split("caseARM_STANDARD_WAIT_OPEN:")[0]
tray_start = endpoint_pre_open.index("startInitialStorageServoHeadStart()")
gripper_start = endpoint_pre_open.index(
    "commandArmStandardizationGripperOpen();", tray_start
)
head_start_deadline = endpoint_pre_open.index(
    "static_cast<uint32_t>(GRIPPER_OPEN_SETTLE_MS)*1000UL+"
    "ENDPOINT_GRIPPER_M7_HANDOFF_US;",
    gripper_start,
)
assert tray_start < gripper_start < head_start_deadline
endpoint_open = standard_service.split(
    "caseARM_STANDARD_WAIT_OPEN:"
)[1].split("caseARM_STANDARD_WAIT_RAW_PARALLEL:")[0]
m7_start = endpoint_open.index(
    "startLiftToHeightMm(ENDPOINT_VISION_HEIGHT_MM)"
)
m5_start = endpoint_open.index(
    "startArmBaseStandardFrameDegrees(armStandardBaseTargetDegrees);"
)
assert m7_start < m5_start
assert "startInitialStorageServoHeadStart()" not in endpoint_open

endpoint_wait = standard_service.split(
    "caseARM_STANDARD_WAIT_ENDPOINT_PARALLEL:"
)[1].split("caseARM_STANDARD_WAIT_BASE_SETTLE:")[0]
assert "liftMoveFinished()" in endpoint_wait
assert "!armMotors.isM5Running()" in endpoint_wait
assert "workStorageServoDeadlineMs" not in endpoint_wait
assert "endpointInitialStorageCommanded" not in endpoint_wait

endpoint_scan = function_body(source, "voidbeginEndpointScan(uint8_tringPosition)")
assert "extensionAtSafeZero" in endpoint_scan
assert "extensionAtSearchSeed" in endpoint_scan
assert (
    "(!extensionAtSafeZero&&!extensionAtSearchSeed)" in endpoint_scan
)
assert "extensionAxis.recoveryPending" in endpoint_scan
assert "extensionAxis.fault" in endpoint_scan
assert "beginMaixRequest(MAIXCAM_ENDPOINT_CIRCLE_REQUEST)" not in endpoint_scan

endpoint_vision = function_body(source, "voidbeginEndpointVision()")
assert "beginMaixRequest(MAIXCAM_ENDPOINT_CIRCLE_REQUEST)" in endpoint_vision

print(
    "PASS storage round 1: M7 zero -> 100 us -> M5 hold, M6/tray parallel; "
    "arrival reuses the safe M6=ring-1 seed, then reaches the mode-10 request"
)
