#!/usr/bin/env python3

"""Safety contract for placement timing and restored loaded-jaw margins."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.cpp"
ROBOT_CONFIG = ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h"


def compact(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"\s+", "", text)


def matching_brace(code: str, opening: int) -> int:
    depth = 1
    for index in range(opening + 1, len(code)):
        if code[index] == "{":
            depth += 1
        elif code[index] == "}":
            depth -= 1
            if depth == 0:
                return index
    raise AssertionError("unterminated C++ block")


def function_body(code: str, marker: str) -> str:
    start = code.index(marker)
    opening = code.index("{", start)
    return code[opening + 1 : matching_brace(code, opening)]


source = compact(MAIN.read_text(encoding="utf-8"))
robot_config = compact(ROBOT_CONFIG.read_text(encoding="utf-8"))


# Keep the independent PROCESS/endpoint/RAW timing changes, while locking the
# rough-processing loaded-jaw and ring-return waits back to their proven values.
for fragment in (
    "PROCESS_ENTRY_M5_SETTLE_US=100UL;",
    "ENDPOINT_GRIPPER_M7_HANDOFF_US=100UL;",
    "INITIAL_STORAGE_SERVO_M7_HEAD_START_MS="
    "GRIPPER_OPEN_SETTLE_MS;",
    "GRIPPER_TARGET_PICK_CLOSE_SETTLE_MS=120UL;",
    "GRIPPER_TRAY_PICK_CLOSE_SETTLE_MS=160UL;",
    "RING_RETURN_M7_RAISE_EXTRA_DELAY_MS=100UL;",
    "RAW_PICK_PHYSICAL_LOWER_MM=77.2f;",
    "RING_RETURN_STORAGE_COMMAND_DELAY_MS=500UL;",
    "RAW_NEXT_STORAGE_COMMAND_DELAY_MS=1000UL;",
    "RAW_STORAGE_SERVO_INTERVAL_MS=200U;",
    "RAW_STORAGE_SERVO_SETTLE_MS=200UL;",
    "STORAGE_SERVO_VERIFY_CONFIRMATIONS=2U;",
    "STORAGE_SERVO_VERIFY_MAX_RETRIES=1U;",
):
    assert fragment in source, f"missing timing contract: {fragment}"


# Historical 6/6/9 values remain documented in RobotConfig, but PROCESS1/2 and
# STORAGE1 now include them inside one final net pose table. STORAGE2 stays
# independently calibrated at 0/10/2.
assert "RING1_TARGET_PLACE_EXTENSION_REDUCTION_MM=6.0f;" in robot_config
assert "RING2_TARGET_PLACE_EXTENSION_REDUCTION_MM=6.0f;" in robot_config
assert "RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM=9.0f;" in robot_config
assert "STORAGE2_RING1_TARGET_PLACE_EXTENSION_REDUCTION_MM=0.0f;" in robot_config
assert "STORAGE2_MIDDLE_TARGET_PLACE_EXTENSION_REDUCTION_MM=10.0f;" in robot_config
assert "STORAGE2_RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM=2.0f;" in robot_config
unloading = function_body(source, "voidbeginUnloadingTransfer()")
assert (
    "processOrStorage1RingNetCorrectionApplies(ringPosition)?0.0f:"
    in unloading
)
assert "arm_config::RING1_TARGET_PLACE_EXTENSION_REDUCTION_MM" not in unloading
assert "arm_config::RING2_TARGET_PLACE_EXTENSION_REDUCTION_MM" not in unloading
assert "arm_config::RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM" not in unloading


# Both 0.1 ms boundaries must use the microsecond clock. A millis() deadline
# would round the requested handoff up and change the intended timing.
work_service = function_body(source, "voidserviceCompetitionAction()")
assert "micros()+PROCESS_ENTRY_M5_SETTLE_US" in work_service
assert (
    "micros()-processM5ZeroSettleDeadlineUs" in work_service
)

standardization = function_body(source, "boolserviceArmStandardization()")
assert "micros()+ENDPOINT_GRIPPER_M7_HANDOFF_US" in standardization
assert "micros()-armStandardHandoffDeadlineUs" in standardization
pre_open = standardization.index("caseARM_STANDARD_WAIT_PRE_OPEN_GAP:")
open_wait = standardization.index("caseARM_STANDARD_WAIT_OPEN:")
pre_open_body = standardization[pre_open:open_wait]
turntable_index = pre_open_body.index("startInitialStorageServoHeadStart()")
gripper_index = pre_open_body.index(
    "commandArmStandardizationGripperOpen();", turntable_index
)
m7_deadline_index = pre_open_body.index(
    "static_cast<uint32_t>(GRIPPER_OPEN_SETTLE_MS)*1000UL+"
    "ENDPOINT_GRIPPER_M7_HANDOFF_US;",
    gripper_index,
)
assert turntable_index < gripper_index < m7_deadline_index
open_wait_body = standardization[open_wait:]
assert "startLiftToHeightMm(ENDPOINT_VISION_HEIGHT_MM)" in open_wait_body
assert "startInitialStorageServoHeadStart()" not in open_wait_body


# The RAW 200 ms value only schedules the first query. Pickup release remains
# dependent on two actual servo-angle samples inside the allowed deadband.
raw_command = function_body(
    source, "voidcommandRawStorageServoPositionWithArrivalGate("
)
assert (
    "commandStorageServoPositionWithArrivalGateProfile("
    "positionIndex,RAW_STORAGE_SERVO_INTERVAL_MS,"
    "RAW_STORAGE_SERVO_VERIFY_FIRST_QUERY_MS);" in raw_command
)

arrival_service = function_body(source, "boolserviceStorageServoArrivalGate()")
query_index = arrival_service.index("storageServo.queryRawAngleMTurn();")
deadband_index = arrival_service.index(
    "validResponse&&errorDegrees<="
    "STORAGE_SERVO_VERIFY_TOLERANCE_DEGREES;"
)
confirmation_index = arrival_service.index(
    "++storageServoArrivalGate.consecutiveConfirmations;"
)
release_index = arrival_service.index(
    "storageServoArrivalGate.confirmed=true;"
)
assert query_index < deadband_index < confirmation_index < release_index
assert (
    "storageServoArrivalGate.consecutiveConfirmations>="
    "STORAGE_SERVO_VERIFY_CONFIRMATIONS" in arrival_service
)


# The single re-drive must retain the active profile. For RAW this means the
# retry still commands 200 ms and then waits 200 ms before querying again.
assert (
    "commandStorageServoPosition(retrySlot,"
    "storageServoArrivalGate.commandIntervalMs);" in arrival_service
)
assert (
    "storageServoArrivalGate.commandStartMs+"
    "storageServoArrivalGate.firstQueryDelayMs;" in arrival_service
)


# Obsolete point-5 shortcuts remain deleted. The current STORAGE1 -> RAW2 M6
# travel pre-position is a new guarded path and still enters RAW2 through the
# normal expected-slot preparation and arm-standardization dispatch.
begin_work = function_body(source, "voidbeginWorkAction(")
assert "if(kind==WORK_ACTION_RAW){" in begin_work
assert "beginRawExpectedSlotPreparation();" in begin_work
assert "beginRawParallelPreparation();" in begin_work
assert "beginArmStandardization(" in begin_work
for removed_point5_symbol in (
    "raw2StorageServoDeadlineMs",
    "WORK_PHASE_RAW_WAIT_ARRIVAL_HANDOFF",
    "WORK_PHASE_STORAGE1_WAIT_ROUTE_RELEASE",
    "STORAGE1_RAW2_ROUTE_RELEASE_DELAY_MS",
):
    assert removed_point5_symbol not in source


# Ring return starts the tray only after release completion plus the restored
# 500 ms margin; it is not measured from the earlier gripper command start.
release = function_body(source, "voidreleaseArmTransferDestination()")
assert (
    "armTransferStorageCommandDueMs=gripperOpenCommandMs+"
    "armTransferGripperOpenSettleMs()+"
    "RING_RETURN_STORAGE_COMMAND_DELAY_MS;" in release
)
# Both RAW visits defer only the first/second next-slot commands until the
# gripper has completed opening plus 1 s. The arm can independently return to
# its remembered pickup pose, and item three has no next-source schedule.
assert (
    "if(armTransferUsesRawProfile()&&armTransferPrepareNextSource){"
    "armTransferStorageCommandDueMs=gripperOpenCommandMs+"
    "armTransferGripperOpenSettleMs()+"
    "RAW_NEXT_STORAGE_COMMAND_DELAY_MS;" in release
)
deferred = function_body(
    source, "voidserviceArmTransferDeferredStorageCommand()"
)
assert "!deadlineReached(armTransferStorageCommandDueMs)" in deferred
assert "commandStorageServoPosition(" in deferred
assert "commandRawStorageServoPositionWithArrivalGate(" in deferred
assert "RAW_STORAGE_SERVO_SETTLE_MS" in deferred


print(
    "PASS placement timing policy: 120/160 ms loaded-jaw settles, 500 ms "
    "ring-return delay and RAW next-slot release+1000 ms delay; independent "
    "100 us handoffs and guarded RAW 200 ms commands retained"
)
