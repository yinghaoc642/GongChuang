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

assert "STORAGE1_RAW2_M7_HANDOFF_US=100UL;" in source
assert "STORAGE1_RAW2_M6_SPEED_RPM==302U" in source
assert "STORAGE1_RAW2_M6_ACCELERATION==82U" in source
assert "STORAGE1_RAW2_SERVO_INTERVAL_MS==1023U" in source
assert "STORAGE1_RAW2_SERVO_SETTLE_MS==1123UL" in source

complete_transfer = function_body(source, "voidcompleteTransferAndRotateStorage()")
assert "storageRound1RawTravelOwnsTrayCommand" in complete_transfer
assert (
    "!processRound1TravelOwnsTrayCommand&&"
    "!storageRound1RawTravelOwnsTrayCommand" in complete_transfer
)

service = function_body(source, "voidserviceCompetitionAction()")
wait_storage = service.split("caseWORK_PHASE_WAIT_STORAGE_SERVO:")[1].split(
    "caseWORK_PHASE_WAIT_STORAGE1_RAW2_HANDOFF:"
)[0]
assert "storage1Raw2TravelHandoff" in wait_storage
assert "activeWorkAction==WORK_ACTION_STORAGE" in wait_storage
assert "workRoundIndex==0U" in wait_storage
assert (
    "storage1Raw2HandoffDeadlineUs="
    "micros()+STORAGE1_RAW2_M7_HANDOFF_US;" in wait_storage
)

handoff = service.split("caseWORK_PHASE_WAIT_STORAGE1_RAW2_HANDOFF:")[1].split(
    "caseWORK_PHASE_WAIT_STORAGE_PARK:"
)[0]
assert "!liftMoveFinished()" in handoff
assert "liftAxis.currentMm-M7_STANDARD_HEIGHT_MM" in handoff
assert (
    "static_cast<int32_t>(micros()-storage1Raw2HandoffDeadlineUs)<0"
    in handoff
)
m6_index = handoff.index(
    "startExtensionToMmWithProfile(RAW_VIEW_EXTENSION_MM,"
    "STORAGE1_RAW2_M6_SPEED_RPM,STORAGE1_RAW2_M6_ACCELERATION)"
)
tray_index = handoff.index(
    "commandStorageServoParkingPosition(STORAGE1_RAW2_SERVO_INTERVAL_MS)"
)
chassis_index = handoff.index("startVisualCorrectionRestore()")
assert m6_index < tray_index < chassis_index
assert "storage1Raw2TravelPhase=STORAGE1_RAW2_TRAVEL_RUNNING;" in handoff

wait_restore = service.split("caseWORK_PHASE_WAIT_RESTORE:")[1]
assert "storage1ParallelDeparture" in wait_restore
assert "activeWorkAction==WORK_ACTION_STORAGE" in wait_restore
assert "finishActiveWorkAction();" in wait_restore

raw_safe = function_body(source, "boolrawParallelPreparationSafe()")
assert "RAW_VIEW_EXTENSION_MM" in raw_safe
assert "extensionAtSafeStart" in raw_safe

integrated = function_body(source, "voidserviceIntegratedRoute()")
assert "serviceStorage1Raw2TravelPreparation();" in integrated
assert "integratedWorkPause==INTEGRATED_WORK_RAW_2" in integrated
assert "storage1Raw2TravelPreparationReady()" in integrated
assert "beginWorkAction(WORK_ACTION_RAW,2U);" in integrated

print(
    "PASS STORAGE1 exit: M7 zero -> 100 us -> 40%-speed M6/tray + chassis; "
    "RAW2 waits only if the travelling M6 has not reached its view pose"
)
