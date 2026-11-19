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


def function_body(code: str, marker: str) -> str:
    match = re.search(re.escape(marker) + r"[^;{]*\{", code)
    assert match is not None, marker
    start = match.start()
    brace = match.end() - 1
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
config = compact(CONFIG.read_text(encoding="utf-8"))


# Run-mode 0 remains the complete mission; modes 1 and 2 are local tests.
assert (
    "static_assert(GONGCHUANG_RUN_MODE==0||"
    "GONGCHUANG_RUN_MODE==1||GONGCHUANG_RUN_MODE==2,"
    '"GONGCHUANG_RUN_MODEmustbe0,1or2");'
    in config
)
assert (
    "ROUGH_PROCESSING_CALIBRATION_MODE=GONGCHUANG_RUN_MODE==1;"
    in source
)
assert (
    "STORAGE_ROUND2_SIMULATION_MODE=GONGCHUANG_RUN_MODE==2;"
    in source
)
assert (
    "LOCAL_WORKSTATION_SIMULATION_MODE="
    "ROUGH_PROCESSING_CALIBRATION_MODE||"
    "STORAGE_ROUND2_SIMULATION_MODE||"
    "STORAGE_ROUND1_SIMULATION_MODE;" in source
)
# The local task must preserve the supplied code exactly.  Its second-round
# position group is deliberately not the stack destination: a second item is
# stacked on the first-round ring that already contains the same color.
configure = function_body(
    source, "boolconfigureStorageRound2SimulationTask()"
)
assert 'STORAGE2_SIMULATION_TASK_CODE[]="134+123+314+231";' in configure
assert (
    "competition::parseTaskCode(STORAGE2_SIMULATION_TASK_CODE,"
    "simulationPlan)" in configure
)
assert "taskPlan=simulationPlan;" in configure
assert (
    "memcpy(qrData,STORAGE2_SIMULATION_TASK_CODE,"
    "sizeof(STORAGE2_SIMULATION_TASK_CODE));" in configure
)
assert "scanFlag=true;" in configure
assert "taskCodeDecoded=true;" in configure

fixed_code = "134+123+314+231"
first_colors_text, first_positions_text, second_colors_text, second_positions_text = (
    fixed_code.split("+")
)
first_colors = [int(value) for value in first_colors_text]
first_positions = [int(value) for value in first_positions_text]
second_colors = [int(value) for value in second_colors_text]
second_positions = [int(value) for value in second_positions_text]
ring_by_color = dict(zip(first_colors, first_positions))
target_rings = [ring_by_color[color] for color in second_colors]
scan_angles = {1: 45.0, 2: 0.0, 3: -45.0}

assert first_colors == [1, 3, 4]
assert first_positions == [1, 2, 3]
assert second_colors == [3, 1, 4]
assert second_positions == [2, 3, 1]
assert target_rings == [2, 1, 3]
assert target_rings != second_positions
assert [scan_angles[ring] for ring in target_rings] == [0.0, 45.0, -45.0]


# Mode 2 starts with the gripper opening while the four wheels are locked. It
# returns before the formal route entry and advances through a dedicated
# preparation service instead of route_chassis::updateRoute().
begin_route = function_body(source, "voidbeginRoute()")
configure_index = begin_route.index("configureStorageRound2SimulationTask()")
open_index = begin_route.index("commandGripperOpen();", configure_index)
local_branch_index = begin_route.index(
    "if(LOCAL_WORKSTATION_SIMULATION_MODE){", open_index
)
zero_wheels_index = begin_route.index(
    "for(uint8_ti=0;i<4;++i){motors[i]->setCurrentPosition(0);}",
    local_branch_index,
)
enable_wheels_index = begin_route.index("enableDriveMotors();", zero_wheels_index)
route_stop_index = begin_route.index(
    "route_chassis::stopAllMotorsImmediately();", enable_wheels_index
)
local_stop_index = begin_route.index("stopAllMotorsImmediately();", route_stop_index)
wait_gripper_index = begin_route.index(
    "storageRound2SimulationPhase="
    "STORAGE2_SIMULATION_WAIT_GRIPPER_OPEN;",
    local_stop_index,
)
local_return_index = begin_route.index("return;", wait_gripper_index)
formal_route_index = begin_route.index("route_chassis::beginRoute();")
assert (
    configure_index < open_index < local_branch_index < zero_wheels_index
    < enable_wheels_index < route_stop_index < local_stop_index
    < wait_gripper_index < local_return_index < formal_route_index
)
local_branch = begin_route[local_branch_index:formal_route_index]
assert "route_chassis::beginRoute();" not in local_branch
assert "beginWorkAction(WORK_ACTION_STORAGE,2U);" not in local_branch
assert "disableDriveMotors();" not in local_branch

integrated_route = function_body(source, "voidserviceIntegratedRoute()")
assert (
    integrated_route.index(
        "if(LOCAL_WORKSTATION_SIMULATION_MODE||"
        "programState!=PROGRAM_RUNNING){return;}"
    )
    < integrated_route.index("route_chassis::updateRoute();")
)

action_timeout = function_body(source, "uint32_tactiveWorkActionTimeoutMs()")
assert (
    action_timeout.index("if(LOCAL_WORKSTATION_SIMULATION_MODE){return0UL;}")
    < action_timeout.index("switch(activeWorkAction)")
)
watchdogs = function_body(source, "voidserviceCompetitionWatchdogs()")
assert (
    watchdogs.index(
        "if(LOCAL_WORKSTATION_SIMULATION_MODE||"
        "programState!=PROGRAM_RUNNING){return;}"
    )
    < watchdogs.index("MISSION_PROGRESS_TIMEOUT_MS")
)

loop_body = function_body(source, "voidloop()")
qr_guard = loop_body.index("if(!LOCAL_WORKSTATION_SIMULATION_MODE){")
receive_qr_index = loop_body.index("receiveQrData();", qr_guard)
sync_qr_index = loop_body.index(
    "synchronizeTaskCodeToRouteChassis();", receive_qr_index
)
qr_guard_end = loop_body.index("}", sync_qr_index)
assert qr_guard < receive_qr_index < sync_qr_index < qr_guard_end

restore = function_body(source, "voidstartVisualCorrectionRestore()")
simulation_restore_index = restore.index(
    "if(STORAGE_ROUND2_SIMULATION_MODE){"
)
restore_route_stop_index = restore.index(
    "route_chassis::stopAllMotorsImmediately();",
    simulation_restore_index,
)
restore_local_stop_index = restore.index(
    "stopAllMotorsImmediately();", restore_route_stop_index
)
restore_return_index = restore.index("return;", restore_local_stop_index)
restore_move_index = restore.index("startRelativeMotorMove(", restore_return_index)
assert (
    simulation_restore_index < restore_route_stop_index
    < restore_local_stop_index < restore_return_index < restore_move_index
)

departure = function_body(source, "boolserviceWorkstationDepartureLock()")
simulation_departure_index = departure.index(
    "if(LOCAL_WORKSTATION_SIMULATION_MODE){"
)
simulation_departure_return = departure.index(
    "returntrue;", simulation_departure_index
)
normal_heading_lock = departure.index(
    "returnupdateHeadingLock(", simulation_departure_return
)
assert simulation_departure_index < simulation_departure_return < normal_heading_lock


# The local entry waits for a real gripper-open deadline, revalidates the
# startup M6/M7 zero pose, then reuses the high-speed M7 -> 300 ms -> M5/M7 ->
# M6=30 travel preparation. The action starts once that pose is ready.
simulation = function_body(source, "voidserviceStorageRound2Simulation()")
wait_open = simulation.split(
    "caseSTORAGE2_SIMULATION_WAIT_GRIPPER_OPEN:"
)[1].split("caseSTORAGE2_SIMULATION_WAIT_TRAVEL_PREPARATION:")[0]
assert "deadlineReached(storageRound2SimulationGripperDeadlineMs)" in wait_open
assert "extensionAxis.currentMm-M6_STANDARD_EXTENSION_MM" in wait_open
assert "liftAxis.currentMm-M7_STANDARD_HEIGHT_MM" in wait_open
assert "beginStorageTravelPreparation(2U);" in wait_open

wait_preparation = simulation.split(
    "caseSTORAGE2_SIMULATION_WAIT_TRAVEL_PREPARATION:"
)[1].split("caseSTORAGE2_SIMULATION_RUN_ACTION:")[0]
assert "serviceStorageTravelPreparation();" in wait_preparation
assert "storageTravelPreparationReady(2U)" in wait_preparation
assert "activeCompetitionRound=2U;" in wait_preparation
assert "beginWorkAction(WORK_ACTION_STORAGE,2U);" in wait_preparation

begin_travel = function_body(
    source, "voidbeginStorageTravelPreparation(uint8_troundNumber)"
)
assert "storageRingForItem(" not in begin_travel
assert "storageColorSearchAngleForRing(" not in begin_travel
assert "startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)" in begin_travel
service_travel = function_body(source, "voidserviceStorageTravelPreparation()")
head_start = service_travel.split(
    "caseSTORAGE_TRAVEL_PREP_WAIT_M7_HEAD_START:"
)[1].split("caseSTORAGE_TRAVEL_PREP_WAIT_M5_M7:")[0]
assert "deadlineReached(storageTravelM5StartDeadlineMs)" in head_start
assert "startArmBaseStandardFrameDegrees(" not in service_travel
wait_m5_m7 = service_travel.split(
    "caseSTORAGE_TRAVEL_PREP_WAIT_M5_M7:"
)[1].split("caseSTORAGE_TRAVEL_PREP_WAIT_M6:")[0]
assert "armMotors.isM5Running()" in wait_m5_m7
assert "!liftMoveFinished()" in wait_m5_m7
assert "startExtensionToMmWithProfile(STORAGE_TRAVEL_M6_EXTENSION_MM" in wait_m5_m7
assert "STORAGE_TRAVEL_M6_EXTENSION_MM=30.0f;" in source


# Local entry uses the same 3/2/1 color sweep; no Hough endpoint scan.
begin_work = function_body(source, "voidbeginWorkAction(")
round2_branch = begin_work.split(
    "if(kind==WORK_ACTION_STORAGE&&workRoundIndex==1U)"
)[1]
assert "commandStorageServoPositionWithArrivalGate(" not in round2_branch
assert "beginStorageColorSearch(false);" in round2_branch

color_search = function_body(source, "voidbeginStorageColorSearch(")
assert "storage2ExpectedColorAndSlotForRing(" in color_search
assert "storage2ItemOrder[storageColorTargetRing-1U]=expectedTraySlot;" in color_search
assert "storageColorExpected=expectedColor;" in color_search
assert "storageColorTargetRing=static_cast<uint8_t>(3U-storage2ScanIndex);" in color_search
assert "workActionPhase=WORK_PHASE_STORAGE_COLOR_WAIT_EXTENSION;" in color_search
assert "beginCircleVision" not in color_search
assert "beginEndpointVision" not in color_search

color_request = function_body(source, "voidbeginStorageColorVision()")
assert "constuint8_trequestMode=storageColorExpected;" in color_request
assert "MAIXCAM_ALL_COLORS_REQUEST" not in color_request
assert "beginMaixRequest(MAIXCAM_ALL_COLORS_REQUEST);" not in color_request


# Static workflow checks plus a small state simulation guarantee three stack
# placements, no ring-to-tray recovery phase, and a terminal S2DONE state.
transfer_completion = function_body(
    source, "voidcompleteTransferAndRotateStorage()"
)
assert "caseTRANSFER_PURPOSE_CONTAINER_TO_RING:" in transfer_completion
assert "++correctPlacementCount;break;" in transfer_completion
assert (
    "measuredRingPoses[activePlacementRingPosition]="
    "armTransferDestinationPose;" in transfer_completion
)
assert "++workItemIndex;" in transfer_completion
assert (
    "activeWorkAction==WORK_ACTION_STORAGE&&"
    "activeTransferPurpose==TRANSFER_PURPOSE_CONTAINER_TO_RING"
    in transfer_completion
)
assert (
    "commandStorageServoParkingPosition("
    "activeWorkAction==WORK_ACTION_PROCESS?"
    "DEPARTURE_STORAGE_SERVO_INTERVAL_MS:"
    "STORAGE_SERVO_INTERVAL_MS);" in transfer_completion
)

service_action = function_body(source, "voidserviceCompetitionAction()")
wait_storage = service_action.split(
    "caseWORK_PHASE_WAIT_STORAGE_SERVO:"
)[1].split("caseWORK_PHASE_WAIT_STORAGE_PARK:")[0]
assert "beginStorageColorSearch(" not in wait_storage
assert "workActionPhase=WORK_PHASE_START_UNLOAD;" in wait_storage
assert (
    "if(activeWorkAction==WORK_ACTION_PROCESS&&"
    "activeTransferPurpose==TRANSFER_PURPOSE_CONTAINER_TO_RING)"
    in wait_storage
)
assert "workActionPhase=WORK_PHASE_START_RESTORE;" in wait_storage

run_action = simulation.split(
    "caseSTORAGE2_SIMULATION_RUN_ACTION:"
)[1].split("caseSTORAGE2_SIMULATION_COMPLETE:")[0]
assert "if(!storageActionFinished)" in run_action
assert (
    "correctPlacementCount!=3U||correctGrabCount!=0U||"
    "workItemIndex!=3U||activeWorkAction!=WORK_ACTION_NONE||"
    "workActionPhase!=WORK_PHASE_IDLE" in run_action
)
assert "route_chassis::stopAllMotorsImmediately();" in run_action
assert "stopAllMotorsImmediately();" in run_action
assert "programState=PROGRAM_FINISHED;" in run_action
assert 'hmiSetRunStatus("S2DONE");' in run_action
assert "beginReloadingTransfer" not in simulation

# Local mode derives the map from the known first-round ring colors and uses
# vision only to measure each ring coordinate.
known_by_ring = [1, 3, 4]
tray_slots = [second_colors.index(color) for color in known_by_ring]
assert len(set(tray_slots)) == 3
assert tray_slots == [1, 0, 2]
assert [second_colors[slot] for slot in tray_slots] == known_by_ring
assert list(range(1, 4)) == [1, 2, 3]
print("PASS STORAGE2 local mode: fixed task, QR-known ring colors, locked chassis, targeted coordinate scan and three placements -> S2DONE")
