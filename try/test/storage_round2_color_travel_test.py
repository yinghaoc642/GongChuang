#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.cpp"
PLATFORMIO = ROOT / "platformio.ini"
MAIX_CLIENT = ROOT / "lib" / "MaixCamClient" / "src" / "MaixCamClient.cpp"
DEPLOYED_VISION = ROOT / "vision" / "yanyanview5.py"


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

assert (
    "STORAGE_TRAVEL_M5_MAXIMUM_STEP_RATE="
    "arm_config::M5_PLACE_MAXIMUM_STEP_RATE;" in source
)
assert (
    "STORAGE_TRAVEL_M5_STEP_ACCELERATION="
    "arm_config::M5_PLACE_STEP_ACCELERATION;" in source
)
assert "STORAGE2_M7_HEAD_START_MS=300UL;" in source
assert "STORAGE_TRAVEL_M6_EXTENSION_MM=30.0f;" in source
assert "STORAGE_TRAVEL_M6_SPEED_RPM=PLACE_M6_SPEED_RPM;" in source
assert "STORAGE_TRAVEL_M6_ACCELERATION=PLACE_M6_ACCELERATION;" in source

begin_travel = function_body(
    source, "voidbeginStorageTravelPreparation(uint8_troundNumber)"
)
assert "roundNumber==2U" in begin_travel
assert "storageRingForItem(" not in begin_travel
assert (
    "startLiftToHeightMmWithProfile(M7_STANDARD_HEIGHT_MM,"
    "STORAGE_TRAVEL_M7_SPEED_RPM,STORAGE_TRAVEL_M7_ACCELERATION)"
    in begin_travel
)
assert "PROCESS_STORAGE_TRAVEL_M6_SPEED_RPM=162U;" in source
assert "PROCESS_STORAGE_TRAVEL_M6_ACCELERATION=42U;" in source
assert "armMotors.setM5MotionProfile(" not in begin_travel
assert "startArmBaseStandardFrameDegrees(" not in begin_travel

assert (
    "axis.address==liftAxis.address&&"
    "m7ProfilePolicy==M7_PROFILE_APPLY_EXPERIMENT"
    in source
)
assert "axis.commandM7ProfilePolicy=m7ProfilePolicy;" in source
assert "retryM7ProfilePolicy" in source
assert "finalM7ProfilePolicy" in source

travel_ready = function_body(
    source, "boolstorageTravelPreparationReady(uint8_troundNumber)"
)
assert "liftMoveFinished()" in travel_ready
assert "liftAxis.currentMm-M7_STANDARD_HEIGHT_MM" in travel_ready
assert "if(roundNumber==1U" in travel_ready
assert (
    "extensionAxis.currentMm-ENDPOINT_SEARCH_SEED_EXTENSION_MM"
    in travel_ready
)
assert "extensionAxis.currentMm-STORAGE_TRAVEL_M6_EXTENSION_MM" in travel_ready

service_travel = function_body(
    source, "voidserviceStorageTravelPreparation()"
)
head_start = service_travel.split(
    "caseSTORAGE_TRAVEL_PREP_WAIT_M7_HEAD_START:"
)[1].split("caseSTORAGE_TRAVEL_PREP_WAIT_M5_M7:")[0]
assert "deadlineReached(storageTravelM5StartDeadlineMs)" in head_start
assert "STORAGE2_M7_HEAD_START_MS" in begin_travel
assert "armMotors.setM5MotionProfile(" not in service_travel
assert "startArmBaseStandardFrameDegrees(" not in service_travel
assert (
    "storageTravelPreparationPhase="
    "STORAGE_TRAVEL_PREP_WAIT_M5_M7;" in head_start
)
wait_m5_m7 = service_travel.split(
    "caseSTORAGE_TRAVEL_PREP_WAIT_M5_M7:"
)[1].split("caseSTORAGE_TRAVEL_PREP_WAIT_M6:")[0]
assert "armMotors.isM5Running()" in wait_m5_m7
assert "!liftMoveFinished()" in wait_m5_m7
assert "startExtensionToMmWithProfile(STORAGE_TRAVEL_M6_EXTENSION_MM" in wait_m5_m7
assert "PROCESS_STORAGE_TRAVEL_M6_SPEED_RPM" in wait_m5_m7
assert "PROCESS_STORAGE_TRAVEL_M6_ACCELERATION" in wait_m5_m7
assert "liftAxis.currentMm-M7_STANDARD_HEIGHT_MM" in wait_m5_m7

color_request = function_body(source, "voidbeginStorageColorVision()")
assert "constuint8_trequestMode=storageColorExpected;" in color_request
assert "beginMaixRequest(MAIXCAM_ALL_COLORS_REQUEST);" not in color_request
assert "MAIXCAM_ALL_COLORS_REQUEST" not in color_request
assert "MAIXCAM_HOUGH_CIRCLE_REQUEST" not in color_request
assert "MAIXCAM_ENDPOINT_CIRCLE_REQUEST" not in color_request

known_map = function_body(
    source, "boolstorage2ExpectedColorAndSlotForRing("
)
assert "taskPositions[0][firstRoundItem]!=ringPosition" in known_map
assert "expectedColor=taskColors[0][firstRoundItem];" in known_map
assert "taskColors[1][traySlot]!=expectedColor" in known_map
target_roi = function_body(
    source, "boolstorage2CoordinateWithinTargetRoi("
)
assert "imageX-IMAGE_CENTER_X" in target_roi
assert "imageY-IMAGE_CENTER_Y" in target_roi
assert "STORAGE2_TARGET_COLOR_ROI_RADIUS_PIXELS" in target_roi

# The deployed yanyanview5 accepts modes 1..4 as directed red/yellow/blue/
# green requests. STORAGE2 uses only the QR-derived target mode, so only its
# matching threshold group may run even when several colors share the view.
deployed_vision = compact(DEPLOYED_VISION.read_text(encoding="utf-8"))
assert "defdetection_mode_for_request(" in deployed_vision
assert "ifrequested_modein(1,2,3,4,8):" in deployed_vision
assert "ifcurrent_color_to_detectin(1,2,3,4,8):" in deployed_vision
assert "active_color_indices=(current_color_to_detect-1,)" in deployed_vision
assert "target_color_index=current_color_to_detect-1" in deployed_vision

# An unanswered targeted request gets a fresh sequence after 200 ms, at most
# three times. MaixCamClient retransmission itself must retain the live
# sequence; only the bounded recovery path is allowed to increment it.
assert "STORAGE_COLOR_REQUEST_REFRESH_MS" not in source
service_action = function_body(source, "voidserviceCompetitionAction()")
assert "STORAGE_COLOR_REQUEST_REFRESH_MS" not in service_action
assert "STORAGE_COLOR_LOST_RESPONSE_RECOVERY_MS" not in source
assert "arm_config::STORAGE2_SCAN_DWELL_MS*3UL==1500UL" in source
maix_client = compact(MAIX_CLIENT.read_text(encoding="utf-8"))
maix_service = function_body(maix_client, "voidMaixCamClient::service()")
assert "writeRequestFrame(requestedMode_);" in maix_service
assert "requestSequence_=" not in maix_service

color_wait = source.split(
    "caseWORK_PHASE_STORAGE_COLOR_WAIT_COORDINATE:"
)[1].split("caseWORK_PHASE_CIRCLE_WAIT_ARM_LOWER:")[0]
assert (
    "if(detectedColor==storageColorExpected&&"
    "storage2CoordinateWithinTargetRoi(x,y))" in color_wait
)
assert "target-rejected-outside-roi" in color_wait
# A completed targeted request is one-shot. A wrong-color/unusable response
# starts a new sequence immediately; a lost response uses the bounded 200 ms
# recovery path below.
assert "beginStorageColorVision();" in color_wait
assert "if(frame){beginStorageColorVision();}" in color_wait
assert "STORAGE_COLOR_LOST_RESPONSE_RECOVERY_MS" not in color_wait
assert "advanceStorage2Scan();" in color_wait
assert 'routeFault("S2 sweep missing' not in color_wait
# A survey miss advances the sweep. Requests do not reset its deadline.
assert "arm_config::STORAGE2_SCAN_DWELL_MS" in color_wait
assert "storageColorAccuracyWindowStartMs=" not in color_wait
assert "if(storageColorAccuracySampleCount==0U)" in color_wait

begin_work = function_body(source, "voidbeginWorkAction(")
round2_branch = begin_work.split(
    "if(kind==WORK_ACTION_STORAGE&&workRoundIndex==1U)"
)[1]
assert "beginStorageColorSearch(false);" in round2_branch
round2_search_index = round2_branch.index(
    "beginStorageColorSearch(false);"
)
round2_return_index = round2_branch.index(
    "return;", round2_search_index
)
generic_endpoint_index = round2_branch.index(
    "beginArmEndpointPreparation(useStorage1TravelPreparation);",
    round2_return_index,
)
assert round2_search_index < round2_return_index < generic_endpoint_index

wait_storage = source.split("caseWORK_PHASE_WAIT_STORAGE_SERVO:")[1].split(
    "caseWORK_PHASE_WAIT_STORAGE_PARK:"
)[0]
assert "beginStorageColorSearch(" not in wait_storage
assert "workActionPhase=WORK_PHASE_START_UNLOAD;" in wait_storage

departure = function_body(
    source, "boolworkstationMechanismsStoppedForDeparture()"
)
assert (
    "activeWorkAction==WORK_ACTION_PROCESS?"
    "arm_config::RING_ROTATION_CLEARANCE_HEIGHT_MM:"
    "M7_STANDARD_HEIGHT_MM" in departure
)

assert "build_flags=-flto" in compact(
    PLATFORMIO.read_text(encoding="utf-8")
)

print("PASS STORAGE2 travel holds M5; survey uses QR-known targeted colors, center ROI, bounded dwell, no inter-item rescans")
