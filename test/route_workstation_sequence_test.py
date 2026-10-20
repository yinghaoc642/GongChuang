#!/usr/bin/env python3

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import List, Sequence

ROOT = Path(__file__).resolve().parents[1]
PLATFORMIO_PATH = ROOT / "platformio.ini"
ROBOT_CONFIG_PATH = (
    ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h"
)
MAIX_CLIENT_HEADER_PATH = (
    ROOT / "lib" / "MaixCamClient" / "src" / "MaixCamClient.h"
)
MAIX_CLIENT_SOURCE_PATH = (
    ROOT / "lib" / "MaixCamClient" / "src" / "MaixCamClient.cpp"
)
SOURCE_PATH = (
    Path(sys.argv[1]).resolve()
    if len(sys.argv) > 1
    else ROOT / "src" / "main.cpp"
)
ROUTE_CHASSIS_SOURCE_PATH = ROOT / "src" / "route_chassis_main.inc"

class ContractFailure(AssertionError):
    pass

def require(condition: bool, message: str) -> None:
    if not condition:
        raise ContractFailure(message)

def strip_cpp_comments(text: str) -> str:
    output: List[str] = []
    index = 0
    quote = ""
    while index < len(text):
        character = text[index]
        if quote:
            output.append(character)
            if character == "\\" and index + 1 < len(text):
                index += 1
                output.append(text[index])
            elif character == quote:
                quote = ""
            index += 1
            continue

        if character in {'"', "'"}:
            quote = character
            output.append(character)
            index += 1
            continue
        if text.startswith("//", index):
            output.extend((" ", " "))
            index += 2
            while index < len(text) and text[index] != "\n":
                output.append(" ")
                index += 1
            continue
        if text.startswith("/*", index):
            output.extend((" ", " "))
            index += 2
            while index < len(text) and not text.startswith("*/", index):
                output.append("\n" if text[index] == "\n" else " ")
                index += 1
            require(index < len(text), "unterminated C++ block comment")
            output.extend((" ", " "))
            index += 2
            continue

        output.append(character)
        index += 1
    return "".join(output)

def find_matching(
    text: str, opening_index: int, opening: str, closing: str
) -> int:
    require(
        0 <= opening_index < len(text)
        and text[opening_index] == opening,
        f"{opening!r} not found at index {opening_index}",
    )
    depth = 0
    quote = ""
    index = opening_index
    while index < len(text):
        character = text[index]
        if quote:
            if character == "\\":
                index += 2
                continue
            if character == quote:
                quote = ""
            index += 1
            continue
        if character in {'"', "'"}:
            quote = character
        elif character == opening:
            depth += 1
        elif character == closing:
            depth -= 1
            if depth == 0:
                return index
        index += 1
    raise ContractFailure(f"unclosed {opening!r} at index {opening_index}")

def split_top_level_fields(text: str) -> List[str]:
    fields: List[str] = []
    start = 0
    round_depth = 0
    square_depth = 0
    brace_depth = 0
    quote = ""
    index = 0
    while index < len(text):
        character = text[index]
        if quote:
            if character == "\\":
                index += 2
                continue
            if character == quote:
                quote = ""
            index += 1
            continue
        if character in {'"', "'"}:
            quote = character
        elif character == "(":
            round_depth += 1
        elif character == ")":
            round_depth -= 1
        elif character == "[":
            square_depth += 1
        elif character == "]":
            square_depth -= 1
        elif character == "{":
            brace_depth += 1
        elif character == "}":
            brace_depth -= 1
        elif (
            character == ","
            and round_depth == 0
            and square_depth == 0
            and brace_depth == 0
        ):
            fields.append(text[start:index].strip())
            start = index + 1
        index += 1
    fields.append(text[start:].strip())
    return fields

def extract_function_body(
    source_without_comments: str, function_name: str
) -> str:
    for match in re.finditer(
        rf"\b{re.escape(function_name)}\s*\(",
        source_without_comments,
    ):
        opening_parenthesis = source_without_comments.find(
            "(", match.start()
        )
        closing_parenthesis = find_matching(
            source_without_comments,
            opening_parenthesis,
            "(",
            ")",
        )
        index = closing_parenthesis + 1
        while (
            index < len(source_without_comments)
            and source_without_comments[index].isspace()
        ):
            index += 1
        if (
            index < len(source_without_comments)
            and source_without_comments[index] == "{"
        ):
            closing_brace = find_matching(
                source_without_comments, index, "{", "}"
            )
            return source_without_comments[index + 1 : closing_brace]
    raise ContractFailure(f"function {function_name}() was not found")

def case_segment(function_body: str, command_type: str) -> str:
    case_match = re.search(
        rf"\bcase\s+{re.escape(command_type)}\s*:",
        function_body,
    )
    require(
        case_match is not None,
        f"{command_type} case was not found",
    )
    next_case = re.search(
        r"\b(?:case\s+[A-Z][A-Z0-9_]*\s*:|default\s*:)",
        function_body[case_match.end() :],
    )
    end = (
        len(function_body)
        if next_case is None
        else case_match.end() + next_case.start()
    )
    return function_body[case_match.end() : end]

def compact_cpp(text: str) -> str:
    return re.sub(r"\s+", "", text)

def verify_work_action_state_machine(source_without_comments: str) -> None:
    """Check the live chassis-to-workstation bridge."""
    pause_map = compact_cpp(extract_function_body(
        source_without_comments, "workPauseAfterRouteStep"
    ))
    for step, work in (
        (6, "RAW_1"), (8, "PROCESS_1"), (11, "STORAGE_1"),
        (14, "RAW_2"), (16, "PROCESS_2"), (19, "STORAGE_2"),
    ):
        require(
            f"case{step}U:returnINTEGRATED_WORK_{work};" in pause_map,
            f"live route step {step} does not pause for {work}",
        )

    starter = compact_cpp(extract_function_body(
        source_without_comments, "beginIntegratedWorkPause"
    ))
    require_fragments(starter, (
        "route_chassis::stopAllMotorsImmediately();",
        "suspendRouteChassisProfileForWorkstation();",
        "beginWorkAction(kind,roundNumber);",
    ), "station takeover must stop chassis before arm/vision action")
    for kind in ("WORK_ACTION_RAW", "WORK_ACTION_PROCESS", "WORK_ACTION_STORAGE"):
        require(f"kind={kind};" in starter, f"missing live station action {kind}")

    finished = compact_cpp(extract_function_body(
        source_without_comments, "integratedWorkPauseFinished"
    ))
    for work, flag in (
        ("RAW", "rawActionFinished"),
        ("PROCESS", "processActionFinished"),
        ("STORAGE", "storageActionFinished"),
    ):
        for round_number in (1, 2):
            require(
                f"caseINTEGRATED_WORK_{work}_{round_number}:" in finished,
                f"missing completion gate for {work} {round_number}",
            )
        require(f"return{flag};" in finished, f"missing {work} finish flag")

    service = compact_cpp(extract_function_body(
        source_without_comments, "serviceIntegratedRoute"
    ))
    require_fragments(service, (
        "if(integratedWorkPause!=INTEGRATED_WORK_NONE){",
        "if(integratedWorkPauseFinished()){finishIntegratedWorkPause();}",
        "return;}",
        "route_chassis::updateRoute();",
        "workPauseAfterRouteStep(completedStep);",
        "beginIntegratedWorkPause(pause);",
    ), "workstation must block next chassis segment")
    require(service.count("route_chassis::updateRoute();") == 1,
            "live route must advance only once per service call")

    loop_body = compact_cpp(extract_function_body(source_without_comments, "loop"))
    require(0 <= loop_body.find("serviceCompetitionAction();")
            < loop_body.find("serviceIntegratedRoute();"),
            "loop() must service workstation before route")

def verify_process_three_circle_contract(
    source_without_comments: str,
) -> None:
    begin_work = compact_cpp(
        extract_function_body(
            source_without_comments, "beginWorkAction"
        )
    )
    require_fragments(
        begin_work,
        (
            "workRoundIndex = "
            "static_cast<uint8_t>(roundNumber - 1U)",
            "activeEndpointScanRing = 0U",
            "endpointMapStartMs = 0UL",
            "endpointMapCompleteMs = 0UL",
            "resetMeasuredRingMap()",
        ),
        "per-round ring-map reset",
    )
    require(
        begin_work.find("resetMeasuredRingMap();")
        < begin_work.find("if(kind==WORK_ACTION_RAW){"),
        "the ring map is not reset before every workstation branch",
    )

    measured_map = extract_function_body(
        source_without_comments, "buildMeasuredRingMap"
    )
    require_fragments(
        measured_map,
        (
            "!measuredRingPointValid[1U] || "
            "!measuredRingPointValid[3U]",
            "midpointOutwardMm = 0.5f * "
            "(measuredRingPoints[1U].outwardMm + "
            "measuredRingPoints[3U].outwardMm)",
            "midpointLeftMm = 0.5f * "
            "(measuredRingPoints[1U].leftMm + "
            "measuredRingPoints[3U].leftMm)",
            "measuredRingPoints[2U].outwardMm = "
            "midpointOutwardMm",
            "measuredRingPoints[2U].leftMm = midpointLeftMm",
        ),
        "ring 1/3 measurement and ring 2 inference",
    )

    service = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceCompetitionAction"
        )
    )
    require_fragments(
        service,
        (
            "beginEndpointScan(1U)",
            "beginDirectRing3EndpointScan()",
            "completeEndpointMapAndStartTransfers()",
            "if(activeWorkAction==WORK_ACTION_PROCESS&&"
            "activeTransferPurpose=="
            "TRANSFER_PURPOSE_CONTAINER_TO_RING){"
            "workItemIndex=0U;"
            "workActionPhase=WORK_PHASE_START_RELOAD;",
        ),
        "process endpoint/transfer sequence",
    )
    require(
        "beginEndpointScan(2U)" not in service,
        "ring 2 must be inferred from rings 1 and 3, not scanned",
    )

    service_body = extract_function_body(
        source_without_comments, "serviceCompetitionAction"
    )
    coordinate_case = compact_cpp(
        case_segment(
            service_body,
            "WORK_PHASE_ENDPOINT_WAIT_COORDINATE",
        )
    )
    ring1_result_index = coordinate_case.find("if(ring==1U){")
    ring3_handoff_index = coordinate_case.find(
        "beginDirectRing3EndpointScan();",
        ring1_result_index,
    )
    ring1_break_index = coordinate_case.find(
        "break;", ring3_handoff_index
    )
    map_completion_index = coordinate_case.find(
        "completeEndpointMapAndStartTransfers();",
        ring1_break_index,
    )
    require(
        0 <= ring1_result_index < ring3_handoff_index
        < ring1_break_index < map_completion_index,
        "ring 1 result no longer hands off to ring 3 before "
        "the endpoint map is completed",
    )
    require(
        "WORK_PHASE_ENDPOINT_WAIT_RING3_RAISE" not in service_body,
        "ring-1 handoff still performs the removed 15 mm M7 raise",
    )
    endpoint_contract = compact_cpp(source_without_comments)
    begin_endpoint_scan = compact_cpp(
        extract_function_body(
            source_without_comments, "beginEndpointScan"
        )
    )
    direct_ring3_scan = compact_cpp(
        extract_function_body(
            source_without_comments, "beginDirectRing3EndpointScan"
        )
    )
    predicted_ring3_pose = compact_cpp(
        extract_function_body(
            source_without_comments, "predictedRing3SearchPose"
        )
    )
    require(
        "ENDPOINT_FINE_VISION_PHYSICAL_LOWER_MM=105.0f;"
        in endpoint_contract
        and "ENDPOINT_VISION_HEIGHT_MM="
        "M7_STANDARD_HEIGHT_MM-ENDPOINT_FINE_VISION_LOWER_MM;"
        in endpoint_contract
        and "liftAxis.currentMm-ENDPOINT_VISION_HEIGHT_MM"
        in begin_endpoint_scan
        and "nominalRingPose(ringPosition,"
        "ENDPOINT_FINE_VISION_LOWER_MM,searchPose)"
        in begin_endpoint_scan
        and "liftAxis.currentMm-ENDPOINT_VISION_HEIGHT_MM"
        in direct_ring3_scan,
        "endpoint scans no longer share the fixed physical 105 mm M7 height",
    )
    require(
        "planarPointToCameraPose(predictedPoint,"
        "ENDPOINT_FINE_VISION_LOWER_MM,pose)"
        in predicted_ring3_pose
        and "rawPredictedAngleDegrees>="
        "ENDPOINT_RING3_SEARCH_MINIMUM_ANGLE_DEGREES"
        in predicted_ring3_pose
        and "rawPredictedAngleDegrees<="
        "ENDPOINT_RING3_SEARCH_MAXIMUM_ANGLE_DEGREES"
        in predicted_ring3_pose
        and "if(!predictedPoseInsideRing3Window){"
        in predicted_ring3_pose
        and "pose.standardFrameAngleDegrees="
        "ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES;"
        in predicted_ring3_pose
        and "pose.extensionMm=ENDPOINT_SEARCH_SEED_EXTENSION_MM;"
        in predicted_ring3_pose,
        "ring-3 handoff does not keep the bounded prediction with "
        "the verified -45 degree/90 mm fallback",
    )
    require(
        "WORK_PHASE_ENDPOINT_WAIT_FINE_LOWER" not in service_body
        and "WORK_PHASE_ENDPOINT_WAIT_RING3_RAISE"
        not in service_body,
        "the removed endpoint M7 transition phases are still present",
    )

    unload = extract_function_body(
        source_without_comments, "beginUnloadingTransfer"
    )
    require_fragments(
        unload,
        (
            "if (activeWorkAction == WORK_ACTION_PROCESS)",
            "processItemIndexForSequence("
            "workItemIndex, transferItemIndex)",
            "taskPositions[workRoundIndex][transferItemIndex]",
            "activeTransferPurpose = "
            "TRANSFER_PURPOSE_CONTAINER_TO_RING",
        ),
        "three process placements",
    )
    reload = extract_function_body(
        source_without_comments, "beginReloadingTransfer"
    )
    require_fragments(
        reload,
        (
            "processItemIndexForSequence("
            "workItemIndex, transferItemIndex)",
            "taskPositions[workRoundIndex][transferItemIndex]",
            "activeTransferPurpose = "
            "TRANSFER_PURPOSE_RING_TO_CONTAINER",
        ),
        "three process pickups",
    )

def require_fragments(
    body: str, fragments: Sequence[str], label: str
) -> None:
    compact_body = compact_cpp(body)
    for fragment in fragments:
        require(
            compact_cpp(fragment) in compact_body,
            f"{label} is missing {fragment}",
        )

def verify_process_ring_transfer_order_contract(
    source_without_comments: str,
) -> None:
    require(
        "PROCESS_RING_TRANSFER_ORDER"
        not in source_without_comments,
        "PROCESS still has a fixed physical-ring transfer order; "
        "only endpoint scanning may use ring 1 then ring 3",
    )

    item_lookup = compact_cpp(
        extract_function_body(
            source_without_comments,
            "processItemIndexForSequence",
        )
    )
    require_fragments(
        item_lookup,
        (
            "if (sequenceIndex >= 3U)",
            "destinationRing = "
            "taskPositions[workRoundIndex][sequenceIndex]",
            "destinationRing < 1U || destinationRing > 3U",
            "itemIndex = sequenceIndex",
            "return true",
        ),
        "PROCESS QR-index lookup",
    )
    destination_ring_index = item_lookup.find(
        "constuint8_tdestinationRing="
        "taskPositions[workRoundIndex][sequenceIndex];"
    )
    destination_validation_index = item_lookup.find(
        "destinationRing<1U||destinationRing>3U",
        destination_ring_index,
    )
    assign_index = item_lookup.find(
        "itemIndex=sequenceIndex;", destination_validation_index
    )
    require(
        0 <= destination_ring_index
        < destination_validation_index
        < assign_index,
        "PROCESS does not validate the destination at the same "
        "QR index before selecting that item/slot",
    )
    require(
        "taskColors[" not in item_lookup
        and "ROUGH_PROCESSING_CALIBRATION_MODE"
        not in item_lookup
        and "for(" not in item_lookup,
        "PROCESS QR order is color-dependent, mode-dependent, "
        "or still searches for a preferred physical ring",
    )

    unload = compact_cpp(
        extract_function_body(
            source_without_comments, "beginUnloadingTransfer"
        )
    )
    require_fragments(
        unload,
        (
            "if (activeWorkAction == WORK_ACTION_PROCESS)",
            "processItemIndexForSequence("
            "workItemIndex, transferItemIndex)",
            "ringPosition = "
            "taskPositions[workRoundIndex][transferItemIndex]",
            "processItemIndexForSequence("
            "static_cast<uint8_t>(workItemIndex + 1U), "
            "nextStorageSlot)",
        ),
        "PROCESS QR-ordered placement sequence",
    )
    require_fragments(
        unload,
        (
            "prepareFirstPlacedRingPickup = "
            "activeWorkAction == WORK_ACTION_PROCESS && "
            "workItemIndex == 2U",
            "processItemIndexForSequence(0U, firstTransferItemIndex)",
            "PROCESS_PLACE_LOWER_MM + "
            "processRingReturnPickExtraLowerMm()",
            "[P2P] first ring/slot=",
            "prepareFirstPlacedRingPickup)",
        ),
        "final placement to first-ring pickup preposition",
    )
    process_branch_index = unload.find(
        "if(activeWorkAction==WORK_ACTION_PROCESS){"
    )
    lookup_index = unload.find(
        "processItemIndexForSequence("
        "workItemIndex,transferItemIndex)",
        process_branch_index,
    )
    ring_index = unload.find(
        "ringPosition="
        "taskPositions[workRoundIndex][transferItemIndex];",
        lookup_index,
    )
    require(
        0 <= process_branch_index < lookup_index < ring_index,
        "PROCESS placement does not select the current QR item "
        "before reading its same-index destination ring",
    )

    reload = compact_cpp(
        extract_function_body(
            source_without_comments, "beginReloadingTransfer"
        )
    )
    require_fragments(
        reload,
        (
            "processItemIndexForSequence("
            "workItemIndex, transferItemIndex)",
            "ringPosition = "
            "taskPositions[workRoundIndex][transferItemIndex]",
            "activeTransferPurpose = "
            "TRANSFER_PURPOSE_RING_TO_CONTAINER",
        ),
        "PROCESS QR-ordered pickup sequence",
    )
    require(
        "ROUGH_PROCESSING_CALIBRATION_MODE"
        not in unload
        and "ROUGH_PROCESSING_CALIBRATION_MODE"
        not in reload,
        "formal rounds or mode 1 can bypass the shared "
        "QR-index transfer order",
    )

    service_body = extract_function_body(
        source_without_comments, "serviceCompetitionAction"
    )
    endpoint_timing = compact_cpp(source_without_comments)
    require(
        "armTransferNextSourceMapped="
        "armTransferPrepareNextSource&&nextSourceMapped;"
        in endpoint_timing
        and "mappedNextPose=armTransferPrepareNextSource&&"
        "armTransferNextSourceMapped;" in endpoint_timing,
        "cross-stage first-ring preposition is not treated as a mapped pose",
    )
    require(
        "ENDPOINT_LOCAL_MOVE_SETTLE_PREVIOUS_MS=80UL;"
        in endpoint_timing
        and "ENDPOINT_LOCAL_MOVE_SETTLE_MS=10UL;"
        in endpoint_timing
        and "ENDPOINT_LOCAL_MOVE_SETTLE_MS*8UL=="
        "ENDPOINT_LOCAL_MOVE_SETTLE_PREVIOUS_MS"
        in endpoint_timing
        and endpoint_timing.count(
            "millis()+ENDPOINT_LOCAL_MOVE_SETTLE_MS;"
        ) == 2,
        "endpoint correction-to-re-recognition settle is not reduced "
        "from 80 ms to 10 ms on both entry and correction paths",
    )
    require(
        "ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS=5.0f;"
        in endpoint_timing
        and "RING_ENDPOINT_MINIMUM_CONFIDENCE=600U;"
        in endpoint_timing
        and "ENDPOINT_STABLE_DIRECT_ACCEPT_MINIMUM_CONFIDENCE=800U;"
        in endpoint_timing
        and "constboolstableCameraResultAlreadyFinal="
        "centerErrorPixels<="
        "ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS&&"
        "confidence>="
        "ENDPOINT_STABLE_DIRECT_ACCEPT_MINIMUM_CONFIDENCE;"
        in endpoint_timing
        and "if(!endpointFineVisionActive&&"
        "!stableCameraResultAlreadyFinal){" in endpoint_timing
        and "if(stableCameraResultAlreadyFinal){"
        "endpointCenteredConfirmationCount="
        "ENDPOINT_FINAL_CENTER_CONFIRMATIONS;"
        in endpoint_timing
        and "}elseif(endpointCenteredConfirmationCount<255U){"
        in endpoint_timing,
        "stable mode-10 endpoint results are not accepted directly or the "
        "confidence-600 independent fine/two-confirmation fallback was lost",
    )
    endpoint_servo_target = compact_cpp(
        extract_function_body(
            source_without_comments, "endpointServoTargetPose"
        )
    )
    require(
        "ENDPOINT_SERVO_MEDIUM_DISTANCE_THRESHOLD_MM=5.0f;"
        in endpoint_timing
        and "ENDPOINT_SERVO_FAR_DISTANCE_THRESHOLD_MM=10.0f;"
        in endpoint_timing
        and "ENDPOINT_SERVO_MEDIUM_DISTANCE_MULTIPLIER=1.20f;"
        in endpoint_timing
        and "ENDPOINT_SERVO_FAR_DISTANCE_MULTIPLIER=1.50f;"
        in endpoint_timing
        and "constboolfarDistance=measuredCorrectionMm>"
        "ENDPOINT_SERVO_FAR_DISTANCE_THRESHOLD_MM;"
        in endpoint_servo_target
        and "constboolmediumDistance=!farDistance&&"
        "measuredCorrectionMm>="
        "ENDPOINT_SERVO_MEDIUM_DISTANCE_THRESHOLD_MM;"
        in endpoint_servo_target
        and "fineStage?(farDistance?"
        "ENDPOINT_FINE_FAR_DISTANCE_GAIN:"
        "(mediumDistance?ENDPOINT_FINE_MEDIUM_DISTANCE_GAIN:"
        "ENDPOINT_FINE_SERVO_GAIN)):(farDistance?"
        "ENDPOINT_COARSE_FAR_DISTANCE_GAIN:"
        "(mediumDistance?ENDPOINT_COARSE_MEDIUM_DISTANCE_GAIN:"
        "ENDPOINT_COARSE_SERVO_GAIN))"
        in endpoint_servo_target
        and "errorOutwardMm*gain" in endpoint_servo_target
        and "errorLeftMm*gain" in endpoint_servo_target
        and "if(commandedCorrectionMm>maximumCorrectionMm)"
        in endpoint_servo_target,
        "endpoint M5/M6 correction no longer uses the requested "
        "5/10 mm distance gain bands before the existing safety cap",
    )
    require(
        "ENDPOINT_BASE_TO_EXTENSION_SETTLE_PREVIOUS_MS=50UL;"
        in endpoint_timing
        and "ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS=10UL;"
        in endpoint_timing
        and "ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS=10UL;"
        in endpoint_timing
        and "ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS=10UL;"
        in endpoint_timing
        and "VISION_ACTION_DELAY_CAP_MS=10UL;" in endpoint_timing
        and "VISION_POST_MOTION_SETTLE_TIME_MS=10UL;"
        in endpoint_timing
        and "VISION_HEADING_STABLE_TIME_MS=10UL;"
        in endpoint_timing
        and endpoint_timing.count(
            "millis()+ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS;"
        )
        == 2
        and "updateHeadingLock(MOTION_TIMEOUT_MS,"
        "ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS,"
        "ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS)"
        in endpoint_timing,
        "vision action handoffs are not all capped at 10 ms",
    )
    require(
        endpoint_timing.count(
            "updateHeadingLock(MOTION_TIMEOUT_MS,"
            "VISION_POST_MOTION_SETTLE_TIME_MS,"
            "VISION_HEADING_STABLE_TIME_MS)"
        )
        == 2,
        "circle correction and post-vision heading locks do not both use "
        "the 10 ms vision gate",
    )
    initial_storage_parallel = compact_cpp(
        extract_function_body(
            source_without_comments,
            "startInitialStorageServoHeadStart",
        )
    )
    require_fragments(
        initial_storage_parallel,
        (
            "endpointInitialStorageCommanded",
            "activeWorkAction == WORK_ACTION_PROCESS",
            "processItemIndexForSequence(0U, initialStorageSlot)",
            "activeWorkAction == WORK_ACTION_STORAGE",
            "workRoundIndex == 0U",
            "commandStorageServoPosition(initialStorageSlot)",
            "workStorageServoDeadlineMs = "
            "millis() + STORAGE_SERVO_SETTLE_MS",
            "endpointInitialStorageCommanded = true",
        ),
        "PROCESS and STORAGE1 initial storage parallel command",
    )
    arm_standard_service = extract_function_body(
        source_without_comments, "serviceArmStandardization"
    )
    endpoint_pre_open_case = compact_cpp(
        case_segment(
            arm_standard_service, "ARM_STANDARD_WAIT_PRE_OPEN_GAP"
        )
    )
    endpoint_storage_index = endpoint_pre_open_case.find(
        "startInitialStorageServoHeadStart()"
    )
    endpoint_gripper_index = endpoint_pre_open_case.find(
        "commandArmStandardizationGripperOpen()",
        endpoint_storage_index,
    )
    endpoint_open_deadline_index = endpoint_pre_open_case.find(
        "static_cast<uint32_t>(GRIPPER_OPEN_SETTLE_MS)*1000UL+"
        "ENDPOINT_GRIPPER_M7_HANDOFF_US;",
        endpoint_gripper_index,
    )
    require(
        0 <= endpoint_storage_index < endpoint_gripper_index
        < endpoint_open_deadline_index,
        "PROCESS/STORAGE1 turntable is not launched at the start of the "
        "60 ms gripper gate before M7",
    )
    endpoint_open_case = compact_cpp(
        case_segment(arm_standard_service, "ARM_STANDARD_WAIT_OPEN")
    )
    endpoint_lift_index = endpoint_open_case.find(
        "startLiftToHeightMm(ENDPOINT_VISION_HEIGHT_MM)"
    )
    endpoint_m5_index = endpoint_open_case.find(
        "startArmBaseStandardFrameDegrees("
        "armStandardBaseTargetDegrees);",
        endpoint_lift_index,
    )
    endpoint_parallel_phase_index = endpoint_open_case.find(
        "armStandardPhase=ARM_STANDARD_WAIT_ENDPOINT_PARALLEL;",
        endpoint_m5_index,
    )
    require(
        0 <= endpoint_lift_index < endpoint_m5_index
        < endpoint_parallel_phase_index
        and "startInitialStorageServoHeadStart()" not in endpoint_open_case,
        "M7/M5 original launch point changed or the turntable is still "
        "launched together with M7",
    )
    endpoint_parallel_wait = compact_cpp(
        case_segment(
            arm_standard_service,
            "ARM_STANDARD_WAIT_ENDPOINT_PARALLEL",
        )
    )
    require(
        "workStorageServoDeadlineMs" not in endpoint_parallel_wait
        and "endpointInitialStorageCommanded" not in endpoint_parallel_wait,
        "PROCESS/STORAGE1 endpoint preparation added a new all-three-arrival gate",
    )
    require_fragments(
        unload,
        (
            "if (!endpointInitialStorageCommanded || "
            "workStorageServoDeadlineMs == 0UL)",
            "if (!deadlineReached(workStorageServoDeadlineMs))",
        ),
        "unchanged first-pick storage-arrival gate",
    )
    local_settle_case = compact_cpp(
        case_segment(
            service_body, "WORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE"
        )
    )
    first_slot_lookup = re.search(
        r"processItemIndexForSequence\(0U,"
        r"([A-Za-z_][A-Za-z0-9_]*)\)",
        local_settle_case,
    )
    require(
        first_slot_lookup is not None,
        "the first PROCESS placement does not select QR item/slot 0",
    )
    first_slot_name = first_slot_lookup.group(1)
    first_slot_command_index = local_settle_case.find(
        f"commandStorageServoPosition({first_slot_name});",
        first_slot_lookup.end(),
    )
    require(
        first_slot_command_index >= 0,
        "the QR item-0 lookup result is not used for the initial "
        "storage-servo command",
    )

    completion = compact_cpp(
        extract_function_body(
            source_without_comments,
            "completeTransferAndRotateStorage",
        )
    )
    require_fragments(
        completion,
        (
            "if (activeWorkAction == WORK_ACTION_PROCESS)",
            "nextSequenceIndex = "
            "workItemIndex >= 3U ? 0U : workItemIndex",
            "processItemIndexForSequence("
            "nextSequenceIndex, nextStorageSlot)",
            "commandStorageServoPosition(nextStorageSlot)",
        ),
        "PROCESS next QR-slot rotation",
    )

def verify_raw_pick_and_process_entry_contract(
    source_without_comments: str,
    maix_client_without_comments: str,
) -> None:
    require_fragments(
        source_without_comments,
        (
            "MAIXCAM_RED_REQUEST = "
            "gongchuang::MaixCamClient::RED_REQUEST",
            "MAIXCAM_YELLOW_REQUEST = "
            "gongchuang::MaixCamClient::YELLOW_REQUEST",
            "MAIXCAM_BLUE_REQUEST = "
            "gongchuang::MaixCamClient::BLUE_REQUEST",
            "MAIXCAM_GREEN_REQUEST = "
            "gongchuang::MaixCamClient::GREEN_REQUEST",
            "constexpr bool REQUIRE_RAW_PICK_QR_ORDER = true",
            "static_assert(REQUIRE_RAW_PICK_QR_ORDER",
            "constexpr float RAW_VIEW_EXTENSION_MM = 20.0f",
            "constexpr float M6_RING2_MINIMUM_EXTENSION_MM = -9.5f",
            "constexpr float M6_RAW_PICK_MINIMUM_EXTENSION_MM = -9.5f",
            "RAW M6 retraction must retain 0.5 mm hard-stop margin",
            "constexpr uint32_t RAW_TARGET_REQUEST_REFRESH_MS =",
            "GONGCHUANG_RAW_COLOR_REQUEST_POLICY == 1",
            "? 5000UL",
            ": 15000UL",
            "constexpr uint32_t RAW_ACTION_TIMEOUT_MS = 65000UL",
            "constexpr uint32_t ARM_BASE_SETTLE_MS = 10UL",
            "constexpr uint32_t PROCESS_ENTRY_M5_SETTLE_US = 100UL",
            "constexpr uint32_t ENDPOINT_GRIPPER_M7_HANDOFF_US = 100UL",
            "constexpr uint16_t STORAGE_SERVO_INTERVAL_MS = 409U",
            "constexpr uint32_t STORAGE_SERVO_SETTLE_MS = 510UL",
            "constexpr uint16_t RAW_STORAGE_SERVO_INTERVAL_MS = 200U",
            "constexpr uint32_t RAW_STORAGE_SERVO_SETTLE_MS = 200UL",
            "constexpr uint32_t RAW_NEXT_STORAGE_COMMAND_DELAY_MS = 1000UL",
            "STORAGE_SERVO_SETTLE_MS >=",
            "static_cast<uint32_t>(STORAGE_SERVO_INTERVAL_MS) + 100UL",
            "constexpr uint32_t GRIPPER_OPEN_SETTLE_MS = 60UL",
            "constexpr uint16_t GRIPPER_TARGET_PLACE_OPEN_INTERVAL_MS = 40U",
            "constexpr uint16_t GRIPPER_DOUBLE_SPEED_INTERVAL_MS = 20U",
            "constexpr uint32_t GRIPPER_TARGET_PLACE_OPEN_SETTLE_MS = 40UL",
            "constexpr uint32_t GRIPPER_TRAY_RELEASE_OPEN_SETTLE_MS = 20UL",
            "constexpr uint32_t GRIPPER_TARGET_PICK_CLOSE_SETTLE_MS = 120UL",
            "constexpr uint32_t GRIPPER_TRAY_PICK_CLOSE_SETTLE_MS = 160UL",
            "constexpr uint32_t WORK_M7_TO_GRIPPER_GAP_MS = 10UL",
            "constexpr uint32_t WORK_GRIPPER_TO_M7_GAP_MS = 10UL",
            "constexpr uint32_t RAW_PROCESS_PICKUP_M7_RAISE_ADDITIONAL_DELAY_MS =",
            "50UL",
            "constexpr float RAW_PICK_PHYSICAL_LOWER_MM = 77.2f",
            "constexpr uint32_t WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_MS = 5UL",
            "constexpr uint32_t WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS = 100UL",
        ),
        "RAW order/view/timeout constants",
    )
    require_fragments(
        maix_client_without_comments,
        (
            "RED_REQUEST = 0x01U",
            "YELLOW_REQUEST = 0x02U",
            "BLUE_REQUEST = 0x03U",
            "GREEN_REQUEST = 0x04U",
        ),
        "MaixCAM targeted-color request IDs",
    )

    begin_request = extract_function_body(
        maix_client_without_comments, "beginRequest"
    )
    require_fragments(
        begin_request,
        (
            "requestIsValid(request)",
            "requestedMode_ = request",
            "modeCommandSent_ = false",
        ),
        "MaixCAM targeted-color request acceptance",
    )
    finish_coordinate = extract_function_body(
        maix_client_without_comments, "finishCoordinateLine"
    )
    require_fragments(
        finish_coordinate,
        (
            "response.sequence != requestSequence_",
            "response.mode != requestedMode_",
            "responseTargetMatchesMode(response.target)",
        ),
        "MaixCAM targeted-color response validation",
    )

    begin_raw_vision = compact_cpp(
        extract_function_body(
            source_without_comments, "beginRawItemVision"
        )
    )
    require_fragments(
        begin_raw_vision,
        (
            "expectedColor = "
            "taskColors[workRoundIndex][rawCollectedCount]",
            "expectedColor < MAIXCAM_RED_REQUEST || "
            "expectedColor > MAIXCAM_GREEN_REQUEST",
            "GONGCHUANG_RAW_COLOR_REQUEST_POLICY == 1",
            "expectedColor",
            "MAIXCAM_ALL_COLORS_REQUEST",
            "beginMaixRequest(requestMode)",
            "workVisionRequestStartMs = millis()",
            "workActionPhase = WORK_PHASE_RAW_WAIT_RESULT",
        ),
        "selectable RAW camera request",
    )
    expected_color_index = begin_raw_vision.find(
        "constuint8_texpectedColor="
        "taskColors[workRoundIndex][rawCollectedCount];"
    )
    request_index = begin_raw_vision.find(
        "beginMaixRequest(requestMode);"
    )
    wait_phase_index = begin_raw_vision.find(
        "workActionPhase=WORK_PHASE_RAW_WAIT_RESULT;"
    )
    require(
        0 <= expected_color_index < request_index < wait_phase_index
        and begin_raw_vision.count("beginMaixRequest(") == 1,
        "RAW vision must select the current QR color before issuing its "
        "single request",
    )
    require(
        "#defineGONGCHUANG_RAW_PICKUP_LOGIC_MODE0"
        in compact_cpp(source_without_comments)
        and "#defineGONGCHUANG_RAW_COLOR_REQUEST_POLICY0"
        in compact_cpp(source_without_comments)
        and "GONGCHUANG_RAW_COLOR_REQUEST_POLICY==1"
        in begin_raw_vision
        and "?expectedColor:MAIXCAM_ALL_COLORS_REQUEST"
        in begin_raw_vision,
        "RAW legacy A-point motion must default to historical mode-8 all-color "
        "requests while retaining policy 1 for targeted QR-color experiments",
    )

    raw_slot_preparation = compact_cpp(
        extract_function_body(
            source_without_comments,
            "beginRawExpectedSlotPreparation",
        )
    )
    require_fragments(
        raw_slot_preparation,
        (
            "expectedColor = "
            "taskColors[workRoundIndex][rawCollectedCount]",
            "mappedSlot = rawStorageSlotForColor(expectedColor)",
            "static_cast<uint8_t>(mappedSlot) != "
            "rawCollectedCount",
            "commandRawStorageServoPositionWithArrivalGate("
            "static_cast<uint8_t>(mappedSlot))",
            "workStorageServoDeadlineMs = 0UL",
            "workActionPhase = "
            "WORK_PHASE_RAW_WAIT_EXPECTED_SLOT",
        ),
        "RAW expected-slot preparation",
    )
    slot_command_index = raw_slot_preparation.find(
        "commandRawStorageServoPositionWithArrivalGate("
        "static_cast<uint8_t>(mappedSlot));"
    )
    slot_deadline_index = raw_slot_preparation.find(
        "workStorageServoDeadlineMs=0UL;",
        slot_command_index,
    )
    slot_wait_phase_index = raw_slot_preparation.find(
        "workActionPhase=WORK_PHASE_RAW_WAIT_EXPECTED_SLOT;",
        slot_deadline_index,
    )
    require(
        0 <= slot_command_index < slot_deadline_index
        < slot_wait_phase_index,
        "RAW does not command the current expected slot before "
        "arming its 200 ms real-angle arrival gate",
    )
    require(
        "beginRawItemVision();" not in raw_slot_preparation
        and "beginMaixRequest(" not in raw_slot_preparation,
        "RAW starts vision before the expected tray slot settles",
    )

    require(
        "confirmRawCoordinate" not in source_without_comments
        and "RAW_MAIN_CONFIRMATION_" not in source_without_comments,
        "STM32 still performs an obsolete second RAW confirmation "
        "after the camera's internal two-frame check",
    )

    raw_target_pose = compact_cpp(
        extract_function_body(
            source_without_comments, "rawTargetPoseFromCameraPose"
        )
    )
    raw_target_pose_legacy_wrapper = compact_cpp(
        extract_function_body(
            source_without_comments, "rawTargetPose"
        )
    )
    raw_pixel_scale = compact_cpp(
        extract_function_body(
            source_without_comments, "rawMillimetersPerPixel"
        )
    )
    require_fragments(
        raw_pixel_scale,
        (
            "pixelRadius / "
            "RAW_PIXEL_MAPPING_BLEND_END_RADIUS_PIXELS",
            "if (blend < 0.0f)",
            "else if (blend > 1.0f)",
            "RAW_NEAR_MM_PER_PIXEL + "
            "(RAW_FAR_MM_PER_PIXEL - RAW_NEAR_MM_PER_PIXEL) * blend",
        ),
        "continuous RAW pixel-to-M6 mapping",
    )
    require(
        "PIXEL_MAPPING_SWITCH_RADIUS_PIXELS" not in source_without_comments
        and "rawMillimetersPerPixel(pixelRadius)" in raw_target_pose,
        "RAW M6 mapping still contains the discontinuous 50 px switch",
    )
    require_fragments(
        raw_target_pose,
        (
            "cameraRadiusMm = "
            "ARM_PIVOT_TO_CAMERA_CENTER_MM + "
            "cameraPose.extensionMm",
            "extensionMm < M6_RAW_PICK_MINIMUM_EXTENSION_MM - "
            "ARM_AXIS_POSITION_TOLERANCE_MM",
            "return RAW_TARGET_WAIT_TOO_NEAR",
            "extensionMm > M6_MAXIMUM_EXTENSION_MM + "
            "ARM_AXIS_POSITION_TOLERANCE_MM",
            "return RAW_TARGET_WAIT_TOO_FAR",
            "return RAW_TARGET_POSE_VALID",
        ),
        "RAW target reach classification",
    )
    require_fragments(
        raw_target_pose_legacy_wrapper,
        (
            "rawTargetPoseFromCameraPose",
            "ArmPose(0.0f, RAW_VIEW_EXTENSION_MM, "
            "M7_STANDARD_HEIGHT_MM)",
        ),
        "legacy A-pose RAW coordinate wrapper",
    )
    near_index = raw_target_pose.find(
        "returnRAW_TARGET_WAIT_TOO_NEAR;"
    )
    far_index = raw_target_pose.find(
        "returnRAW_TARGET_WAIT_TOO_FAR;"
    )
    valid_index = raw_target_pose.find(
        "returnRAW_TARGET_POSE_VALID;"
    )
    require(
        0 <= near_index < far_index < valid_index,
        "RAW target reach results are not ordered "
        "near-wait, far-wait, valid",
    )
    require(
        "routeFault(" not in raw_target_pose,
        "an out-of-reach RAW observation still faults the route",
    )

    arm_pose_valid = compact_cpp(
        extract_function_body(
            source_without_comments, "armPoseIsValid"
        )
    )
    begin_arm_transfer = compact_cpp(
        extract_function_body(
            source_without_comments, "beginArmTransfer"
        )
    )
    require_fragments(
        arm_pose_valid,
        (
            "pose.extensionMm >= minimumExtensionMm - "
            "ARM_AXIS_POSITION_TOLERANCE_MM",
        ),
        "transfer pose explicit M6 lower-bound validation",
    )
    require_fragments(
        begin_arm_transfer,
        (
            "sourceMinimumExtensionMm = rawFastProfile ? "
            "M6_RAW_PICK_MINIMUM_EXTENSION_MM",
            "mapSource ? M6_RING2_MINIMUM_EXTENSION_MM : "
            "M6_STANDARD_EXTENSION_MM",
            "armPoseIsValid(source, sourceMinimumExtensionMm)",
            "destinationMinimumExtensionMm = mapDestination ? "
            "M6_RING2_MINIMUM_EXTENSION_MM : "
            "M6_STANDARD_EXTENSION_MM",
        ),
        "both-round RAW pickup -9.5 mm transfer validation",
    )
    require(
        "armPoseIsValid(source,mapSource)"
        not in begin_arm_transfer,
        "RAW pickup still reuses the ordinary zero-minimum source "
        "validation",
    )

    begin_work = compact_cpp(
        extract_function_body(
            source_without_comments, "beginWorkAction"
        )
    )
    require_fragments(
        begin_work,
        (
            "if (kind == WORK_ACTION_RAW)",
            "parallelPreparation = rawParallelPreparationSafe()",
            "beginRawExpectedSlotPreparation()",
            "if (parallelPreparation)",
            "beginRawParallelPreparation()",
            "beginArmStandardization("
            "0.0f, false, RAW_VIEW_EXTENSION_MM)",
            "if (kind == WORK_ACTION_PROCESS)",
            "if (!rawTravelM5ZeroPending)",
            "startArmBaseStandardFrameDegrees(0.0f)",
            "rawTravelM5ZeroPending = true",
            "workActionPhase = "
            "WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO",
        ),
        "parallel RAW view and PROCESS standard-zero entry",
    )
    raw_branch_index = begin_work.find(
        "if(kind==WORK_ACTION_RAW){"
    )
    raw_branch_return_index = begin_work.find(
        "return;", begin_work.find(
            "workActionPhase=WORK_PHASE_PREPARE;",
            raw_branch_index,
        )
    )
    raw_slot_index = begin_work.find(
        "beginRawExpectedSlotPreparation();",
        raw_branch_index,
    )
    raw_parallel_index = begin_work.find(
        "beginRawParallelPreparation();", raw_slot_index
    )
    raw_fallback_index = begin_work.find(
        "beginArmStandardization(", raw_parallel_index
    )
    require(
        0 <= raw_branch_index < raw_slot_index < raw_parallel_index
        < raw_fallback_index < raw_branch_return_index,
        "RAW entry does not command the first slot before selecting its "
        "safe parallel/fallback arm path",
    )
    require(
        "commandStorageServoParkingPosition();"
        not in begin_work[raw_branch_index:raw_branch_return_index],
        "RAW entry still performs the redundant 165-degree parking move",
    )

    raw_parallel_safe = compact_cpp(
        extract_function_body(
            source_without_comments, "rawParallelPreparationSafe"
        )
    )
    require_fragments(
        raw_parallel_safe,
        (
            "liftMoveFinished()",
            "extensionMoveFinished()",
            "armLinearPositionQueryAxis == nullptr",
            "!armMotors.isM5Running()",
            "!armBaseMotionWatchdogActive",
            "liftAxis.currentMm - M7_STANDARD_HEIGHT_MM",
            "extensionAxis.currentMm - M6_STANDARD_EXTENSION_MM",
        ),
        "RAW parallel-entry zero/idle safety gate",
    )
    raw_parallel_start = compact_cpp(
        extract_function_body(
            source_without_comments, "beginRawParallelPreparation"
        )
    )
    require_fragments(
        raw_parallel_start,
        (
            "commandArmStandardizationGripperOpen()",
            "armStandardDeadlineMs = millis() + GRIPPER_OPEN_SETTLE_MS",
            "startArmBaseStandardFrameDegrees(0.0f)",
            "startExtensionToMmWithProfile("
            "RAW_VIEW_EXTENSION_MM, RAW_M6_SPEED_RPM, RAW_M6_ACCELERATION)",
            "armStandardPhase = ARM_STANDARD_WAIT_RAW_PARALLEL",
        ),
        "RAW M5/M6/gripper parallel preparation",
    )
    require(
        "startLift" not in raw_parallel_start,
        "RAW parallel entry moves M7 despite the shared M6/M7 serial gate",
    )
    process_branch_index = begin_work.find(
        "if(kind==WORK_ACTION_PROCESS){"
    )
    process_branch_return_index = begin_work.find(
        "return;", process_branch_index
    )
    process_zero_command_index = begin_work.find(
        "startArmBaseStandardFrameDegrees(0.0f);",
        process_branch_index,
    )
    process_wait_phase_index = begin_work.find(
        "workActionPhase="
        "WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO;",
        process_branch_index,
    )
    require(
        0 <= process_branch_index < process_zero_command_index
        < process_wait_phase_index < process_branch_return_index,
        "PROCESS entry does not command/retain standard M5 zero "
        "and enter the zero-arrival gate",
    )
    require(
        "beginArmEndpointPreparation();"
        not in begin_work[
            process_branch_index:process_branch_return_index
        ],
        "PROCESS entry can overwrite the M5 zero command with "
        "ring-1 preparation",
    )

    service_body = extract_function_body(
        source_without_comments, "serviceCompetitionAction"
    )
    service = compact_cpp(service_body)
    arm_standard_service = extract_function_body(
        source_without_comments, "serviceArmStandardization"
    )
    raw_parallel_wait = compact_cpp(
        case_segment(
            arm_standard_service,
            "ARM_STANDARD_WAIT_RAW_PARALLEL",
        )
    )
    require_fragments(
        raw_parallel_wait,
        (
            "deadlineReached(armStandardDeadlineMs)",
            "extensionMoveFinished()",
            "liftMoveFinished()",
            "!armMotors.isM5Running()",
            "!armBaseMotionWatchdogActive",
            "armStandardPhase = ARM_STANDARD_COMPLETE",
        ),
        "RAW same-loop all-mechanism completion gate",
    )
    raw_prepare_case = compact_cpp(
        case_segment(service_body, "WORK_PHASE_PREPARE")
    )
    require_fragments(
        raw_prepare_case,
        (
            "if (activeWorkAction == WORK_ACTION_RAW)",
            "serviceArmStandardization()",
            "if (rawInitialStorageArrivalOverlapPending)",
            "beginRawItemVision()",
            "serviceStorageServoArrivalGate()",
            "resetStorageServoArrivalGate()",
        ),
        "round-specific RAW parallel-completion vision dispatch",
    )
    require(
        "rawInitialStorageArrivalOverlapPending="
        "rawCollectedCount==0U;"
        in begin_work[raw_branch_index:raw_branch_return_index],
        "initial tray/vision overlap is not enabled for both RAW rounds",
    )
    overlap_vision_index = raw_prepare_case.find(
        "beginRawItemVision();"
    )
    initial_arrival_gate_index = raw_prepare_case.find(
        "serviceStorageServoArrivalGate()", overlap_vision_index
    )
    round1_vision_index = raw_prepare_case.find(
        "beginRawItemVision();", overlap_vision_index + 1
    )
    initial_process_prepare_index = raw_prepare_case.find(
        "startPreEndpointHeadingCorrection();"
    )
    require(
        0 <= overlap_vision_index < initial_arrival_gate_index
        < round1_vision_index
        < initial_process_prepare_index,
        "RAW2 does not start vision before its independent tray gate, or "
        "RAW1 no longer waits for verified tray arrival",
    )
    require(
        "beginRawExpectedSlotPreparation();" not in raw_prepare_case,
        "RAW completion still commands a second redundant first-slot move",
    )

    expected_slot_wait_case = compact_cpp(
        case_segment(
            service_body, "WORK_PHASE_RAW_WAIT_EXPECTED_SLOT"
        )
    )
    require_fragments(
        expected_slot_wait_case,
        (
            "if (!serviceStorageServoArrivalGate())",
            "resetStorageServoArrivalGate()",
            "beginRawItemVision()",
        ),
        "RAW expected-slot settle gate",
    )
    slot_settled_index = expected_slot_wait_case.find(
        "if(!serviceStorageServoArrivalGate()){"
    )
    fresh_vision_index = expected_slot_wait_case.find(
        "beginRawItemVision();", slot_settled_index
    )
    require(
        0 <= slot_settled_index < fresh_vision_index,
        "RAW requests coordinates before the 200 ms tray command has "
        "two real-angle arrival confirmations",
    )
    require(
        "commandStorageServoPosition(" not in expected_slot_wait_case,
        "RAW reissues the tray command after entering its settle gate",
    )

    raw_wait_case = compact_cpp(
        case_segment(
            service_body, "WORK_PHASE_RAW_WAIT_RESULT"
        )
    )
    require_fragments(
        raw_wait_case,
        (
            "readNewMaixCoordinate("
            "workLastMaixSequence, detectedColor, x, y)",
            "expectedColor = "
            "taskColors[workRoundIndex][rawCollectedCount]",
            "if (detectedColor != expectedColor)",
            "slotIndex != rawCollectedCount",
            "ArmPose source = rawPendingSourcePose",
            "if (!rawPickupPrepositionedPending)",
            "rawTargetPose("
            "static_cast<float>(x), "
            "static_cast<float>(y), source)",
            "if (poseResult != RAW_TARGET_POSE_VALID)",
            "beginRawItemVision()",
            "beginRawPickupTransfer(detectedColor, slotIndex, source)",
        ),
        "strict RAW result handling",
    )
    require(
        raw_wait_case.count("readNewMaixCoordinate(") == 1,
        "STM32 RAW handling no longer consumes exactly one "
        "camera-confirmed coordinate",
    )
    reach_wait_index = raw_wait_case.find(
        "if(poseResult!=RAW_TARGET_POSE_VALID){"
    )
    reach_retry_index = raw_wait_case.find(
        "beginRawItemVision();", reach_wait_index
    )
    pickup_helper_index = raw_wait_case.find(
        "beginRawPickupTransfer(", reach_retry_index
    )
    require_fragments(
        raw_wait_case,
        (
            "rawInitialStorageArrivalOverlapPending",
            "!storageServoArrivalGate.confirmed",
            "!armMotors.isM5Running()",
            "extensionMoveFinished()",
            "liftMoveFinished()",
            "serviceStorageServoArrivalGate()",
        ),
        "RAW2 tray verification during the independent vision wait",
    )
    arm_transfer_service = extract_function_body(
        source_without_comments, "serviceArmTransfer"
    )
    destination_rotation_case = compact_cpp(
        case_segment(
            arm_transfer_service,
            "ARM_TRANSFER_WAIT_DESTINATION_ROTATION",
        )
    )
    destination_gate_index = destination_rotation_case.find(
        "serviceStorageServoArrivalGate()"
    )
    destination_descent_release_index = destination_rotation_case.find(
        "armTransferPhase=ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE;"
    )
    require(
        "armTransferUsesRawProfile()&&"
        "rawInitialStorageArrivalOverlapPending" in destination_rotation_case
        and 0 <= destination_gate_index < destination_descent_release_index
        and "resetStorageServoArrivalGate();" in destination_rotation_case,
        "RAW initial tray arrival is not enforced before destination descent",
    )
    require(
        0 <= reach_wait_index < reach_retry_index
        < pickup_helper_index,
        "an unreachable RAW target is not retried before the "
        "accepted-target path",
    )
    unreachable_branch = raw_wait_case[
        reach_wait_index:pickup_helper_index
    ]
    require(
        "routeFault(" not in unreachable_branch
        and "++rawCollectedCount;" not in unreachable_branch,
        "an unreachable RAW target faults or advances QR order",
    )

    raw_pickup_transfer = compact_cpp(
        extract_function_body(
            source_without_comments, "beginRawPickupTransfer"
        )
    )
    require_fragments(
        raw_pickup_transfer,
        (
            "stopMaixRequest()",
            "rawPendingColor = detectedColor",
            "rawPendingSlotIndex = slotIndex",
            "sourceAlreadyPrepared = false",
            "sourceAlreadyPrepared = rawPickupPrepositionedPending",
            "rawPickupPrepositionedPending = false",
            "if (!sourceAlreadyPrepared)",
            "rawPendingSourcePose = source",
            "prepareNextRawPickup = rawCollectedCount < 2U",
            "nextSearchPose = rawSearchPose(rawSearchZone)",
            "beginArmTransfer("
            "rawPendingSourcePose, arm_transfer::containerPlacePose()",
            "activeTransferPurpose = "
            "TRANSFER_PURPOSE_RAW_TO_CONTAINER",
            "workActionPhase = WORK_PHASE_WAIT_TRANSFER",
        ),
        "accepted RAW pickup transfer helper",
    )
    accepted_stop_index = raw_pickup_transfer.find(
        "stopMaixRequest();"
    )
    direct_transfer_index = raw_pickup_transfer.find(
        "beginArmTransfer(", accepted_stop_index
    )
    require(
        direct_transfer_index >= 0,
        "accepted RAW coordinates do not start an immediate transfer",
    )
    direct_transfer_open = raw_pickup_transfer.find(
        "(", direct_transfer_index
    )
    direct_transfer_close = find_matching(
        raw_pickup_transfer, direct_transfer_open, "(", ")"
    )
    direct_transfer_arguments = split_top_level_fields(
        raw_pickup_transfer[
            direct_transfer_open + 1:direct_transfer_close
        ]
    )
    require(
        len(direct_transfer_arguments) == 12
        and direct_transfer_arguments[0]
        == "rawPendingSourcePose"
        and direct_transfer_arguments[6]
        == "sourceAlreadyPrepared"
        and "GONGCHUANG_RAW_PICKUP_LOGIC_MODE==1"
        in direct_transfer_arguments[7]
        and "?&nextSearchPose" in direct_transfer_arguments[7]
        and "?&rawPendingSourcePose" in direct_transfer_arguments[7]
        and direct_transfer_arguments[9] == "false"
        and direct_transfer_arguments[-2:] == ["true", "true"],
        "RAW direct transfer must return to the active search pose in new "
        "mode, retain the old reuse pose in legacy mode, enable rawFastProfile "
        "and keep the gripper open",
    )
    accepted_to_transfer = raw_pickup_transfer[
        accepted_stop_index:direct_transfer_index
    ]
    require(
        "commandStorageServoPosition(" not in accepted_to_transfer
        and "workStorageServoDeadlineMs"
        not in accepted_to_transfer
        and "deadlineReached(" not in accepted_to_transfer
        and "WORK_PHASE_RAW_WAIT_EXPECTED_SLOT"
        not in accepted_to_transfer
        and "workActionPhase=" not in accepted_to_transfer,
        "accepted RAW coordinates still use a stale tray command "
        "or another deadline/phase gate instead of starting the "
        "grab immediately",
    )
    require(
        "WORK_PHASE_RAW_WAIT_STORAGE_POSITION"
        not in source_without_comments,
        "obsolete post-coordinate RAW tray-wait state remains",
    )

    arm_transfer = compact_cpp(
        extract_function_body(
            source_without_comments, "beginArmTransfer"
        )
    )
    require_fragments(
        source_without_comments,
        (
            "bool sourceGripperAlreadyOpen = false",
        ),
        "RAW transfer parameter",
    )
    require_fragments(
        arm_transfer,
        (
            "if (armTransferSourceAlreadyPrepared)",
            "retained M5/M6 source pose and open gripper",
            "startArmTransferLiftToSourceHeightMm("
            "armTransferSourcePose.heightMm)",
            "if (sourceGripperAlreadyOpen)",
            "startArmTransferLiftToHeightMm("
            "armTransferClearanceTargetMm("
            "armTransferMapSource))",
            "armTransferPhase = "
            "ARM_TRANSFER_WAIT_PREPARE_LIFT",
            "commandArmTransferGripperOpen()",
        ),
        "already-open RAW source transfer",
    )

    release_destination = compact_cpp(
        extract_function_body(
            source_without_comments, "releaseArmTransferDestination"
        )
    )
    require_fragments(
        release_destination,
        (
            "if (armTransferUsesRawProfile() && "
            "armTransferPrepareNextSource)",
            "armTransferStorageCommandDueMs = "
            "gripperOpenCommandMs + "
            "armTransferGripperOpenSettleMs() + "
            "RAW_NEXT_STORAGE_COMMAND_DELAY_MS",
        ),
        "RAW next-slot one-second post-release schedule",
    )
    deferred_storage_command = compact_cpp(
        extract_function_body(
            source_without_comments,
            "serviceArmTransferDeferredStorageCommand",
        )
    )
    require_fragments(
        deferred_storage_command,
        (
            "rawNextSlotDeferred = "
            "armTransferUsesRawProfile() && "
            "armTransferPrepareNextSource",
            "deadlineReached(armTransferStorageCommandDueMs)",
            "commandRawStorageServoPositionWithArrivalGate("
            "static_cast<uint8_t>(armTransferNextStorageSlot))",
            "rawNextSlotDeferred ? "
            "RAW_STORAGE_SERVO_SETTLE_MS : "
            "STORAGE_SERVO_SETTLE_MS",
        ),
        "RAW deferred next-slot command and real-arrival gate",
    )
    already_open_index = arm_transfer.find(
        "if(sourceGripperAlreadyOpen){"
    )
    already_open_brace = arm_transfer.find(
        "{", already_open_index
    )
    already_open_end = find_matching(
        arm_transfer, already_open_brace, "{", "}"
    )
    already_open_branch = arm_transfer[
        already_open_index:already_open_end + 1
    ]
    gripper_open_index = arm_transfer.find(
        "commandArmTransferGripperOpen();", already_open_end
    )
    require(
        "return;" in already_open_branch
        and gripper_open_index > already_open_end,
        "already-open RAW transfer can fall through to the "
        "ordinary source-open command",
    )
    require(
        "commandGripperOpen();" not in already_open_branch
        and "GRIPPER_OPEN_SETTLE_MS"
        not in already_open_branch
        and "armTransferDeadlineMs"
        not in already_open_branch,
        "already-open RAW transfer still repeats the ordinary "
        "gripper-open settle",
    )

    refresh_guard = (
        "if(workActionPhase==WORK_PHASE_RAW_WAIT_RESULT&&"
        "workVisionRequestStartMs!=0UL&&"
        "nowMs-workVisionRequestStartMs>="
        "RAW_TARGET_REQUEST_REFRESH_MS){"
    )
    refresh_index = service.find(refresh_guard)
    refresh_request_index = service.find(
        "beginRawItemVision();", refresh_index
    )
    refresh_return_index = service.find(
        "return;", refresh_request_index
    )
    require(
        0 <= refresh_index < refresh_request_index
        < refresh_return_index,
        "RAW 15-second wait does not refresh the same "
        "current-color request",
    )
    active_timeout = compact_cpp(
        extract_function_body(
            source_without_comments, "activeWorkActionTimeoutMs"
        )
    )
    require(
        "caseWORK_ACTION_RAW:returnworkActionPhase=="
        "WORK_PHASE_RAW_WAIT_RESULT?0UL:RAW_ACTION_TIMEOUT_MS;"
        in active_timeout,
        "RAW no-target wait is not indefinite while mechanical phases "
        "retain the 65-second timeout",
    )
    require(
        "workActionStartMs=millis();"
        in raw_pickup_transfer[accepted_stop_index:],
        "RAW mechanical timeout is not restarted after target acceptance",
    )
    watchdog = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceCompetitionWatchdogs"
        )
    )
    require(
        "activeWorkAction==WORK_ACTION_RAW&&"
        "workActionPhase==WORK_PHASE_RAW_WAIT_RESULT" in watchdog,
        "mission watchdog can still fault an intentional RAW no-target wait",
    )

    transfer_completion_body = extract_function_body(
        source_without_comments,
        "completeTransferAndRotateStorage",
    )
    raw_transfer_completion = compact_cpp(
        case_segment(
            transfer_completion_body,
            "TRANSFER_PURPOSE_RAW_TO_CONTAINER",
        )
    )
    require_fragments(
        raw_transfer_completion,
        (
            "preparedNextRawPickup = "
            "armTransferNextSourcePreparedAtEnd && "
            "armTransferNextStorageCommanded",
            "consumeArmTransferCompletion()",
            "rawFilledSlotMask = "
            "static_cast<uint8_t>("
            "rawFilledSlotMask | "
            "(1U << rawPendingSlotIndex))",
            "++rawCollectedCount",
            "if (rawCollectedCount >= 3U)",
            "beginRawTravelParkingAfterFinalStore()",
            "rawPickupPrepositionedPending = true",
            "workActionPhase = "
            "WORK_PHASE_RAW_WAIT_EXPECTED_SLOT",
        ),
        "successful RAW store progression",
    )
    consume_index = raw_transfer_completion.find(
        "consumeArmTransferCompletion();"
    )
    fill_index = raw_transfer_completion.find(
        "rawFilledSlotMask=static_cast<uint8_t>(",
        consume_index,
    )
    increment_index = raw_transfer_completion.find(
        "++rawCollectedCount;", fill_index
    )
    third_item_index = raw_transfer_completion.find(
        "if(rawCollectedCount>=3U){", increment_index
    )
    travel_park_index = raw_transfer_completion.find(
        "beginRawTravelParkingAfterFinalStore();",
        third_item_index,
    )
    require(
        0 <= consume_index < fill_index < increment_index
        < third_item_index < travel_park_index,
        "QR order advances before a successful container store "
        "or does not dispatch the third-item travel park",
    )
    require(
        source_without_comments.count("++rawCollectedCount;") == 1,
        "rawCollectedCount can advance outside successful "
        "RAW-to-container completion",
    )
    require(
        "beginRawItemVision();" not in raw_transfer_completion,
        "the next RAW color starts vision before its expected "
        "tray slot is prepared",
    )
    require(
        "beginRawExpectedSlotPreparation();"
        not in raw_transfer_completion,
        "RAW reissues the already-running next-slot command after "
        "M5/M6 reach the remembered pickup pose",
    )

    raw_travel_zero_start = compact_cpp(
        extract_function_body(
            source_without_comments,
            "beginRawTravelParkingAfterFinalStore",
        )
    )
    require_fragments(
        raw_travel_zero_start,
        (
            "transferClearanceReady = "
            "extensionMoveFinished() && liftMoveFinished()",
            "extensionAxis.currentMm - "
            "M6_STANDARD_EXTENSION_MM",
            "if (!transferClearanceReady)",
            "startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)",
            "workActionPhase = "
            "WORK_PHASE_RAW_WAIT_TRAVEL_LINEAR_ZERO",
        ),
        "third RAW item M7 travel-zero start",
    )
    require(
        "liftAxis.currentMm-M7_STANDARD_HEIGHT_MM"
        not in raw_travel_zero_start,
        "third RAW completion must accept the unified 0 mm M7 "
        "clearance before the idempotent final zero command",
    )
    clearance_check_index = raw_travel_zero_start.find(
        "if(!transferClearanceReady){"
    )
    m7_zero_command_index = raw_travel_zero_start.find(
        "startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)"
    )
    raw_zero_phase_index = raw_travel_zero_start.find(
        "workActionPhase="
        "WORK_PHASE_RAW_WAIT_TRAVEL_LINEAR_ZERO;",
        m7_zero_command_index,
    )
    require(
        0 <= clearance_check_index < m7_zero_command_index
        < raw_zero_phase_index,
        "third RAW completion does not command M7 zero before "
        "entering its arrival wait",
    )

    raw_travel = compact_cpp(
        extract_function_body(
            source_without_comments,
            "startRawTravelParkingAtSafeLinearZero",
        )
    )
    require_fragments(
        raw_travel,
        (
            "linearAxesSafe = "
            "extensionMoveFinished() && liftMoveFinished()",
            "extensionAxis.currentMm - "
            "M6_STANDARD_EXTENSION_MM",
            "liftAxis.currentMm - M7_STANDARD_HEIGHT_MM",
            "if (!linearAxesSafe)",
            "startArmBaseStandardFrameDegrees(0.0f)",
            "rawTravelM5ZeroPending = true",
            "RAW_EXIT_M5_MAXIMUM_STEP_RATE",
            "RAW_EXIT_M5_STEP_ACCELERATION",
            "commandStorageServoParkingPosition("
            "DEPARTURE_STORAGE_SERVO_INTERVAL_MS)",
            "finishActiveWorkAction()",
        ),
        "third RAW item M5/tray/chassis parallel departure handoff",
    )
    safe_check_index = raw_travel.find(
        "if(!linearAxesSafe){"
    )
    m5_zero_index = raw_travel.find(
        "startArmBaseStandardFrameDegrees(0.0f);"
    )
    pending_index = raw_travel.find(
        "rawTravelM5ZeroPending=true;", m5_zero_index
    )
    finish_raw_index = raw_travel.find(
        "finishActiveWorkAction();",
        pending_index,
    )
    require(
        0 <= safe_check_index < m5_zero_index < pending_index
        < finish_raw_index,
        "M5 standard zero is issued before M6/M7 safe-zero "
        "validation, is not preserved for PROCESS, or the chassis "
        "is not released afterward",
    )

    raw_travel_zero_wait = compact_cpp(
        case_segment(
            service_body,
            "WORK_PHASE_RAW_WAIT_TRAVEL_LINEAR_ZERO",
        )
    )
    require_fragments(
        raw_travel_zero_wait,
        (
            "if (!liftMoveFinished())",
            "startRawTravelParkingAtSafeLinearZero()",
        ),
        "third RAW item M7 zero-arrival gate",
    )
    require(
        raw_travel_zero_wait.find("liftMoveFinished()")
        < raw_travel_zero_wait.find(
            "startRawTravelParkingAtSafeLinearZero();"
        ),
        "RAW parking starts before M7 has reached the 0 mm "
        "working zero",
    )

    require(
        "WORK_PHASE_RAW_WAIT_TRAVEL_PARK"
        not in source_without_comments,
        "RAW still waits for M5/tray completion instead of releasing "
        "the chassis at M7 zero",
    )
    departure_stop_gate = compact_cpp(
        extract_function_body(
            source_without_comments,
            "workstationMechanismsStoppedForDeparture",
        )
    )
    process_overlap_gate = compact_cpp(
        extract_function_body(
            source_without_comments,
            "process2FinalStorageMayOverlapDeparture",
        )
    )
    require_fragments(
        process_overlap_gate,
        (
            "activeWorkAction == WORK_ACTION_PROCESS",
            "workRoundIndex == 1U",
            "workItemIndex >= 3U",
            "activeTransferPurpose == "
            "TRANSFER_PURPOSE_RING_TO_CONTAINER",
        ),
        "PROCESS2-only final tray/chassis overlap scope",
    )
    process2_prelock_begin = compact_cpp(
        extract_function_body(
            source_without_comments,
            "beginProcess2DepartureImuPrelock",
        )
    )
    require_fragments(
        process2_prelock_begin,
        (
            "resetProcess2DepartureImuPrelock()",
            "route_chassis::imuValidFrameSequence",
            "route_chassis::lastImuReceiveMs",
        ),
        "PROCESS2 departure IMU prelock start",
    )
    process2_prelock_service = compact_cpp(
        extract_function_body(
            source_without_comments,
            "serviceProcess2DepartureImuPrelock",
        )
    )
    require_fragments(
        process2_prelock_service,
        (
            "motorPulsesAreZero(visualCorrectionAccumulator)",
            "route_chassis::imuValidFrameSequence",
            "route_chassis::lastImuReceiveMs",
            "allMotorsArrived()",
            "imuIsFresh()",
            "WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS",
            "frameSequence == "
            "process2DepartureImuLastFrameSequence",
            "fabsf(headingErrorDegrees()) > "
            "WORKSTATION_HEADING_TOLERANCE_DEGREES",
            "WORK_DEPARTURE_POST_ARM_SETTLE_MS + "
            "WORK_DEPARTURE_IMU_STABLE_MS",
            "process2DepartureImuPrelockReady",
        ),
        "stationary 320 ms PROCESS2 IMU prelock observer",
    )
    require(
        "startHeadingCorrection(" not in process2_prelock_service
        and "startBodyDisplacement(" not in process2_prelock_service
        and "startRelativeMotorMove(" not in process2_prelock_service,
        "PROCESS2 IMU prelock observer must never move the chassis",
    )
    require_fragments(
        departure_stop_gate,
        (
            "!armMotors.isM5Running()",
            "!armBaseMotionWatchdogActive",
            "extensionMoveFinished()",
            "liftMoveFinished()",
            "extensionAxis.currentMm - M6_STANDARD_EXTENSION_MM",
            "activeWorkAction == WORK_ACTION_PROCESS",
            "arm_config::RING_ROTATION_CLEARANCE_HEIGHT_MM",
            "liftAxis.currentMm - requiredLiftHeightMm",
            "process2FinalStorageMayOverlapDeparture()",
            "storageServoReadyForDeparture",
            "deadlineReached(workStorageServoDeadlineMs)",
        ),
        "all-mechanism workstation departure stop gate",
    )
    departure_service = compact_cpp(
        extract_function_body(
            source_without_comments,
            "serviceWorkstationDepartureLock",
        )
    )
    require_fragments(
        departure_service,
        (
            "WORK_DEPARTURE_MAX_WAIT_MS",
            "allMotorsArrived()",
            "WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS",
            "route_chassis::lastImuReceiveMs",
            "updateHeadingLock(WORK_DEPARTURE_MAX_WAIT_MS,"
            "WORK_DEPARTURE_POST_ARM_SETTLE_MS,"
            "WORK_DEPARTURE_IMU_STABLE_MS)",
        ),
        "120+200 ms adaptive workstation departure IMU lock",
    )
    require(
        "WORK_DEPARTURE_MAX_WAIT_MS=1000UL;"
        in compact_cpp(source_without_comments),
        "workstation departure IMU fallback must remain 1000 ms",
    )
    departure_case = compact_cpp(
        case_segment(
            service_body, "WORK_PHASE_WAIT_DEPARTURE_LOCK"
        )
    )
    require_fragments(
        departure_case,
        (
            "serviceWorkstationDepartureLock()",
            "finishActiveWorkAction()",
        ),
        "departure lock route-release gate",
    )
    finish_action = extract_function_body(
        source_without_comments, "finishActiveWorkAction"
    )
    require(
        "rawTravelM5ZeroPending" not in finish_action,
        "RAW completion clears the pending travel-zero handoff "
        "before PROCESS can confirm it",
    )

    zero_wait_case = compact_cpp(
        case_segment(
            service_body,
            "WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO",
        )
    )
    require_fragments(
        zero_wait_case,
        (
            "armMotors.isM5Running() || "
            "armBaseMotionWatchdogActive",
            "processM5ZeroSettleDeadlineUs = "
            "micros() + PROCESS_ENTRY_M5_SETTLE_US",
            "workActionPhase = "
            "WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO_SETTLE",
        ),
        "PROCESS M5 pulse-arrival gate",
    )
    require(
        "beginArmEndpointPreparation();" not in zero_wait_case,
        "ring-1 preparation starts before the M5 0.1 ms handoff gate",
    )

    zero_settle_case = compact_cpp(
        case_segment(
            service_body,
            "WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO_SETTLE",
        )
    )
    require_fragments(
        zero_settle_case,
        (
            "micros() - processM5ZeroSettleDeadlineUs",
            "armMotors.isM5Running() || "
            "armBaseMotionWatchdogActive",
            "rawTravelM5ZeroPending = false",
            "processM5ZeroSettleDeadlineUs = 0UL",
            "beginArmEndpointPreparation()",
            "workActionPhase = WORK_PHASE_PREPARE",
        ),
        "PROCESS M5 0.1 ms handoff gate",
    )
    settle_guard_index = zero_settle_case.find(
        "if(static_cast<int32_t>(micros()-"
        "processM5ZeroSettleDeadlineUs)<0||"
        "armMotors.isM5Running()||"
        "armBaseMotionWatchdogActive){"
    )
    clear_pending_index = zero_settle_case.find(
        "rawTravelM5ZeroPending=false;", settle_guard_index
    )
    endpoint_prepare_index = zero_settle_case.find(
        "beginArmEndpointPreparation();", clear_pending_index
    )
    require(
        0 <= settle_guard_index < clear_pending_index
        < endpoint_prepare_index,
        "PROCESS starts ring 1 before M5 arrival plus 0.1 ms handoff",
    )

    cancel_action = compact_cpp(
        extract_function_body(
            source_without_comments, "cancelCompetitionAction"
        )
    )
    require(
        "rawTravelM5ZeroPending=false;" in cancel_action,
        "a direct/calibration PROCESS run can inherit a stale "
        "travel-zero flag",
    )
    begin_route = compact_cpp(
        extract_function_body(source_without_comments, "beginRoute")
    )
    cancel_index = begin_route.find(
        "cancelCompetitionAction();"
    )
    calibration_process_index = begin_route.find(
        "beginWorkAction(WORK_ACTION_PROCESS,1U);"
    )
    require(
        0 <= cancel_index < calibration_process_index,
        "mode 1 does not clear the handoff state before entering "
        "the same PROCESS standard-zero gate",
    )

def verify_run_mode_contract(
    source_without_comments: str,
    config_without_comments: str,
) -> None:
    run_mode_defines = re.findall(
        r"^\s*#\s*define\s+GONGCHUANG_RUN_MODE\s+([^\s]+)\s*$",
        config_without_comments,
        flags=re.MULTILINE,
    )
    require(
        len(run_mode_defines) == 1 and
        run_mode_defines[0] == "0",
        "RobotConfig.h must select complete-flow mode 0; "
        f"found {run_mode_defines}",
    )
    default_mode = run_mode_defines[0]
    require(
        re.search(
            r"#\s*ifndef\s+GONGCHUANG_RUN_MODE\s*"
            r"#\s*define\s+GONGCHUANG_RUN_MODE\s+" +
            re.escape(default_mode) +
            r"\s*"
            r"#\s*endif",
            config_without_comments,
        )
        is not None,
        "GONGCHUANG_RUN_MODE must be guarded in RobotConfig.h",
    )
    compact_config = compact_cpp(config_without_comments)
    require(
        "static_assert(GONGCHUANG_RUN_MODE==0||"
        "GONGCHUANG_RUN_MODE==1||"
        "GONGCHUANG_RUN_MODE==2,"
        '"GONGCHUANG_RUN_MODEmustbe0,1or2");'
        in compact_config,
        "GONGCHUANG_RUN_MODE is not statically restricted to 0/1/2",
    )
    compact_source = compact_cpp(source_without_comments)
    require(
        "constexprboolROUGH_PROCESSING_CALIBRATION_MODE="
        "GONGCHUANG_RUN_MODE==1;" in compact_source,
        "run mode 1 is not mapped to rough-processing calibration",
    )
    require(
        "constexprboolSTORAGE_ROUND2_SIMULATION_MODE="
        "GONGCHUANG_RUN_MODE==2;" in compact_source
        and "constexprboolLOCAL_WORKSTATION_SIMULATION_MODE="
        "RAW_WORKSTATION_SIMULATION_MODE||"
        "ROUGH_PROCESSING_CALIBRATION_MODE||"
        "STORAGE_ROUND2_SIMULATION_MODE||"
        "STORAGE_ROUND1_SIMULATION_MODE;" in compact_source,
        "local workstation run-mode guard is incomplete",
    )

    forbidden_fake_completion_symbols = (
        "PATH_ONLY_TEST",
        "WORKSTATION_TEST_HOLD_MS",
        "timedActionFinished",
        "finishRawActionIfNeeded",
        "finishProcessActionIfNeeded",
        "finishStorageActionIfNeeded",
        "GONGCHUANG_VISION_YANYAN_TEST",
        "VISION_YANYAN_TEST_MODE",
        "configureVisionYanyanTask",
        "visionYanyan",
    )
    for symbol in forbidden_fake_completion_symbols:
        require(
            symbol not in source_without_comments,
            f"source still contains obsolete/fake test entry {symbol}",
        )

    forbidden_qr_bypasses = (
        "ENABLE_QR_RECEIVER",
        "REQUIRE_QR_SUCCESS",
    )
    for symbol in forbidden_qr_bypasses:
        require(
            symbol not in source_without_comments,
            f"production source still contains QR bypass {symbol}",
        )

    forbidden_hard_time_limit_symbols = (
        "ENABLE_COMPETITION_TIME_LIMIT",
        "COMPETITION_TIME_LIMIT_MS",
        "COMPETITION_HARD_STOP_MARGIN_MS",
        "Competition hard time limit",
    )
    for symbol in forbidden_hard_time_limit_symbols:
        require(
            symbol not in source_without_comments,
            f"removed 180-second hard limit still contains {symbol}",
        )
    watchdog = extract_function_body(
        source_without_comments, "serviceCompetitionWatchdogs"
    )
    require_fragments(
        watchdog,
        (
            "routeWaitingForQrCode ||",
            "if (nowMs - lastMissionProgressMs >= "
            "MISSION_PROGRESS_TIMEOUT_MS)",
            'routeFault("Mission progress watchdog timeout")',
        ),
        "fault-only mission progress watchdog",
    )

    require(
        PLATFORMIO_PATH.is_file(),
        f"PlatformIO configuration does not exist: {PLATFORMIO_PATH}",
    )
    platformio_source = PLATFORMIO_PATH.read_text(encoding="utf-8")
    environments = re.findall(
        r"^\s*\[env:([^\]]+)\]\s*$",
        platformio_source,
        flags=re.MULTILINE,
    )
    require(
        environments == ["genericSTM32H750VB", "debug_qr_continuous", "debug_workstation", "debug_raw_workstation", "debug_raw_workstation_341", "debug_storage2_workstation", "debug_translation", "debug_chassis_route", "debug_maix_mode8", "debug_maix_protocol_cycle"],
        "platformio.ini must expose production plus the nine isolated debug environments; "
        f"found {environments}",
    )
    require(
        re.search(
            r"^\s*default_envs\s*=\s*genericSTM32H750VB\s*$",
            platformio_source,
            flags=re.MULTILINE,
        )
        is not None,
        "the default PlatformIO target must be the production environment",
    )
    require(
        "visionyanyan" not in platformio_source.lower()
        and "GONGCHUANG_VISION_YANYAN_TEST" not in platformio_source
        and "GONGCHUANG_RUN_MODE" not in platformio_source,
        "platformio.ini must not add a second run-mode/test switch",
    )

    debug_workstation_source = (
        ROOT / "src" / "debug_workstation.cpp"
    ).read_text(encoding="utf-8")
    debug_definitions = set(re.findall(
        r"^\s*#define\s+(\w+)", debug_workstation_source, re.MULTILINE
    ))
    require(
        debug_definitions == {
            "GONGCHUANG_DEBUG_WORKSTATION_STATE",
            "GONGCHUANG_RUN_MODE",
            "GONGCHUANG_DEBUG_STORAGE1",
            "GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG",
        },
        "workstation wrapper may select its station and logging only; "
        "motion, gripper, tray, vision and correction parameters must be inherited",
    )
    require(
        debug_workstation_source.find("#if GONGCHUANG_DEBUG_WORKSTATION_STATE == 1")
        < debug_workstation_source.find('#include "main.cpp"'),
        "all three debug states must enter the shared production main.cpp",
    )
    require(
        "#define GONGCHUANG_RING3_EXTENSION_CORRECTION_MM 5.0f"
        in source_without_comments,
        "production main must own the merged ring-3 +5 mm correction",
    )
    require(
        "GONGCHUANG_DEBUG_RING3_EXTENSION_CORRECTION_MM" in source_without_comments,
        "main may retain the optional legacy ring-3 override interface",
    )
    debug_ring3_adjustment = compact_cpp(
        extract_function_body(
            source_without_comments,
            "applyRing3ExtensionCorrection",
        )
    )
    require_fragments(
        debug_ring3_adjustment,
        (
            "if(ringPosition!=3U||"
            "RING3_EXTENSION_CORRECTION_MM<=0.0f){returntrue;}",
            "correctedExtensionMm=baseExtensionMm+"
            "RING3_EXTENSION_CORRECTION_MM;",
            "correctedExtensionMm>M6_MAXIMUM_EXTENSION_MM+"
            "ARM_AXIS_POSITION_TOLERANCE_MM",
            "pose.extensionMm=correctedExtensionMm;",
        ),
        "production ring-3 placement extension",
    )

    unloading = compact_cpp(
        extract_function_body(source_without_comments, "beginUnloadingTransfer")
    )
    require(
        unloading.find("applyTargetPlacementExtraLower(")
        < unloading.find("applyRingPlacementPoseCorrection(")
        < unloading.find("activePlacementRingPosition=ringPosition;"),
        "the single placement net correction must precede transfer",
    )
    require(
        unloading.count("applyRingPlacementPoseCorrection(") == 1
        and unloading.count("applyRingPositionPoseCorrection(") == 1,
        "placement net and first-return net must use their dedicated helpers",
    )
    require(
        "processOrStorage1RingNetCorrectionApplies(ringPosition)?0.0f:"
        in unloading,
        "PROCESS/STORAGE1 must suppress the legacy 6/6/9 mm reduction before the net table",
    )
    reloading = compact_cpp(
        extract_function_body(source_without_comments, "beginReloadingTransfer")
    )
    require(
        reloading.count("applyRingPositionPoseCorrection(") == 2,
        "current and next ring return pickups must use the same correction helper",
    )
    transfer_completion = compact_cpp(
        extract_function_body(
            source_without_comments,
            "completeTransferAndRotateStorage",
        )
    )
    require_fragments(
        transfer_completion,
        (
            "measuredRingPoses[activePlacementRingPosition]="
            "armTransferDestinationPose;",
            "removeRingPositionPoseCorrectionForSavedBase("
            "measuredRingPoses[activePlacementRingPosition],"
            "activePlacementRingPosition);",
        ),
        "normalized ring base pose prevents correction accumulation",
    )

    process_storage1_net_gate = compact_cpp(
        extract_function_body(
            source_without_comments,
            "processOrStorage1RingNetCorrectionApplies",
        )
    )
    require_fragments(
        process_storage1_net_gate,
        (
            "ringPosition>=1U&&ringPosition<=3U",
            "activeWorkAction==WORK_ACTION_PROCESS",
            "activeWorkAction==WORK_ACTION_STORAGE&&workRoundIndex==0U",
        ),
        "PROCESS/STORAGE1 shared net-correction gate",
    )
    workstation_net_gate = compact_cpp(
        extract_function_body(
            source_without_comments,
            "workstationRingNetCorrectionApplies",
        )
    )
    require_fragments(
        workstation_net_gate,
        (
            "processOrStorage1RingNetCorrectionApplies(ringPosition)",
            "ringPosition>=1U&&ringPosition<=3U",
            "secondStorageColorWorkflowActive()",
        ),
        "all-workstation net-correction gate",
    )
    process_storage1_return_adjustment = compact_cpp(
        extract_function_body(
            source_without_comments,
            "applyRingPositionPoseCorrection",
        )
    )
    require_fragments(
        process_storage1_return_adjustment,
        (
            "pose.standardFrameAngleDegrees+="
            "WORKSTATION_RING_RETURN_M5_NET_CCW_DEGREES[ringPosition-1U]",
            "pose.extensionMm+="
            "WORKSTATION_RING_RETURN_M6_NET_EXTENSION_MM[ringPosition-1U]",
            "secondStorageColorWorkflowActive()",
            "applyRing3ExtensionCorrection(pose,ringPosition)",
        ),
        "all workstation returns use one shared net while STORAGE2 keeps ring3",
    )
    process_storage1_place_adjustment = compact_cpp(
        extract_function_body(
            source_without_comments,
            "applyRingPlacementPoseCorrection",
        )
    )
    require_fragments(
        process_storage1_place_adjustment,
        (
            "pose.standardFrameAngleDegrees+="
            "PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[ringPosition-1U]",
            "STORAGE2_OUTER_RING_PLACE_M5_DIFFERENCE_CCW_DEGREES",
            "pose.extensionMm+="
            "WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[ringPosition-1U]",
            "secondStorageColorWorkflowActive()",
            "STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE",
            "STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM",
        ),
        "all workstation placements use one shared net and STORAGE2 keeps its scale",
    )
    require(
        process_storage1_place_adjustment.find(
            "STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE"
        )
        < process_storage1_place_adjustment.find(
            "WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM"
        ),
        "STORAGE2 must apply the shared millimetre net after its 0.90 scale",
    )
    require(
        process_storage1_place_adjustment.find(
            "WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM"
        )
        < process_storage1_place_adjustment.find(
            "STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM"
        ),
        "STORAGE2 outer +5 mm must be the final direct placement offset",
    )
    require_fragments(
        compact_cpp(source_without_comments),
        (
            "PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[3]={1.92f,2.75f,1.62f};",
            "STORAGE2_OUTER_RING_PLACE_M5_DIFFERENCE_CCW_DEGREES=-0.50f;",
            "WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[3]={-17.5f,-13.5f,-17.0f};",
            "STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES=0.20f;",
            "STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM=1.0f;",
            "WORKSTATION_RING_RETURN_M5_NET_CCW_DEGREES[3]={2.0f,2.5f,2.0f};",
            "WORKSTATION_RING_RETURN_M6_NET_EXTENSION_MM[3]={-17.0f,-12.5f,-16.0f};",
        ),
        "scoped PROCESS/STORAGE1 and STORAGE2 placement net values",
    )
    storage1_ring1_extra_gate = compact_cpp(
        extract_function_body(
            source_without_comments,
            "storage1Ring1PlacementExtraCorrectionApplies",
        )
    )
    require_fragments(
        storage1_ring1_extra_gate,
        (
            "activeWorkAction==WORK_ACTION_STORAGE",
            "workRoundIndex==0U",
            "ringPosition==1U",
        ),
        "STORAGE1 ring-1 placement-only correction gate",
    )
    require_fragments(
        process_storage1_place_adjustment,
        (
            "storage1Ring1PlacementExtraCorrectionApplies(ringPosition)",
            "pose.standardFrameAngleDegrees-="
            "STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES;",
            "pose.extensionMm+="
            "STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM;",
        ),
        "STORAGE1 ring-1 extra placement correction application",
    )
    m5_profile_setter = compact_cpp(
        extract_function_body(
            source_without_comments,
            "setArmBaseMotionProfileForPayload",
        )
    )
    require(
        "M5_GLOBAL_RAMP_TIME_SCALE=1.15f;"
        in compact_cpp(source_without_comments)
        and "M5_GLOBAL_ACCELERATION_SCALE="
        "1.0f/M5_GLOBAL_RAMP_TIME_SCALE;"
        in compact_cpp(source_without_comments)
        and m5_profile_setter.count(
            "stepAcceleration*=M5_GLOBAL_ACCELERATION_SCALE;"
        ) == 1
        and m5_profile_setter.find(
            "stepAcceleration*=EMPTY_GRIPPER_ACCELERATION_SCALE;"
        )
        < m5_profile_setter.find(
            "stepAcceleration*=M5_GLOBAL_ACCELERATION_SCALE;"
        )
        < m5_profile_setter.find(
            "armMotors.setM5MotionProfile("
        ),
        "global M5 ramp-time scale is missing, duplicated, or applied "
        "before the empty-gripper profile",
    )
    require(
        all(
            legacy not in compact_cpp(source_without_comments)
            for legacy in (
                "RING12_VISUAL_M5_COUNTERCLOCKWISE_CORRECTION_DEGREES",
                "RING12_VISUAL_M6_EXTENSION_CORRECTION_MM",
                "RING1_PLACEMENT_ADDITIONAL_M5_CLOCKWISE_CORRECTION_DEGREES",
                "RING1_PLACEMENT_ADDITIONAL_M6_EXTENSION_CORRECTION_MM",
            )
        ),
        "legacy PROCESS/STORAGE1 correction pieces still exist beside the net tables",
    )
    saved_pose_normalization = compact_cpp(
        extract_function_body(
            source_without_comments,
            "removeRingPositionPoseCorrectionForSavedBase",
        )
    )
    require_fragments(
        saved_pose_normalization,
        (
            "pose.standardFrameAngleDegrees-="
            "PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[ringPosition-1U];",
            "pose.standardFrameAngleDegrees-="
            "STORAGE2_OUTER_RING_PLACE_M5_DIFFERENCE_CCW_DEGREES;",
            "pose.extensionMm-="
            "WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[ringPosition-1U];",
            "pose.standardFrameAngleDegrees+="
            "STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES;",
            "pose.extensionMm-="
            "STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM;",
            "STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM;",
            "processOrStorage1RingNetCorrectionApplies(ringPosition)",
            "pose.extensionMm-=RING3_EXTENSION_CORRECTION_MM;",
        ),
        "shared net removal and STORAGE2 ring-3 normalization",
    )

    require(
        "GONGCHUANG_DEBUG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT"
        not in debug_definitions,
        "state1 must inherit the formal return cushion instead of freezing a copy",
    )
    require(
        "#define GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT 150U"
        in source_without_comments,
        "production main must own the ring-1 150-percent return cushion",
    )
    require(
        "GONGCHUANG_DEBUG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT"
        in source_without_comments,
        "main may retain the optional legacy return-cushion override interface",
    )
    require_fragments(
        compact_cpp(source_without_comments),
        (
            "RING1_LOADED_RETURN_M5_STEP_ACCELERATION="
            "arm_config::M5_LOADED_RETURN_STEP_ACCELERATION*100.0f/"
            "static_cast<float>(RING1_LOADED_RETURN_RAMP_TIME_PERCENT);",
            "RING1_LOADED_RETURN_M6_ACCELERATION_DISTANCE="
            "((EMM_ACCELERATION_PARAMETER_LIMIT-"
            "static_cast<uint16_t>(arm_config::M6_LOADED_RETURN_ACCELERATION))*"
            "RING1_LOADED_RETURN_RAMP_TIME_PERCENT+50U)/100U;",
            "RING1_LOADED_RETURN_M5_STEP_ACCELERATION==48000.0f",
            "RING1_LOADED_RETURN_M6_ACCELERATION==184U",
        ),
        "production ring-1 x1.5 M5/M6 ramp conversion",
    )
    ring1_return_cushion = compact_cpp(
        extract_function_body(
            source_without_comments,
            "armTransferUsesRing1LoadedReturnCushion",
        )
    )
    require_fragments(
        ring1_return_cushion,
        (
            "armTransferUsesReturnProfile()",
            "armTransferSourceRingPosition==1U",
            "RING1_LOADED_RETURN_RAMP_TIME_PERCENT>100U",
        ),
        "PROCESS ring-return-only ring-1 loaded-return cushion gate",
    )
    require(
        "ROUGH_PROCESSING_CALIBRATION_MODE" not in ring1_return_cushion,
        "ring-1 cushion must also run in the formal PROCESS workflow",
    )
    destination_extension = compact_cpp(
        extract_function_body(
            source_without_comments,
            "startArmTransferDestinationExtensionToMm",
        )
    )
    require_fragments(
        destination_extension,
        (
            "loadedTrayMotion&&armTransferUsesRing1LoadedReturnCushion()",
            "?RING1_LOADED_RETURN_M6_ACCELERATION",
            "?TRAY_LOAD_RAW_M6_ACCELERATION:RAW_M6_ACCELERATION",
            "?TRAY_LOAD_RETURN_M6_ACCELERATION:RETURN_M6_ACCELERATION",
            "arm_config::M6_LOADED_RETURN_SPEED_RPM",
        ),
        "tray-load M6 cushion profiles with ring-1 override",
    )
    planar_move = compact_cpp(
        extract_function_body(
            source_without_comments,
            "startArmTransferPlanarMove",
        )
    )
    require_fragments(
        planar_move,
        (
            "ring1LoadedReturnCushion=loadedTrayMotion&&"
            "armTransferUsesRing1LoadedReturnCushion();",
            "pose.standardFrameAngleDegrees<"
            "armTransferSourcePose.standardFrameAngleDegrees-"
            "FINAL_RETURN_M5_TOLERANCE_DEGREES",
            "pose.extensionMm<extensionAxis.currentMm-"
            "ARM_AXIS_POSITION_TOLERANCE_MM",
            "useArmBaseRing1LoadedReturnCushionMotionProfile();",
            "loadedTrayMotion&&armTransferUsesRawProfile()",
            "useArmBaseRawTrayLoadCushionMotionProfile();",
            "loadedTrayMotion&&armTransferUsesReturnProfile()",
            "useArmBaseTrayLoadReturnCushionMotionProfile();",
            "[RING1 LOADED RETURN CUSHION]",
            "[TRAY LOAD CUSHION]",
        ),
        "all tray-load cushions plus ring-1 clockwise/retract override",
    )
    require_fragments(
        compact_cpp(source_without_comments),
        (
            "TRAY_LOAD_RAMP_TIME_NUMERATOR=3U;",
            "TRAY_LOAD_RAMP_TIME_DENOMINATOR=2U;",
            "TRAY_LOAD_RETURN_M5_STEP_ACCELERATION==48000.0f",
            "TRAY_LOAD_RAW_M5_STEP_ACCELERATION==60000.0f",
            "TRAY_LOAD_RETURN_M6_ACCELERATION==184U",
            "TRAY_LOAD_RAW_M6_ACCELERATION==179U",
            "armTransferUsesRawProfile()||armTransferUsesReturnProfile()",
        ),
        "x1.5 loaded-to-tray ramp conversion and destination gate",
    )
    require(
        "workItemIndex==0U,false,ringPosition);" in reloading,
        "ring return must latch the actual source ring for the shared cushion gate",
    )
    require(
        compact_cpp(source_without_comments).count(
            "armTransferSourceRingPosition=0U;"
        )
        >= 3,
        "ring-source cushion latch must reset at transfer, action, and cancel boundaries",
    )

    setup = compact_cpp(
        extract_function_body(source_without_comments, "setup")
    )
    split_route_chassis = "route_chassis::setup();" in setup
    if split_route_chassis:
        require(
            ROUTE_CHASSIS_SOURCE_PATH.is_file(),
            "route chassis include is missing",
        )
        route_chassis_without_comments = strip_cpp_comments(
            ROUTE_CHASSIS_SOURCE_PATH.read_text(encoding="utf-8")
        )
        route_setup = compact_cpp(
            extract_function_body(
                route_chassis_without_comments, "setup"
            )
        )
        require_fragments(
            route_setup,
            (
                "SerialQr.begin(QR_BAUDRATE)",
                "resetQrReceiver()",
            ),
            "route chassis unconditional QR startup",
        )
        require(
            route_setup.find("SerialQr.begin(QR_BAUDRATE);")
            < route_setup.find("resetQrReceiver();"),
            "route chassis QR reset occurs before serial startup",
        )
        require_fragments(
            setup,
            (
                "route_chassis::setup()",
                "initializeManipulationHardware()",
                "resetQrReceiver()",
            ),
            "split chassis/manipulator setup",
        )
        require(
            setup.find("route_chassis::setup();")
            < setup.find("initializeManipulationHardware();")
            < setup.find("resetQrReceiver();")
            < setup.find("if(!armWorkingZerosReady){"),
            "split setup no longer initializes chassis before the "
            "manipulator or unconditionally resets the GC QR parser",
        )
    else:
        require_fragments(
            setup,
            (
                "SerialQr.begin(QR_BAUDRATE)",
                "resetQrReceiver()",
            ),
            "unconditional QR startup",
        )
        require(
            setup.find("SerialQr.begin(QR_BAUDRATE);")
            < setup.find("resetQrReceiver();")
            < setup.find("if(!armWorkingZerosReady){"),
            "QR serial/reset is no longer unconditional during setup",
        )

    begin_route = extract_function_body(
        source_without_comments, "beginRoute"
    )
    require_fragments(
        begin_route,
        (
            "resetQrReceiver()",
            'hmiSetText("t1", "QRWAIT")',
        ),
        "unconditional QR reset at mission start",
    )
    if split_route_chassis:
        require_fragments(
            begin_route,
            (
                "route_chassis::beginRoute()",
                "route_chassis::routeImuReferenceDegrees",
                "route_chassis::targetCounterClockwiseHeadingDegrees",
            ),
            "split route startup bridge",
        )

    loop_body = compact_cpp(
        extract_function_body(source_without_comments, "loop")
    )
    qr_receive_index = loop_body.find("receiveQrData();")
    first_runtime_branch = (
        loop_body.find("if(route_chassis::abortRequested")
        if split_route_chassis
        else loop_body.find("if(abortRequested){")
    )
    require(
        0 <= qr_receive_index < first_runtime_branch,
        "receiveQrData() must run unconditionally before loop branches",
    )
    if split_route_chassis:
        require(
            qr_receive_index
            < loop_body.find("synchronizeTaskCodeToRouteChassis();")
            < first_runtime_branch,
            "GC QR decoding must feed the route scanner before runtime "
            "branches",
        )
        require(
            source_without_comments.count("resetQrReceiver();") == 2
            and source_without_comments.count("receiveQrData();") == 1
            and source_without_comments.count(
                "route_chassis::setup();"
            ) == 1
            and route_chassis_without_comments.count(
                "SerialQr.begin(QR_BAUDRATE);"
            ) == 1,
            "split QR receiver startup/reset/service call counts changed",
        )
    else:
        require(
            source_without_comments.count("resetQrReceiver();") == 2
            and source_without_comments.count("receiveQrData();") == 1
            and source_without_comments.count(
                "SerialQr.begin(QR_BAUDRATE);"
            )
            == 1,
            "QR receiver startup/reset/service call counts changed",
        )
    require(
        '"BYPASS"' not in source_without_comments,
        "production HMI still exposes a QR BYPASS state",
    )

def verify_calibration_mode_contract(
    source_without_comments: str,
) -> None:
    calibration_task = extract_function_body(
        source_without_comments,
        "configureRoughProcessingCalibrationTask",
    )
    require_fragments(
        calibration_task,
        (
            'CALIBRATION_TASK_CODE[] = "123+132+123+132"',
            "competition::parseTaskCode("
            "CALIBRATION_TASK_CODE, calibrationPlan)",
            "taskPlan = calibrationPlan",
            "memcpy(qrData, CALIBRATION_TASK_CODE, "
            "sizeof(CALIBRATION_TASK_CODE))",
            "scanFlag = true",
            "taskCodeDecoded = true",
        ),
        "rough-processing calibration task",
    )

    begin_route = compact_cpp(
        extract_function_body(source_without_comments, "beginRoute")
    )
    require_fragments(
        begin_route,
        (
            "if (ROUGH_PROCESSING_CALIBRATION_MODE)",
            "configureRoughProcessingCalibrationTask()",
            'hmiSetRunStatus("CALRUN")',
            "beginWorkAction(WORK_ACTION_PROCESS, 1U)",
        ),
        "calibration direct PROCESS1 entry",
    )
    direct_process_index = begin_route.find(
        "beginWorkAction(WORK_ACTION_PROCESS,1U);"
    )
    zero_wheel_positions_index = begin_route.find(
        "for(uint8_ti=0;i<4;++i){"
        "motors[i]->setCurrentPosition(0);}"
    )
    enable_wheels_index = begin_route.find(
        "enableDriveMotors();"
    )
    running_index = begin_route.find(
        "programState=PROGRAM_RUNNING;"
    )
    calibration_branch_index = begin_route.rfind(
        "if(ROUGH_PROCESSING_CALIBRATION_MODE){",
        0,
        direct_process_index,
    )
    lock_wheels_index = begin_route.find(
        "stopAllMotorsImmediately();",
        calibration_branch_index,
    )
    return_after_direct_process = begin_route.find(
        "return;", direct_process_index
    )
    formal_route_message = begin_route.find(
        "route_chassis::beginRoute();"
    )
    if formal_route_message < 0:
        formal_route_message = begin_route.find(
            'SerialDebug.println("21-steprelativeroute+'
            'workstationactionsstarted");'
        )
    require(
        0 <= direct_process_index < return_after_direct_process,
        "calibration mode does not return immediately after PROCESS1 start",
    )
    require(
        0 <= return_after_direct_process < formal_route_message,
        "calibration mode can fall through into the full route entry",
    )
    require(
        0 <= zero_wheel_positions_index < enable_wheels_index
        < running_index < lock_wheels_index < direct_process_index
        < return_after_direct_process,
        "calibration mode does not enable and lock all four wheels "
        "before starting PROCESS1",
    )
    require(
        0 <= calibration_branch_index < zero_wheel_positions_index,
        "calibration wheel-lock sequence escaped its mode guard",
    )
    require(
        "disableDriveMotors();"
        not in begin_route[
            calibration_branch_index:return_after_direct_process
        ],
        "calibration mode releases the four-wheel holding torque "
        "while PROCESS1 is running",
    )

    require("route_chassis::beginRoute();" in begin_route,
            "complete-flow mode no longer starts active chassis route")
    updater = compact_cpp(extract_function_body(
        source_without_comments, "serviceIntegratedRoute"
    ))
    require(0 <= updater.find("if(LOCAL_WORKSTATION_SIMULATION_MODE||"
                              "programState!=PROGRAM_RUNNING){return;}")
            < updater.find("route_chassis::updateRoute();"),
            "local simulation can advance production chassis route")

    transfer_completion = compact_cpp(
        extract_function_body(
            source_without_comments,
            "completeTransferAndRotateStorage",
        )
    )
    require_fragments(
        transfer_completion,
        (
            "case TRANSFER_PURPOSE_CONTAINER_TO_RING:",
            "measuredRingPoses[activePlacementRingPosition] = "
            "armTransferDestinationPose",
            "activePlacementRingPosition = 0U",
            "++correctPlacementCount; break",
            "case TRANSFER_PURPOSE_RING_TO_CONTAINER: "
            "++correctGrabCount; break",
            "consumeArmTransferCompletion()",
            "++workItemIndex",
        ),
        "real placement/pickup completion accounting",
    )
    require_fragments(
        transfer_completion,
        (
            "finishedFinalContainerSequence = workItemIndex >= 3U",
            "processRound1TravelOwnsTrayCommand",
            "storageRound1RawTravelOwnsTrayCommand",
            "if (finishedFinalContainerSequence)",
            "if (!processRound1TravelOwnsTrayCommand && "
            "!storageRound1RawTravelOwnsTrayCommand)",
            "commandStorageServoParkingPosition("
            "activeWorkAction == WORK_ACTION_PROCESS ? "
            "DEPARTURE_STORAGE_SERVO_INTERVAL_MS : "
            "STORAGE_SERVO_INTERVAL_MS)",
            "startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)",
            "DEPARTURE_STORAGE_SERVO_SETTLE_MS",
            "workActionPhase = WORK_PHASE_WAIT_STORAGE_SERVO",
        ),
        "final slow tray motion with concurrent M7 zero return",
    )
    final_sequence_index = transfer_completion.find(
        "constboolfinishedFinalContainerSequence="
    )
    process1_tray_owner_index = transfer_completion.find(
        "constboolprocessRound1TravelOwnsTrayCommand=",
        final_sequence_index,
    )
    process2_parking_command_index = transfer_completion.find(
        "commandStorageServoParkingPosition("
        "activeWorkAction==WORK_ACTION_PROCESS?"
        "DEPARTURE_STORAGE_SERVO_INTERVAL_MS:"
        "STORAGE_SERVO_INTERVAL_MS);",
        process1_tray_owner_index,
    )
    final_m7_zero_index = transfer_completion.find(
        "startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)",
        process2_parking_command_index,
    )
    process2_prelock_start_index = transfer_completion.find(
        "beginProcess2DepartureImuPrelock();",
        final_m7_zero_index,
    )
    final_wait_storage_index = transfer_completion.find(
        "workActionPhase=WORK_PHASE_WAIT_STORAGE_SERVO;",
        process2_prelock_start_index,
    )
    require(
        0 <= final_sequence_index < process1_tray_owner_index
        < process2_parking_command_index < final_m7_zero_index
        < process2_prelock_start_index < final_wait_storage_index,
        "PROCESS2 final transfer must command tray parking first, then M7 "
        "zero + IMU prelock, without a blocking servo call",
    )
    require_fragments(
        transfer_completion,
        (
            "preparedFirstPlacedRingPickup = preparedNextSource && "
            "activeWorkAction == WORK_ACTION_PROCESS && "
            "activeTransferPurpose == "
            "TRANSFER_PURPOSE_CONTAINER_TO_RING && workItemIndex == 2U",
            "if (preparedFirstPlacedRingPickup)",
            "workItemIndex = 0U",
            "ringPickupPrepositionedPending = true",
            "activeTransferPurpose = "
            "TRANSFER_PURPOSE_RING_TO_CONTAINER",
            "[P2 PIPE] pickup",
        ),
        "three placements to immediate first-ring pickup handoff",
    )
    placement_index = transfer_completion.find(
        "caseTRANSFER_PURPOSE_CONTAINER_TO_RING:"
    )
    pickup_index = transfer_completion.find(
        "caseTRANSFER_PURPOSE_RING_TO_CONTAINER:"
        "++correctGrabCount;break;",
        placement_index,
    )
    consume_index = transfer_completion.find(
        "consumeArmTransferCompletion();", pickup_index
    )
    item_increment_index = transfer_completion.find(
        "++workItemIndex;", consume_index
    )
    formal_pipeline_index = transfer_completion.find(
        "if(preparedNextSource){",
        item_increment_index,
    )
    require(
        0 <= placement_index < pickup_index < consume_index
        < item_increment_index < formal_pipeline_index,
        "placement/pickup completion no longer reaches the common "
        "three-item pipeline",
    )
    require(
        "ROUGH_PROCESSING_CALIBRATION_MODE"
        not in transfer_completion
        and "roughProcessingCalibrationPlacementComplete"
        not in source_without_comments,
        "calibration still contains a special three-placement "
        "early-finish branch",
    )

    service_body = extract_function_body(
        source_without_comments, "serviceCompetitionAction"
    )
    service = compact_cpp(service_body)
    wait_storage_case = compact_cpp(
        case_segment(
            service_body, "WORK_PHASE_WAIT_STORAGE_SERVO"
        )
    )
    require_fragments(
        wait_storage_case,
        (
            "process2DepartureOverlap = "
            "process2FinalStorageMayOverlapDeparture()",
            "if (process2DepartureOverlap)",
            "serviceProcess2DepartureImuPrelock()",
            "if (!liftMoveFinished())",
            "process2DepartureM7ArrivalFrameSequence = "
            "route_chassis::imuValidFrameSequence",
            "startVisualCorrectionRestore()",
            "if (!deadlineReached(workStorageServoDeadlineMs) || "
            "!liftMoveFinished())",
            "if (workItemIndex < 3U)",
            "activeTransferPurpose == "
            "TRANSFER_PURPOSE_CONTAINER_TO_RING",
            "workActionPhase = WORK_PHASE_START_UNLOAD",
            "activeTransferPurpose == "
            "TRANSFER_PURPOSE_RING_TO_CONTAINER",
            "workActionPhase = WORK_PHASE_START_RELOAD",
            "activeWorkAction == WORK_ACTION_PROCESS && "
            "activeTransferPurpose == "
            "TRANSFER_PURPOSE_CONTAINER_TO_RING",
            "workItemIndex = 0U",
            "workActionPhase = WORK_PHASE_START_RELOAD",
            "workActionPhase = WORK_PHASE_START_RESTORE",
        ),
        "calibration three-place/three-pick transition",
    )
    process2_overlap_index = wait_storage_case.find(
        "if(process2DepartureOverlap){"
    )
    process2_m7_gate_index = wait_storage_case.find(
        "if(!liftMoveFinished()){", process2_overlap_index
    )
    process2_direct_restore_index = wait_storage_case.find(
        "startVisualCorrectionRestore();", process2_m7_gate_index
    )
    generic_deadline_index = wait_storage_case.find(
        "if(!deadlineReached(workStorageServoDeadlineMs)||"
        "!liftMoveFinished()){",
        process2_direct_restore_index,
    )
    require(
        0 <= process2_overlap_index < process2_m7_gate_index
        < process2_direct_restore_index < generic_deadline_index,
        "PROCESS2 final departure does not start restoration directly "
        "at M7 zero before the generic tray deadline gate",
    )
    process2_overlap_branch = wait_storage_case[
        process2_overlap_index:generic_deadline_index
    ]
    require(
        "deadlineReached(workStorageServoDeadlineMs)"
        not in process2_overlap_branch,
        "PROCESS2 final departure still reads the 1463 ms tray deadline",
    )
    placement_continue_index = wait_storage_case.find(
        "if(workItemIndex<3U){"
    )
    process_reload_index = wait_storage_case.find(
        "if(activeWorkAction==WORK_ACTION_PROCESS&&"
        "activeTransferPurpose=="
        "TRANSFER_PURPOSE_CONTAINER_TO_RING){",
        placement_continue_index,
    )
    reset_for_pickups_index = wait_storage_case.find(
        "workItemIndex=0U;", process_reload_index
    )
    begin_pickups_index = wait_storage_case.find(
        "workActionPhase=WORK_PHASE_START_RELOAD;",
        reset_for_pickups_index,
    )
    final_restore_index = wait_storage_case.find(
        "workActionPhase=WORK_PHASE_START_RESTORE;",
        begin_pickups_index,
    )
    require(
        0 <= placement_continue_index < process_reload_index
        < reset_for_pickups_index < begin_pickups_index
        < final_restore_index,
        "PROCESS1 does not switch from three placements to three "
        "pickups before final restoration",
    )

    start_restore_case = compact_cpp(
        case_segment(
            service_body, "WORK_PHASE_START_RESTORE"
        )
    )
    require_fragments(
        start_restore_case,
        (
            "startVisualCorrectionRestore()",
        ),
        "post-pickup restoration dispatch",
    )

    wait_restore_case = compact_cpp(
        case_segment(service_body, "WORK_PHASE_WAIT_RESTORE")
    )
    require_fragments(
        wait_restore_case,
        (
            "allMotorsArrived()",
            "process2NoVisualRestoreMotion = "
            "motorPulsesAreZero(visualCorrectionAccumulator)",
            "visualCorrectionAccumulator = MotorPulses()",
            "process2PrelockedDeparture = "
            "process2FinalStorageMayOverlapDeparture() && "
            "process2DepartureImuPrelockReady && "
            "process2NoVisualRestoreMotion",
            "postM7FrameSequence != "
            "process2DepartureM7ArrivalFrameSequence",
            "imuIsFresh()",
            "millis() - route_chassis::lastImuReceiveMs <= "
            "WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS",
            "fabsf(headingErrorDegrees()) <= "
            "WORKSTATION_HEADING_TOLERANCE_DEGREES",
            "workstationMechanismsStoppedForDeparture()",
            "finishActiveWorkAction()",
            "beginWorkstationDepartureLock()",
        ),
        "PROCESS2 prelocked fast release with full-lock fallback",
    )
    require(
        "beginStorageParkingBeforeWorkFinish();"
        not in wait_restore_case
        and "beginArmStandardization(" not in wait_restore_case,
        "normal PROCESS/STORAGE exit still repeats parking and raises M7 "
        "after the turntable has stopped",
    )

    safe_wait_gate = (
        "caseWORK_PHASE_WAIT_STORAGE_PARK:"
        "if(serviceArmStandardization()&&"
        "deadlineReached(workStorageServoDeadlineMs)){"
    )
    safe_wait_index = service.find(safe_wait_gate)
    departure_gate_index = service.find(
        "beginWorkstationDepartureLock();", safe_wait_index
    )
    finish_action_index = service.find(
        "finishActiveWorkAction();", departure_gate_index
    )
    require(
        0 <= safe_wait_index < departure_gate_index < finish_action_index,
        "completion is not gated by safe parking and departure IMU lock",
    )

    correction_restore = compact_cpp(
        extract_function_body(
            source_without_comments, "startVisualCorrectionRestore"
        )
    )
    correction_calibration_index = correction_restore.find(
        "if(ROUGH_PROCESSING_CALIBRATION_MODE||"
        "STORAGE_ROUND1_SIMULATION_MODE){"
    )
    correction_stop_index = correction_restore.find(
        "stopAllMotorsImmediately();",
        correction_calibration_index,
    )
    correction_park_index = correction_restore.find(
        "beginStorageParkingBeforeWorkFinish();",
        correction_stop_index,
    )
    correction_return_index = correction_restore.find(
        "return;", correction_park_index
    )
    correction_move_index = correction_restore.find(
        "startRelativeMotorMove(",
        correction_return_index,
    )
    require(
        0 <= correction_calibration_index < correction_stop_index
        < correction_park_index < correction_return_index
        < correction_move_index,
        "calibration restoration can move the chassis instead of "
        "keeping all four wheels locked and parking the arm",
    )

    heading_lock = compact_cpp(
        extract_function_body(
            source_without_comments, "updateHeadingLock"
        )
    )
    heading_calibration_index = heading_lock.find(
        "if(LOCAL_WORKSTATION_SIMULATION_MODE){"
    )
    heading_stop_index = heading_lock.find(
        "stopAllMotorsImmediately();",
        heading_calibration_index,
    )
    heading_return_index = heading_lock.find(
        "returntrue;", heading_stop_index
    )
    normal_motion_gate_index = heading_lock.find(
        "if(!allMotorsArrived()){", heading_return_index
    )
    heading_correction_index = heading_lock.find(
        "startHeadingCorrection(error);",
        normal_motion_gate_index,
    )
    require(
        0 <= heading_calibration_index < heading_stop_index
        < heading_return_index < normal_motion_gate_index
        < heading_correction_index,
        "calibration heading checks can emit chassis correction pulses",
    )

    finish_action = compact_cpp(
        extract_function_body(
            source_without_comments, "finishActiveWorkAction"
        )
    )
    require_fragments(
        finish_action,
        (
            "if (ROUGH_PROCESSING_CALIBRATION_MODE && "
            "completedAction == WORK_ACTION_PROCESS)",
            "stopAllMotorsImmediately()",
            "enableDriveMotors()",
            "disableArmBaseMotor()",
            "commandGripperClose()",
            "programState = PROGRAM_FINISHED",
            "startRequested = false",
            'hmiSetRunStatus("CALDONE")',
            '"three placements and three pickups finished; "',
        ),
        "safe calibration stop",
    )
    calibration_finish_index = finish_action.find(
        "if(ROUGH_PROCESSING_CALIBRATION_MODE&&"
        "completedAction==WORK_ACTION_PROCESS){"
    )
    caldone_index = finish_action.find(
        'hmiSetRunStatus("CALDONE");',
        calibration_finish_index,
    )
    require(
        0 <= calibration_finish_index < caldone_index,
        "CALDONE is not gated by completion of the full PROCESS action",
    )
    calibration_finish_end = finish_action.find(
        "}", calibration_finish_index
    )
    require(
        "disableDriveMotors();"
        not in finish_action[
            calibration_finish_index:calibration_finish_end
        ],
        "CALDONE releases the four-wheel holding torque",
    )

    action_timeout = compact_cpp(
        extract_function_body(
            source_without_comments, "activeWorkActionTimeoutMs"
        )
    )
    timeout_guard_index = action_timeout.find(
        "if(LOCAL_WORKSTATION_SIMULATION_MODE){return0UL;}"
    )
    timeout_switch_index = action_timeout.find(
        "switch(activeWorkAction){"
    )
    require(
        0 <= timeout_guard_index < timeout_switch_index,
        "a local PROCESS1/STORAGE2 run can be cut off by the formal "
        "workstation total-duration timeout",
    )

def main() -> int:
    require(
        SOURCE_PATH.is_file(),
        f"source file does not exist: {SOURCE_PATH}",
    )
    require(
        ROBOT_CONFIG_PATH.is_file(),
        f"shared robot config does not exist: {ROBOT_CONFIG_PATH}",
    )
    require(
        MAIX_CLIENT_HEADER_PATH.is_file() and
        MAIX_CLIENT_SOURCE_PATH.is_file(),
        "MaixCamClient module is incomplete",
    )
    source = SOURCE_PATH.read_text(encoding="utf-8")
    source_without_comments = strip_cpp_comments(source)
    config = ROBOT_CONFIG_PATH.read_text(encoding="utf-8")
    config_without_comments = strip_cpp_comments(config)
    maix_client = (
        MAIX_CLIENT_HEADER_PATH.read_text(encoding="utf-8") +
        "\n" +
        MAIX_CLIENT_SOURCE_PATH.read_text(encoding="utf-8")
    )
    maix_client_without_comments = strip_cpp_comments(maix_client)
    compact_config = compact_cpp(config_without_comments)
    compact_maix_client = compact_cpp(maix_client_without_comments)
    require(
        "MODE_SWITCH_GUARD_PREVIOUS_MS=100UL;"
        in compact_config
        and "MODE_SWITCH_GUARD_MS=10UL;" in compact_config
        and "nowMs-modeSwitchStartMs_<"
        "config::vision_link::MODE_SWITCH_GUARD_MS"
        in compact_maix_client,
        "camera recognition request does not use the 90%-shorter 10 ms guard",
    )
    verify_run_mode_contract(
        source_without_comments,
        config_without_comments,
    )
    verify_work_action_state_machine(source_without_comments)
    verify_process_three_circle_contract(source_without_comments)
    verify_process_ring_transfer_order_contract(
        source_without_comments
    )
    verify_raw_pick_and_process_entry_contract(
        source_without_comments,
        maix_client_without_comments,
    )
    verify_calibration_mode_contract(source_without_comments)
    print("PASS live chassis bridge: six route pauses and completion gates")
    print(
        "PASS run-mode contract: guarded 0/1/2 switch in RobotConfig, "
        "local modes isolated, no fake completion or 180-second hard stop"
    )
    print("PASS workstation gates: arm/vision blocks next chassis segment")
    print(
        "PASS PROCESS order: scan 1/3, infer 2, then place and "
        "pick QR items 0/1/2 at their same-index destination rings"
    )
    print(
        "PASS RAW contract: QR-ordered mode-8 colors, camera-side "
        "five-sample confirmation, reach wait, 65/15 s timing and "
        "travel-zero PROCESS gate"
    )
    print(
        "PASS calibration mode: wheels locked, route frozen, "
        "three real placements + three pickups, safe park, CALDONE"
    )
    print(f"ALL ROUTE/WORKSTATION CHECKS PASSED: {SOURCE_PATH}")
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except (
        ContractFailure,
        OSError,
        UnicodeError,
        ValueError,
    ) as error:
        print(f"ROUTE/WORKSTATION CHECK FAILED: {error}", file=sys.stderr)
        sys.exit(1)
