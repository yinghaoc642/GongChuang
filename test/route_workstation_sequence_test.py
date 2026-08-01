#!/usr/bin/env python3

from __future__ import annotations

import ast
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

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

def normalize_expression(expression: str) -> str:
    return re.sub(r"\s+", "", expression)

def constant_initializer(
    source_without_comments: str,
    constant_name: str,
) -> str:
    declaration = re.search(
        rf"\b(?:constexpr|const)\s+float\s+"
        rf"{re.escape(constant_name)}\s*=\s*([^;]+);",
        source_without_comments,
        flags=re.DOTALL,
    )
    require(
        declaration is not None,
        f"float constant {constant_name} was not found",
    )
    return normalize_expression(declaration.group(1))

def parse_integer_literal(expression: str, label: str) -> int:
    match = re.fullmatch(
        r"\s*([+-]?\d+)(?:[uUlL]+)?\s*", expression
    )
    require(match is not None, f"{label} is not an integer literal: {expression!r}")
    return int(match.group(1))

@dataclass(frozen=True)
class RouteEntry:
    specification_step: int
    command_type: str
    value_expression: str
    motion_scale_expression: str
    name: str

def parse_route(source_without_comments: str) -> List[RouteEntry]:
    declaration = re.search(
        r"\b(?:const|constexpr)\s+RouteCommand\s+"
        r"route\s*\[\s*\]\s*=",
        source_without_comments,
    )
    require(declaration is not None, "RouteCommand route[] was not found")
    opening = source_without_comments.find("{", declaration.end())
    require(opening >= 0, "route[] opening brace was not found")
    closing = find_matching(
        source_without_comments, opening, "{", "}"
    )
    initializer = source_without_comments[opening + 1 : closing]

    raw_entries: List[str] = []
    index = 0
    while index < len(initializer):
        if initializer[index].isspace() or initializer[index] == ",":
            index += 1
            continue
        require(
            initializer[index] == "{",
            "route[] contains a non-aggregate entry near "
            f"{initializer[index:index + 30]!r}",
        )
        entry_end = find_matching(initializer, index, "{", "}")
        raw_entries.append(initializer[index + 1 : entry_end])
        index = entry_end + 1

    entries: List[RouteEntry] = []
    for route_index, raw_entry in enumerate(raw_entries):
        fields = split_top_level_fields(raw_entry)
        require(
            len(fields) >= 5,
            f"route[{route_index}] has {len(fields)} fields; expected at least 5",
        )
        step = parse_integer_literal(
            fields[0], f"route[{route_index}].specificationStep"
        )
        command_type = fields[1].strip()
        require(
            re.fullmatch(r"COMMAND_[A-Z0-9_]+", command_type)
            is not None,
            f"route[{route_index}] has invalid command type {command_type!r}",
        )
        try:
            name = ast.literal_eval(fields[4])
        except (SyntaxError, ValueError) as error:
            raise ContractFailure(
                f"route[{route_index}] has an invalid name literal"
            ) from error
        require(
            isinstance(name, str),
            f"route[{route_index}] name is not a string",
        )
        entries.append(
            RouteEntry(
                specification_step=step,
                command_type=command_type,
                value_expression=normalize_expression(fields[2]),
                motion_scale_expression=normalize_expression(fields[3]),
                name=name,
            )
        )

    require(entries, "route[] contains no entries")
    return entries

EXPECTED_MOTIONS: Sequence[Tuple[int, str, str, str]] = (
    (1, "COMMAND_ZONE_LONGITUDINAL_FAST", "DISTANCE_A_MM", "STEP_01_MOTION_SCALE"),
    (2, "COMMAND_SCAN_SLOW", "MAXIMUM_SCAN_DISTANCE_B_MM", "STEP_02_MOTION_SCALE"),
    (3, "COMMAND_ADJUST_TO_POINT_A", "SCAN_START_TO_POINT_A_MM", "STEP_03_MOTION_SCALE"),
    (4, "COMMAND_RIGHT_FAST", "DISTANCE_C_MM", "STEP_04_MOTION_SCALE"),
    (5, "COMMAND_BACKWARD_FAST", "DISTANCE_D_MM", "STEP_05_MOTION_SCALE"),
    (6, "COMMAND_TURN_COUNTERCLOCKWISE", "90", "STEP_06_MOTION_SCALE"),
    (7, "COMMAND_RIGHT_FAST", "DISTANCE_E_MM", "STEP_07_MOTION_SCALE"),
    (8, "COMMAND_TURN_CLOCKWISE", "180", "STEP_08_MOTION_SCALE"),
    (9, "COMMAND_FORWARD_FAST", "DISTANCE_F_MM", "STEP_09_MOTION_SCALE"),
    (10, "COMMAND_TURN_CLOCKWISE", "90", "STEP_10_MOTION_SCALE"),
    (11, "COMMAND_FORWARD_FAST", "DISTANCE_G_MM", "STEP_11_MOTION_SCALE"),
    (12, "COMMAND_FORWARD_FAST", "DISTANCE_D_MM", "STEP_12_MOTION_SCALE"),
    (13, "COMMAND_TURN_CLOCKWISE", "90", "STEP_13_MOTION_SCALE"),
    (14, "COMMAND_FORWARD_FAST", "DISTANCE_F_MM", "STEP_14_MOTION_SCALE"),
    (15, "COMMAND_RIGHT_FAST", "DISTANCE_E_MM", "STEP_15_MOTION_SCALE"),
    (16, "COMMAND_TURN_CLOCKWISE", "180", "STEP_16_MOTION_SCALE"),
    (17, "COMMAND_FORWARD_FAST", "DISTANCE_F_MM", "STEP_17_MOTION_SCALE"),
    (18, "COMMAND_TURN_CLOCKWISE", "90", "STEP_18_MOTION_SCALE"),
    (19, "COMMAND_FORWARD_FAST", "DISTANCE_G_MM", "STEP_19_MOTION_SCALE"),
    (20, "COMMAND_RIGHT_FAST", "STORAGE_F_TO_SCAN_A_MM", "STEP_20_MOTION_SCALE"),
    (21, "COMMAND_ZONE_LONGITUDINAL_FAST", "POINT_A_TO_START_ZONE_MM", "STEP_21_MOTION_SCALE"),
)

EXPECTED_ACTIONS: Sequence[Tuple[int, str, int]] = (
    (6, "COMMAND_RAW_ACTION", 1),
    (8, "COMMAND_PROCESS_ACTION", 1),
    (11, "COMMAND_STORAGE_ACTION", 1),
    (14, "COMMAND_RAW_ACTION", 2),
    (16, "COMMAND_PROCESS_ACTION", 2),
    (19, "COMMAND_STORAGE_ACTION", 2),
)

ACTION_TYPES = {
    "COMMAND_RAW_ACTION",
    "COMMAND_PROCESS_ACTION",
    "COMMAND_STORAGE_ACTION",
}

def verify_route_entries(entries: Sequence[RouteEntry]) -> None:
    motions = [
        entry for entry in entries if 1 <= entry.specification_step <= 21
    ]
    actual_motions = [
        (
            entry.specification_step,
            entry.command_type,
            entry.value_expression,
            entry.motion_scale_expression,
        )
        for entry in motions
    ]
    require(
        actual_motions == list(EXPECTED_MOTIONS),
        "21 motion entries differ from the chassis-only reference:\n"
        f"actual={actual_motions}",
    )

    actions: List[Tuple[int, str, int]] = []
    last_motion_step = 0
    for entry in entries:
        if 1 <= entry.specification_step <= 21:
            last_motion_step = entry.specification_step
        elif entry.command_type in ACTION_TYPES:
            require(
                entry.specification_step == 0,
                f"{entry.command_type} must use specificationStep=0",
            )
            round_number = parse_integer_literal(
                entry.value_expression,
                f"{entry.command_type}.round",
            )
            actions.append(
                (last_motion_step, entry.command_type, round_number)
            )

    require(
        actions == list(EXPECTED_ACTIONS),
        "workstation actions are not immediately after "
        "steps 6/8/11/14/16/19 in the required order:\n"
        f"actual={actions}",
    )

    finish_indices = [
        index
        for index, entry in enumerate(entries)
        if entry.command_type == "COMMAND_FINISH"
    ]
    require(
        finish_indices == [len(entries) - 1],
        "COMMAND_FINISH must appear exactly once as the final route entry",
    )
    finish = entries[-1]
    require(
        finish.specification_step == 0,
        "COMMAND_FINISH must use specificationStep=0",
    )

    allowed_non_motion = ACTION_TYPES | {"COMMAND_FINISH"}
    unexpected = [
        entry.command_type
        for entry in entries
        if not (1 <= entry.specification_step <= 21)
        and entry.command_type not in allowed_non_motion
    ]
    require(
        not unexpected,
        f"unexpected non-motion route entries remain: {unexpected}",
    )
    require(
        len(entries) == 28,
        f"route[] has {len(entries)} entries; expected 21 + 6 + FINISH",
    )

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

def verify_work_action_state_machine(
    source_without_comments: str,
) -> None:
    starter = extract_function_body(
        source_without_comments, "startMotionCommand"
    )
    updater = extract_function_body(
        source_without_comments, "updateRoute"
    )
    finisher = compact_cpp(
        extract_function_body(
            source_without_comments, "finishActiveWorkAction"
        )
    )

    action_contracts = (
        (
            "COMMAND_RAW_ACTION",
            "WORK_ACTION_RAW",
            "rawActionFinished",
        ),
        (
            "COMMAND_PROCESS_ACTION",
            "WORK_ACTION_PROCESS",
            "processActionFinished",
        ),
        (
            "COMMAND_STORAGE_ACTION",
            "WORK_ACTION_STORAGE",
            "storageActionFinished",
        ),
    )
    for command_type, work_kind, finished_flag in action_contracts:
        start_case = compact_cpp(case_segment(starter, command_type))
        require(
            f"beginWorkAction({work_kind},"
            "static_cast<uint8_t>(command.value));" in start_case,
            f"{command_type} does not start {work_kind} with route round",
        )
        require(
            "advanceRoute();" not in start_case,
            f"{command_type} advances from its start case",
        )
        require(
            start_case.count(
                f"beginWorkAction({work_kind},"
                "static_cast<uint8_t>(command.value));"
            )
            == 1,
            f"{command_type} must start its production action exactly once",
        )

        update_case = compact_cpp(case_segment(updater, command_type))
        expected_gate = f"if({finished_flag}){{"
        require(
            expected_gate in update_case,
            f"{command_type} is not directly gated by {finished_flag}",
        )
        require(
            update_case.count("advanceRoute();") == 1
            and update_case.find("advanceRoute();")
            > update_case.find(expected_gate),
            f"{command_type} does not advance exactly once inside its gate",
        )
        require(
            f"case{work_kind}:{finished_flag}=true;" in finisher,
            f"{work_kind} does not set {finished_flag} on completion",
        )

    process_routes = [
        entry
        for entry in parse_route(source_without_comments)
        if entry.command_type == "COMMAND_PROCESS_ACTION"
    ]
    require(
        [parse_integer_literal(entry.value_expression, "process round")
         for entry in process_routes]
        == [1, 2],
        "rough-station process action must run once in each round",
    )

    loop_body = compact_cpp(
        extract_function_body(source_without_comments, "loop")
    )
    service_index = loop_body.find("serviceCompetitionAction();")
    route_index = loop_body.find("updateRoute();")
    require(
        0 <= service_index < route_index,
        "loop() must service workstation actions before route advancement",
    )

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
        "workActionPhase="
        "WORK_PHASE_ENDPOINT_WAIT_RING3_RAISE;",
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
    ring3_raise_case = compact_cpp(
        case_segment(
            service_body,
            "WORK_PHASE_ENDPOINT_WAIT_RING3_RAISE",
        )
    )
    require(
        "beginDirectRing3EndpointScan();" in ring3_raise_case,
        "ring-1 handoff does not start the direct ring-3 scan",
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
            "arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM",
            "[PLACE->PICK PIPELINE] final placement immediately ",
            "prepositions first ring/slot=",
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
        and "ENDPOINT_LOCAL_MOVE_SETTLE_MS=20UL;"
        in endpoint_timing
        and "ENDPOINT_LOCAL_MOVE_SETTLE_MS*4UL=="
        "ENDPOINT_LOCAL_MOVE_SETTLE_PREVIOUS_MS"
        in endpoint_timing
        and endpoint_timing.count(
            "millis()+ENDPOINT_LOCAL_MOVE_SETTLE_MS;"
        ) == 2,
        "endpoint correction-to-re-recognition settle is not reduced "
        "from 80 ms to 20 ms on both entry and correction paths",
    )
    require(
        "ENDPOINT_FAST_ACCEPT_CENTER_TOLERANCE_PIXELS=2.0f;"
        in endpoint_timing
        and "ENDPOINT_FAST_ACCEPT_MINIMUM_CONFIDENCE=1000U;"
        in endpoint_timing
        and "constboolfastAcceptedStableCenter="
        "centerErrorPixels<="
        "ENDPOINT_FAST_ACCEPT_CENTER_TOLERANCE_PIXELS&&"
        "confidence>=ENDPOINT_FAST_ACCEPT_MINIMUM_CONFIDENCE;"
        in endpoint_timing
        and "if(fastAcceptedStableCenter){"
        "endpointCenteredConfirmationCount="
        "ENDPOINT_FINAL_CENTER_CONFIRMATIONS;"
        in endpoint_timing
        and "}elseif(endpointCenteredConfirmationCount<255U){"
        in endpoint_timing,
        "endpoint strict 2 px/full-confidence fast acceptance does not "
        "preserve the normal two-confirmation fallback",
    )
    require(
        "ENDPOINT_BASE_TO_EXTENSION_SETTLE_PREVIOUS_MS=50UL;"
        in endpoint_timing
        and "ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS=15UL;"
        in endpoint_timing
        and "ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS=15UL;"
        in endpoint_timing
        and "ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS=20UL;"
        in endpoint_timing
        and "VISION_ACTION_DELAY_CAP_MS=20UL;" in endpoint_timing
        and "VISION_POST_MOTION_SETTLE_TIME_MS=20UL;"
        in endpoint_timing
        and "VISION_HEADING_STABLE_TIME_MS=20UL;"
        in endpoint_timing
        and endpoint_timing.count(
            "millis()+ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS;"
        )
        == 2
        and "updateHeadingLock(MOTION_TIMEOUT_MS,"
        "ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS,"
        "ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS)"
        in endpoint_timing,
        "vision action handoffs are not all capped at 20 ms",
    )
    require(
        endpoint_timing.count(
            "updateHeadingLock(MOTION_TIMEOUT_MS,"
            "VISION_POST_MOTION_SETTLE_TIME_MS,"
            "VISION_HEADING_STABLE_TIME_MS)"
        )
        == 2,
        "circle correction and post-vision heading locks do not both use "
        "the 20 ms vision gate",
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
            "constexpr float RAW_VIEW_EXTENSION_MM = 50.0f",
            "constexpr uint32_t RAW_TARGET_REQUEST_REFRESH_MS = "
            "15000UL",
            "constexpr uint32_t RAW_ACTION_TIMEOUT_MS = 65000UL",
            "constexpr uint32_t ARM_BASE_SETTLE_MS = 20UL",
            "constexpr uint16_t STORAGE_SERVO_INTERVAL_MS = 409U",
            "constexpr uint32_t STORAGE_SERVO_SETTLE_MS = 510UL",
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
            "beginMaixRequest(MAIXCAM_ALL_COLORS_REQUEST)",
            "workVisionRequestStartMs = millis()",
            "workActionPhase = WORK_PHASE_RAW_WAIT_RESULT",
        ),
        "legacy all-color request with STM32 QR filtering",
    )
    expected_color_index = begin_raw_vision.find(
        "constuint8_texpectedColor="
        "taskColors[workRoundIndex][rawCollectedCount];"
    )
    request_index = begin_raw_vision.find(
        "beginMaixRequest(MAIXCAM_ALL_COLORS_REQUEST);"
    )
    wait_phase_index = begin_raw_vision.find(
        "workActionPhase=WORK_PHASE_RAW_WAIT_RESULT;"
    )
    require(
        0 <= expected_color_index < request_index < wait_phase_index
        and begin_raw_vision.count("beginMaixRequest(") == 1,
        "RAW vision must request legacy mode 8 after selecting the "
        "current QR color",
    )
    require(
        "beginMaixRequest(expectedColor);" not in begin_raw_vision,
        "production RAW pickup still requires unsupported targeted-color "
        "camera modes",
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
            "commandStorageServoPosition("
            "static_cast<uint8_t>(mappedSlot))",
            "workStorageServoDeadlineMs = "
            "millis() + STORAGE_SERVO_SETTLE_MS",
            "workActionPhase = "
            "WORK_PHASE_RAW_WAIT_EXPECTED_SLOT",
        ),
        "RAW expected-slot preparation",
    )
    slot_command_index = raw_slot_preparation.find(
        "commandStorageServoPosition("
        "static_cast<uint8_t>(mappedSlot));"
    )
    slot_deadline_index = raw_slot_preparation.find(
        "workStorageServoDeadlineMs="
        "millis()+STORAGE_SERVO_SETTLE_MS;",
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
        "starting its 300 ms settle gate",
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
            source_without_comments, "rawTargetPose"
        )
    )
    require_fragments(
        raw_target_pose,
        (
            "cameraRadiusMm = "
            "ARM_PIVOT_TO_CAMERA_CENTER_MM + "
            "RAW_VIEW_EXTENSION_MM",
            "extensionMm < M6_STANDARD_EXTENSION_MM - "
            "ARM_AXIS_POSITION_TOLERANCE_MM",
            "return RAW_TARGET_WAIT_TOO_NEAR",
            "extensionMm > M6_MAXIMUM_EXTENSION_MM + "
            "ARM_AXIS_POSITION_TOLERANCE_MM",
            "return RAW_TARGET_WAIT_TOO_FAR",
            "return RAW_TARGET_POSE_VALID",
        ),
        "RAW target reach classification",
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

    begin_work = compact_cpp(
        extract_function_body(
            source_without_comments, "beginWorkAction"
        )
    )
    require_fragments(
        begin_work,
        (
            "if (kind == WORK_ACTION_RAW)",
            "beginArmStandardization("
            "0.0f, false, RAW_VIEW_EXTENSION_MM)",
            "if (kind == WORK_ACTION_PROCESS)",
            "if (!rawTravelM5ZeroPending)",
            "startArmBaseStandardFrameDegrees(0.0f)",
            "rawTravelM5ZeroPending = true",
            "workActionPhase = "
            "WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO",
        ),
        "RAW view and PROCESS standard-zero entry",
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
    raw_prepare_case = compact_cpp(
        case_segment(service_body, "WORK_PHASE_PREPARE")
    )
    require_fragments(
        raw_prepare_case,
        (
            "if (activeWorkAction == WORK_ACTION_RAW)",
            "serviceArmStandardization()",
            "deadlineReached(workStorageServoDeadlineMs)",
            "beginRawExpectedSlotPreparation()",
        ),
        "initial RAW expected-slot dispatch",
    )
    initial_slot_prepare_index = raw_prepare_case.find(
        "beginRawExpectedSlotPreparation();"
    )
    initial_process_prepare_index = raw_prepare_case.find(
        "startPreEndpointHeadingCorrection();"
    )
    require(
        0 <= initial_slot_prepare_index
        < initial_process_prepare_index,
        "initial RAW item bypasses expected-slot preparation",
    )

    expected_slot_wait_case = compact_cpp(
        case_segment(
            service_body, "WORK_PHASE_RAW_WAIT_EXPECTED_SLOT"
        )
    )
    require_fragments(
        expected_slot_wait_case,
        (
            "if (!deadlineReached("
            "workStorageServoDeadlineMs))",
            "beginRawItemVision()",
        ),
        "RAW expected-slot settle gate",
    )
    slot_settled_index = expected_slot_wait_case.find(
        "if(!deadlineReached(workStorageServoDeadlineMs)){"
    )
    fresh_vision_index = expected_slot_wait_case.find(
        "beginRawItemVision();", slot_settled_index
    )
    require(
        0 <= slot_settled_index < fresh_vision_index,
        "RAW requests coordinates before the tray's 510 ms "
        "expected-slot settle completes",
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
            "rawTargetPose("
            "static_cast<float>(x), "
            "static_cast<float>(y), source)",
            "if (poseResult != RAW_TARGET_POSE_VALID)",
            "beginRawItemVision()",
            "rawPendingColor = detectedColor",
            "rawPendingSlotIndex = slotIndex",
            "rawPendingSourcePose = source",
            "beginArmTransfer("
            "rawPendingSourcePose, arm_transfer::containerPlacePose()",
            "activeTransferPurpose = "
            "TRANSFER_PURPOSE_RAW_TO_CONTAINER",
            "workActionPhase = WORK_PHASE_WAIT_TRANSFER",
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
    accepted_stop_index = raw_wait_case.find(
        "stopMaixRequest();", reach_retry_index
    )
    require(
        0 <= reach_wait_index < reach_retry_index
        < accepted_stop_index,
        "an unreachable RAW target is not retried before the "
        "accepted-target path",
    )
    unreachable_branch = raw_wait_case[
        reach_wait_index:accepted_stop_index
    ]
    require(
        "routeFault(" not in unreachable_branch
        and "++rawCollectedCount;" not in unreachable_branch,
        "an unreachable RAW target faults or advances QR order",
    )

    direct_transfer_index = raw_wait_case.find(
        "beginArmTransfer(", accepted_stop_index
    )
    require(
        direct_transfer_index >= 0,
        "accepted RAW coordinates do not start an immediate transfer",
    )
    direct_transfer_open = raw_wait_case.find(
        "(", direct_transfer_index
    )
    direct_transfer_close = find_matching(
        raw_wait_case, direct_transfer_open, "(", ")"
    )
    direct_transfer_arguments = split_top_level_fields(
        raw_wait_case[
            direct_transfer_open + 1:direct_transfer_close
        ]
    )
    require(
        len(direct_transfer_arguments) == 12
        and direct_transfer_arguments[0]
        == "rawPendingSourcePose"
        and direct_transfer_arguments[-2:] == ["true", "true"],
        "RAW direct transfer must enable rawFastProfile and "
        "sourceGripperAlreadyOpen",
    )
    accepted_to_transfer = raw_wait_case[
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
        "workActionStartMs=millis();" in raw_wait_case[accepted_stop_index:],
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
            "consumeArmTransferCompletion()",
            "rawFilledSlotMask = "
            "static_cast<uint8_t>("
            "rawFilledSlotMask | "
            "(1U << rawPendingSlotIndex))",
            "++rawCollectedCount",
            "if (rawCollectedCount >= 3U)",
            "beginRawTravelParkingAfterFinalStore()",
            "beginRawExpectedSlotPreparation()",
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
        "third RAW completion rejects the expected -10 mm M7 "
        "clearance before commanding the final 0 mm return",
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
            "workActionPhase = "
            "WORK_PHASE_RAW_WAIT_TRAVEL_PARK",
        ),
        "third RAW item post-arrival safe travel handoff",
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
    raw_travel_phase_index = raw_travel.find(
        "workActionPhase=WORK_PHASE_RAW_WAIT_TRAVEL_PARK;",
        pending_index,
    )
    require(
        0 <= safe_check_index < m5_zero_index < pending_index
        < raw_travel_phase_index,
        "M5 standard zero is issued before M6/M7 safe-zero "
        "validation or is not preserved for PROCESS",
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

    raw_travel_wait = compact_cpp(
        case_segment(
            service_body, "WORK_PHASE_RAW_WAIT_TRAVEL_PARK"
        )
    )
    require_fragments(
        raw_travel_wait,
        (
            "deadlineReached(workStorageServoDeadlineMs)",
            "finishActiveWorkAction()",
        ),
        "RAW route-release gate",
    )
    require(
        "armMotors.isM5Running()" not in raw_travel_wait
        and "armBaseMotionWatchdogActive" not in raw_travel_wait
        and "serviceArmStandardization()" not in raw_travel_wait,
        "RAW completion still waits for M5 instead of releasing "
        "the chassis route in parallel",
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
            "processM5ZeroSettleDeadlineMs = "
            "millis() + ARM_BASE_SETTLE_MS",
            "workActionPhase = "
            "WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO_SETTLE",
        ),
        "PROCESS M5 pulse-arrival gate",
    )
    require(
        "beginArmEndpointPreparation();" not in zero_wait_case,
        "ring-1 preparation starts before the M5 50 ms settle gate",
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
            "!deadlineReached(processM5ZeroSettleDeadlineMs) || "
            "armMotors.isM5Running() || "
            "armBaseMotionWatchdogActive",
            "rawTravelM5ZeroPending = false",
            "processM5ZeroSettleDeadlineMs = 0UL",
            "beginArmEndpointPreparation()",
            "workActionPhase = WORK_PHASE_PREPARE",
        ),
        "PROCESS M5 50 ms settle gate",
    )
    settle_guard_index = zero_settle_case.find(
        "if(!deadlineReached(processM5ZeroSettleDeadlineMs)||"
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
        "PROCESS starts ring 1 before M5 arrival plus 50 ms settle",
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

def verify_open_loop_motion_contract(
    source_without_comments: str,
) -> None:
    vehicle_displacement = extract_function_body(
        source_without_comments, "startRouteVehicleDisplacement"
    )
    require_fragments(
        vehicle_displacement,
        (
            "-userForwardMm / 1000.0f",
            "userRightMm / 1000.0f",
            "counterClockwiseDegrees * PI_F / 180.0f",
        ),
        "relative chassis-axis mapping",
    )

    longitudinal = extract_function_body(
        source_without_comments,
        "startRouteFastLongitudinalTranslation",
    )
    require_fragments(
        longitudinal,
        (
            "commandedForwardMm = nominalForwardMm * motionScale",
            "ROUTE_FAST_MAXIMUM_STEP_RATE * motionScale * "
            "ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE",
            "ROUTE_FAST_STEP_ACCELERATION * motionScale * "
            "ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE",
            "setRouteDriveMotionProfile("
            "commandedMaximumStepRate, commandedAcceleration)",
            "startRouteVehicleDisplacement("
            "commandedForwardMm, 0.0f, 0.0f)",
        ),
        "longitudinal motionScale contract",
    )

    lateral = extract_function_body(
        source_without_comments,
        "startRouteFastLateralTranslation",
    )
    require_fragments(
        lateral,
        (
            "commandedRightMm = nominalRightMm * motionScale",
            "ROUTE_FAST_MAXIMUM_STEP_RATE * motionScale * "
            "maximumSpeedProfileScale",
            "lateralAccelerationProfileLimit = "
            "ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE * "
            "ROUTE_LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL",
            "limitedAccelerationProfileScale = fminf("
            "accelerationProfileScale, "
            "lateralAccelerationProfileLimit)",
            "ROUTE_FAST_STEP_ACCELERATION * motionScale * "
            "limitedAccelerationProfileScale",
            "setRouteDriveMotionProfile("
            "commandedMaximumStepRate, commandedAcceleration)",
            "startRouteVehicleDisplacement("
            "0.0f, commandedRightMm, 0.0f)",
        ),
        "lateral motionScale contract",
    )

    turn = extract_function_body(
        source_without_comments, "startRouteTurn"
    )
    require_fragments(
        turn,
        (
            "commandedCounterClockwiseDegrees = "
            "nominalCounterClockwiseDegrees * motionScale",
            "targetCounterClockwiseHeadingDegrees += "
            "commandedCounterClockwiseDegrees",
            "integratedTurnControlActive = true",
            "integratedTurnBrakeCommandIssued = false",
            "activeTurnPulsesPerDegree = "
            "rotationPulsesPerDegree(integratedTurnDirectionSign)",
            "ROUTE_TURN_MAXIMUM_STEP_RATE * motionScale",
            "ROUTE_TURN_STEP_ACCELERATION * motionScale",
            "setRouteDriveMotionProfile("
            "ROUTE_TURN_MAXIMUM_STEP_RATE * motionScale, "
            "ROUTE_TURN_STEP_ACCELERATION * motionScale)",
            "startRouteVehicleDisplacement("
            "0.0f, 0.0f, commandedCounterClockwiseDegrees)",
        ),
        "turn motionScale/IMU-target contract",
    )

    starter = extract_function_body(
        source_without_comments, "startMotionCommand"
    )
    starter_compact = compact_cpp(starter)
    require_fragments(
        starter,
        (
            "selectedStartZoneDirection() * "
            "static_cast<float>(command.value)",
            "SCAN_START_TO_POINT_A_MM) - scanDistanceBmm",
            "selectedStartZoneDirection() * "
            "deltaAlongScanDirectionMm",
            "startRouteFastLongitudinalTranslation("
            "-static_cast<float>(command.value), "
            "command.motionScale)",
            "STEP_07_LATERAL_MAX_SPEED_SCALE",
            "STEP_07_LATERAL_ACCELERATION_SCALE",
            "STEP_15_LATERAL_MAX_SPEED_SCALE",
            "STEP_15_LATERAL_ACCELERATION_SCALE",
            "startRouteTurn("
            "-static_cast<float>(command.value), "
            "command.motionScale)",
        ),
        "route command dispatch",
    )
    require(
        starter_compact.count(
            "startRouteFastLongitudinalTranslation("
        )
        >= 4,
        "route dispatch lost a longitudinal direction",
    )

    physical_service = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceRoutePhysicalCommand"
        )
    )
    require(
        "STOP_AFTER_STEP" not in source_without_comments,
        "removed per-step early-finish switch STOP_AFTER_STEP returned",
    )
    skip_guard = "if(nextRouteCommandIsTurn()){"
    skip_index = physical_service.find(skip_guard)
    skip_advance_index = physical_service.find(
        "advanceRoute();", skip_index
    )
    skip_return_index = physical_service.find(
        "return;", skip_advance_index
    )
    heading_lock_index = physical_service.find(
        "beginRouteHeadingLock();", skip_return_index
    )
    require(
        0 <= skip_index < skip_advance_index
        < skip_return_index < heading_lock_index,
        "translation-before-turn lock skip or normal heading lock changed",
    )

    advance = compact_cpp(
        extract_function_body(source_without_comments, "advanceRoute")
    )
    stop_index = advance.find("stopAllMotorsImmediately();")
    increment_index = advance.find("++routeIndex;")
    command_reset_index = advance.find(
        "commandStarted=false;", increment_index
    )
    progress_index = advance.find(
        "markMissionProgress();", command_reset_index
    )
    require(
        advance.count("++routeIndex;") == 1
        and 0 <= stop_index < increment_index
        < command_reset_index < progress_index,
        "advanceRoute no longer stops, increments once, resets, and records "
        "normal progress in order",
    )
    require(
        "finishProgram(" not in advance,
        "advanceRoute contains an obsolete early-finish path",
    )

def verify_predictive_braking_contract(
    source_without_comments: str,
) -> None:
    expected_constants = {
        "ROUTE_MOTION_PROFILE_INCREASE_SCALE": "1.50f",
        "ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE": "1.50f",
        "ROUTE_FINAL_ALL_MAXIMUM_SPEED_SCALE": "2.0f/3.0f",
        "ROUTE_FINAL_ALL_ACCELERATION_SCALE": "4.00f",
        "ROUTE_DECELERATION_ACCELERATION_SCALE": "2.0f/3.0f",
        "ROUTE_DECELERATION_SWITCH_MARGIN_STEPS": "8.0f",
        "ROUTE_LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL": (
            "0.50f"
        ),
        "STEP_07_LATERAL_ACCELERATION_SCALE": "0.60f",
        "STEP_15_LATERAL_ACCELERATION_SCALE": "0.60f",
        "ROUTE_TURN_HEADING_TOLERANCE_DEGREES": "0.15f",
        "TURN_IMU_CONTROL_LATENCY_SECONDS": "0.015f",
        "TURN_PREDICTIVE_BRAKE_MARGIN_DEGREES": "0.10f",
    }
    for constant_name, expected_expression in expected_constants.items():
        actual_expression = constant_initializer(
            source_without_comments, constant_name
        )
        require(
            actual_expression == expected_expression,
            f"{constant_name} changed: "
            f"expected {expected_expression}, got {actual_expression}",
        )

    speed_constants = (
        "ROUTE_FAST_MAXIMUM_STEP_RATE",
        "ROUTE_SCAN_MAXIMUM_STEP_RATE",
        "ROUTE_TURN_MAXIMUM_STEP_RATE",
        "ROUTE_HEADING_CORRECTION_MAXIMUM_STEP_RATE",
    )
    for constant_name in speed_constants:
        require(
            "ROUTE_FINAL_ALL_MAXIMUM_SPEED_SCALE"
            in constant_initializer(
                source_without_comments, constant_name
            ),
            f"{constant_name} lost the global 2/3 speed scale",
        )

    acceleration_constants = (
        "ROUTE_FAST_STEP_ACCELERATION",
        "ROUTE_SCAN_STEP_ACCELERATION",
        "ROUTE_TURN_STEP_ACCELERATION",
        "ROUTE_HEADING_CORRECTION_STEP_ACCELERATION",
    )
    for constant_name in acceleration_constants:
        require(
            "ROUTE_FINAL_ALL_ACCELERATION_SCALE"
            in constant_initializer(
                source_without_comments, constant_name
            ),
            f"{constant_name} lost the global 4.0 acceleration scale",
        )

    route_profile = extract_function_body(
        source_without_comments, "setRouteDriveMotionProfile"
    )
    require_fragments(
        route_profile,
        (
            "activeDriveAcceleration = acceleration",
            "activeDriveDeceleration = acceleration * "
            "ROUTE_DECELERATION_ACCELERATION_SCALE",
            "driveDecelerationActive = false",
            "routeDriveProfileEnabled = true",
            "motors[i]->setMaxSpeed(maximumStepRate)",
            "motors[i]->setAcceleration(activeDriveAcceleration)",
        ),
        "route asymmetric acceleration profile",
    )

    ordinary_braking = compact_cpp(
        extract_function_body(
            source_without_comments,
            "serviceDriveDecelerationProfile",
        )
    )
    require_fragments(
        ordinary_braking,
        (
            "!routeDriveProfileEnabled || "
            "integratedTurnControlActive || "
            "driveDecelerationActive || "
            "activeDriveDeceleration <= 0.0f",
            "remainingSteps = motors[i]->distanceToGo()",
            "currentSpeed = fabsf(motors[i]->speed())",
            "stoppingSteps = currentSpeed * currentSpeed / "
            "(2.0f * activeDriveDeceleration)",
            "static_cast<float>(remainingSteps) <= "
            "stoppingSteps + "
            "ROUTE_DECELERATION_SWITCH_MARGIN_STEPS",
            "shouldStartDeceleration = true",
            "motors[i]->setAcceleration(activeDriveDeceleration)",
            "driveDecelerationActive = true",
        ),
        "ordinary predictive braking",
    )
    require(
        ordinary_braking.count(
            "for(uint8_ti=0U;i<4U;++i){"
        )
        >= 2,
        "ordinary braking must inspect all wheels and then switch all wheels",
    )
    trigger_index = ordinary_braking.find(
        "shouldStartDeceleration=true;"
    )
    switch_index = ordinary_braking.rfind(
        "motors[i]->setAcceleration(activeDriveDeceleration);"
    )
    require(
        0 <= trigger_index < switch_index,
        "four-wheel deceleration is not applied after any-wheel trigger",
    )

    motor_runner = compact_cpp(
        extract_function_body(
            source_without_comments, "runAllMotors"
        )
    )
    motor_service_index = motor_runner.find(
        "serviceDriveDecelerationProfile();"
    )
    run_indices = [
        motor_runner.find(f"motor{motor}.run();")
        for motor in range(1, 5)
    ]
    require(
        motor_service_index >= 0
        and all(index > motor_service_index for index in run_indices)
        and run_indices == sorted(run_indices),
        "runAllMotors() must check braking before all four run() calls",
    )

    stop = extract_function_body(
        source_without_comments, "stopAllMotorsImmediately"
    )
    require_fragments(
        stop,
        (
            "driveDecelerationActive = false",
            "routeDriveProfileEnabled = false",
            "integratedTurnControlActive = false",
            "integratedTurnBrakeCommandIssued = false",
            "integratedTurnDirectionSign = 0",
        ),
        "immediate-stop controller reset",
    )

    integrated_braking = compact_cpp(
        extract_function_body(
            source_without_comments, "beginIntegratedTurnBraking"
        )
    )
    require(
        integrated_braking.count(
            "for(uint8_ti=0U;i<4U;++i){"
        )
        >= 2,
        "integrated turn braking must update and stop all four wheels",
    )
    acceleration_index = integrated_braking.find(
        "motors[i]->setAcceleration(activeDriveDeceleration);"
    )
    stop_index = integrated_braking.find("motors[i]->stop();")
    brake_flag_index = integrated_braking.find(
        "integratedTurnBrakeCommandIssued=true;"
    )
    require(
        0 <= acceleration_index < stop_index < brake_flag_index,
        "integrated turn must select deceleration before four-wheel stop()",
    )

    integrated_turn = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceIntegratedTurnCommand"
        )
    )
    require_fragments(
        integrated_turn,
        (
            "maximumAbsoluteStepRate = 0.0f",
            "stepRate = fabsf(motors[i]->speed())",
            "predictedAngularRateDegreesPerSecond = "
            "maximumAbsoluteStepRate / activeTurnPulsesPerDegree",
            "predictedBrakingDegrees = maximumAbsoluteStepRate * "
            "maximumAbsoluteStepRate / "
            "(2.0f * activeDriveDeceleration * "
            "activeTurnPulsesPerDegree)",
            "predictedLatencyDegrees = "
            "predictedAngularRateDegreesPerSecond * "
            "TURN_IMU_CONTROL_LATENCY_SECONDS",
            "predictedBrakingDegrees + predictedLatencyDegrees + "
            "TURN_PREDICTIVE_BRAKE_MARGIN_DEGREES",
            "beginIntegratedTurnBraking()",
            "fabsf(stoppedErrorDegrees) <= "
            "ROUTE_TURN_HEADING_TOLERANCE_DEGREES",
            "integratedTurnControlActive = false",
            "advanceRoute()",
            "integratedTurnBrakeCommandIssued = false",
            "activeTurnPulsesPerDegree = "
            "rotationPulsesPerDegree(integratedTurnDirectionSign)",
            "startRouteHeadingCorrection(stoppedErrorDegrees)",
        ),
        "real-time IMU turn braking",
    )
    tolerance_index = integrated_turn.find(
        "if(fabsf(stoppedErrorDegrees)<="
        "ROUTE_TURN_HEADING_TOLERANCE_DEGREES){"
    )
    advance_index = integrated_turn.find("advanceRoute();")
    correction_index = integrated_turn.find(
        "startRouteHeadingCorrection(stoppedErrorDegrees);"
    )
    require(
        integrated_turn.count("advanceRoute();") == 1
        and 0 <= tolerance_index < advance_index < correction_index,
        "turn may advance before <=0.15 degree stop validation",
    )
    require(
        "beginRouteHeadingLock(" not in integrated_turn
        and "serviceRouteHeadingLock(" not in integrated_turn,
        "explicit turns escaped into the ordinary post-motion heading lock",
    )

    updater = compact_cpp(
        extract_function_body(source_without_comments, "updateRoute")
    )
    require(
        "caseCOMMAND_TURN_COUNTERCLOCKWISE:"
        "caseCOMMAND_TURN_CLOCKWISE:"
        "serviceIntegratedTurnCommand();break;" in updater,
        "updateRoute() does not dispatch both turns to real-time IMU control",
    )
    require(
        "caseCOMMAND_ZONE_LONGITUDINAL_FAST:"
        "caseCOMMAND_ADJUST_TO_POINT_A:"
        "caseCOMMAND_FORWARD_FAST:"
        "caseCOMMAND_BACKWARD_FAST:"
        "caseCOMMAND_RIGHT_FAST:"
        "serviceRoutePhysicalCommand();break;" in updater,
        "ordinary translations no longer share the physical-command service",
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
        "GONGCHUANG_RUN_MODE==1,"
        '"GONGCHUANG_RUN_MODEmustbe0or1");'
        in compact_config,
        "GONGCHUANG_RUN_MODE is not statically restricted to 0/1",
    )
    compact_source = compact_cpp(source_without_comments)
    require(
        "constexprboolROUGH_PROCESSING_CALIBRATION_MODE="
        "GONGCHUANG_RUN_MODE==1;" in compact_source,
        "run mode 1 is not mapped to rough-processing calibration",
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
            "if (qrScanPhase == QR_SCAN_WAIT_AT_LIMIT ||",
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
        environments == ["genericSTM32H750VB"],
        "platformio.ini must expose only the production STM32 environment; "
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

    setup = compact_cpp(
        extract_function_body(source_without_comments, "setup")
    )
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

    loop_body = compact_cpp(
        extract_function_body(source_without_comments, "loop")
    )
    qr_receive_index = loop_body.find("receiveQrData();")
    first_runtime_branch = loop_body.find("if(abortRequested){")
    require(
        0 <= qr_receive_index < first_runtime_branch,
        "receiveQrData() must run unconditionally before loop branches",
    )
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
    calibration_branch_index = begin_route.find(
        "if(ROUGH_PROCESSING_CALIBRATION_MODE){",
        running_index,
    )
    lock_wheels_index = begin_route.find(
        "stopAllMotorsImmediately();",
        calibration_branch_index,
    )
    return_after_direct_process = begin_route.find(
        "return;", direct_process_index
    )
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
        < running_index < calibration_branch_index
        < lock_wheels_index < direct_process_index
        < return_after_direct_process,
        "calibration mode does not enable and lock all four wheels "
        "before starting PROCESS1",
    )
    require(
        "disableDriveMotors();"
        not in begin_route[
            calibration_branch_index:return_after_direct_process
        ],
        "calibration mode releases the four-wheel holding torque "
        "while PROCESS1 is running",
    )

    updater = compact_cpp(
        extract_function_body(source_without_comments, "updateRoute")
    )
    calibration_guard = updater.find(
        "if(ROUGH_PROCESSING_CALIBRATION_MODE){return;}"
    )
    normal_route_guard = updater.find(
        "if(programState!=PROGRAM_RUNNING||"
        "routeIndex>=ROUTE_COMMAND_COUNT){return;}"
    )
    command_start = updater.find("startCurrentCommand();")
    require(
        0 <= calibration_guard < normal_route_guard < command_start,
        "updateRoute() can start route[0] during calibration mode",
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
            "case TRANSFER_PURPOSE_CONTAINER_TO_RING: "
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
            "preparedFirstPlacedRingPickup = preparedNextSource && "
            "activeWorkAction == WORK_ACTION_PROCESS && "
            "activeTransferPurpose == "
            "TRANSFER_PURPOSE_CONTAINER_TO_RING && workItemIndex == 2U",
            "if (preparedFirstPlacedRingPickup)",
            "workItemIndex = 0U",
            "ringPickupPrepositionedPending = true",
            "activeTransferPurpose = "
            "TRANSFER_PURPOSE_RING_TO_CONTAINER",
            "first ring pose retained while tray finishes rotating",
        ),
        "three placements to immediate first-ring pickup handoff",
    )
    placement_index = transfer_completion.find(
        "caseTRANSFER_PURPOSE_CONTAINER_TO_RING:"
        "++correctPlacementCount;break;"
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

    safe_wait_gate = (
        "caseWORK_PHASE_WAIT_STORAGE_PARK:"
        "if(serviceArmStandardization()&&"
        "deadlineReached(workStorageServoDeadlineMs)){"
    )
    safe_wait_index = service.find(safe_wait_gate)
    finish_action_index = service.find(
        "finishActiveWorkAction();", safe_wait_index
    )
    require(
        0 <= safe_wait_index < finish_action_index,
        "calibration completion is not gated by safe arm/servo parking",
    )

    correction_restore = compact_cpp(
        extract_function_body(
            source_without_comments, "startVisualCorrectionRestore"
        )
    )
    correction_calibration_index = correction_restore.find(
        "if(ROUGH_PROCESSING_CALIBRATION_MODE){"
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
        "if(ROUGH_PROCESSING_CALIBRATION_MODE){"
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
            "routeMotionPhase = ROUTE_MOTION_IDLE",
            "commandStarted = false",
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
        "if(ROUGH_PROCESSING_CALIBRATION_MODE){return0UL;}"
    )
    timeout_switch_index = action_timeout.find(
        "switch(activeWorkAction){"
    )
    require(
        0 <= timeout_guard_index < timeout_switch_index,
        "calibration PROCESS1 can be cut off by the formal "
        "workstation total-duration timeout",
    )

def verify_qr_scan_contract(source_without_comments: str) -> None:
    projection = compact_cpp(
        extract_function_body(
            source_without_comments, "forwardTravelFromOriginMm"
        )
    )
    projection_terms = (
        "now.motor1-origin.motor1",
        "now.motor2-origin.motor2",
        "now.motor3-origin.motor3",
        "now.motor4-origin.motor4",
        "FORWARD_PULSES_PER_METER*1000.0f",
    )
    for term in projection_terms:
        require(
            term in projection,
            f"QR scan-distance projection is missing {term}",
        )
    motor1 = projection.find("now.motor1-origin.motor1")
    motor2 = projection.find("now.motor2-origin.motor2")
    motor3 = projection.find("now.motor3-origin.motor3")
    motor4 = projection.find("now.motor4-origin.motor4")
    require(
        0 <= motor1 < motor2 < motor3 < motor4,
        "QR scan-distance wheel projection order changed",
    )

    capture = extract_function_body(
        source_without_comments, "captureScanDistanceB"
    )
    require_fragments(
        capture,
        (
            "scanDistanceBmm = selectedStartZoneDirection() * "
            "forwardTravelFromOriginMm(qrScanOriginMotorPositions)",
            "scanDistanceBmm = 0.0f",
            "scanDistanceBmm = scanCommandedMaximumDistanceMm",
        ),
        "QR scan-distance capture",
    )

    scan_start = extract_function_body(
        source_without_comments, "startQrScanAction"
    )
    require_fragments(
        scan_start,
        (
            "scanOriginValid = true",
            "scanDistanceBmm = 0.0f",
            "scanCommandedMaximumDistanceMm = fminf("
            "static_cast<float>(MAXIMUM_SCAN_DISTANCE_B_MM) * "
            "activeRouteCommand.motionScale, "
            "static_cast<float>(MAXIMUM_SCAN_DISTANCE_B_MM))",
            "ROUTE_SCAN_MAXIMUM_STEP_RATE * "
            "activeRouteCommand.motionScale * "
            "ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE",
            "selectedStartZoneDirection() * "
            "scanCommandedMaximumDistanceMm",
        ),
        "QR slow-scan start",
    )

    scan_update = compact_cpp(
        extract_function_body(
            source_without_comments, "updateQrScanAction"
        )
    )
    require(
        "QR_SCAN_ACTION_TIMEOUT_MS" not in scan_update,
        "step 2 still has the obsolete total QR timeout",
    )
    scan_success = scan_update.find(
        "if(scanFlag){stopAllMotorsImmediately();"
        "captureScanDistanceB();"
    )
    lock_after_success = scan_update.find(
        "beginRouteHeadingLock();",
        scan_success,
    )
    capture_at_limit = scan_update.find(
        "captureScanDistanceB();",
        lock_after_success + 1,
    )
    wait_at_limit = scan_update.find(
        "qrScanPhase=QR_SCAN_WAIT_AT_LIMIT;"
    )
    require(
        0 <= scan_success < lock_after_success < capture_at_limit
        < wait_at_limit,
        "QR scan no longer records b before IMU correction and waits at "
        "the 200 mm limit",
    )
    wait_case = case_segment(
        extract_function_body(
            source_without_comments, "updateQrScanAction"
        ),
        "QR_SCAN_WAIT_AT_LIMIT",
    )
    require_fragments(
        wait_case,
        (
            "if (scanFlag)",
            "beginRouteHeadingLock()",
            "qrScanPhase = QR_SCAN_LOCK_AFTER_CODE",
        ),
        "QR wait-at-limit resume",
    )

    watchdog = extract_function_body(
        source_without_comments, "serviceCompetitionWatchdogs"
    )
    require_fragments(
        watchdog,
        (
            "if (qrScanPhase == QR_SCAN_WAIT_AT_LIMIT ||",
            "return",
        ),
        "mission watchdog QR-wait exemption",
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
    entries = parse_route(source_without_comments)
    verify_route_entries(entries)
    verify_run_mode_contract(
        source_without_comments,
        config_without_comments,
    )
    verify_open_loop_motion_contract(source_without_comments)
    verify_predictive_braking_contract(source_without_comments)
    verify_qr_scan_contract(source_without_comments)
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
    print(
        "PASS route contract: 21 open-loop motion steps + "
        "RAW/PROCESS/STORAGE x2 + FINISH"
    )
    print(
        "PASS run-mode contract: sole guarded 0/1 switch in RobotConfig, "
        "no fake completion or 180-second hard stop, QR always serviced"
    )
    print(
        "PASS motion contract: relative axes, per-step scales, "
        "predictive braking and real-time IMU turns"
    )
    print(
        "PASS QR contract: b projection, 200 mm cap, 92-b adjustment "
        "and stationary wait"
    )
    print(
        "PASS workstation gates: actions block route advancement; "
        "PROCESS rounds 1/2 reset and run the full three-ring workflow"
    )
    print(
        "PASS PROCESS order: scan 1/3, infer 2, then place and "
        "pick QR items 0/1/2 at their same-index destination rings"
    )
    print(
        "PASS RAW contract: QR-ordered targeted colors, camera-side "
        "two-frame confirmation, reach wait, 65/15 s timing and "
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
