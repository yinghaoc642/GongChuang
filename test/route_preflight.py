#!/usr/bin/env python3

from __future__ import annotations

import ast
import math
import re
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = (
    Path(sys.argv[1]).resolve()
    if len(sys.argv) > 1
    else ROOT / "src" / "yanyanversionnew.inc"
)
CONFIG_PATH = ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h"

TRANSLATION_COMMANDS = {
    "COMMAND_MOVE_SIDE_12_MM",
    "COMMAND_MOVE_SIDE_34_MM",
    "COMMAND_MOVE_SIDE_13_MM",
    "COMMAND_MOVE_SIDE_24_MM",
}
TURN_COMMANDS = {
    "COMMAND_TURN_COUNTERCLOCKWISE_DEGREES",
    "COMMAND_TURN_CLOCKWISE_DEGREES",
}
MOTION_COMMANDS = TRANSLATION_COMMANDS | TURN_COMMANDS

ROUTE_BINDING_FIXED = "ROUTE_BINDING_FIXED"
ROUTE_BINDING_START_TO_QR = "ROUTE_BINDING_START_TO_QR"
ROUTE_BINDING_RETURN_TO_START_ROW = (
    "ROUTE_BINDING_RETURN_TO_START_ROW"
)

class PreflightFailure(AssertionError):
    pass

def require(condition: bool, message: str) -> None:
    if not condition:
        raise PreflightFailure(message)

def near(
    actual: float, expected: float, tolerance: float = 1.0e-6
) -> bool:
    return abs(actual - expected) <= tolerance

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
            if index < len(text):
                output.extend((" ", " "))
                index += 2
            continue

        output.append(character)
        index += 1

    return "".join(output)

def clean_cpp_number_suffixes(expression: str) -> str:
    expression = re.sub(
        r"\b(0[xX][0-9A-Fa-f]+)(?:[uUlL]+)\b", r"\1", expression
    )
    return re.sub(
        r"(?<![A-Za-z_])(\d+(?:\.\d*)?|\.\d+)"
        r"(?:[fFuUlL]+)\b",
        r"\1",
        expression,
    )

def evaluate_expression(
    expression: str, values: Dict[str, float]
) -> float:

    cleaned = clean_cpp_number_suffixes(expression.strip())
    cleaned = re.sub(
        r"static_cast\s*<[^>]+>\s*\(([^()]*)\)",
        r"\1",
        cleaned,
    )
    cleaned = re.sub(
        r"(?:(?:[A-Za-z_]\w*)\s*::\s*)+([A-Za-z_]\w*)",
        r"\1",
        cleaned,
    )
    cleaned = re.sub(r"\s+", " ", cleaned)
    tree = ast.parse(cleaned, mode="eval")

    def evaluate(node: ast.AST) -> float:
        if isinstance(node, ast.Expression):
            return evaluate(node.body)
        if (
            isinstance(node, ast.Constant)
            and isinstance(node.value, (int, float))
            and not isinstance(node.value, bool)
        ):
            return float(node.value)
        if isinstance(node, ast.Name) and node.id in values:
            return float(values[node.id])
        if isinstance(node, ast.UnaryOp):
            operand = evaluate(node.operand)
            if isinstance(node.op, ast.USub):
                return -operand
            if isinstance(node.op, ast.UAdd):
                return operand
        if isinstance(node, ast.BinOp):
            left = evaluate(node.left)
            right = evaluate(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            if isinstance(node.op, ast.Div):
                return left / right
            if isinstance(node.op, ast.Mod):
                return left % right
        raise ValueError(f"unsupported C++ expression: {expression!r}")

    return evaluate(tree)

def parse_numeric_constants(
    source_without_comments: str,
    initial_values: Dict[str, float] | None = None,
) -> Dict[str, float]:
    declaration_pattern = re.compile(
        r"\b(?:constexpr|const)\s+"
        r"((?:signed\s+|unsigned\s+)?"
        r"(?:float|double|int|long|size_t|"
        r"u?int(?:8|16|32|64)_t|bool))\s+"
        r"([A-Za-z_]\w*)\s*=\s*([^;]+);",
        flags=re.DOTALL,
    )
    pending = {
        name: expression
        for value_type, name, expression in declaration_pattern.findall(
            source_without_comments
        )
        if value_type.strip() != "bool"
    }
    values: Dict[str, float] = dict(initial_values or {})

    made_progress = True
    while pending and made_progress:
        made_progress = False
        for name, expression in list(pending.items()):
            try:
                values[name] = evaluate_expression(expression, values)
            except (SyntaxError, ValueError, ZeroDivisionError):
                continue
            del pending[name]
            made_progress = True

    return values

def parse_boolean_constants(
    source_without_comments: str,
) -> Dict[str, bool]:
    declaration_pattern = re.compile(
        r"\b(?:constexpr|const)\s+bool\s+"
        r"([A-Za-z_]\w*)\s*=\s*(true|false)\s*;"
    )
    return {
        name: value == "true"
        for name, value in declaration_pattern.findall(
            source_without_comments
        )
    }

def find_matching_delimiter(
    text: str, opening_index: int, opening: str, closing: str
) -> int:
    require(
        0 <= opening_index < len(text)
        and text[opening_index] == opening,
        f"delimiter {opening!r} not found at index {opening_index}",
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
    raise PreflightFailure(
        f"unclosed delimiter {opening!r} at index {opening_index}"
    )

def extract_braced_initializer(
    source_without_comments: str, declaration_pattern: str
) -> str:
    declaration = re.search(
        declaration_pattern, source_without_comments, flags=re.DOTALL
    )
    require(declaration is not None, "route[] declaration was not found")
    opening_index = source_without_comments.find(
        "{", declaration.end()
    )
    require(opening_index >= 0, "route[] opening brace was not found")
    closing_index = find_matching_delimiter(
        source_without_comments, opening_index, "{", "}"
    )
    return source_without_comments[opening_index + 1 : closing_index]

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

@dataclass(frozen=True)
class RouteCommand:
    command_type: str
    value: float
    name: str
    precise_arrival: bool = False
    central_channel: bool = False
    binding: str = ROUTE_BINDING_FIXED

def parse_route(
    source_without_comments: str, values: Dict[str, float]
) -> List[RouteCommand]:
    initializer = extract_braced_initializer(
        source_without_comments,
        r"\bconst\s+RouteCommand\s+route\s*\[\s*\]\s*=",
    )

    entries: List[str] = []
    index = 0
    while index < len(initializer):
        if initializer[index].isspace() or initializer[index] == ",":
            index += 1
            continue
        require(
            initializer[index] == "{",
            "unexpected token between route[] entries: "
            f"{initializer[index:index + 24]!r}",
        )
        closing_index = find_matching_delimiter(
            initializer, index, "{", "}"
        )
        entries.append(initializer[index + 1 : closing_index])
        index = closing_index + 1

    commands: List[RouteCommand] = []
    valid_bindings = {
        ROUTE_BINDING_FIXED,
        ROUTE_BINDING_START_TO_QR,
        ROUTE_BINDING_RETURN_TO_START_ROW,
    }
    for route_index, entry in enumerate(entries):
        fields = split_top_level_fields(entry)
        require(
            3 <= len(fields) <= 6,
            f"route[{route_index}] has {len(fields)} constructor fields",
        )
        command_type = fields[0]
        require(
            re.fullmatch(r"COMMAND_[A-Z0-9_]+", command_type)
            is not None,
            f"route[{route_index}] has invalid command type {command_type!r}",
        )
        try:
            name_value = ast.literal_eval(fields[2])
        except (SyntaxError, ValueError) as error:
            raise PreflightFailure(
                f"route[{route_index}] has invalid name literal"
            ) from error
        require(
            isinstance(name_value, str),
            f"route[{route_index}] name is not a string",
        )

        optional_bools = [False, False]
        for optional_index in range(min(2, len(fields) - 3)):
            value_text = fields[3 + optional_index]
            require(
                value_text in {"true", "false"},
                f"route[{route_index}] optional bool is {value_text!r}",
            )
            optional_bools[optional_index] = value_text == "true"

        binding = fields[5] if len(fields) == 6 else ROUTE_BINDING_FIXED
        require(
            binding in valid_bindings,
            f"route[{route_index}] has invalid binding {binding!r}",
        )
        commands.append(
            RouteCommand(
                command_type=command_type,
                value=evaluate_expression(fields[1], values),
                name=name_value,
                precise_arrival=optional_bools[0],
                central_channel=optional_bools[1],
                binding=binding,
            )
        )

    require(commands, "no route commands were parsed")
    require(
        len(commands) == len(entries),
        "one or more route entries were silently dropped",
    )
    return commands

def extract_function_body(
    source_without_comments: str, function_name: str
) -> str:
    for match in re.finditer(
        rf"\b{re.escape(function_name)}\s*\(",
        source_without_comments,
    ):
        opening_parenthesis = source_without_comments.find("(", match.start())
        closing_parenthesis = find_matching_delimiter(
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
        if index >= len(source_without_comments) or (
            source_without_comments[index] != "{"
        ):
            continue
        closing_brace = find_matching_delimiter(
            source_without_comments, index, "{", "}"
        )
        return source_without_comments[index + 1 : closing_brace]
    raise PreflightFailure(
        f"function definition {function_name}() was not found"
    )

def compact_cpp(text: str) -> str:
    return re.sub(r"\s+", "", text)

def verify_route_command_contract(
    source_without_comments: str,
) -> None:
    compact_source = compact_cpp(source_without_comments)
    required_fragments = (
        "enumRouteBinding{ROUTE_BINDING_FIXED,"
        "ROUTE_BINDING_START_TO_QR,"
        "ROUTE_BINDING_RETURN_TO_START_ROW};",
        "boolpreciseArrival;boolcentralChannel;RouteBindingbinding;",
        "boolprecise=false,boolcentral=false,"
        "RouteBindingrouteBinding=ROUTE_BINDING_FIXED",
        "preciseArrival(precise),centralChannel(central),"
        "binding(routeBinding)",
    )
    for fragment in required_fragments:
        require(
            fragment in compact_source,
            f"RouteCommand contract is missing {fragment}",
        )

    resolver = compact_cpp(
        extract_function_body(
            source_without_comments, "resolveRouteCommand"
        )
    )
    start_case_index = resolver.find(
        "caseROUTE_BINDING_START_TO_QR:"
    )
    return_case_index = resolver.find(
        "caseROUTE_BINDING_RETURN_TO_START_ROW:"
    )
    require(
        0 <= start_case_index < return_case_index,
        "route-binding switch cases are missing or out of order",
    )
    start_case = resolver[start_case_index:return_case_index]
    return_case = resolver[return_case_index:]

    require(
        re.search(
            r"if\(selectedStartZone==START_ZONE_2\)"
            r"\{[^{}]*resolved\.type="
            r"COMMAND_MOVE_SIDE_12_MM;[^{}]*\}"
            r"else\{[^{}]*resolved\.type="
            r"COMMAND_MOVE_SIDE_34_MM;[^{}]*\}"
            r"resolved\.value=START_TO_QR_PASS_MM;",
            start_case,
        )
        is not None,
        "START_TO_QR binding does not resolve Zone2 north / Zone1 south",
    )
    require(
        re.search(
            r"if\(selectedStartZone==START_ZONE_2\)"
            r"\{[^{}]*resolved\.type="
            r"COMMAND_MOVE_SIDE_12_MM;"
            r"[^{}]*resolved\.value="
            r"RETURN_TO_START_ZONE_2_Y_MM;[^{}]*\}"
            r"else\{[^{}]*resolved\.type="
            r"COMMAND_MOVE_SIDE_34_MM;"
            r"[^{}]*resolved\.value="
            r"RETURN_TO_START_ZONE_1_Y_MM;[^{}]*\}",
            return_case,
        )
        is not None,
        "RETURN_TO_START_ROW binding does not select Zone1/Zone2 motion",
    )

    start_current = compact_cpp(
        extract_function_body(
            source_without_comments, "startCurrentCommand"
        )
    )
    require(
        "activeRouteCommand="
        "resolveRouteCommand(route[routeIndex]);" in start_current,
        "route commands are not resolved before execution",
    )
    zone_selector = compact_cpp(
        extract_function_body(
            source_without_comments, "onStartButtonDoubleClick"
        )
    )
    require(
        "selectedStartZone=selectedStartZone==START_ZONE_1"
        "?START_ZONE_2:START_ZONE_1;" in zone_selector,
        "the operator cannot toggle between Start1 and Start2",
    )
    setup_body = compact_cpp(
        extract_function_body(source_without_comments, "setup")
    )
    require(
        "startButton.attachDoubleClick(onStartButtonDoubleClick);"
        in setup_body,
        "the start-zone selector is not attached to the PB9 double-click",
    )
    print("PASS RouteCommand contract: 2 booleans + explicit route binding")
    print("PASS start-zone selection: PB9 double-click toggles Zone1/Zone2")

def resolve_route(
    commands: Sequence[RouteCommand],
    start_zone: int,
    values: Dict[str, float],
) -> List[RouteCommand]:
    require(start_zone in {1, 2}, f"invalid start zone {start_zone}")
    resolved_commands: List[RouteCommand] = []
    for command in commands:
        if command.binding == ROUTE_BINDING_FIXED:
            resolved_commands.append(command)
        elif command.binding == ROUTE_BINDING_START_TO_QR:
            resolved_commands.append(
                replace(
                    command,
                    command_type=(
                        "COMMAND_MOVE_SIDE_12_MM"
                        if start_zone == 2
                        else "COMMAND_MOVE_SIDE_34_MM"
                    ),
                    value=values["START_TO_QR_PASS_MM"],
                    name=(
                        "Start2 -> QR area direct"
                        if start_zone == 2
                        else "Start1 -> QR area direct"
                    ),
                )
            )
        elif command.binding == ROUTE_BINDING_RETURN_TO_START_ROW:
            resolved_commands.append(
                replace(
                    command,
                    command_type=(
                        "COMMAND_MOVE_SIDE_12_MM"
                        if start_zone == 2
                        else "COMMAND_MOVE_SIDE_34_MM"
                    ),
                    value=values[
                        "RETURN_TO_START_ZONE_2_Y_MM"
                        if start_zone == 2
                        else "RETURN_TO_START_ZONE_1_Y_MM"
                    ],
                    name=(
                        "Return lane -> Start2 row"
                        if start_zone == 2
                        else "Return lane -> Start1 row"
                    ),
                )
            )
        else:
            raise PreflightFailure(
                f"unknown route binding {command.binding!r}"
            )
    return resolved_commands

@dataclass(frozen=True)
class Pose:
    x_mm: float
    y_mm: float
    side_24_heading_deg: float

@dataclass(frozen=True)
class Movement:
    command: RouteCommand
    before: Pose
    after: Pose

def normalized_heading(degrees: float) -> float:
    result = degrees % 360.0
    return 0.0 if near(result, 360.0) else result

def translation_angle(
    command_type: str, side_24_heading_deg: float
) -> float:
    offsets = {
        "COMMAND_MOVE_SIDE_24_MM": 0.0,
        "COMMAND_MOVE_SIDE_12_MM": 90.0,
        "COMMAND_MOVE_SIDE_13_MM": 180.0,
        "COMMAND_MOVE_SIDE_34_MM": -90.0,
    }
    return side_24_heading_deg + offsets[command_type]

def initial_pose(start_zone: int, values: Dict[str, float]) -> Pose:
    return Pose(
        values["START_CENTER_X_MM"],
        values[
            "START_ZONE_1_CENTER_Y_MM"
            if start_zone == 1
            else "START_ZONE_2_CENTER_Y_MM"
        ],
        0.0,
    )

def simulate_route(
    commands: Sequence[RouteCommand],
    start_zone: int,
    values: Dict[str, float],
) -> Tuple[List[Movement], List[Tuple[RouteCommand, Pose]]]:
    pose = initial_pose(start_zone, values)
    movements: List[Movement] = []
    events: List[Tuple[RouteCommand, Pose]] = []
    for command in commands:
        before = pose
        if command.command_type in TRANSLATION_COMMANDS:
            angle_radians = math.radians(
                translation_angle(
                    command.command_type,
                    pose.side_24_heading_deg,
                )
            )
            pose = Pose(
                pose.x_mm
                + command.value * math.cos(angle_radians),
                pose.y_mm
                + command.value * math.sin(angle_radians),
                pose.side_24_heading_deg,
            )
            movements.append(Movement(command, before, pose))
        elif (
            command.command_type
            == "COMMAND_TURN_COUNTERCLOCKWISE_DEGREES"
        ):
            pose = Pose(
                pose.x_mm,
                pose.y_mm,
                pose.side_24_heading_deg + command.value,
            )
            movements.append(Movement(command, before, pose))
        elif command.command_type == "COMMAND_TURN_CLOCKWISE_DEGREES":
            pose = Pose(
                pose.x_mm,
                pose.y_mm,
                pose.side_24_heading_deg - command.value,
            )
            movements.append(Movement(command, before, pose))
        events.append((command, pose))
    return movements, events

def assert_pose(actual: Pose, expected: Pose, label: str) -> None:
    require(near(actual.x_mm, expected.x_mm), f"{label}: x={actual.x_mm}")
    require(near(actual.y_mm, expected.y_mm), f"{label}: y={actual.y_mm}")
    require(
        near(
            normalized_heading(actual.side_24_heading_deg),
            normalized_heading(expected.side_24_heading_deg),
        ),
        f"{label}: heading={actual.side_24_heading_deg}",
    )

def body_half_extents(
    heading_deg: float, footprint_x_mm: float, footprint_y_mm: float
) -> Tuple[float, float]:
    heading = normalized_heading(heading_deg)
    cardinals = (0.0, 90.0, 180.0, 270.0)
    require(
        any(near(heading, cardinal) for cardinal in cardinals),
        f"non-cardinal route heading {heading_deg}",
    )
    if near(heading, 0.0) or near(heading, 180.0):
        return footprint_x_mm / 2.0, footprint_y_mm / 2.0
    return footprint_y_mm / 2.0, footprint_x_mm / 2.0

def footprint_bounds(
    pose: Pose, values: Dict[str, float]
) -> Tuple[float, float, float, float]:
    half_x, half_y = body_half_extents(
        pose.side_24_heading_deg,
        values["CHASSIS_FOOTPRINT_X_MM"],
        values["CHASSIS_FOOTPRINT_Y_MM"],
    )
    return (
        pose.x_mm - half_x,
        pose.x_mm + half_x,
        pose.y_mm - half_y,
        pose.y_mm + half_y,
    )

def selected_zone_bounds(
    start_zone: int, values: Dict[str, float]
) -> Tuple[float, float, float, float]:
    field_size = values["FIELD_SIZE_MM"]
    zone_size = values["START_ZONE_SIZE_MM"]
    return (
        field_size - zone_size,
        field_size,
        (
            values["START_ZONE_1_MIN_Y_MM"]
            if start_zone == 1
            else 0.0
        ),
        (
            field_size
            if start_zone == 1
            else zone_size
        ),
    )

def require_bounds_inside(
    inner: Tuple[float, float, float, float],
    outer: Tuple[float, float, float, float],
    label: str,
) -> None:
    epsilon = 1.0e-6
    require(
        inner[0] >= outer[0] - epsilon
        and inner[1] <= outer[1] + epsilon
        and inner[2] >= outer[2] - epsilon
        and inner[3] <= outer[3] + epsilon,
        f"{label}: footprint {inner} is outside {outer}",
    )

def verify_action_sequence(commands: Sequence[RouteCommand]) -> None:
    action_types = {
        "COMMAND_QR_ACTION",
        "COMMAND_RAW_ACTION",
        "COMMAND_PROCESS_ACTION",
        "COMMAND_STORAGE_ACTION",
        "COMMAND_FINAL_ALIGN",
    }
    actual = [
        (command.command_type, int(round(command.value)))
        for command in commands
        if command.command_type in action_types
    ]
    expected = [
        ("COMMAND_QR_ACTION", 0),
        ("COMMAND_RAW_ACTION", 1),
        ("COMMAND_PROCESS_ACTION", 1),
        ("COMMAND_STORAGE_ACTION", 1),
        ("COMMAND_RAW_ACTION", 2),
        ("COMMAND_PROCESS_ACTION", 2),
        ("COMMAND_STORAGE_ACTION", 2),
        ("COMMAND_FINAL_ALIGN", 0),
    ]
    require(actual == expected, f"route action sequence is {actual}")

    final_align_indices = [
        index
        for index, command in enumerate(commands)
        if command.command_type == "COMMAND_FINAL_ALIGN"
    ]
    require(
        len(final_align_indices) == 1,
        f"FINAL_ALIGN count={len(final_align_indices)}",
    )
    final_align_index = final_align_indices[0]
    commands_after_alignment = commands[final_align_index + 1 :]
    require(
        not any(
            command.command_type in MOTION_COMMANDS
            for command in commands_after_alignment
        ),
        "a chassis movement remains after FINAL_ALIGN",
    )
    require(
        commands_after_alignment
        and commands_after_alignment[-1].command_type
        == "COMMAND_FINISH",
        "route does not finish after FINAL_ALIGN",
    )
    forbidden_names = {
        "Final Y+50 before arm base home",
        "Final X+40 before arm base home",
    }
    present_forbidden_names = sorted(
        command.name
        for command in commands
        if command.name in forbidden_names
    )
    require(
        not present_forbidden_names,
        f"obsolete final-tail commands remain: {present_forbidden_names}",
    )
    print(
        "PASS mission actions: QR, RAW/PROCESS/STORAGE x2, FINAL_ALIGN"
    )
    print("PASS route tail: no chassis translation after FINAL_ALIGN")

def verify_route_geometry(
    commands: Sequence[RouteCommand], values: Dict[str, float]
) -> List[List[RouteCommand]]:
    exact_geometry = {
        "FIELD_SIZE_MM": 2400.0,
        "FIELD_CENTER_MM": 1200.0,
        "START_ZONE_SIZE_MM": 300.0,
        "START_ZONE_1_MIN_Y_MM": 2100.0,
        "CHASSIS_FOOTPRINT_X_MM": 230.0,
        "CHASSIS_FOOTPRINT_Y_MM": 300.0,
        "START_CENTER_X_MM": 2215.0,
        "START_ZONE_1_CENTER_Y_MM": 2250.0,
        "START_ZONE_2_CENTER_Y_MM": 150.0,
        "FINAL_ZONE_CENTER_X_MM": 2250.0,
        "START_TO_QR_PASS_MM": 1050.0,
        "STORAGE_ROUND2_OPEN_LOOP_Y_MM": 1110.0,
        "RETURN_TO_START_ZONE_1_Y_MM": 1140.0,
        "RETURN_TO_START_ZONE_2_Y_MM": 960.0,
    }
    for name, expected in exact_geometry.items():
        require(
            near(values[name], expected),
            f"{name}={values[name]}, expected {expected}",
        )

    start_bindings = [
        command
        for command in commands
        if command.binding == ROUTE_BINDING_START_TO_QR
    ]
    return_bindings = [
        command
        for command in commands
        if command.binding == ROUTE_BINDING_RETURN_TO_START_ROW
    ]
    require(
        len(start_bindings) == 1 and commands[0] == start_bindings[0],
        "the first and only start-to-QR binding is not route[0]",
    )
    require(
        start_bindings[0].command_type
        == "COMMAND_MOVE_SIDE_34_MM"
        and near(start_bindings[0].value, 1050.0),
        f"unresolved start binding is {start_bindings[0]}",
    )
    require(
        len(return_bindings) == 1,
        f"return-to-row binding count={len(return_bindings)}",
    )

    resolved_routes: List[List[RouteCommand]] = []
    qr_poses: List[Pose] = []
    for start_zone in (1, 2):
        resolved = resolve_route(commands, start_zone, values)
        resolved_routes.append(resolved)
        movements, events = simulate_route(
            resolved, start_zone, values
        )

        expected_first_type = (
            "COMMAND_MOVE_SIDE_34_MM"
            if start_zone == 1
            else "COMMAND_MOVE_SIDE_12_MM"
        )
        require(
            resolved[0].command_type == expected_first_type
            and near(resolved[0].value, 1050.0),
            f"Zone{start_zone} start binding resolved to {resolved[0]}",
        )

        resolved_return = next(
            command
            for command in resolved
            if command.binding
            == ROUTE_BINDING_RETURN_TO_START_ROW
        )
        expected_return_type = (
            "COMMAND_MOVE_SIDE_34_MM"
            if start_zone == 1
            else "COMMAND_MOVE_SIDE_12_MM"
        )
        expected_return_value = 1140.0 if start_zone == 1 else 960.0
        require(
            resolved_return.command_type == expected_return_type
            and near(resolved_return.value, expected_return_value),
            f"Zone{start_zone} return binding is {resolved_return}",
        )

        qr_events = [
            pose
            for command, pose in events
            if command.command_type == "COMMAND_QR_ACTION"
        ]
        require(
            len(qr_events) == 1,
            f"Zone{start_zone} QR event count={len(qr_events)}",
        )
        qr_pose = qr_events[0]
        qr_poses.append(qr_pose)
        assert_pose(
            qr_pose,
            Pose(2215.0, 1200.0, 0.0),
            f"Zone{start_zone} QR pose",
        )

        start_pose = initial_pose(start_zone, values)
        expected_final_y = 2250.0 if start_zone == 1 else 150.0
        final_pose = events[-1][1]
        assert_pose(
            final_pose,
            Pose(2250.0, expected_final_y, 180.0),
            f"Zone{start_zone} final pose",
        )

        start_half_extents = body_half_extents(
            start_pose.side_24_heading_deg,
            values["CHASSIS_FOOTPRINT_X_MM"],
            values["CHASSIS_FOOTPRINT_Y_MM"],
        )
        final_half_extents = body_half_extents(
            final_pose.side_24_heading_deg,
            values["CHASSIS_FOOTPRINT_X_MM"],
            values["CHASSIS_FOOTPRINT_Y_MM"],
        )
        require(
            start_half_extents == (115.0, 150.0)
            and final_half_extents == (115.0, 150.0),
            f"Zone{start_zone} start/final half extents changed",
        )
        zone_bounds = selected_zone_bounds(start_zone, values)
        require_bounds_inside(
            footprint_bounds(start_pose, values),
            zone_bounds,
            f"Zone{start_zone} start",
        )
        require_bounds_inside(
            footprint_bounds(final_pose, values),
            zone_bounds,
            f"Zone{start_zone} final",
        )

        field_size = values["FIELD_SIZE_MM"]
        turn_radius = math.hypot(
            values["CHASSIS_FOOTPRINT_X_MM"] / 2.0,
            values["CHASSIS_FOOTPRINT_Y_MM"] / 2.0,
        )
        for movement in movements:
            if movement.command.command_type in TRANSLATION_COMMANDS:
                half_x, half_y = body_half_extents(
                    movement.before.side_24_heading_deg,
                    values["CHASSIS_FOOTPRINT_X_MM"],
                    values["CHASSIS_FOOTPRINT_Y_MM"],
                )
                swept_bounds = (
                    min(movement.before.x_mm, movement.after.x_mm)
                    - half_x,
                    max(movement.before.x_mm, movement.after.x_mm)
                    + half_x,
                    min(movement.before.y_mm, movement.after.y_mm)
                    - half_y,
                    max(movement.before.y_mm, movement.after.y_mm)
                    + half_y,
                )
                require_bounds_inside(
                    swept_bounds,
                    (0.0, field_size, 0.0, field_size),
                    f"Zone{start_zone} {movement.command.name}",
                )
            else:
                turn_clearance = min(
                    movement.before.x_mm,
                    field_size - movement.before.x_mm,
                    movement.before.y_mm,
                    field_size - movement.before.y_mm,
                )
                require(
                    turn_clearance + 1.0e-6 >= turn_radius,
                    f"Zone{start_zone} turn sweep leaves field at "
                    f"{movement.before}",
                )

    assert_pose(qr_poses[0], qr_poses[1], "QR convergence")
    print(
        "PASS dual-zone route: "
        "Start1/Start2 -> (2215,1200), "
        "finish -> (2250,2250)/(2250,150)"
    )
    print(
        "PASS start/final footprints: "
        "230x300 mm chassis is fully inside each 300x300 zone"
    )
    return resolved_routes

def rounded_pulse_count(pulses: float) -> int:
    return int(pulses + 0.5 if pulses >= 0.0 else pulses - 0.5)

def motor_pulses_for_command(
    command: RouteCommand, values: Dict[str, float]
) -> Tuple[int, int, int, int]:
    forward_m = 0.0
    left_m = 0.0
    turn_rad = 0.0
    distance_m = command.value / 1000.0
    if command.command_type == "COMMAND_MOVE_SIDE_12_MM":
        forward_m = distance_m
    elif command.command_type == "COMMAND_MOVE_SIDE_34_MM":
        forward_m = -distance_m
    elif command.command_type == "COMMAND_MOVE_SIDE_13_MM":
        left_m = distance_m
    elif command.command_type == "COMMAND_MOVE_SIDE_24_MM":
        left_m = -distance_m
    elif (
        command.command_type
        == "COMMAND_TURN_COUNTERCLOCKWISE_DEGREES"
    ):
        turn_rad = math.radians(command.value)
    elif command.command_type == "COMMAND_TURN_CLOCKWISE_DEGREES":
        turn_rad = -math.radians(command.value)
    else:
        return (0, 0, 0, 0)

    forward_pulses = forward_m * values["FORWARD_PULSES_PER_METER"]
    lateral_pulses = left_m * values["LATERAL_PULSES_PER_METER"]
    lever_arm_m = (
        values["WHEELBASE_MM"] + values["TRACK_WIDTH_MM"]
    ) / 2000.0
    rotation_scale = values[
        "COUNTERCLOCKWISE_ROTATION_PULSE_SCALE"
        if turn_rad >= 0.0
        else "CLOCKWISE_ROTATION_PULSE_SCALE"
    ]
    rotation_pulses = (
        lever_arm_m
        * turn_rad
        / (values["WHEEL_DIAMETER_MM"] / 2000.0)
        * values["PULSES_PER_WHEEL_REVOLUTION"]
        / (2.0 * math.pi)
        * rotation_scale
    )
    physical = (
        forward_pulses - lateral_pulses - rotation_pulses,
        forward_pulses + lateral_pulses + rotation_pulses,
        forward_pulses + lateral_pulses - rotation_pulses,
        forward_pulses - lateral_pulses + rotation_pulses,
    )
    motor_directions = (-1, 1, -1, 1)
    return tuple(
        rounded_pulse_count(pulse * direction)
        for pulse, direction in zip(physical, motor_directions)
    )

def verify_wheel_pulses(
    resolved_routes: Sequence[Sequence[RouteCommand]],
    values: Dict[str, float],
    source_without_comments: str,
) -> None:
    representative_commands = (
        RouteCommand("COMMAND_MOVE_SIDE_12_MM", 1000.0, "SIDE12"),
        RouteCommand("COMMAND_MOVE_SIDE_34_MM", 1000.0, "SIDE34"),
        RouteCommand("COMMAND_MOVE_SIDE_13_MM", 1000.0, "SIDE13"),
        RouteCommand("COMMAND_MOVE_SIDE_24_MM", 1000.0, "SIDE24"),
        RouteCommand(
            "COMMAND_TURN_COUNTERCLOCKWISE_DEGREES", 90.0, "CCW"
        ),
        RouteCommand("COMMAND_TURN_CLOCKWISE_DEGREES", 90.0, "CW"),
    )
    expected_signs = {
        "COMMAND_MOVE_SIDE_12_MM": (-1, 1, -1, 1),
        "COMMAND_MOVE_SIDE_34_MM": (1, -1, 1, -1),
        "COMMAND_MOVE_SIDE_13_MM": (1, 1, -1, -1),
        "COMMAND_MOVE_SIDE_24_MM": (-1, -1, 1, 1),
        "COMMAND_TURN_COUNTERCLOCKWISE_DEGREES": (1, 1, 1, 1),
        "COMMAND_TURN_CLOCKWISE_DEGREES": (-1, -1, -1, -1),
    }
    expected_magnitudes: Dict[str, int] = {}
    for command in representative_commands:
        pulses = motor_pulses_for_command(command, values)
        signs = tuple(1 if pulse > 0 else -1 for pulse in pulses)
        require(
            signs == expected_signs[command.command_type],
            f"{command.command_type} signs={signs}, pulses={pulses}",
        )
        magnitudes = {abs(pulse) for pulse in pulses}
        require(
            len(magnitudes) == 1,
            f"{command.command_type} unequal magnitudes {pulses}",
        )
        magnitude = next(iter(magnitudes))
        require(magnitude > 0, f"{command.command_type} has zero pulses")
        expected_magnitudes[command.command_type] = magnitude

    require(
        expected_magnitudes["COMMAND_MOVE_SIDE_12_MM"]
        == rounded_pulse_count(values["FORWARD_PULSES_PER_METER"])
        and expected_magnitudes["COMMAND_MOVE_SIDE_34_MM"]
        == rounded_pulse_count(values["FORWARD_PULSES_PER_METER"]),
        "1 m forward/back pulse magnitude is wrong",
    )
    require(
        expected_magnitudes["COMMAND_MOVE_SIDE_13_MM"]
        == rounded_pulse_count(values["LATERAL_PULSES_PER_METER"])
        and expected_magnitudes["COMMAND_MOVE_SIDE_24_MM"]
        == rounded_pulse_count(values["LATERAL_PULSES_PER_METER"]),
        "1 m lateral pulse magnitude is wrong",
    )

    checked_moves = 0
    for route in resolved_routes:
        for command in route:
            if command.command_type not in MOTION_COMMANDS:
                continue
            pulses = motor_pulses_for_command(command, values)
            require(
                all(pulse != 0 for pulse in pulses),
                f"{command.name}: zero route pulse {pulses}",
            )
            require(
                len({abs(pulse) for pulse in pulses}) == 1,
                f"{command.name}: unequal route pulses {pulses}",
            )
            checked_moves += 1

    compact_source = compact_cpp(source_without_comments)
    require(
        "constWheelDirectionsMOTOR_DIRECTIONS(-1,+1,-1,+1);"
        in compact_source,
        "firmware motor installation directions are not -,+,-,+",
    )
    pulse_body = compact_cpp(
        extract_function_body(
            source_without_comments,
            "bodyDisplacementToMotorPulses",
        )
    )
    for fragment in (
        "geometry.wheelRadiusMeters()*FORWARD_PULSES_PER_METER",
        "geometry.wheelRadiusMeters()*LATERAL_PULSES_PER_METER",
        "PULSES_PER_WHEEL_REVOLUTION/TWO_PI_F*"
        "rotationPulseScale",
        "physical1*MOTOR_DIRECTIONS.frontLeft",
        "physical2*MOTOR_DIRECTIONS.frontRight",
        "physical3*MOTOR_DIRECTIONS.rearLeft",
        "physical4*MOTOR_DIRECTIONS.rearRight",
    ):
        require(
            fragment in pulse_body,
            f"firmware wheel-pulse conversion is missing {fragment}",
        )
    move_body = compact_cpp(
        extract_function_body(
            source_without_comments, "startRelativeMotorMove"
        )
    )
    run_body = compact_cpp(
        extract_function_body(source_without_comments, "runAllMotors")
    )
    for motor_number in range(1, 5):
        require(
            f"motor{motor_number}.move(pulses.motor{motor_number});"
            in move_body,
            f"M{motor_number} move target is missing",
        )
        require(
            f"motor{motor_number}.run();" in run_body,
            f"M{motor_number} run() service is missing",
        )

    print(
        "PASS wheel pulses: "
        f"all 6 sign patterns, calibrated magnitudes, "
        f"{checked_moves} resolved route moves"
    )

class AccelStepperModel:

    CW = 1
    CCW = 0

    def __init__(self) -> None:
        self.current = 0
        self.target = 0
        self.speed = 0.0
        self.maximum_speed = 1.0
        self.acceleration = 1.0
        self.n = 0
        self.c0 = 0.676 * math.sqrt(2.0) * 1_000_000.0
        self.cn = 0.0
        self.cmin = 1_000_000.0
        self.direction = self.CCW

    def distance_to_go(self) -> int:
        return self.target - self.current

    def compute_new_speed(self) -> None:
        distance_to = self.distance_to_go()
        steps_to_stop = int(
            (self.speed * self.speed) / (2.0 * self.acceleration)
        )
        if distance_to == 0 and steps_to_stop <= 1:
            self.speed = 0.0
            self.n = 0
            return

        if distance_to > 0:
            if self.n > 0:
                if steps_to_stop >= distance_to or self.direction == self.CCW:
                    self.n = -steps_to_stop
            elif self.n < 0:
                if steps_to_stop < distance_to and self.direction == self.CW:
                    self.n = -self.n
        elif distance_to < 0:
            if self.n > 0:
                if steps_to_stop >= -distance_to or self.direction == self.CW:
                    self.n = -steps_to_stop
            elif self.n < 0:
                if steps_to_stop < -distance_to and self.direction == self.CCW:
                    self.n = -self.n

        if self.n == 0:
            self.cn = self.c0
            self.direction = self.CW if distance_to > 0 else self.CCW
        else:
            self.cn = self.cn - (2.0 * self.cn) / ((4.0 * self.n) + 1.0)
            self.cn = max(self.cn, self.cmin)
        self.n += 1
        self.speed = 1_000_000.0 / self.cn
        if self.direction == self.CCW:
            self.speed = -self.speed

    def set_max_speed(self, speed: float) -> None:
        speed = abs(speed)
        if self.maximum_speed != speed:
            self.maximum_speed = speed
            self.cmin = 1_000_000.0 / speed
            if self.n > 0:
                self.n = int(
                    (self.speed * self.speed)
                    / (2.0 * self.acceleration)
                )
                self.compute_new_speed()

    def set_acceleration(self, acceleration: float) -> None:
        acceleration = abs(acceleration)
        if self.acceleration != acceleration:
            self.n = int(self.n * (self.acceleration / acceleration))
            self.c0 = (
                0.676
                * math.sqrt(2.0 / acceleration)
                * 1_000_000.0
            )
            self.acceleration = acceleration
            self.compute_new_speed()

    def move(self, relative: int) -> None:
        self.target = self.current + relative
        self.compute_new_speed()

    def step_once(self) -> bool:
        if self.speed == 0.0 and self.distance_to_go() == 0:
            return False
        self.current += 1 if self.direction == self.CW else -1
        self.compute_new_speed()
        return True

def run_stepper_move(
    pulses: int, maximum_speed: float, acceleration: float
) -> Tuple[int, int]:
    require(pulses != 0, "cannot simulate a zero-pulse move")
    model = AccelStepperModel()
    model.set_max_speed(maximum_speed)
    model.set_acceleration(acceleration)
    start = model.current
    model.move(pulses)
    target = model.target
    requested_direction = 1 if pulses > 0 else -1
    maximum_overshoot = 0
    reverse_steps = 0
    step_count = 0

    while model.speed != 0.0 or model.distance_to_go() != 0:
        previous = model.current
        require(model.step_once(), "stepper stopped before target")
        actual_direction = 1 if model.current > previous else -1
        if actual_direction != requested_direction:
            reverse_steps += 1
        directional_overshoot = (
            model.current - target
        ) * requested_direction
        maximum_overshoot = max(
            maximum_overshoot, directional_overshoot
        )
        step_count += 1
        require(
            step_count < 500_000,
            "stepper simulation did not converge",
        )

    require(model.current == target, f"stepper ended at {model.current}")
    require(
        model.current - start == pulses,
        "stepper displacement differs from requested pulses",
    )
    return maximum_overshoot, reverse_steps

def verify_motion_profiles(
    resolved_routes: Sequence[Sequence[RouteCommand]],
    values: Dict[str, float],
    source_without_comments: str,
) -> None:
    simulations = set()
    for commands in resolved_routes:
        precise_mode = False
        for command in commands:
            if command.command_type == "COMMAND_SET_PRECISE_MOTION":
                precise_mode = True
                continue
            if command.command_type not in MOTION_COMMANDS:
                continue

            if command.command_type in TRANSLATION_COMMANDS:
                maximum_segment = int(
                    round(
                        values[
                            "CENTRAL_CHANNEL_MAX_TRANSLATION_SEGMENT_MM"
                            if command.central_channel
                            else "MAX_TRANSLATION_SEGMENT_MM"
                        ]
                    )
                )
                remaining_mm = int(round(command.value))
                require(
                    remaining_mm > 0 and maximum_segment > 0,
                    f"invalid segment length for {command.name}",
                )
                while remaining_mm > 0:
                    segment_mm = min(remaining_mm, maximum_segment)
                    remaining_mm -= segment_mm
                    segment_command = replace(
                        command, value=float(segment_mm)
                    )
                    if precise_mode:
                        maximum_speed = values[
                            "FINAL_MAXIMUM_STEP_RATE"
                        ]
                        acceleration = values[
                            "FINAL_STEP_ACCELERATION"
                        ]
                    elif command.precise_arrival:
                        maximum_speed = values[
                            "WORKSTATION_MAXIMUM_STEP_RATE"
                        ]
                        acceleration = values[
                            "WORKSTATION_STEP_ACCELERATION"
                        ]
                    elif command.central_channel:
                        maximum_speed = values[
                            "CENTRAL_CHANNEL_MAXIMUM_STEP_RATE"
                        ]
                        acceleration = values["STEP_ACCELERATION"]
                    else:
                        maximum_speed = values["MAXIMUM_STEP_RATE"]
                        acceleration = values["STEP_ACCELERATION"]
                    for pulses in motor_pulses_for_command(
                        segment_command, values
                    ):
                        simulations.add(
                            (pulses, maximum_speed, acceleration)
                        )
            else:
                for pulses in motor_pulses_for_command(command, values):
                    simulations.add(
                        (
                            pulses,
                            values["TURN_MAXIMUM_STEP_RATE"],
                            values["TURN_STEP_ACCELERATION"],
                        )
                    )

    for pulses, maximum_speed, acceleration in sorted(simulations):
        overshoot, reverse_steps = run_stepper_move(
            pulses, maximum_speed, acceleration
        )
        require(
            overshoot == 0,
            f"AccelStepper overshoot={overshoot}: "
            f"{pulses} pulses at {maximum_speed}/{acceleration}",
        )
        require(
            reverse_steps == 0,
            f"AccelStepper reverse={reverse_steps}: "
            f"{pulses} pulses at {maximum_speed}/{acceleration}",
        )

    translation_body = compact_cpp(
        extract_function_body(
            source_without_comments, "startTranslationSegment"
        )
    )
    require(
        "setDriveMotionProfile("
        "translationMaximumStepRate,"
        "translationStepAcceleration);" in translation_body,
        "translation profile is not selected before each segment",
    )
    translation_fragments = (
        "maximumSegmentMm=translationCentralChannelEnabled?"
        "CENTRAL_CHANNEL_MAX_TRANSLATION_SEGMENT_MM:"
        "MAX_TRANSLATION_SEGMENT_MM;",
        "translationMaximumStepRate=preciseMotionEnabled?"
        "FINAL_MAXIMUM_STEP_RATE:",
        "translationPreciseArrivalEnabled;",
        "translationCentralChannelEnabled?"
        "CENTRAL_CHANNEL_MAXIMUM_STEP_RATE:MAXIMUM_STEP_RATE",
        "caseCOMMAND_MOVE_SIDE_12_MM:"
        "startBodyDisplacement(distanceMeters,0.0f,0.0f);",
        "caseCOMMAND_MOVE_SIDE_34_MM:"
        "startBodyDisplacement(-distanceMeters,0.0f,0.0f);",
        "caseCOMMAND_MOVE_SIDE_13_MM:"
        "startBodyDisplacement(0.0f,distanceMeters,0.0f);",
        "caseCOMMAND_MOVE_SIDE_24_MM:"
        "startBodyDisplacement(0.0f,-distanceMeters,0.0f);",
    )
    for fragment in translation_fragments:
        require(
            fragment in translation_body,
            f"translation segmentation/profile/mapping is missing {fragment}",
        )

    start_motion_body = compact_cpp(
        extract_function_body(
            source_without_comments, "startMotionCommand"
        )
    )
    require(
        "translationPreciseArrivalEnabled=command.preciseArrival;"
        in start_motion_body
        and "translationCentralChannelEnabled=command.centralChannel;"
        in start_motion_body
        and "startTranslationSegment(command.type);" in start_motion_body,
        "RouteCommand precise/central flags are not wired to translation",
    )
    require(
        translation_body.find(
            "setDriveMotionProfile("
            "translationMaximumStepRate,"
            "translationStepAcceleration);"
        )
        < translation_body.find("switch(type)"),
        "translation profile is selected after motion starts",
    )
    for obsolete_symbol in (
        "serviceWorkstationApproachProfile",
        "WORKSTATION_APPROACH_PULSE_THRESHOLD",
        "currentTranslationEndsAtPreciseArrival",
    ):
        require(
            obsolete_symbol not in source_without_comments,
            f"obsolete mid-motion profile symbol {obsolete_symbol}",
        )

    print(
        "PASS AccelStepper trajectories: "
        f"{len(simulations)} unique motor/profile cases, "
        "zero overshoot and zero reversal"
    )

def wrap_delta_degrees(degrees: float) -> float:
    while degrees >= 180.0:
        degrees -= 360.0
    while degrees < -180.0:
        degrees += 360.0
    return degrees

def unwrap_sequence(raw_degrees: Iterable[float]) -> float:
    iterator = iter(raw_degrees)
    previous = next(iterator)
    continuous = 0.0
    for raw in iterator:
        continuous += wrap_delta_degrees(raw - previous)
        previous = raw
    return continuous

def verify_imu_unwrap(
    source_without_comments: str, values: Dict[str, float]
) -> None:
    require(
        near(
            unwrap_sequence((170.0, 179.0, -179.0, -170.0)),
            20.0,
        ),
        "counterclockwise +/-180 unwrap failed",
    )
    require(
        near(
            unwrap_sequence((-170.0, -179.0, 179.0, 170.0)),
            -20.0,
        ),
        "clockwise +/-180 unwrap failed",
    )
    positive_turn = [0.0]
    negative_turn = [0.0]
    for degrees in range(10, 731, 10):
        positive_turn.append(
            ((degrees + 180.0) % 360.0) - 180.0
        )
        negative_turn.append(
            ((-degrees + 180.0) % 360.0) - 180.0
        )
    require(
        near(unwrap_sequence(positive_turn), 730.0),
        "counterclockwise +730 degree unwrap failed",
    )
    require(
        near(unwrap_sequence(negative_turn), -730.0),
        "clockwise -730 degree unwrap failed",
    )

    imu_body = compact_cpp(
        extract_function_body(
            source_without_comments, "updateContinuousImuHeading"
        )
    )
    wrap_body = compact_cpp(
        extract_function_body(
            source_without_comments, "wrapDeltaDegrees"
        )
    )
    require(
        "while(degrees>=180.0f){degrees-=360.0f;}" in wrap_body
        and "while(degrees<-180.0f){degrees+=360.0f;}" in wrap_body,
        "firmware +/-180 degree wrapping arithmetic changed",
    )
    require(
        near(values["IMU_COUNTERCLOCKWISE_SIGN"], 1.0),
        "IMU_COUNTERCLOCKWISE_SIGN must remain +1 for this mounting",
    )
    require(
        "continuousDeltaDegrees=wrapDeltaDegrees("
        "counterClockwiseSignedRaw-imuLastSignedRawDegrees);"
        in imu_body
        and "imuCounterClockwiseDegrees+=continuousDeltaDegrees;"
        in imu_body,
        "firmware does not accumulate wrapped IMU frame deltas",
    )
    print("PASS IMU unwrap: +/-180 crossings and +/-730 degree turns")

def verify_qr_reliability_contract(
    source_without_comments: str, values: Dict[str, float]
) -> None:
    require(
        values["QR_REQUIRED_MATCHING_FRAMES"] == 1.0,
        "QR_REQUIRED_MATCHING_FRAMES must be exactly 1 for "
        "single-valid-frame lock",
    )

    reset_body = compact_cpp(
        extract_function_body(
            source_without_comments, "resetQrReceiver"
        )
    )
    for fragment in (
        "scanFlag=false;",
        "taskCodeDecoded=false;",
        "qrCandidate[0]='\\0';",
        "qrMatchingFrameCount=0U;",
        "taskPlan.clear();",
    ):
        require(
            fragment in reset_body,
            f"QR receiver reset is missing {fragment}",
        )

    finish_body = compact_cpp(
        extract_function_body(
            source_without_comments, "finishQrFrame"
        )
    )
    require(
        "taskStatus=qrOverflow?"
        "competition::TASK_CODE_WRONG_LENGTH:"
        "competition::parseTaskCode(qrData,candidatePlan);"
        in finish_body,
        "QR overflow/validation does not feed a temporary TaskPlan",
    )
    require(
        "if(strcmp(qrCandidate,qrData)==0){"
        "if(qrMatchingFrameCount<UINT8_MAX){"
        "++qrMatchingFrameCount;}}"
        "else{memcpy(qrCandidate,qrData,QR_CODE_LENGTH+1U);"
        "qrMatchingFrameCount=1U;}" in finish_body,
        "QR candidate initialization/counting contract changed",
    )

    first_valid_count_index = finish_body.find(
        "qrMatchingFrameCount=1U;"
    )
    threshold_index = finish_body.find(
        "if(qrMatchingFrameCount>=QR_REQUIRED_MATCHING_FRAMES)"
    )
    plan_assignment_index = finish_body.find(
        "taskPlan=candidatePlan;"
    )
    scan_flag_index = finish_body.find("scanFlag=true;")
    invalid_candidate_reset_index = finish_body.find(
        "qrCandidate[0]='\\0';", scan_flag_index + 1
    )
    invalid_count_reset_index = finish_body.find(
        "qrMatchingFrameCount=0U;", scan_flag_index + 1
    )
    require(
        0
        <= first_valid_count_index
        < threshold_index
        < plan_assignment_index
        < scan_flag_index
        < invalid_candidate_reset_index,
        "one strictly valid QR frame cannot reach task-plan and "
        "scan-flag commit",
    )
    require(
        invalid_candidate_reset_index < invalid_count_reset_index,
        "an invalid QR frame does not clear candidate and match count",
    )
    require(
        finish_body.count("taskPlan=candidatePlan;") == 1
        and finish_body.count("scanFlag=true;") == 1
        and "taskCodeDecoded=true;" in finish_body,
        "QR lock state is assigned outside the validated-frame branch",
    )

    scan_body = compact_cpp(
        extract_function_body(
            source_without_comments, "startQrScanAction"
        )
    )
    require(
        "qrScanOriginMotorPositions=currentDriveMotorPositions();"
        in scan_body,
        "QR sweep does not save its four-wheel absolute origin",
    )
    require(
        "qrSweepForwardSign=selectedStartZone==START_ZONE_1?"
        "-1.0f:1.0f;" in scan_body,
        "QR sweep must be -forward from Zone1 and +forward from Zone2",
    )
    require(
        "startBodyDisplacement(qrSweepForwardSign*"
        "static_cast<float>(QR_SCAN_SWEEP_MAXIMUM_MM)/"
        "1000.0f,0.0f,0.0f);" in scan_body,
        "QR sweep sign is not applied to the forward displacement",
    )

    return_body = compact_cpp(
        extract_function_body(
            source_without_comments, "startQrScanReturnToOrigin"
        )
    )
    for motor_number in range(1, 5):
        require(
            f"motor{motor_number}.moveTo("
            f"qrScanOriginMotorPositions.motor{motor_number});"
            in return_body,
            f"QR return does not restore M{motor_number} absolute origin",
        )
    require(
        "qrScanPhase=QR_SCAN_RETURNING;" in return_body,
        "QR absolute return does not enter RETURNING state",
    )

    print(
        "PASS QR reliability: single validated-frame lock, invalid reset, "
        "dual-zone sweep, absolute four-wheel return"
    )

def verify_vision_protocol_contract(
    source_without_comments: str, values: Dict[str, float]
) -> None:
    compact_source = compact_cpp(source_without_comments)
    require(
        "#include<VisionProtocol.h>" in compact_source,
        "main controller does not include VisionProtocol v2",
    )
    require(
        near(values["MAIXCAM_ALL_COLORS_REQUEST"], 8.0)
        and near(values["MAIXCAM_HOUGH_CIRCLE_REQUEST"], 9.0)
        and near(values["MAIXCAM_ENDPOINT_CIRCLE_REQUEST"], 10.0),
        "Vision mode IDs must remain 8 (colors), 9 (three circles), "
        "and 10 (one endpoint ring)",
    )
    require(
        values["MAIXCAM_LINE_CAPACITY"] >= 64.0,
        "MAIXCAM_LINE_CAPACITY must hold a complete v2 response",
    )
    require(
        "charmaixReceiveLine[MAIXCAM_LINE_CAPACITY]={0};"
        in compact_source,
        "MaixCAM receive buffer does not use the checked line capacity",
    )

    write_body = compact_cpp(
        extract_function_body(
            source_without_comments, "writeMaixRequestFrame"
        )
    )
    require(
        "uint8_tframe[vision_protocol::REQUEST_FRAME_SIZE]={0U};"
        in write_body,
        "Vision request does not use REQUEST_FRAME_SIZE",
    )
    require(
        "vision_protocol::buildRequest("
        "maixRequestSequence,request,frame,sizeof(frame))"
        in write_body
        and "SerialMaixcam.write(frame,sizeof(frame))" in write_body,
        "Vision request is not built and transmitted as one v2 frame",
    )

    begin_body = compact_cpp(
        extract_function_body(
            source_without_comments, "beginMaixRequest"
        )
    )
    sequence_increment = (
        "maixRequestSequence=static_cast<uint8_t>("
        "maixRequestSequence+1U);"
    )
    require(
        begin_body.count(sequence_increment) == 1,
        "each new logical Vision request must increment sequence once",
    )
    for request_name in (
        "MAIXCAM_ALL_COLORS_REQUEST",
        "MAIXCAM_HOUGH_CIRCLE_REQUEST",
        "MAIXCAM_ENDPOINT_CIRCLE_REQUEST",
    ):
        require(
            begin_body.count(
                f"request!={request_name}"
            )
            == 1,
            "Vision request allow-list is missing or duplicates "
            f"{request_name}",
        )
    require(
        begin_body.find("stopMaixRequest();")
        < begin_body.find(sequence_increment)
        < begin_body.find("maixRequestedMode=request;"),
        "Vision sequence/mode is not initialized in request order",
    )

    finish_body = compact_cpp(
        extract_function_body(
            source_without_comments, "finishMaixCoordinateLine"
        )
    )
    parse_index = finish_body.find(
        "vision_protocol::parseResponse("
        "maixReceiveLine,maixReceiveLength,response)"
    )
    match_guard_index = finish_body.find(
        "if(!maixModeCommandSent||"
        "maixRequestedMode==MAIXCAM_STOP_REQUEST||"
        "response.sequence!=maixRequestSequence||"
        "response.mode!=maixRequestedMode)"
    )
    status_guard_index = finish_body.find(
        "if(response.status!=vision_protocol::STATUS_OK)"
    )
    target_definition_index = finish_body.find(
        "targetMatchesMode="
    )
    coordinate_commit_index = finish_body.find(
        "latestMaixCoordinate.targetId=response.target;"
    )
    require(
        0
        <= parse_index
        < match_guard_index
        < status_guard_index
        < target_definition_index
        < coordinate_commit_index,
        "Vision response is committed before parse/seq/mode/status/target checks",
    )
    require(
        "if(parseError!=vision_protocol::PARSE_OK)" in finish_body,
        "Vision parse errors are not rejected",
    )
    require(
        "if(response.status!=vision_protocol::STATUS_OK)"
        in finish_body
        and "return;}" in finish_body[status_guard_index:target_definition_index],
        "non-OK Vision status is not rejected",
    )
    require(
        "(maixRequestedMode==MAIXCAM_ALL_COLORS_REQUEST&&"
        "response.target>=1U&&response.target<=4U)||"
        "(maixRequestedMode==MAIXCAM_HOUGH_CIRCLE_REQUEST&&"
        "response.target==2U)||"
        "(maixRequestedMode==MAIXCAM_ENDPOINT_CIRCLE_REQUEST&&"
        "response.target==1U)" in finish_body,
        "Vision target identity is not constrained by mode "
        "(mode 9 => 2, mode 10 => endpoint token 1)",
    )
    target_rejection_index = finish_body.find(
        "if(!targetMatchesMode)"
    )
    require(
        target_definition_index < target_rejection_index,
        "Vision target gate definition/rejection order is invalid",
    )
    target_gate = finish_body[
        target_definition_index:target_rejection_index
    ]
    require(
        target_gate.count("maixRequestedMode==") == 3,
        "Vision target gate must contain exactly the mode 8/9/10 "
        "identity branches",
    )

    overflow_guard_index = finish_body.find(
        "if(maixReceiveOverflow)"
    )
    require(
        0 <= overflow_guard_index < parse_index,
        "overflowed Vision lines can reach parseResponse",
    )
    overflow_guard = finish_body[
        overflow_guard_index:parse_index
    ]
    require(
        "maixReceiveLength=0U;" in overflow_guard
        and "maixReceiveOverflow=false;" in overflow_guard
        and "return;" in overflow_guard,
        "overflowed Vision line is not discarded atomically",
    )

    service_body = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceMaixcam"
        )
    )
    newline_index = service_body.find(
        "if(incoming=='\\n'){finishMaixCoordinateLine();continue;}"
    )
    discard_index = service_body.find(
        "if(maixReceiveOverflow){continue;}"
    )
    append_index = service_body.find(
        "if(maixReceiveLength<MAIXCAM_LINE_CAPACITY-1U)"
    )
    overflow_set_index = service_body.find(
        "maixReceiveOverflow=true;maixReceiveLength=0U;"
    )
    require(
        0
        <= newline_index
        < discard_index
        < append_index
        < overflow_set_index,
        "Vision overflow bytes are not discarded through the next newline",
    )

    print(
        "PASS VisionProtocol v2: framed requests, per-request sequence, "
        "strict response gates, mode-9/10 target gates, "
        "atomic overflow discard"
    )

def verify_endpoint_mapping_contract(
    source_without_comments: str,
    values: Dict[str, float],
) -> None:
    fully_retracted_pivot_mm = 125.74
    working_zero_offset_mm = 10.0
    lift_working_zero_offset_mm = 10.0
    pivot_to_camera_mm = (
        fully_retracted_pivot_mm + working_zero_offset_mm
    )
    require(
        near(
            values[
                "ARM_PIVOT_TO_CAMERA_FULLY_RETRACTED_MM"
            ],
            fully_retracted_pivot_mm,
        )
        and near(
            values[
                "ARM_PIVOT_TO_GRIPPER_FULLY_RETRACTED_MM"
            ],
            fully_retracted_pivot_mm,
        )
        and near(
            values["M6_STARTUP_WORKING_ZERO_OFFSET_MM"],
            working_zero_offset_mm,
        )
        and near(
            values["ARM_PIVOT_TO_CAMERA_CENTER_MM"],
            pivot_to_camera_mm,
        )
        and near(
            values["ARM_PIVOT_TO_GRIPPER_CENTER_MM"],
            pivot_to_camera_mm,
        )
        and near(
            values["M6_MAXIMUM_PHYSICAL_EXTENSION_MM"],
            150.0,
        )
        and near(
            values["M6_MAXIMUM_EXTENSION_MM"],
            140.0,
        )
        and near(
            values["M7_STARTUP_WORKING_ZERO_OFFSET_MM"],
            lift_working_zero_offset_mm,
        )
        and near(
            values["M7_MINIMUM_PHYSICAL_HEIGHT_MM"],
            -160.0,
        )
        and near(
            values["M7_MINIMUM_HEIGHT_MM"],
            -150.0,
        ),
        "M6/M7 safe-zero geometry does not preserve both "
        "10 mm startup offsets and physical travel limits",
    )
    expected_m6_pulses_per_mm = (
        200.0 * 256.0 / (math.pi * 35.0)
    )
    expected_m7_pulses_per_mm = 200.0 * 256.0 / 12.0
    require(
        near(values["M6_MICROSTEPS"], 256.0)
        and near(values["M7_MICROSTEPS"], 256.0)
        and near(
            values["M6_PULSES_PER_MM"],
            expected_m6_pulses_per_mm,
            1.0e-3,
        )
        and near(
            values["M7_PULSES_PER_MM"],
            expected_m7_pulses_per_mm,
            1.0e-3,
        )
        and "ARM_LINEAR_MICROSTEPS"
        not in source_without_comments,
        "M6 and M7 pulse conversion changed unexpectedly",
    )
    require(
        near(values["M6_SPEED_RPM"], 390.0)
        and near(values["M6_ACCELERATION"], 171.0)
        and near(values["ENDPOINT_FINE_M6_SPEED_RPM"], 160.0)
        and near(values["ENDPOINT_FINE_M6_ACCELERATION"], 96.0)
        and near(values["ENDPOINT_COARSE_M6_SPEED_RPM"], 240.0)
        and near(values["ENDPOINT_COARSE_M6_ACCELERATION"], 149.0)
        and near(values["M6_RECOVERY_SPEED_RPM"], 120.0)
        and near(values["M6_RECOVERY_ACCELERATION"], 64.0)
        and near(values["ARM_LINEAR_STARTUP_ZERO_SPEED_RPM"], 100.0)
        and near(values["ARM_LINEAR_STARTUP_ZERO_ACCELERATION"], 80.0),
        "M6 gentle motion profiles changed unexpectedly",
    )
    expected_axis_safety_values = {
        "ARM_AXIS_ENABLE_RESPONSE_WAIT_MS": 70.0,
        "ARM_AXIS_TERMINAL_CONFIRMATION_SAMPLES": 2.0,
        "ARM_AXIS_TERMINAL_VERIFY_MAX_FAILURES": 2.0,
        "ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM": 0.60,
        "ARM_AXIS_STALL_CONFIRMATION_SAMPLES": 3.0,
        "ARM_AXIS_MAXIMUM_RECOVERY_ATTEMPTS": 1.0,
        "ARM_AXIS_RECOVERY_TOTAL_TIMEOUT_MS": 5_500.0,
    }
    for name, expected in expected_axis_safety_values.items():
        require(
            near(values[name], expected),
            f"{name}={values[name]}, expected {expected}",
        )
    require(
        near(values["ARM_LINEAR_STARTUP_PROBE_MM"], 0.5)
        and round(
            values["ARM_LINEAR_STARTUP_PROBE_MM"]
            * values["M6_PULSES_PER_MM"]
        )
        == 233
        and round(
            values["M6_STARTUP_WORKING_ZERO_OFFSET_MM"]
            * values["M6_PULSES_PER_MM"]
        )
        == 4656
        and round(
            values["ENDPOINT_SEARCH_SEED_EXTENSION_MM"]
            * values["M6_PULSES_PER_MM"]
        )
        == 41908
        and (
            values["ARM_LINEAR_STARTUP_PROBE_MM"]
            * values["M6_MICROSTEPS"]
            / 8.0
        )
        <= 16.0,
        "M6=256 pulse counts or safe mismatch probe changed unexpectedly",
    )

    linear_move_helper = compact_cpp(
        extract_function_body(
            source_without_comments, "startLinearAxisMove"
        )
    )
    require(
        "pulsesPerMm*travelPerRevolutionMm/60.0f"
        in linear_move_helper
        and "axis.address==extensionAxis.address?"
        "M6_TRAVEL_PER_REVOLUTION_MM:"
        "M7_TRAVEL_PER_REVOLUTION_MM"
        in linear_move_helper
        and "ARM_LINEAR_MICROSTEPS" not in linear_move_helper,
        "linear-axis timeout still assumes one shared microstep value",
    )
    consecutive_reset_index = linear_move_helper.find(
        "axis.stallProtectionSamples=0U;"
    )
    retry_guard_index = linear_move_helper.find(
        "if(!recoveryRetry){"
    )
    require(
        0 <= consecutive_reset_index < retry_guard_index
        and "axis.recoveryAttemptCount=0U;"
        in linear_move_helper[retry_guard_index:],
        "recovery retry does not reset consecutive flags while "
        "preserving its bounded attempt count",
    )

    startup_zero_helper = compact_cpp(
        extract_function_body(
            source_without_comments,
            "establishLinearAxisSafeWorkingZero",
        )
    )
    for fragment in (
        "readArmLinearAngleWithRetry(",
        "writeArmLinearResetStallProtection(",
        "ARM_LINEAR_STARTUP_PROBE_MM",
        "probeTargetMm",
        "ARM_LINEAR_STARTUP_ZERO_SPEED_RPM,"
        "ARM_LINEAR_STARTUP_ZERO_ACCELERATION)",
        "waitForLinearAxisStartupMove(",
        "validateLinearAxisStartupAngleDelta(",
        "\"full-offset\"",
        "writeArmLinearResetCurrentPositionZero(",
        "axis.currentMm=0.0f;",
        "axis.targetMm=0.0f;",
    ):
        require(
            fragment in startup_zero_helper,
            "shared M6/M7 startup safe-zero helper is missing "
            f"{fragment}",
        )

    startup_angle_validator = compact_cpp(
        extract_function_body(
            source_without_comments,
            "validateLinearAxisStartupAngleDelta",
        )
    )
    for fragment in (
        "observedAngleRatio=",
        "ARM_LINEAR_ZERO_ANGLE_RATIO_MINIMUM",
        "ARM_LINEAR_ZERO_ANGLE_RATIO_MAXIMUM",
        "angleDirectionValid",
        "calibrationrejected",
    ):
        require(
            fragment in startup_angle_validator,
            "startup probe angle validator is missing "
            + fragment,
        )

    for fragment in (
        "axis.driverWorkingZeroAngleDegrees="
        "workingZeroAngleDegrees;",
        "axis.driverWorkingZeroAngleValid=true;",
        "cannotestablishrecoveryanglereference",
    ):
        require(
            fragment in startup_zero_helper,
            "startup zero does not establish safe recovery reference: "
            + fragment,
        )

    angle_to_position = compact_cpp(
        extract_function_body(
            source_without_comments,
            "linearAxisPositionMmFromMotorAngle",
        )
    )
    for fragment in (
        "if(!axis.driverWorkingZeroAngleValid){returnfalse;}",
        "relativeAngleDegrees=motorAngleDegrees-"
        "axis.driverWorkingZeroAngleDegrees;",
    ):
        require(
            fragment in angle_to_position,
            "stall recovery does not use driver-angle delta: "
            + fragment,
        )

    linear_frame_handler = compact_cpp(
        extract_function_body(
            source_without_comments, "handleArmLinearFrame"
        )
    )
    e2_index = linear_frame_handler.find(
        "if(frame[1]==0xFDU&&frame[2]==0xE2U)"
    )
    fd_ack_index = linear_frame_handler.find(
        "if(frame[1]==0xFDU&&frame[2]==0x02U)",
        e2_index,
    )
    require(
        0 <= e2_index < fd_ack_index,
        "FD E2 and accepted-FD branches cannot be isolated",
    )
    e2_segment = linear_frame_handler[e2_index:fd_ack_index]
    require(
        "axis->positionCommandRejected=true;" in e2_segment
        and "axis->stallProtectionSamples=0U;" in e2_segment
        and "writeArmLinearStatusRequest(axis->address);"
        in e2_segment
        and "scheduleLinearAxisRecovery(" not in e2_segment
        and "ARM_AXIS_RECOVERY_STALL" not in e2_segment,
        "FD E2 is incorrectly treated as an immediate stall/recovery",
    )
    status_branch_index = linear_frame_handler.find(
        "if(frame[1]==0x3AU){"
    )
    fd9f_index = linear_frame_handler.find(
        "if(frame[1]==0xFDU&&frame[2]==0x9FU)",
        status_branch_index,
    )
    require(
        0 <= status_branch_index < fd9f_index,
        "3A and explicit FD-9F terminal branches cannot be isolated",
    )
    status_segment = linear_frame_handler[
        status_branch_index:fd9f_index
    ]
    verified_declaration_index = status_segment.find(
        "constboolverifiedOnPosition="
    )
    verified_branch_index = status_segment.find(
        "if(verifiedOnPosition){",
        verified_declaration_index,
    )
    if verified_declaration_index >= 0:
        require(
            verified_branch_index > verified_declaration_index
            and "!locked"
            in status_segment[
                verified_declaration_index:verified_branch_index
            ],
            "0x07 can still be accepted from ACK/motion evidence "
            "without excluding the locked bit",
        )
    else:
        require(
            "markLinearAxisArrived(*axis);"
            not in status_segment,
            "3A on-position can bypass asynchronous encoder "
            "verification",
        )

    locked_reason_index = status_segment.find(
        "ARM_AXIS_VERIFY_LOCKED_ON_POSITION"
    )
    locked_request_index = status_segment.rfind(
        "requestLinearAxisTerminalVerification(",
        0,
        locked_reason_index,
    )
    healthy_reason_index = status_segment.find(
        "ARM_AXIS_VERIFY_HEALTHY_WITHOUT_COMMAND_EVIDENCE"
    )
    healthy_request_index = status_segment.rfind(
        "requestLinearAxisTerminalVerification(",
        0,
        healthy_reason_index,
    )
    terminal_sample_index = status_segment.rfind(
        "axis->terminalOnPositionSamples",
        0,
        healthy_request_index,
    )
    terminal_threshold_index = status_segment.rfind(
        "ARM_AXIS_TERMINAL_CONFIRMATION_SAMPLES",
        0,
        healthy_request_index,
    )
    command_evidence_gate_index = status_segment.rfind(
        "if(axis->commandAcknowledged||"
        "axis->motionObserved){",
        0,
        terminal_sample_index,
    )
    evidence_arrival_index = status_segment.find(
        "markLinearAxisArrived(*axis);",
        command_evidence_gate_index,
        terminal_sample_index,
    )
    require(
        0
        <= locked_request_index
        < locked_reason_index
        < healthy_request_index
        < healthy_reason_index
        and 0
        <= command_evidence_gate_index
        < evidence_arrival_index
        < terminal_sample_index
        <= terminal_threshold_index
        < healthy_request_index,
        "3A terminal arbitration does not send locked 0x07 and "
        "healthy no-evidence on-position states through bounded "
        "encoder verification",
    )

    stall_candidate_declaration_index = linear_frame_handler.find(
        "constboolstallCandidate="
    )
    stall_branch_index = linear_frame_handler.find(
        "if(stallCandidate){"
    )
    stall_increment_index = linear_frame_handler.find(
        "if(axis->stallProtectionSamples<255U){"
        "++axis->stallProtectionSamples;}",
        stall_branch_index,
    )
    confirmation_return_index = linear_frame_handler.find(
        "if(axis->stallProtectionSamples<"
        "ARM_AXIS_STALL_CONFIRMATION_SAMPLES){return;}",
        stall_branch_index,
    )
    stall_recovery_index = linear_frame_handler.find(
        "scheduleLinearAxisRecovery("
        "*axis,ARM_AXIS_RECOVERY_STALL);",
        confirmation_return_index,
    )
    healthy_reset_index = linear_frame_handler.find(
        "axis->stallProtectionSamples=0U;",
        stall_recovery_index,
    )
    require(
        0
        <= stall_candidate_declaration_index
        < stall_branch_index
        < stall_increment_index
        < confirmation_return_index
        < stall_recovery_index
        < healthy_reset_index,
        "3A stall handling does not wait for consecutive confirmation "
        "or clear the counter on a healthy frame",
    )
    require(
        "protectionLatched"
        in linear_frame_handler[
            stall_candidate_declaration_index:stall_branch_index
        ]
        and "locked"
        in linear_frame_handler[
            stall_candidate_declaration_index:stall_branch_index
        ],
        "3A stall candidate no longer includes both locked and "
        "latched-protection states",
    )
    require(
        near(values["ARM_AXIS_STALL_CONFIRMATION_SAMPLES"], 3.0),
        "3-frame stall threshold is missing",
    )
    rejected_classification_index = linear_frame_handler.find(
        "if(axis->positionCommandRejected){",
        healthy_reset_index,
    )
    rejected_recovery_index = linear_frame_handler.find(
        "ARM_AXIS_RECOVERY_COMMAND_REJECTED",
        rejected_classification_index,
    )
    require(
        0
        <= healthy_reset_index
        < rejected_classification_index
        < rejected_recovery_index,
        "a classified FD E2 does not enter its bounded rejected-command "
        "recovery path",
    )
    require(
        "EMMstallconfirmedwhileestablishingworkingzero"
        in linear_frame_handler,
        "startup stall is not separated from reference-based recovery",
    )

    position_frame_handler = compact_cpp(
        extract_function_body(
            source_without_comments,
            "handleArmLinearPositionFrame",
        )
    )
    first_position_sample_index = position_frame_handler.find(
        "if(axis->terminalPositionSamples==0U){"
    )
    averaged_position_index = position_frame_handler.find(
        "constfloataveragedPositionMm="
    )
    target_error_index = position_frame_handler.find(
        "constfloattargetErrorMm="
    )
    target_verified_index = position_frame_handler.find(
        "constbooltargetVerified="
    )
    target_branch_index = position_frame_handler.find(
        "if(targetVerified){",
        target_verified_index,
    )
    encoder_arrival_index = position_frame_handler.find(
        "markLinearAxisArrived(*axis);",
        target_branch_index,
    )
    require(
        0
        <= first_position_sample_index
        < averaged_position_index
        < target_error_index
        < target_verified_index
        < target_branch_index
        < encoder_arrival_index
        and "ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM"
        in position_frame_handler[
            target_error_index:target_branch_index
        ],
        "0x36 terminal verification is not based on two stable "
        "encoder samples at the commanded target",
    )

    protection_match = re.search(
        r"constbool([A-Za-z_]\w*)="
        r"\(axis->terminalStatusFlags&0x08U\)!=0U;",
        position_frame_handler,
        flags=re.IGNORECASE,
    )
    require(
        protection_match is not None
        and "protection" in protection_match.group(1).lower()
        and "ARM_AXIS_VERIFY_LOCKED_ON_POSITION"
        in position_frame_handler,
        "0x36 terminal verification does not distinguish a resolvable "
        "locked status from latched protection",
    )
    protection_branch_index = position_frame_handler.find(
        f"if({protection_match.group(1)}){{"
    )
    locked_reason_index = position_frame_handler.find(
        "ARM_AXIS_VERIFY_LOCKED_ON_POSITION",
        encoder_arrival_index,
    )
    protection_recovery_index = position_frame_handler.find(
        "ARM_AXIS_RECOVERY_STALL",
        protection_branch_index,
    )
    locked_failure_index = position_frame_handler.rfind(
        "noteLinearAxisTerminalVerificationFailure(",
        encoder_arrival_index,
        locked_reason_index,
    )
    require(
        0
        <= protection_branch_index
        < protection_recovery_index
        < target_branch_index
        < encoder_arrival_index
        < locked_failure_index
        < locked_reason_index,
        "0x0B/0x0F protection can be encoder-cleared, or locked "
        "0x07 is not resolved by measured target position before "
        "falling back to bounded verification failure",
    )

    terminal_service = compact_cpp(
        extract_function_body(
            source_without_comments,
            "serviceLinearAxisTerminalVerification",
        )
    )
    for fragment in (
        "axis.terminalPositionRequestSent",
        "axis.terminalPositionRequestDeadlineMs",
        "armLinearPositionQueryAxis",
        "writeArmLinearCurrentPositionRequest(axis.address);",
        "otherAxis.active",
        "armMotors.isM5Running()",
    ):
        require(
            fragment in terminal_service,
            "asynchronous 0x36 terminal service is missing "
            + fragment,
        )
    require(
        "readArmLinearCurrentMotorAngleDegrees(" not in terminal_service
        and "delay(" not in terminal_service,
        "terminal verification blocks the main service loop",
    )

    linear_axes_service = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceArmLinearAxes"
        )
    )
    require(
        "handleArmLinearPositionFrame(" in linear_axes_service
        and "serviceLinearAxisTerminalVerification("
        in linear_axes_service,
        "the 8-byte 0x36 parser/service is not wired into the "
        "non-blocking linear-axis loop",
    )

    recovery_helper = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceLinearAxisRecovery"
        )
    )
    for fragment in (
        "armMotors.isM5Running()",
        "M6_RECOVERY_SPEED_RPM",
        "M6_RECOVERY_ACCELERATION",
        "M7_RECOVERY_SPEED_RPM",
        "M7_RECOVERY_ACCELERATION",
    ):
        require(
            fragment in recovery_helper,
            "recovery is missing reduced/sequential safeguard: "
            + fragment,
        )
    recovery_scheduler = compact_cpp(
        extract_function_body(
            source_without_comments, "scheduleLinearAxisRecovery"
        )
    )
    recovery_deadline_set_index = recovery_scheduler.find(
        "axis.recoveryDeadlineMs="
        "millis()+ARM_AXIS_RECOVERY_TOTAL_TIMEOUT_MS;"
    )
    recovery_stop_index = recovery_scheduler.find(
        "writeArmLinearStop(axis.address);"
    )
    recovery_deadline_check_index = recovery_helper.find(
        "if(deadlineReached(axis.recoveryDeadlineMs)){"
    )
    recovery_ready_check_index = recovery_helper.find(
        "if(!deadlineReached(axis.recoveryReadyMs)){"
    )
    recovery_wait_m5_index = recovery_helper.find(
        "armMotors.isM5Running()"
    )
    require(
        0
        <= recovery_deadline_set_index
        < recovery_stop_index
        and 0
        <= recovery_deadline_check_index
        < recovery_ready_check_index
        < recovery_wait_m5_index,
        "recovery total deadline is not armed before stop or checked "
        "before settle/M5 waits",
    )
    retry_call_index = recovery_helper.find(
        "startLinearAxisMove(",
        recovery_wait_m5_index,
    )
    require(
        retry_call_index >= 0
        and "axis.recoveryDeadlineMs=0UL;"
        not in recovery_helper[
            recovery_wait_m5_index:retry_call_index
        ],
        "the reduced recovery retry refreshes or clears its original "
        "total deadline",
    )

    arrived_helper = compact_cpp(
        extract_function_body(
            source_without_comments, "markLinearAxisArrived"
        )
    )
    fault_axis_helper = compact_cpp(
        extract_function_body(
            source_without_comments, "faultLinearAxis"
        )
    )
    require(
        "axis.recoveryDeadlineMs=0UL;" in arrived_helper
        and "axis.recoveryDeadlineMs=0UL;" in fault_axis_helper,
        "terminal success/fault does not clear the recovery deadline",
    )

    active_recovery_deadline_guard = (
        "axis.recoveryAttemptCount>0U"
        in linear_axes_service
        and "axis.recoveryDeadlineMs!=0UL"
        in linear_axes_service
        and "deadlineReached(axis.recoveryDeadlineMs)"
        in linear_axes_service
    )
    retry_budget_clamp = (
        "recoveryRetry"
        in linear_move_helper
        and "recoveryDeadlineMs" in linear_move_helper
    )
    require(
        active_recovery_deadline_guard or retry_budget_clamp,
        "the recovery deadline stops being enforced once the reduced "
        "retry becomes an active motion",
    )
    require(
        near(values["ARM_AXIS_MAXIMUM_RECOVERY_ATTEMPTS"], 1.0),
        "a confirmed obstruction can be retried more than once",
    )

    startup_zeros = compact_cpp(
        extract_function_body(
            source_without_comments,
            "establishArmLinearSafeWorkingZeros",
        )
    )
    for fragment in (
        "establishLinearAxisSafeWorkingZero("
        "extensionAxis,\"M6\","
        "M6_STARTUP_WORKING_ZERO_OFFSET_MM,"
        "M6_STANDARD_EXTENSION_MM,"
        "M6_MAXIMUM_PHYSICAL_EXTENSION_MM,"
        "M6_PULSES_PER_MM,"
        "M6_EXTEND_DIRECTION,"
        "M6_RETRACT_DIRECTION,"
        "M6_TRAVEL_PER_REVOLUTION_MM)",
        "establishLinearAxisSafeWorkingZero("
        "liftAxis,\"M7\","
        "-M7_STARTUP_WORKING_ZERO_OFFSET_MM,"
        "M7_MINIMUM_PHYSICAL_HEIGHT_MM,"
        "M7_STANDARD_HEIGHT_MM,"
        "M7_PULSES_PER_MM,"
        "M7_RAISE_DIRECTION,"
        "M7_LOWER_DIRECTION,"
        "M7_TRAVEL_PER_REVOLUTION_MM)",
        "armLinearReferenceValid=true;",
    ):
        require(
            fragment in startup_zeros,
            "dual-axis startup safe-zero sequence is missing "
            f"{fragment}",
        )

    reset_origin = compact_cpp(
        extract_function_body(
            source_without_comments,
            "resetArmLinearSoftwareOrigin",
        )
    )
    require(
        "armLinearReferenceValid=false;" in reset_origin,
        "arm reference becomes valid before both 10 mm startup moves",
    )

    setup_body = compact_cpp(
        extract_function_body(source_without_comments, "setup")
    )
    require(
        "initializeHmi();"
        "constboolarmWorkingZerosReady="
        "establishArmLinearSafeWorkingZeros();"
        in setup_body,
        "setup does not establish M6/M7 safe working zeros before READY",
    )

    require(
        near(values["ARM_BASE_MAXIMUM_STEP_RATE"], 45_000.0)
        and near(values["ARM_BASE_STEP_ACCELERATION"], 18_000.0)
        and near(
            values["ARM_BASE_TRANSFER_MAXIMUM_STEP_RATE"],
            117_000.0,
        )
        and near(
            values["ARM_BASE_TRANSFER_STEP_ACCELERATION"],
            54_000.0,
        )
        and near(values["ARM_BASE_MOTION_TIMEOUT_MS"], 4_500.0),
        "M5 endpoint/transfer profiles or local timeout changed "
        "unexpectedly",
    )
    start_arm_base = compact_cpp(
        extract_function_body(
            source_without_comments, "startArmBaseLibraryDegrees"
        )
    )
    arm_base_command_index = start_arm_base.find(
        "armMotors.moveM5ToDegrees(libraryDegrees,false);"
    )
    arm_base_watchdog_arm_index = start_arm_base.find(
        "armBaseMotionWatchdogActive=true;"
    )
    arm_base_deadline_index = start_arm_base.find(
        "armBaseMotionDeadlineMs="
        "millis()+ARM_BASE_MOTION_TIMEOUT_MS;"
    )
    require(
        0
        <= arm_base_command_index
        < arm_base_watchdog_arm_index
        < arm_base_deadline_index,
        "each M5 command does not arm its local 4.5 s watchdog",
    )

    arm_base_watchdog = compact_cpp(
        extract_function_body(
            source_without_comments,
            "serviceArmBaseMotionWatchdog",
        )
    )
    arm_base_timeout_check_index = arm_base_watchdog.find(
        "deadlineReached(armBaseMotionDeadlineMs)"
    )
    arm_base_timeout_stop_index = arm_base_watchdog.find(
        "armMotors.stopM5Immediately();",
        arm_base_timeout_check_index,
    )
    arm_base_timeout_fault_index = arm_base_watchdog.find(
        'routeFault("M5motiontimeout");',
        arm_base_timeout_stop_index,
    )
    require(
        0
        <= arm_base_timeout_check_index
        < arm_base_timeout_stop_index
        < arm_base_timeout_fault_index
        and "!armMotors.isM5Running()" in arm_base_watchdog,
        "M5 watchdog does not distinguish normal completion from a "
        "timed-out motion and stop before faulting",
    )
    stop_arm_base = compact_cpp(
        extract_function_body(
            source_without_comments, "stopArmBaseImmediately"
        )
    )
    disable_arm_base = compact_cpp(
        extract_function_body(
            source_without_comments, "disableArmBaseMotor"
        )
    )
    loop_service = compact_cpp(
        extract_function_body(source_without_comments, "loop")
    )
    require(
        "armBaseMotionWatchdogActive=false;" in stop_arm_base
        and "armBaseMotionDeadlineMs=0UL;" in stop_arm_base
        and "armMotors.stopM5Immediately();" in stop_arm_base
        and "stopArmBaseImmediately();" in disable_arm_base
        and "armMotors.serviceM5();"
        "serviceArmBaseMotionWatchdog();" in loop_service,
        "M5 stop/disable/loop paths do not consistently maintain the "
        "local watchdog",
    )

    begin_transfer = compact_cpp(
        extract_function_body(
            source_without_comments, "beginArmTransfer"
        )
    )
    endpoint_initializer = compact_cpp(
        extract_function_body(
            source_without_comments, "initializeEndpointScanState"
        )
    )
    require(
        "useArmBaseTransferMotionProfile();" in begin_transfer
        and "useArmBaseEndpointTravelMotionProfile();"
        in endpoint_initializer,
        "M5 transfer and endpoint phases do not select their separate "
        "motion profiles",
    )

    expected_slow_place_values = {
        "RING_PLACE_FINAL_DESCENT_MM": 10.0,
        "M7_RING_PLACE_SPEED_RPM": 1050.0,
        "M7_RING_PLACE_ACCELERATION": 188.0,
        "RING_PLACE_EXTENSION_SETTLE_MS": 30.0,
        "RING_PLACE_LOWER_SETTLE_MS": 40.0,
    }
    for name, expected in expected_slow_place_values.items():
        require(
            near(values[name], expected),
            f"{name}={values[name]}, expected {expected}",
        )
    require(
        values["M7_RING_PLACE_SPEED_RPM"]
        < values["M7_SPEED_RPM"],
        "final ring descent is not slower than normal M7 motion",
    )
    require(
        near(values["RAW_PICK_LOWER_MM"], 63.0)
        and near(values["HOUGH_VISION_LOWER_MM"], 80.0)
        and near(values["ENDPOINT_FINE_VISION_LOWER_MM"], 95.0)
        and near(values["CONTAINER_PICK_LOWER_MM"], 28.0)
        and near(values["CONTAINER_PLACE_LOWER_MM"], 30.0)
        and near(values["PROCESS_PLACE_LOWER_MM"], 138.0)
        and near(
            values["STORAGE_ROUND1_PLACE_LOWER_MM"],
            138.0,
        )
        and near(values["STORAGE_ROUND2_PLACE_LOWER_MM"], 80.0),
        "M7 logical heights do not preserve the original physical "
        "heights after the 10 mm startup offset",
    )
    require(
        near(
            values["ENDPOINT_COARSE_CENTER_TOLERANCE_PIXELS"],
            6.0,
        )
        and near(
            values["ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS"],
            5.0,
        )
        and near(
            values["ENDPOINT_FINAL_CENTER_CONFIRMATIONS"],
            2.0,
        )
        and near(
            values["ENDPOINT_MAXIMUM_SERVO_MOVES_PER_STAGE"],
            5.0,
        )
        and near(
            values["ENDPOINT_MAXIMUM_TOTAL_SERVO_MOVES"],
            7.0,
        )
        and near(values["ENDPOINT_COARSE_SERVO_GAIN"], 0.85)
        and near(values["ENDPOINT_FINE_SERVO_GAIN"], 0.55)
        and near(
            values["ENDPOINT_COARSE_MAXIMUM_CORRECTION_MM"],
            25.0,
        )
        and near(
            values["ENDPOINT_FINE_MAXIMUM_CORRECTION_MM"],
            6.0,
        )
        and near(
            values["ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES"],
            45.0,
        )
        and near(
            values["ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES"],
            -45.0,
        )
        and near(
            values["ENDPOINT_SEARCH_SEED_EXTENSION_MM"],
            90.0,
        )
        and near(
            values[
                "ENDPOINT_RING1_SEARCH_FALLBACK_STEP_DEGREES"
            ],
            15.0,
        )
        and near(
            values[
                "ENDPOINT_RING3_SEARCH_FALLBACK_STEP_DEGREES"
            ],
            15.0,
        )
        and near(
            values[
                "ENDPOINT_RING1_SEARCH_MINIMUM_ANGLE_DEGREES"
            ],
            30.0,
        )
        and near(
            values[
                "ENDPOINT_RING1_SEARCH_MAXIMUM_ANGLE_DEGREES"
            ],
            60.0,
        )
        and near(
            values[
                "ENDPOINT_RING3_SEARCH_MINIMUM_ANGLE_DEGREES"
            ],
            -60.0,
        )
        and near(
            values[
                "ENDPOINT_RING3_SEARCH_MAXIMUM_ANGLE_DEGREES"
            ],
            -30.0,
        )
        and near(
            values["ENDPOINT_SEARCH_MAXIMUM_FALLBACK_MOVES"],
            2.0,
        ),
        "endpoint local-servo limits changed unexpectedly",
    )
    require(
        near(values["ENDPOINT_VISION_RESULT_TIMEOUT_MS"], 2_500.0)
        and near(values["ENDPOINT_VISION_MAXIMUM_RETRIES"], 1.0),
        "endpoint vision timeout changed unexpectedly",
    )

    begin_request = compact_cpp(
        extract_function_body(
            source_without_comments, "beginMaixRequest"
        )
    )
    require(
        "request!=MAIXCAM_ENDPOINT_CIRCLE_REQUEST"
        in begin_request,
        "mode 10 is not admitted by beginMaixRequest()",
    )

    endpoint_vision = compact_cpp(
        extract_function_body(
            source_without_comments, "beginEndpointVision"
        )
    )
    require(
        "beginMaixRequest(MAIXCAM_ENDPOINT_CIRCLE_REQUEST);"
        in endpoint_vision
        and "workActionPhase="
        "WORK_PHASE_ENDPOINT_WAIT_COORDINATE;"
        in endpoint_vision,
        "endpoint vision does not request mode 10 and enter its wait phase",
    )

    endpoint_transform = compact_cpp(
        extract_function_body(
            source_without_comments,
            "endpointVisionToPlanarPoint",
        )
    )
    for fragment in (
        "constfloatmmPerPixel="
        "RING_PHYSICAL_RADIUS_MM/"
        "static_cast<float>(radiusPixels);",
        "constfloatlocalRadialMm="
        "ARM_PIVOT_TO_CAMERA_CENTER_MM+"
        "scanPose.extensionMm-pixelDeltaY*mmPerPixel;",
        "constfloatlocalLeftMm=-pixelDeltaX*mmPerPixel;",
        "point.outwardMm=localRadialMm*cosine-"
        "localLeftMm*sine;",
        "point.leftMm=localRadialMm*sine+"
        "localLeftMm*cosine;",
    ):
        require(
            fragment in endpoint_transform,
            "endpoint image-to-arm transform is missing "
            f"{fragment}",
        )

    def endpoint_point(
        scan_angle_degrees: float,
        scan_extension_mm: float,
        image_x: float,
        image_y: float,
        radius_pixels: float,
    ) -> Tuple[float, float]:
        scale = 41.75 / radius_pixels
        local_radial = (
            pivot_to_camera_mm
            + scan_extension_mm
            - (image_y - 120.0) * scale
        )
        local_left = -(image_x - 160.0) * scale
        angle = math.radians(scan_angle_degrees)
        return (
            local_radial * math.cos(angle)
            - local_left * math.sin(angle),
            local_radial * math.sin(angle)
            + local_left * math.cos(angle),
        )

    nominal_angle = math.degrees(
        math.atan2(150.0, pivot_to_camera_mm)
    )
    nominal_extension = (
        math.hypot(pivot_to_camera_mm, 150.0)
        - pivot_to_camera_mm
    )
    nominal_ring_1 = endpoint_point(
        nominal_angle,
        nominal_extension,
        160.0,
        120.0,
        72.0,
    )
    nominal_ring_3 = endpoint_point(
        -nominal_angle,
        nominal_extension,
        160.0,
        120.0,
        72.0,
    )
    require(
        near(nominal_ring_1[0], pivot_to_camera_mm)
        and near(nominal_ring_1[1], 150.0)
        and near(nominal_ring_3[0], pivot_to_camera_mm)
        and near(nominal_ring_3[1], -150.0),
        "endpoint transform does not reconstruct nominal ring 1/3",
    )
    midpoint = (
        0.5 * (nominal_ring_1[0] + nominal_ring_3[0]),
        0.5 * (nominal_ring_1[1] + nominal_ring_3[1]),
    )
    require(
        near(midpoint[0], pivot_to_camera_mm)
        and near(midpoint[1], 0.0)
        and near(nominal_angle, 47.8570047, 1.0e-5)
        and near(nominal_extension, 66.5601424, 1.0e-5),
        "nominal endpoint midpoint or M5/M6 inverse geometry changed",
    )

    inverse_kinematics = compact_cpp(
        extract_function_body(
            source_without_comments, "planarPointToRingPose"
        )
    )
    require(
        "hypotf(point.outwardMm,point.leftMm)"
        in inverse_kinematics
        and "atan2f(point.leftMm,point.outwardMm)"
        in inverse_kinematics
        and "ARM_PIVOT_TO_GRIPPER_CENTER_MM"
        in inverse_kinematics,
        "dynamic ring inverse kinematics is not atan2/hypot based",
    )

    camera_inverse_kinematics = compact_cpp(
        extract_function_body(
            source_without_comments,
            "planarPointToCameraPose",
        )
    )
    for fragment in (
        "hypotf(point.outwardMm,point.leftMm)",
        "atan2f(point.leftMm,point.outwardMm)",
        "ARM_PIVOT_TO_CAMERA_CENTER_MM",
        "RING_SCAN_MINIMUM_ANGLE_DEGREES",
        "RING_SCAN_MAXIMUM_ANGLE_DEGREES",
    ):
        require(
            fragment in camera_inverse_kinematics,
            "camera-centering inverse kinematics is missing "
            f"{fragment}",
        )

    centering_pose = compact_cpp(
        extract_function_body(
            source_without_comments,
            "endpointServoTargetPose",
        )
    )
    for fragment in (
        "fineStage?ENDPOINT_FINE_SERVO_GAIN:"
        "ENDPOINT_COARSE_SERVO_GAIN",
        "fineStage?ENDPOINT_FINE_MAXIMUM_CORRECTION_MM:"
        "ENDPOINT_COARSE_MAXIMUM_CORRECTION_MM",
        "if(commandedCorrectionMm>maximumCorrectionMm)",
        "planarPointToCameraPose("
        "requestedCameraPoint,-currentPose.heightMm,"
        "targetPose)",
    ):
        require(
            fragment in centering_pose,
            "bounded endpoint centering calculation is missing "
            f"{fragment}",
        )

    map_builder = compact_cpp(
        extract_function_body(
            source_without_comments, "buildMeasuredRingMap"
        )
    )
    for fragment in (
        "if(!measuredRingPointValid[1U]||"
        "!measuredRingPointValid[3U])",
        "constfloatmidpointOutwardMm="
        "0.5f*(measuredRingPoints[1U].outwardMm+"
        "measuredRingPoints[3U].outwardMm);",
        "constfloatmidpointLeftMm="
        "0.5f*(measuredRingPoints[1U].leftMm+"
        "measuredRingPoints[3U].leftMm);",
        "measuredRingPoints[2U].outwardMm="
        "midpointOutwardMm;",
        "measuredRingPoints[2U].leftMm=midpointLeftMm;",
        "planarPointToRingPose("
        "ring,measuredRingPoints[ring],0.0f,pose)",
        "measuredRingPoseValid[ring]=true;",
    ):
        require(
            fragment in map_builder,
            "endpoint map construction is missing "
            f"{fragment}",
        )
    for forbidden in (
        "RING_MAP_MAXIMUM_MIDPOINT_ERROR_MM",
        "RING_ENDPOINT_MAXIMUM_AXIS_TILT_DEGREES",
        "RING_ENDPOINT_MINIMUM_SIDE_SEPARATION_MM",
        "ARM_PIVOT_TO_CAMERA_CENTER_MM",
    ):
        require(
            forbidden not in map_builder,
            "endpoint map still assumes nominal parking geometry: "
            f"{forbidden}",
        )

    dynamic_ring_pose = compact_cpp(
        extract_function_body(
            source_without_comments, "ringPose"
        )
    )
    require(
        "if(!measuredRingPoseValid[ringPosition])"
        in dynamic_ring_pose
        and "pose=measuredRingPoses[ringPosition];"
        in dynamic_ring_pose,
        "ringPose() does not consume the measured per-ring map",
    )
    require(
        "nominalRingPose(" not in dynamic_ring_pose,
        "ringPose() silently falls back to the old fixed ±150 mm model",
    )

    endpoint_scan = compact_cpp(
        extract_function_body(
            source_without_comments, "beginEndpointScan"
        )
    )
    require(
        "nominalRingPose("
        "ringPosition,HOUGH_VISION_LOWER_MM,searchPose)"
        in endpoint_scan
        and "initializeEndpointScanState("
        "ringPosition,searchPose);"
        in endpoint_scan
        and "startEndpointSearchPlanarMove()"
        in endpoint_scan,
        "direct endpoint seed initialization is incomplete",
    )
    endpoint_planar_start = compact_cpp(
        extract_function_body(
            source_without_comments,
            "startEndpointSearchPlanarMove",
        )
    )
    require(
        "startArmBaseStandardFrameDegrees("
        "activeEndpointScanPose.standardFrameAngleDegrees);"
        in endpoint_planar_start
        and "startExtensionToMm" not in endpoint_planar_start,
        "endpoint seed must start M5 alone before the state machine "
        "starts M6",
    )
    nominal_endpoint_pose = compact_cpp(
        extract_function_body(
            source_without_comments, "nominalRingPose"
        )
    )
    require(
        "ringPosition==1U?"
        "ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES:"
        "ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES"
        in nominal_endpoint_pose
        and "pose.extensionMm=ENDPOINT_SEARCH_SEED_EXTENSION_MM;"
        in nominal_endpoint_pose,
        "ring 1/3 initial views are not +45/-20 degrees at the "
        "90 mm seed",
    )
    predicted_ring3_pose = compact_cpp(
        extract_function_body(
            source_without_comments, "predictedRing3SearchPose"
        )
    )
    require(
        "pose.standardFrameAngleDegrees="
        "ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES;"
        in predicted_ring3_pose,
        "ring-3 handoff does not start at the required -20 degree view",
    )
    direct_ring3_scan = compact_cpp(
        extract_function_body(
            source_without_comments,
            "beginDirectRing3EndpointScan",
        )
    )
    require(
        "returnstartEndpointSearchPlanarMove();"
        in direct_ring3_scan
        and "startExtensionToMm" not in direct_ring3_scan,
        "ring-3 handoff bypasses the shared M5-first sequence",
    )
    fallback_helper = compact_cpp(
        extract_function_body(
            source_without_comments,
            "startEndpointSearchFallbackMove",
        )
    )
    for fragment in (
        "constboolscanningRing1="
        "activeEndpointScanRing==1U;",
        "scanningRing1?"
        "ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES:"
        "ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES",
        "scanningRing1?"
        "ENDPOINT_RING1_SEARCH_FALLBACK_STEP_DEGREES:"
        "ENDPOINT_RING3_SEARCH_FALLBACK_STEP_DEGREES",
        "scanningRing1?"
        "ENDPOINT_RING1_SEARCH_MINIMUM_ANGLE_DEGREES:"
        "ENDPOINT_RING3_SEARCH_MINIMUM_ANGLE_DEGREES",
        "scanningRing1?"
        "ENDPOINT_RING1_SEARCH_MAXIMUM_ANGLE_DEGREES:"
        "ENDPOINT_RING3_SEARCH_MAXIMUM_ANGLE_DEGREES",
        "seedAngleDegrees+fallbackOffsetDirection*"
        "fallbackStepDegrees",
    ):
        require(
            fragment in fallback_helper,
            "ring-specific endpoint fallback is missing "
            + fragment,
        )
    require(
        "endpointSearchBaseAngleDegrees+" not in fallback_helper,
        "fallback still offsets an arbitrary predicted angle",
    )
    ring1_fallback_angles = [
        values["ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES"]
        + values[
            "ENDPOINT_RING1_SEARCH_FALLBACK_STEP_DEGREES"
        ],
        values["ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES"]
        - values[
            "ENDPOINT_RING1_SEARCH_FALLBACK_STEP_DEGREES"
        ],
    ]
    ring3_fallback_angles = [
        values["ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES"]
        - values[
            "ENDPOINT_RING3_SEARCH_FALLBACK_STEP_DEGREES"
        ],
        values["ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES"]
        + values[
            "ENDPOINT_RING3_SEARCH_FALLBACK_STEP_DEGREES"
        ],
    ]
    require(
        ring1_fallback_angles == [60.0, 30.0]
        and ring3_fallback_angles == [-60.0, -30.0],
        "fallback candidates are not ring1 +60/+30 and "
        "ring3 -60/-30",
    )
    endpoint_initializer = compact_cpp(
        extract_function_body(
            source_without_comments,
            "initializeEndpointScanState",
        )
    )
    for fragment in (
        "endpointFineVisionActive=false;",
        "activeEndpointServoMoveCount=0U;",
        "activeEndpointStageServoMoveCount=0U;",
        "endpointSearchFallbackMoveCount=0U;",
        "endpointSearchBaseAngleDegrees="
        "searchPose.standardFrameAngleDegrees;",
    ):
        require(
            fragment in endpoint_initializer,
            "endpoint scan state is missing " + fragment,
        )
    require(
        "RING_SCAN_PRELOAD_ANGLE_DEGREES"
        not in source_without_comments,
        "obsolete -80 degree endpoint preload still exists",
    )
    require(
        "beginEndpointPoseApproach" not in source_without_comments,
        "rejected full-home endpoint retry helper still exists",
    )

    action = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceCompetitionAction"
        )
    )
    required_action_fragments = (
        "workActionPhase=="
        "WORK_PHASE_ENDPOINT_WAIT_COORDINATE",
        "beginEndpointVision();",
        "startPreEndpointHeadingCorrection();",
        "caseWORK_PHASE_ENDPOINT_WAIT_PRE_SCAN_HEADING:",
        "endpointMapLockedHeadingDegrees="
        "currentRouteCounterClockwiseHeading();",
        "beginEndpointScan(1U);",
        "caseWORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE:",
        "caseWORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE_SETTLE:",
        "caseWORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION:",
        "caseWORK_PHASE_ENDPOINT_WAIT_ARM_LOWER:",
        "caseWORK_PHASE_ENDPOINT_WAIT_COORDINATE:",
        "endpointVisionToPlanarPoint("
        "activeEndpointScanPose,x,y,radiusPixels,measuredPoint)",
        "if(centerErrorPixels>centerTolerancePixels)",
        "activeEndpointStageServoMoveCount>="
        "ENDPOINT_MAXIMUM_SERVO_MOVES_PER_STAGE",
        "activeEndpointServoMoveCount>="
        "ENDPOINT_MAXIMUM_TOTAL_SERVO_MOVES",
        "endpointServoTargetPose("
        "activeEndpointScanPose,measuredPoint,"
        "endpointFineVisionActive,targetPose,"
        "measuredCorrectionMm,commandedCorrectionMm)",
        "ENDPOINT_FINE_M6_SPEED_RPM",
        "ENDPOINT_COARSE_M6_SPEED_RPM",
        "ENDPOINT_FINE_M6_ACCELERATION",
        "ENDPOINT_COARSE_M6_ACCELERATION",
        "startArmBaseStandardFrameDegrees("
        "targetPose.standardFrameAngleDegrees);",
        "workActionPhase="
        "WORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE;",
        "if(!endpointFineVisionActive){",
        "activeEndpointStageServoMoveCount=0U;",
        "startLiftToHeightMm("
        "activeEndpointScanPose.heightMm)",
        "ENDPOINT_FINAL_CENTER_CONFIRMATIONS",
        "measuredRingPointValid[ring]=true;",
        "caseWORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE:",
        "caseWORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE_SETTLE:",
        "caseWORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION:",
        "caseWORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE:",
        "caseWORK_PHASE_ENDPOINT_WAIT_FINE_LOWER:",
        "caseWORK_PHASE_ENDPOINT_WAIT_RING3_RAISE:",
        "beginDirectRing3EndpointScan();",
        "workActionPhase=WORK_PHASE_START_UNLOAD;",
    )
    for fragment in required_action_fragments:
        require(
            fragment in action,
            "endpoint state machine is missing "
            f"{fragment}",
        )

    endpoint_phase_index = action.find(
        "constboolendpointVisionPhase="
        "workActionPhase=="
        "WORK_PHASE_ENDPOINT_WAIT_COORDINATE;"
    )
    retry_limit_index = action.find(
        "constuint8_tvisionRetryLimit="
        "endpointVisionPhase?"
        "ENDPOINT_VISION_MAXIMUM_RETRIES:"
        "VISION_MAXIMUM_RETRIES;",
        endpoint_phase_index,
    )
    retry_guard_index = action.find(
        "if(workVisionRetryCount>=visionRetryLimit){",
        retry_limit_index,
    )
    retry_fault_index = action.find(
        'routeFault("Visionresulttimeout");',
        retry_guard_index,
    )
    require(
        0
        <= endpoint_phase_index
        < retry_limit_index
        < retry_guard_index
        < retry_fault_index,
        "endpoint mode does not use its dedicated one-retry limit "
        "while other vision modes retain their general retry limit",
    )

    centering_gate_index = action.find(
        "if(centerErrorPixels>centerTolerancePixels)"
    )
    tolerance_binding_index = action.find(
        "constfloatcenterTolerancePixels="
        "endpointFineVisionActive?"
        "ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS:"
        "ENDPOINT_COARSE_CENTER_TOLERANCE_PIXELS;"
    )
    centering_command_index = action.find(
        "workActionPhase="
        "WORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE;",
        centering_gate_index,
    )
    accepted_index = action.find(
        "measuredRingPointValid[ring]=true;",
        centering_gate_index,
    )
    require(
        0
        <= centering_gate_index
        < centering_command_index
        < accepted_index,
        "endpoint coordinates can be accepted before the center gate",
    )
    require(
        0
        <= tolerance_binding_index
        < centering_gate_index,
        "the 5 px fine tolerance is not selected by the live center gate",
    )
    coordinate_case_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_COORDINATE:"
    )
    confidence_gate_index = action.find(
        "if(confidence<RING_ENDPOINT_MINIMUM_CONFIDENCE)",
        coordinate_case_index,
    )
    endpoint_geometry_index = action.find(
        "endpointVisionToPlanarPoint(",
        confidence_gate_index,
    )
    valid_frame_renewal_index = action.find(
        "workVisionRequestStartMs=millis();",
        endpoint_geometry_index,
    )
    require(
        0
        <= coordinate_case_index
        < confidence_gate_index
        < endpoint_geometry_index
        < valid_frame_renewal_index
        < centering_gate_index
        and "workVisionRequestStartMs=millis();"
        not in action[
            coordinate_case_index:confidence_gate_index
        ],
        "invalid endpoint frames can renew the 2.5 s search deadline",
    )
    local_servo_segment = action[
        centering_gate_index:accepted_index
    ]
    require(
        "beginArmStandardization();" not in local_servo_segment
        and "RING_SCAN_PRELOAD_ANGLE_DEGREES"
        not in local_servo_segment,
        "local endpoint correction still performs a full home/preload cycle",
    )

    first_scan_index = action.find("beginEndpointScan(1U);")
    third_scan_index = action.find("beginDirectRing3EndpointScan();")
    map_index = action.find(
        "completeEndpointMapAndStartTransfers();"
    )
    require(
        0
        <= first_scan_index
        < third_scan_index
        < map_index,
        "normal endpoint sequence is not 1 -> 3 -> map -> unload",
    )
    complete_map = compact_cpp(
        extract_function_body(
            source_without_comments,
            "completeEndpointMapAndStartTransfers",
        )
    )
    require(
        "if(!buildMeasuredRingMap())" in complete_map
        and "workActionPhase=WORK_PHASE_START_UNLOAD;"
        in complete_map,
        "endpoint completion does not build the map before unload",
    )
    require(
        action.count("beginEndpointScan(1U);") == 1
        and action.count("beginDirectRing3EndpointScan();") == 1,
        "endpoint state machine must schedule each endpoint exactly once",
    )

    preload_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE:"
    )
    coordinate_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_COORDINATE:"
    )
    old_circle_index = action.find(
        "caseWORK_PHASE_CIRCLE_WAIT_ARM_LOWER:"
    )
    require(
        0 <= preload_index < coordinate_index < old_circle_index,
        "endpoint and legacy circle phases cannot be isolated",
    )
    search_base_settle_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE_SETTLE:"
    )
    search_extension_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION:"
    )
    search_lower_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_ARM_LOWER:"
    )
    require(
        0
        <= preload_index
        < search_base_settle_index
        < search_extension_index
        < search_lower_index
        < coordinate_index,
        "endpoint seed is not ordered M5 -> settle -> M6 -> M7 -> vision",
    )
    search_base_segment = action[
        preload_index:search_base_settle_index
    ]
    search_base_settle_segment = action[
        search_base_settle_index:search_extension_index
    ]
    search_extension_segment = action[
        search_extension_index:search_lower_index
    ]
    require(
        "!armMotors.isM5Running()" in search_base_segment
        and "startExtensionToMm" not in search_base_segment
        and "startExtensionToMmWithProfile("
        in search_base_settle_segment
        and "extensionMoveFinished()" in search_extension_segment
        and "startLiftToHeightMm(" in search_extension_segment,
        "endpoint seed case bodies do not enforce M5 settle before M6/M7",
    )
    local_base_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE:"
    )
    local_base_settle_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE_SETTLE:"
    )
    local_extension_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION:"
    )
    local_settle_index = action.find(
        "caseWORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE:"
    )
    require(
        0
        <= local_base_index
        < local_base_settle_index
        < local_extension_index
        < local_settle_index,
        "endpoint local correction is not ordered M5 -> settle -> M6",
    )
    require(
        "!armMotors.isM5Running()"
        in action[local_base_index:local_base_settle_index]
        and "startExtensionToMm" not in action[
            local_base_index:local_base_settle_index
        ]
        and "startExtensionToMmWithProfile("
        in action[local_base_settle_index:local_extension_index]
        and "extensionMoveFinished()"
        in action[local_extension_index:local_settle_index],
        "endpoint local case bodies can still overlap M5 and M6",
    )
    frozen_endpoint_segment = action[
        preload_index:old_circle_index
    ]
    for forbidden in (
        "startVisualCorrection(",
        "startAccumulatedWorkstationMove(",
        "startPostVisionHeadingCorrection(",
        "startBodyDisplacement(",
        "startRelativeMotorMove(",
        "beginCircleVision(",
        "updateHeadingLock(",
    ):
        require(
            forbidden not in frozen_endpoint_segment,
            "chassis/legacy command appears after pre-scan lock: "
            f"{forbidden}",
        )

    transfer_bodies = {}
    for function_name in (
        "beginUnloadingTransfer",
        "beginReloadingTransfer",
    ):
        transfer_start = compact_cpp(
            extract_function_body(
                source_without_comments, function_name
            )
        )
        transfer_bodies[function_name] = transfer_start
        require(
            transfer_start.startswith(
                "if(!ringMapHeadingStillValid()){return;}"
            ),
            f"{function_name}() bypasses the post-map IMU guard",
        )
        require(
            "nominalRingPose(" not in transfer_start,
            f"{function_name}() uses nominal instead of measured rings",
        )
    require(
        "beginArmTransfer("
        "containerPose(CONTAINER_PICK_LOWER_MM),destination,"
        "true,false,true);"
        in transfer_bodies["beginUnloadingTransfer"],
        "unloading is not marked as mildly slowed map-based destination",
    )
    require(
        "beginArmTransfer("
        "source,containerPose(CONTAINER_PLACE_LOWER_MM),"
        "false,true,false);"
        in transfer_bodies["beginReloadingTransfer"],
        "reloading is not marked as map-based source",
    )

    map_guard = compact_cpp(
        extract_function_body(
            source_without_comments,
            "ringMapHeadingStillValid",
        )
    )
    require(
        "if(!imuIsFresh())" in map_guard
        and "RING_MAP_MAXIMUM_HEADING_DRIFT_DEGREES"
        in map_guard
        and 'routeFault("IMUstalewhileusingendpointmap");'
        in map_guard
        and "[RINGMAP]headingdriftwarning;continue="
        in map_guard
        and "Chassismovedafterendpointmapping" not in map_guard,
        "ring-map guard must retain IMU checks without drift hard-stop",
    )
    for forbidden in (
        "startBodyDisplacement(",
        "startRelativeMotorMove(",
        "startHeadingCorrection(",
    ):
        require(
            forbidden not in map_guard,
            "ring-map guard must not move the chassis",
        )

    descent = compact_cpp(
        extract_function_body(
            source_without_comments,
            "startArmTransferDestinationDescent",
        )
    )
    transfer_service = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceArmTransfer"
        )
    )
    require(
        "armTransferDestinationPose.heightMm+"
        "RING_PLACE_FINAL_DESCENT_MM"
        in descent
        and "ARM_TRANSFER_WAIT_DESTINATION_APPROACH"
        in descent,
        "ring placement does not split out the final slow descent",
    )
    require(
        "startLiftToHeightMmWithProfile("
        "armTransferDestinationPose.heightMm,"
        "M7_RING_PLACE_SPEED_RPM,"
        "M7_RING_PLACE_ACCELERATION);"
        in transfer_service
        and "RING_PLACE_EXTENSION_SETTLE_MS"
        in transfer_service
        and "RING_PLACE_LOWER_SETTLE_MS"
        in transfer_service,
        "ring placement slow profile or settle windows are missing",
    )
    require(
        "if(armTransferMapSource&&"
        "!ringMapHeadingStillValid()){return;}"
        in transfer_service
        and "if(armTransferMapDestination&&"
        "!ringMapHeadingStillValid()){return;}"
        in transfer_service,
        "map IMU state is not rechecked immediately before ring descent",
    )

    test_task = compact_cpp(
        extract_function_body(
            source_without_comments,
            "configureVisionYanyanTask",
        )
    )
    require(
        'constexprcharTEST_TASK_CODE[]="123+123+123+123";'
        in test_task,
        "placement test is not fixed to tray 0/1/2 -> ring 1/2/3",
    )
    begin_route = compact_cpp(
        extract_function_body(
            source_without_comments, "beginRoute"
        )
    )
    require(
        "if(VISION_YANYAN_TEST_MODE){"
        "visionYanyanPlacementSequenceComplete=false;"
        in begin_route
        and "beginWorkAction(WORK_ACTION_PROCESS,1U);"
        in begin_route,
        "placement-test entry does not directly start process round 1",
    )
    transfer_completion = compact_cpp(
        extract_function_body(
            source_without_comments,
            "completeTransferAndRotateStorage",
        )
    )
    test_completion_index = transfer_completion.find(
        "if(VISION_YANYAN_TEST_MODE&&"
    )
    normal_reload_index = transfer_completion.find(
        "constboolfinishedFinalContainerSequence="
    )
    require(
        0 <= test_completion_index < normal_reload_index,
        "placement-test completion is not handled before normal reload",
    )
    test_completion = transfer_completion[
        test_completion_index:normal_reload_index
    ]
    require(
        "visionYanyanPlacementSequenceComplete=true;"
        in test_completion
        and "beginStorageParkingBeforeWorkFinish();"
        in test_completion
        and "return;" in test_completion
        and "WORK_PHASE_START_RELOAD"
        not in test_completion,
        "placement test does not stop after three ring releases",
    )
    finish_action = compact_cpp(
        extract_function_body(
            source_without_comments, "finishActiveWorkAction"
        )
    )
    measure_index = finish_action.find(
        'hmiSetRunStatus("MEASURE");'
    )
    require(
        0
        <= finish_action.find("disableDriveMotors();")
        < measure_index
        and 0
        <= finish_action.find("disableArmBaseMotor();")
        < measure_index,
        "placement test does not disable chassis/M5 before MEASURE",
    )

    print(
        "PASS endpoint mapping: ring 1 +45/+60/+30 deg, ring 3 "
        "-20/-30/-10 deg + sequential M5/M6 servo, high-level ring "
        "1 -> 3 handoff, lower fine view, midpoint ring 2, frozen "
        "chassis, physical 155 mm with mild final 15 mm"
    )

def verify_safety_contract(
    source_without_comments: str,
    values: Dict[str, float],
    boolean_values: Dict[str, bool],
) -> None:
    expected_timeouts = {
        "QR_SCAN_ACTION_TIMEOUT_MS": 20_000.0,
        "VISION_RESULT_TIMEOUT_MS": 12_000.0,
        "VISION_MAXIMUM_RETRIES": 2.0,
        "RAW_ACTION_TIMEOUT_MS": 45_000.0,
        "PROCESS_ACTION_TIMEOUT_MS": 65_000.0,
        "STORAGE_ACTION_TIMEOUT_MS": 45_000.0,
        "MISSION_PROGRESS_TIMEOUT_MS": 45_000.0,
        "COMPETITION_TIME_LIMIT_MS": 180_000.0,
        "COMPETITION_HARD_STOP_MARGIN_MS": 1_500.0,
    }
    for name, expected in expected_timeouts.items():
        require(
            near(values[name], expected),
            f"{name}={values[name]}, expected {expected}",
        )

    expected_booleans = {
        "ENABLE_MOTION_TIMEOUTS": True,
        "ENABLE_COMPETITION_TIME_LIMIT": False,
        "ENABLE_QR_RECEIVER": True,
        "REQUIRE_QR_SUCCESS": True,
        "REQUIRE_RAW_PICK_QR_ORDER": False,
        "PATH_ONLY_TEST": False,
    }
    for name, expected in expected_booleans.items():
        require(name in boolean_values, f"boolean {name} was not parsed")
        require(
            boolean_values[name] is expected,
            f"{name}={boolean_values[name]}, expected {expected}",
        )

    heading_lock = compact_cpp(
        extract_function_body(
            source_without_comments, "updateHeadingLock"
        )
    )
    require(
        "ENABLE_MOTION_TIMEOUTS&&"
        "millis()-commandStartMs>=timeoutMs" in heading_lock,
        "motion timeout flag is not used by heading/motion completion",
    )

    qr_action = compact_cpp(
        extract_function_body(
            source_without_comments, "updateQrScanAction"
        )
    )
    require(
        "if(REQUIRE_QR_SUCCESS&&!scanFlag&&"
        "(qrScanPhase==QR_SCAN_FORWARD||"
        "qrScanPhase==QR_SCAN_WAIT_AT_LIMIT)&&"
        "qrScanActionStartMs!=0UL&&"
        "millis()-qrScanActionStartMs>="
        "QR_SCAN_ACTION_TIMEOUT_MS)" in qr_action
        and 'routeFault("QRscanactiontimeout");' in qr_action,
        "QR timeout must only guard an unsuccessful forward/wait scan",
    )
    qr_timeout_fault_index = qr_action.find(
        'routeFault("QRscanactiontimeout");'
    )
    require(
        "QR_SCAN_RETURNING" not in qr_action[:qr_timeout_fault_index],
        "QR return-to-origin is incorrectly covered by scan timeout",
    )

    timeout_selector = compact_cpp(
        extract_function_body(
            source_without_comments, "activeWorkActionTimeoutMs"
        )
    )
    for action_name, timeout_name in (
        ("WORK_ACTION_RAW", "RAW_ACTION_TIMEOUT_MS"),
        ("WORK_ACTION_PROCESS", "PROCESS_ACTION_TIMEOUT_MS"),
        ("WORK_ACTION_STORAGE", "STORAGE_ACTION_TIMEOUT_MS"),
    ):
        require(
            f"case{action_name}:return{timeout_name};"
            in timeout_selector,
            f"{action_name} does not select {timeout_name}",
        )

    competition_action = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceCompetitionAction"
        )
    )
    required_action_fragments = (
        "actionTimeoutMs=activeWorkActionTimeoutMs();",
        "nowMs-workActionStartMs>=actionTimeoutMs",
        'routeFault("Workstationactiontimeout");',
        "nowMs-workVisionRequestStartMs>=("
        "workActionPhase==WORK_PHASE_ENDPOINT_WAIT_COORDINATE?"
        "ENDPOINT_VISION_RESULT_TIMEOUT_MS:"
        "VISION_RESULT_TIMEOUT_MS)",
        "startEndpointSearchFallbackMove()",
        '"Endpointnotfoundinboundedlocalsearchwindow"',
        "ENDPOINT_VISION_MAXIMUM_RETRIES",
        'routeFault("Visionresulttimeout");',
        "++workVisionRetryCount;",
    )
    for fragment in required_action_fragments:
        require(
            fragment in competition_action,
            f"workstation/vision safety is missing {fragment}",
        )

    watchdogs = compact_cpp(
        extract_function_body(
            source_without_comments, "serviceCompetitionWatchdogs"
        )
    )
    require(
        "ENABLE_COMPETITION_TIME_LIMIT&&"
        "nowMs-competitionStartMs>=COMPETITION_TIME_LIMIT_MS-"
        "COMPETITION_HARD_STOP_MARGIN_MS" in watchdogs
        and 'routeFault("Competitionhardtimelimit");' in watchdogs,
        "optional competition hard-stop guard is not wired",
    )
    require(
        "nowMs-lastMissionProgressMs>=MISSION_PROGRESS_TIMEOUT_MS"
        in watchdogs
        and 'routeFault("Missionprogresswatchdogtimeout");'
        in watchdogs,
        "mission-progress watchdog is not wired",
    )

    loop_body = compact_cpp(
        extract_function_body(source_without_comments, "loop")
    )
    require(
        "serviceCompetitionWatchdogs();" in loop_body
        and "serviceCompetitionAction();" in loop_body,
        "loop() does not service both safety state machines",
    )

    begin_route = compact_cpp(
        extract_function_body(source_without_comments, "beginRoute")
    )
    require(
        "competitionStartMs=millis();" in begin_route
        and "lastMissionProgressMs=competitionStartMs;" in begin_route,
        "competition/progress clocks are not initialized together",
    )
    progress_body = compact_cpp(
        extract_function_body(
            source_without_comments, "markMissionProgress"
        )
    )
    require(
        "lastMissionProgressMs=millis();" in progress_body,
        "markMissionProgress() does not refresh its watchdog clock",
    )

    optional_order_fragments = (
        "if(REQUIRE_RAW_PICK_QR_ORDER){",
        "constuint8_texpectedColor="
        "taskColors[workRoundIndex][rawCollectedCount];",
        "if(detectedColor!=expectedColor)",
    )
    for fragment in optional_order_fragments:
        require(
            fragment in competition_action,
            "optional raw pickup task-order gate is missing "
            f"{fragment}",
        )

    slot_lookup_body = compact_cpp(
        extract_function_body(
            source_without_comments, "rawStorageSlotForColor"
        )
    )
    for fragment in (
        "for(uint8_tslot=0U;slot<3U;++slot)",
        "if(taskColors[workRoundIndex][slot]==color)",
        "returnstatic_cast<int8_t>(slot);",
        "return-1;",
    ):
        require(
            fragment in slot_lookup_body,
            "raw color-to-QR-slot lookup is missing "
            f"{fragment}",
        )

    raw_bounds_index = competition_action.find(
        "if(rawCollectedCount>=3U)"
    )
    order_guard_index = competition_action.find(
        "if(REQUIRE_RAW_PICK_QR_ORDER){"
    )
    expected_color_index = competition_action.find(
        "constuint8_texpectedColor="
        "taskColors[workRoundIndex][rawCollectedCount];"
    )
    mismatch_index = competition_action.find(
        "if(detectedColor!=expectedColor)"
    )
    slot_lookup_index = competition_action.find(
        "constint8_tslot=rawStorageSlotForColor(detectedColor);"
    )
    outside_batch_index = competition_action.find(
        "if(slot<0)"
    )
    slot_index_index = competition_action.find(
        "constuint8_tslotIndex=static_cast<uint8_t>(slot);"
    )
    optional_slot_order_index = competition_action.find(
        "if(REQUIRE_RAW_PICK_QR_ORDER&&"
        "slotIndex!=rawCollectedCount)"
    )
    slot_bit_index = competition_action.find(
        "constuint8_tslotBit="
        "static_cast<uint8_t>(1U<<slotIndex);"
    )
    duplicate_slot_index = competition_action.find(
        "if((rawFilledSlotMask&slotBit)!=0U)"
    )
    confirmation_index = competition_action.find(
        "if(!confirmRawCoordinate(detectedColor,x,y))"
    )
    pending_slot_index = competition_action.find(
        "rawPendingSlotIndex=slotIndex;"
    )
    storage_position_index = competition_action.find(
        "commandStorageServoPosition(rawPendingSlotIndex);"
    )
    require(
        0
        <= raw_bounds_index
        < order_guard_index
        < expected_color_index
        < mismatch_index
        < slot_lookup_index
        < outside_batch_index
        < slot_index_index
        < optional_slot_order_index
        < slot_bit_index
        < duplicate_slot_index
        < confirmation_index
        < pending_slot_index
        < storage_position_index,
        "raw pickup bounds, optional order gate, QR-slot mapping, "
        "duplicate rejection, and storage positioning are out of order",
    )
    mismatch_to_slot = competition_action[
        mismatch_index:slot_lookup_index
    ]
    require(
        "resetRawConfirmationWindow();" not in mismatch_to_slot
        and "beginRawItemVision();" in mismatch_to_slot
        and "break;" in mismatch_to_slot,
        "wrong-color handling must preserve confirmation state, retry, "
        "and stop the pickup",
    )
    bounds_to_order_guard = competition_action[
        raw_bounds_index:order_guard_index
    ]
    require(
        'routeFault("Rawitemindexoverflow");' in bounds_to_order_guard
        and "break;" in bounds_to_order_guard,
        "rawCollectedCount overflow is not faulted before array indexing",
    )

    outside_batch_handler = competition_action[
        outside_batch_index:slot_index_index
    ]
    require(
        "beginRawItemVision();" in outside_batch_handler
        and "break;" in outside_batch_handler
        and "routeFault(" not in outside_batch_handler,
        "a detected color outside the current QR batch is not ignored "
        "and retried safely",
    )
    duplicate_slot_handler = competition_action[
        duplicate_slot_index:confirmation_index
    ]
    require(
        "beginRawItemVision();" in duplicate_slot_handler
        and "break;" in duplicate_slot_handler
        and "beginArmTransfer(" not in duplicate_slot_handler,
        "an already-filled QR slot is not rejected before pickup",
    )

    begin_work_action = compact_cpp(
        extract_function_body(
            source_without_comments, "beginWorkAction"
        )
    )
    require(
        "rawFilledSlotMask=0U;" in begin_work_action,
        "raw filled-slot mask is not reset at work-action start",
    )

    raw_confirmation = compact_cpp(
        extract_function_body(
            source_without_comments, "confirmRawCoordinate"
        )
    )
    for fragment in (
        "if(color<1U||color>4U)",
        "constuint8_tcolorIndex=static_cast<uint8_t>(color-1U);",
        "rawConfirmationSampleCounts[colorIndex]",
        "rawConfirmationX[colorIndex]",
        "rawConfirmationY[colorIndex]",
        "rawConfirmationLastMs[colorIndex]",
    ):
        require(
            fragment in raw_confirmation,
            "raw coordinate confirmation is not isolated per color: "
            f"{fragment}",
        )

    transfer_completion = compact_cpp(
        extract_function_body(
            source_without_comments,
            "completeTransferAndRotateStorage",
        )
    )
    require(
        "rawFilledSlotMask=static_cast<uint8_t>("
        "rawFilledSlotMask|(1U<<rawPendingSlotIndex));"
        in transfer_completion,
        "completed raw pickup does not mark its QR slot as filled",
    )

    for function_name in ("startCurrentCommand", "advanceRoute"):
        progress_site = compact_cpp(
            extract_function_body(
                source_without_comments, function_name
            )
        )
        require(
            "markMissionProgress();" in progress_site,
            f"{function_name}() does not refresh mission progress",
        )

    print(
        "PASS safety contract: motion/QR/vision/workstation timeouts, "
        "debug hard-stop disabled, 45 s progress watchdog"
    )
    print(
        "PASS raw pickup gate: any pending task color may be picked first; "
        "per-color confirmation, QR-slot mapping and duplicate rejection "
        "enforced"
    )

def verify_general_static_contract(
    source_without_comments: str, commands: Sequence[RouteCommand]
) -> None:
    require(
        "Serial.begin(" not in source_without_comments,
        "default Serial conflicts with the PA1 M4 STEP pin",
    )
    require(
        commands[-1].command_type == "COMMAND_FINISH",
        "route does not end with COMMAND_FINISH",
    )
    home_indices = [
        index
        for index, command in enumerate(commands)
        if command.command_type == "COMMAND_ARM_BASE_HOME"
    ]
    require(
        len(home_indices) == 1
        and home_indices[0] == len(commands) - 2,
        "arm base home must appear exactly once immediately before finish",
    )
    print(
        "PASS static drive contract: four motors serviced, "
        "no default Serial pin conflict, arm home before finish"
    )

def main() -> int:
    require(
        SOURCE_PATH.is_file(),
        f"source file does not exist: {SOURCE_PATH}",
    )
    source = SOURCE_PATH.read_text(encoding="utf-8")
    source_without_comments = strip_cpp_comments(source)
    config_source = CONFIG_PATH.read_text(encoding="utf-8")
    config_without_comments = strip_cpp_comments(config_source)
    config_values = parse_numeric_constants(config_without_comments)
    values = parse_numeric_constants(
        source_without_comments, config_values
    )
    boolean_values = parse_boolean_constants(source_without_comments)

    required_constants = {
        "FIELD_SIZE_MM",
        "FIELD_CENTER_MM",
        "START_ZONE_SIZE_MM",
        "START_ZONE_1_MIN_Y_MM",
        "CHASSIS_FOOTPRINT_X_MM",
        "CHASSIS_FOOTPRINT_Y_MM",
        "START_CENTER_X_MM",
        "START_ZONE_1_CENTER_Y_MM",
        "START_ZONE_2_CENTER_Y_MM",
        "FINAL_ZONE_CENTER_X_MM",
        "START_TO_QR_PASS_MM",
        "STORAGE_ROUND2_OPEN_LOOP_Y_MM",
        "RETURN_TO_START_ZONE_1_Y_MM",
        "RETURN_TO_START_ZONE_2_Y_MM",
        "MAX_TRANSLATION_SEGMENT_MM",
        "CENTRAL_CHANNEL_MAX_TRANSLATION_SEGMENT_MM",
        "FORWARD_PULSES_PER_METER",
        "LATERAL_PULSES_PER_METER",
        "PULSES_PER_WHEEL_REVOLUTION",
        "WHEELBASE_MM",
        "TRACK_WIDTH_MM",
        "WHEEL_DIAMETER_MM",
        "COUNTERCLOCKWISE_ROTATION_PULSE_SCALE",
        "CLOCKWISE_ROTATION_PULSE_SCALE",
        "IMU_COUNTERCLOCKWISE_SIGN",
        "MAIXCAM_ALL_COLORS_REQUEST",
        "MAIXCAM_HOUGH_CIRCLE_REQUEST",
        "MAIXCAM_ENDPOINT_CIRCLE_REQUEST",
        "MAIXCAM_LINE_CAPACITY",
        "M6_MICROSTEPS",
        "M7_MICROSTEPS",
        "M6_PULSES_PER_MM",
        "M7_PULSES_PER_MM",
        "M6_SPEED_RPM",
        "M6_ACCELERATION",
        "ENDPOINT_FINE_M6_SPEED_RPM",
        "ENDPOINT_FINE_M6_ACCELERATION",
        "ENDPOINT_COARSE_M6_SPEED_RPM",
        "ENDPOINT_COARSE_M6_ACCELERATION",
        "M6_RECOVERY_SPEED_RPM",
        "M6_RECOVERY_ACCELERATION",
        "M7_RECOVERY_SPEED_RPM",
        "M7_RECOVERY_ACCELERATION",
        "ARM_AXIS_ENABLE_RESPONSE_WAIT_MS",
        "ARM_AXIS_TERMINAL_CONFIRMATION_SAMPLES",
        "ARM_AXIS_TERMINAL_VERIFY_MAX_FAILURES",
        "ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM",
        "ARM_AXIS_STALL_CONFIRMATION_SAMPLES",
        "ARM_AXIS_MAXIMUM_RECOVERY_ATTEMPTS",
        "ARM_AXIS_RECOVERY_TOTAL_TIMEOUT_MS",
        "ARM_LINEAR_STARTUP_PROBE_MM",
        "ARM_LINEAR_ZERO_ANGLE_RATIO_MINIMUM",
        "ARM_LINEAR_ZERO_ANGLE_RATIO_MAXIMUM",
        "M6_STARTUP_WORKING_ZERO_OFFSET_MM",
        "M7_STARTUP_WORKING_ZERO_OFFSET_MM",
        "ARM_LINEAR_STARTUP_ZERO_SPEED_RPM",
        "ARM_LINEAR_STARTUP_ZERO_ACCELERATION",
        "M6_MAXIMUM_PHYSICAL_EXTENSION_MM",
        "M6_MAXIMUM_EXTENSION_MM",
        "M7_MINIMUM_PHYSICAL_HEIGHT_MM",
        "M7_MINIMUM_HEIGHT_MM",
        "RAW_PICK_LOWER_MM",
        "HOUGH_VISION_LOWER_MM",
        "CONTAINER_PICK_LOWER_MM",
        "CONTAINER_PLACE_LOWER_MM",
        "PROCESS_PLACE_LOWER_MM",
        "STORAGE_ROUND1_PLACE_LOWER_MM",
        "STORAGE_ROUND2_PLACE_LOWER_MM",
        "ARM_PIVOT_TO_CAMERA_FULLY_RETRACTED_MM",
        "ARM_PIVOT_TO_GRIPPER_FULLY_RETRACTED_MM",
        "ARM_PIVOT_TO_CAMERA_CENTER_MM",
        "ARM_PIVOT_TO_GRIPPER_CENTER_MM",
        "ENDPOINT_FINE_VISION_LOWER_MM",
        "ENDPOINT_COARSE_CENTER_TOLERANCE_PIXELS",
        "ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS",
        "ENDPOINT_FINAL_CENTER_CONFIRMATIONS",
        "ENDPOINT_MAXIMUM_SERVO_MOVES_PER_STAGE",
        "ENDPOINT_MAXIMUM_TOTAL_SERVO_MOVES",
        "ENDPOINT_COARSE_SERVO_GAIN",
        "ENDPOINT_FINE_SERVO_GAIN",
        "ENDPOINT_COARSE_MAXIMUM_CORRECTION_MM",
        "ENDPOINT_FINE_MAXIMUM_CORRECTION_MM",
        "ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES",
        "ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES",
        "ENDPOINT_SEARCH_SEED_EXTENSION_MM",
        "ENDPOINT_RING1_SEARCH_FALLBACK_STEP_DEGREES",
        "ENDPOINT_RING3_SEARCH_FALLBACK_STEP_DEGREES",
        "ENDPOINT_RING1_SEARCH_MINIMUM_ANGLE_DEGREES",
        "ENDPOINT_RING1_SEARCH_MAXIMUM_ANGLE_DEGREES",
        "ENDPOINT_RING3_SEARCH_MINIMUM_ANGLE_DEGREES",
        "ENDPOINT_RING3_SEARCH_MAXIMUM_ANGLE_DEGREES",
        "ENDPOINT_VISION_RESULT_TIMEOUT_MS",
        "ENDPOINT_VISION_MAXIMUM_RETRIES",
        "RING_PLACE_FINAL_DESCENT_MM",
        "M7_SPEED_RPM",
        "M7_RING_PLACE_SPEED_RPM",
        "M7_RING_PLACE_ACCELERATION",
        "RING_PLACE_EXTENSION_SETTLE_MS",
        "RING_PLACE_LOWER_SETTLE_MS",
        "MAXIMUM_STEP_RATE",
        "CENTRAL_CHANNEL_MAXIMUM_STEP_RATE",
        "STEP_ACCELERATION",
        "TURN_MAXIMUM_STEP_RATE",
        "TURN_STEP_ACCELERATION",
        "WORKSTATION_MAXIMUM_STEP_RATE",
        "WORKSTATION_STEP_ACCELERATION",
        "FINAL_MAXIMUM_STEP_RATE",
        "FINAL_STEP_ACCELERATION",
        "QR_SCAN_ACTION_TIMEOUT_MS",
        "QR_REQUIRED_MATCHING_FRAMES",
        "VISION_RESULT_TIMEOUT_MS",
        "VISION_MAXIMUM_RETRIES",
        "RAW_ACTION_TIMEOUT_MS",
        "PROCESS_ACTION_TIMEOUT_MS",
        "STORAGE_ACTION_TIMEOUT_MS",
        "MISSION_PROGRESS_TIMEOUT_MS",
        "COMPETITION_TIME_LIMIT_MS",
        "COMPETITION_HARD_STOP_MARGIN_MS",
        "ARM_BASE_MAXIMUM_STEP_RATE",
        "ARM_BASE_STEP_ACCELERATION",
        "ARM_BASE_TRANSFER_MAXIMUM_STEP_RATE",
        "ARM_BASE_TRANSFER_STEP_ACCELERATION",
        "ARM_BASE_MOTION_TIMEOUT_MS",
    }
    missing = sorted(required_constants - values.keys())
    require(not missing, f"unparsed constants: {missing}")

    commands = parse_route(source_without_comments, values)
    verify_route_command_contract(source_without_comments)
    verify_action_sequence(commands)
    resolved_routes = verify_route_geometry(commands, values)
    verify_wheel_pulses(
        resolved_routes, values, source_without_comments
    )
    verify_motion_profiles(
        resolved_routes, values, source_without_comments
    )
    verify_imu_unwrap(source_without_comments, values)
    verify_qr_reliability_contract(source_without_comments, values)
    verify_vision_protocol_contract(source_without_comments, values)
    verify_endpoint_mapping_contract(
        source_without_comments, values
    )
    verify_safety_contract(
        source_without_comments, values, boolean_values
    )
    verify_general_static_contract(source_without_comments, commands)
    print(f"ALL PREFLIGHT CHECKS PASSED: {SOURCE_PATH}")
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except (
        PreflightFailure,
        StopIteration,
        SyntaxError,
        ValueError,
        KeyError,
    ) as error:
        print(f"PREFLIGHT FAILED: {error}", file=sys.stderr)
        sys.exit(1)
