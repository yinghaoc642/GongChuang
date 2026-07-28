#!/usr/bin/env python3
"""Deterministic preflight checks for a route firmware source.

The script reads the route and calibration constants from the firmware source,
then checks route geometry, chassis clearance, workstation pose, wheel pulses,
AccelStepper position profiles, IMU angle unwrapping, and critical static
invariants. Pass a source path as argv[1]; otherwise src/main.cpp is checked.
It uses only the Python standard library.
"""

from __future__ import annotations

import ast
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = (
    Path(sys.argv[1]).resolve()
    if len(sys.argv) > 1
    else ROOT / "src" / "main.cpp"
)

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


class PreflightFailure(AssertionError):
    """A preflight invariant failed."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise PreflightFailure(message)


def strip_cpp_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", text)


def clean_cpp_number_suffixes(expression: str) -> str:
    return re.sub(
        r"(?<![A-Za-z_])(\d+(?:\.\d*)?|\.\d+)(?:[fFuUlL]+)\b",
        r"\1",
        expression,
    )


def evaluate_expression(expression: str, values: Dict[str, float]) -> float:
    expression = clean_cpp_number_suffixes(expression.strip())
    expression = re.sub(r"\s+", " ", expression)
    tree = ast.parse(expression, mode="eval")

    def evaluate(node: ast.AST) -> float:
        if isinstance(node, ast.Expression):
            return evaluate(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.Name) and node.id in values:
            return float(values[node.id])
        if isinstance(node, ast.UnaryOp):
            value = evaluate(node.operand)
            if isinstance(node.op, ast.USub):
                return -value
            if isinstance(node.op, ast.UAdd):
                return value
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
        raise ValueError(f"unsupported expression: {expression}")

    return evaluate(tree)


def parse_numeric_constants(source_without_comments: str) -> Dict[str, float]:
    declaration_pattern = re.compile(
        r"\b(?:constexpr\s+(?:uint16_t|int32_t)|"
        r"const\s+(?:float|uint16_t|uint32_t))"
        r"\s+([A-Za-z_]\w*)\s*=\s*(.*?);",
        flags=re.DOTALL,
    )
    pending = {
        name: expression
        for name, expression in declaration_pattern.findall(source_without_comments)
    }
    values: Dict[str, float] = {}

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


@dataclass(frozen=True)
class RouteCommand:
    command_type: str
    value: float
    name: str
    precise_arrival: bool


def parse_route(
    source_without_comments: str, values: Dict[str, float]
) -> List[RouteCommand]:
    route_match = re.search(
        r"const\s+RouteCommand\s+route\[\]\s*=\s*\{(.*?)\};",
        source_without_comments,
        flags=re.DOTALL,
    )
    require(route_match is not None, "route[] was not found")

    command_pattern = re.compile(
        r"\{\s*(COMMAND_[A-Z0-9_]+)\s*,\s*([^,{}]+?)\s*,"
        r'\s*"([^"]*)"\s*(?:,\s*(true|false)\s*)?\}'
    )
    commands: List[RouteCommand] = []
    for command_type, expression, name, precise_text in command_pattern.findall(
        route_match.group(1)
    ):
        commands.append(
            RouteCommand(
                command_type,
                evaluate_expression(expression, values),
                name,
                precise_text == "true",
            )
        )

    require(commands, "no route commands were parsed")
    return commands


@dataclass(frozen=True)
class Pose:
    x_mm: float
    y_mm: float
    side_24_heading_deg: float


def normalized_heading(degrees: float) -> float:
    result = degrees % 360.0
    return 0.0 if abs(result - 360.0) < 1.0e-9 else result


def body_half_extents(
    heading_deg: float, footprint_x_mm: float, footprint_y_mm: float
) -> Tuple[float, float]:
    heading = normalized_heading(heading_deg)
    require(
        any(abs(heading - cardinal) < 1.0e-6 for cardinal in (0, 90, 180, 270)),
        f"non-cardinal route heading: {heading_deg}",
    )
    if heading in (0.0, 180.0):
        return footprint_x_mm / 2.0, footprint_y_mm / 2.0
    return footprint_y_mm / 2.0, footprint_x_mm / 2.0


def translation_angle(command_type: str, side_24_heading_deg: float) -> float:
    offsets = {
        "COMMAND_MOVE_SIDE_24_MM": 0.0,
        "COMMAND_MOVE_SIDE_12_MM": 90.0,
        "COMMAND_MOVE_SIDE_13_MM": 180.0,
        "COMMAND_MOVE_SIDE_34_MM": -90.0,
    }
    return side_24_heading_deg + offsets[command_type]


def simulate_route(
    commands: Sequence[RouteCommand], values: Dict[str, float]
) -> Tuple[List[Tuple[RouteCommand, Pose, Pose]], List[Tuple[RouteCommand, Pose]]]:
    pose = Pose(values["START_CENTER_X_MM"], values["START_CENTER_Y_MM"], 0.0)
    movements: List[Tuple[RouteCommand, Pose, Pose]] = []
    events: List[Tuple[RouteCommand, Pose]] = []

    for command in commands:
        before = pose
        if command.command_type in TRANSLATION_COMMANDS:
            angle_radians = math.radians(
                translation_angle(command.command_type, pose.side_24_heading_deg)
            )
            pose = Pose(
                pose.x_mm + command.value * math.cos(angle_radians),
                pose.y_mm + command.value * math.sin(angle_radians),
                pose.side_24_heading_deg,
            )
            movements.append((command, before, pose))
        elif command.command_type == "COMMAND_TURN_COUNTERCLOCKWISE_DEGREES":
            pose = Pose(
                pose.x_mm,
                pose.y_mm,
                pose.side_24_heading_deg + command.value,
            )
            movements.append((command, before, pose))
        elif command.command_type == "COMMAND_TURN_CLOCKWISE_DEGREES":
            pose = Pose(
                pose.x_mm,
                pose.y_mm,
                pose.side_24_heading_deg - command.value,
            )
            movements.append((command, before, pose))
        events.append((command, pose))

    return movements, events


def near(actual: float, expected: float, tolerance: float = 1.0e-5) -> bool:
    return abs(actual - expected) <= tolerance


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


def verify_route_geometry(
    commands: Sequence[RouteCommand],
    movements: Sequence[Tuple[RouteCommand, Pose, Pose]],
    events: Sequence[Tuple[RouteCommand, Pose]],
    values: Dict[str, float],
) -> None:
    require(
        near(values["CHASSIS_HALF_WIDTH_MM"], 115.0)
        and near(values["ARM_CENTER_OFFSET_MM"], 225.0)
        and near(values["ARM_CENTER_BEYOND_NEAR_WHEEL_MM"], 110.0)
        and near(values["ARM_CENTER_TO_FARTHEST_WHEEL_MM"], 340.0),
        "230/225 mm arm geometry is inconsistent",
    )
    direct_qr_move = next(
        command
        for command in commands
        if command.name == "Start1 -> QR area direct"
    )
    qr_to_center_move = next(
        command
        for command in commands
        if command.name == "QR area -> raw centerline"
    )
    require(
        direct_qr_move.command_type == "COMMAND_MOVE_SIDE_34_MM"
        and near(
            direct_qr_move.value,
            values["START_TO_QR_PASS_MM"],
        ),
        f"direct QR move is {direct_qr_move}",
    )
    require(
        qr_to_center_move.command_type == "COMMAND_MOVE_SIDE_13_MM"
        and near(qr_to_center_move.value, 1015.0),
        f"QR-to-center move is {qr_to_center_move}",
    )
    require(
        direct_qr_move.value <= values["MAX_TRANSLATION_SEGMENT_MM"]
        and qr_to_center_move.value <= values["MAX_TRANSLATION_SEGMENT_MM"],
        "one of the first two translations would be split into a crawl segment",
    )
    require(
        not any(
            command.command_type == "COMMAND_QR_ACTION"
            for command in commands
        ),
        "route still contains a QR scan/wait action",
    )

    expected_action_poses = {
        "COMMAND_RAW_ACTION": [
            Pose(
                values["FIELD_CENTER_MM"],
                values["RAW_CENTER_Y_MM"],
                90.0,
            ),
            Pose(
                values["FIELD_CENTER_MM"],
                values["RAW_CENTER_Y_MM"],
                90.0,
            ),
        ],
        "COMMAND_PROCESS_ACTION": [
            Pose(
                values["FIELD_CENTER_MM"],
                values["PROCESS_CENTER_Y_MM"],
                270.0,
            ),
            Pose(
                values["FIELD_CENTER_MM"],
                values["PROCESS_CENTER_Y_MM"],
                270.0,
            ),
        ],
        "COMMAND_STORAGE_ACTION": [
            Pose(
                values["STORAGE_CENTER_X_MM"],
                values["FIELD_CENTER_MM"],
                180.0,
            ),
            Pose(
                values["STORAGE_CENTER_X_MM"],
                values["FIELD_CENTER_MM"],
                180.0,
            ),
        ],
    }

    for command_type, expected_poses in expected_action_poses.items():
        actual_poses = [pose for command, pose in events if command.command_type == command_type]
        require(
            len(actual_poses) == len(expected_poses),
            f"{command_type}: expected {len(expected_poses)}, got {len(actual_poses)}",
        )
        for index, (actual, expected) in enumerate(zip(actual_poses, expected_poses), 1):
            assert_pose(actual, expected, f"{command_type} #{index}")

    final_pose = events[-1][1]
    assert_pose(
        final_pose,
        Pose(
            values["FINAL_ZONE_CENTER_X_MM"],
            values["FINAL_ZONE_CENTER_Y_MM"],
            180.0,
        ),
        "final pose",
    )

    field_size = values["FIELD_SIZE_MM"]
    footprint_x = values["CHASSIS_FOOTPRINT_X_MM"]
    footprint_y = values["CHASSIS_FOOTPRINT_Y_MM"]
    minimum_field_clearance = float("inf")
    minimum_center_corridor_clearance = float("inf")

    poses_to_check = [events[0][1]]
    for _, before, after in movements:
        poses_to_check.extend((before, after))

    for pose in poses_to_check:
        half_x, half_y = body_half_extents(
            pose.side_24_heading_deg, footprint_x, footprint_y
        )
        clearances = (
            pose.x_mm - half_x,
            field_size - (pose.x_mm + half_x),
            pose.y_mm - half_y,
            field_size - (pose.y_mm + half_y),
        )
        minimum_field_clearance = min(minimum_field_clearance, *clearances)
        require(min(clearances) >= -1.0e-6, f"chassis leaves field at {pose}")

        if near(pose.x_mm, values["FIELD_CENTER_MM"]):
            corridor_clearance = 200.0 - half_x
            minimum_center_corridor_clearance = min(
                minimum_center_corridor_clearance, corridor_clearance
            )
            require(corridor_clearance >= 0.0, f"vertical corridor violation at {pose}")
        if near(pose.y_mm, values["FIELD_CENTER_MM"]):
            corridor_clearance = 200.0 - half_y
            minimum_center_corridor_clearance = min(
                minimum_center_corridor_clearance, corridor_clearance
            )
            require(
                corridor_clearance >= 0.0, f"horizontal corridor violation at {pose}"
            )

    turn_radius = math.hypot(footprint_x / 2.0, footprint_y / 2.0)
    turn_locations: List[Tuple[float, float]] = []
    previous_was_turn = False
    for command, pose in events:
        if command.command_type in TURN_COMMANDS:
            require(not previous_was_turn, "two planned turns occur without translation")
            turn_locations.append((pose.x_mm, pose.y_mm))
            turn_clearance = min(
                pose.x_mm,
                field_size - pose.x_mm,
                pose.y_mm,
                field_size - pose.y_mm,
            )
            require(
                turn_clearance + 1.0e-6 >= turn_radius,
                f"turning sweep leaves field at {pose}",
            )
            previous_was_turn = True
        elif command.command_type in TRANSLATION_COMMANDS:
            previous_was_turn = False

    # 图纸中的四个450x450区域围绕400 mm中央十字通道布置。
    # 平移检查车体扫过的完整包围盒；原地转弯检查车体外接圆扫掠。
    corridor_half_width = 200.0
    workstation_block_size = 450.0
    inner_low = values["FIELD_CENTER_MM"] - corridor_half_width
    inner_high = values["FIELD_CENTER_MM"] + corridor_half_width
    outer_low = inner_low - workstation_block_size
    outer_high = inner_high + workstation_block_size
    obstacle_rectangles = (
        (outer_low, inner_low, inner_high, outer_high),
        (inner_high, outer_high, inner_high, outer_high),
        (outer_low, inner_low, outer_low, inner_low),
        (inner_high, outer_high, outer_low, inner_low),
    )

    def rectangle_separation(
        first: Tuple[float, float, float, float],
        second: Tuple[float, float, float, float],
    ) -> Tuple[float, bool]:
        first_min_x, first_max_x, first_min_y, first_max_y = first
        second_min_x, second_max_x, second_min_y, second_max_y = second
        delta_x = max(
            second_min_x - first_max_x,
            first_min_x - second_max_x,
            0.0,
        )
        delta_y = max(
            second_min_y - first_max_y,
            first_min_y - second_max_y,
            0.0,
        )
        overlaps_x = (
            min(first_max_x, second_max_x)
            - max(first_min_x, second_min_x)
            > 1.0e-6
        )
        overlaps_y = (
            min(first_max_y, second_max_y)
            - max(first_min_y, second_min_y)
            > 1.0e-6
        )
        return math.hypot(delta_x, delta_y), overlaps_x and overlaps_y

    def point_to_rectangle_distance(
        x_mm: float,
        y_mm: float,
        rectangle: Tuple[float, float, float, float],
    ) -> float:
        min_x, max_x, min_y, max_y = rectangle
        delta_x = max(min_x - x_mm, 0.0, x_mm - max_x)
        delta_y = max(min_y - y_mm, 0.0, y_mm - max_y)
        return math.hypot(delta_x, delta_y)

    minimum_obstacle_clearance = float("inf")
    for command, before, after in movements:
        if command.command_type in TRANSLATION_COMMANDS:
            half_x, half_y = body_half_extents(
                before.side_24_heading_deg,
                footprint_x,
                footprint_y,
            )
            swept_rectangle = (
                min(before.x_mm, after.x_mm) - half_x,
                max(before.x_mm, after.x_mm) + half_x,
                min(before.y_mm, after.y_mm) - half_y,
                max(before.y_mm, after.y_mm) + half_y,
            )
            for obstacle in obstacle_rectangles:
                clearance, intersects = rectangle_separation(
                    swept_rectangle,
                    obstacle,
                )
                require(
                    not intersects,
                    f"{command.name}: chassis sweep intersects a 450x450 area",
                )
                minimum_obstacle_clearance = min(
                    minimum_obstacle_clearance,
                    clearance,
                )
        elif command.command_type in TURN_COMMANDS:
            for obstacle in obstacle_rectangles:
                clearance = (
                    point_to_rectangle_distance(
                        after.x_mm,
                        after.y_mm,
                        obstacle,
                    )
                    - turn_radius
                )
                require(
                    clearance >= -1.0e-6,
                    f"{command.name}: turning sweep intersects a 450x450 area",
                )
                minimum_obstacle_clearance = min(
                    minimum_obstacle_clearance,
                    clearance,
                )

    planned_turn_degrees = sum(
        command.value for command in commands if command.command_type in TURN_COMMANDS
    )
    last_storage_index = max(
        index
        for index, command in enumerate(commands)
        if command.command_type == "COMMAND_STORAGE_ACTION"
    )
    final_return_commands = commands[last_storage_index + 1 :]
    require(
        not any(
            command.command_type in TURN_COMMANDS
            for command in final_return_commands
        ),
        "final return still contains a turn",
    )
    require(
        [
            command.command_type
            for command in final_return_commands
            if command.command_type in TRANSLATION_COMMANDS
        ][-2:]
        == ["COMMAND_MOVE_SIDE_34_MM", "COMMAND_MOVE_SIDE_13_MM"],
        "final direct parking translations are not north then east",
    )
    required_orientations = (
        0.0,
        90.0,
        270.0,
        180.0,
        90.0,
        270.0,
        180.0,
        180.0,
    )
    theoretical_minimum_turn = 0.0
    for first, second in zip(required_orientations, required_orientations[1:]):
        delta = abs((second - first + 180.0) % 360.0 - 180.0)
        theoretical_minimum_turn += delta
    require(
        near(planned_turn_degrees, theoretical_minimum_turn),
        f"planned turn {planned_turn_degrees} != minimum {theoretical_minimum_turn}",
    )

    for index, (command, pose) in enumerate(events):
        if command.command_type not in {
            "COMMAND_RAW_ACTION",
            "COMMAND_PROCESS_ACTION",
            "COMMAND_STORAGE_ACTION",
        }:
            continue
        require(index >= 1, f"{command.name}: missing entry command")
        entry = commands[index - 1]
        require(
            entry.command_type == "COMMAND_MOVE_SIDE_24_MM",
            f"{command.name}: final entry is not through side 2,4",
        )
        require(entry.precise_arrival, f"{command.name}: entry is not precise")
        require(
            near(entry.value, values["WORKSTATION_APPROACH_MM"]),
            f"{command.name}: entry length is {entry.value}",
        )

    side_24_half_width = footprint_x / 2.0
    if "ARM_CENTER_OFFSET_MM" in values:
        tool_distance = values["ARM_CENTER_OFFSET_MM"]
    else:
        tool_distance = (
            side_24_half_width + values["TOOL_OUTWARD_REACH_MM"]
        )
    workstation_events = [
        (command, pose)
        for command, pose in events
        if command.command_type
        in {"COMMAND_RAW_ACTION", "COMMAND_PROCESS_ACTION", "COMMAND_STORAGE_ACTION"}
    ]
    tool_points = []
    for command, pose in workstation_events:
        heading = math.radians(pose.side_24_heading_deg)
        tool_points.append(
            (
                command.command_type,
                pose.x_mm + tool_distance * math.cos(heading),
                pose.y_mm + tool_distance * math.sin(heading),
            )
        )
    for command_type, tool_x, tool_y in tool_points:
        if command_type in {"COMMAND_RAW_ACTION", "COMMAND_PROCESS_ACTION"}:
            require(near(tool_x, 1200.0), f"{command_type}: tool misses x centerline")
        else:
            require(near(tool_y, 1200.0), f"{command_type}: tool misses y centerline")
        require(
            -1.0e-6 <= tool_x <= field_size + 1.0e-6
            and -1.0e-6 <= tool_y <= field_size + 1.0e-6,
            f"{command_type}: tool point leaves field",
        )

        if "ARM_CENTER_OFFSET_MM" in values:
            if command_type == "COMMAND_RAW_ACTION":
                expected_tool_x = values["FIELD_CENTER_MM"]
                expected_tool_y = values["RAW_RING_CENTER_Y_MM"]
            elif command_type == "COMMAND_PROCESS_ACTION":
                expected_tool_x = values["FIELD_CENTER_MM"]
                expected_tool_y = values["PROCESS_RING_CENTER_Y_MM"]
            else:
                expected_tool_x = values["STORAGE_RING_CENTER_X_MM"]
                expected_tool_y = values["FIELD_CENTER_MM"]
            require(
                near(tool_x, expected_tool_x)
                and near(tool_y, expected_tool_y),
                f"{command_type}: arm center=({tool_x},{tool_y})"
                f" misses ring=({expected_tool_x},{expected_tool_y})",
            )

    final_half_x, final_half_y = body_half_extents(
        final_pose.side_24_heading_deg, footprint_x, footprint_y
    )
    final_bounds = (
        final_pose.x_mm - final_half_x,
        final_pose.x_mm + final_half_x,
        final_pose.y_mm - final_half_y,
        final_pose.y_mm + final_half_y,
    )
    zone_min = values["START_ZONE_MIN_MM"]
    coordinate_epsilon = 1.0e-6
    require(
        final_bounds[0] >= zone_min - coordinate_epsilon
        and final_bounds[1] <= field_size + coordinate_epsilon
        and final_bounds[2] >= zone_min - coordinate_epsilon
        and final_bounds[3] <= field_size + coordinate_epsilon,
        f"final chassis bounds are outside 300x300 zone: {final_bounds}",
    )
    require(near(final_bounds[0], zone_min), "final chassis is not flush to zone left")
    require(near(final_bounds[2], zone_min), "final chassis is not flush to zone bottom")

    print(
        "PASS route geometry:"
        f" {len(commands)} commands, final=({final_pose.x_mm:.0f},"
        f"{final_pose.y_mm:.0f}), heading={normalized_heading(final_pose.side_24_heading_deg):.0f}"
    )
    print(
        "PASS direct QR bypass:"
        f" {direct_qr_move.value:.0f} mm then"
        f" {qr_to_center_move.value:.0f} mm,"
        " one segment each, no scan/wait command"
    )
    print(
        "PASS arm geometry:"
        " width half=115 mm, center offset=225 mm,"
        " near-wheel overhang=110 mm, farthest-wheel distance=340 mm"
    )
    print(
        "PASS clearances:"
        f" field minimum={minimum_field_clearance:.1f} mm,"
        f" center-corridor minimum={minimum_center_corridor_clearance:.1f} mm,"
        f" 450x450-area minimum={minimum_obstacle_clearance:.1f} mm"
    )
    if minimum_obstacle_clearance < 1.0:
        print(
            "WARNING zero geometric margin:"
            " the Y=1150 horizontal sweep only touches the 450x450-area boundary"
        )
    print(
        "PASS workstation poses: 2 rounds x 3 stations,"
        " side 2,4 entry and nominal middle-ring alignment in both rounds"
    )
    print(
        "PASS planned turns:"
        f" {len(turn_locations)} turns, {planned_turn_degrees:.0f} deg total"
        f" (theoretical minimum {theoretical_minimum_turn:.0f} deg)"
    )
    print("PASS direct final parking: no final turn, north 100 mm then east 65 mm")
    print(
        "PASS final 300x300 zone:"
        f" chassis bounds x={final_bounds[0]:.0f}..{final_bounds[1]:.0f},"
        f" y={final_bounds[2]:.0f}..{final_bounds[3]:.0f}"
    )


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
    elif command.command_type == "COMMAND_TURN_COUNTERCLOCKWISE_DEGREES":
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
    rotation_scale_name = (
        "COUNTERCLOCKWISE_ROTATION_PULSE_SCALE"
        if turn_rad >= 0.0
        else "CLOCKWISE_ROTATION_PULSE_SCALE"
    )
    rotation_pulses = (
        lever_arm_m
        * turn_rad
        / (values["WHEEL_DIAMETER_MM"] / 2000.0)
        * values["PULSES_PER_WHEEL_REVOLUTION"]
        / (2.0 * math.pi)
        * values[rotation_scale_name]
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
    commands: Sequence[RouteCommand], values: Dict[str, float]
) -> None:
    movement_count = 0
    sign_patterns = set()
    for command in commands:
        if command.command_type not in TRANSLATION_COMMANDS | TURN_COMMANDS:
            continue
        movement_count += 1
        pulses = motor_pulses_for_command(command, values)
        require(all(pulse != 0 for pulse in pulses), f"{command.name}: zero wheel pulse")
        require(pulses[3] != 0, f"{command.name}: M4 has no target pulse")
        magnitudes = {abs(pulse) for pulse in pulses}
        require(len(magnitudes) == 1, f"{command.name}: unequal pure-motion pulses {pulses}")
        sign_patterns.add(tuple(1 if pulse > 0 else -1 for pulse in pulses))

    expected_patterns = {
        (-1, 1, -1, 1),
        (1, -1, 1, -1),
        (1, 1, -1, -1),
        (-1, -1, 1, 1),
        (1, 1, 1, 1),
        (-1, -1, -1, -1),
    }
    require(sign_patterns == expected_patterns, f"wheel sign patterns: {sign_patterns}")
    print(
        "PASS wheel pulses:"
        f" {movement_count} planned moves, M1..M4 nonzero,"
        " all 6 pure-motion sign patterns verified"
    )


class AccelStepperModel:
    """Step-event model of AccelStepper 1.64 computeNewSpeed()."""

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
                    (self.speed * self.speed) / (2.0 * self.acceleration)
                )
                self.compute_new_speed()

    def set_acceleration(self, acceleration: float) -> None:
        acceleration = abs(acceleration)
        if self.acceleration != acceleration:
            self.n = int(self.n * (self.acceleration / acceleration))
            self.c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1_000_000.0
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
        require(model.step_once(), "stepper stopped before reaching target")
        actual_direction = 1 if model.current > previous else -1
        if actual_direction != requested_direction:
            reverse_steps += 1
        directional_overshoot = (model.current - target) * requested_direction
        maximum_overshoot = max(maximum_overshoot, directional_overshoot)
        step_count += 1
        require(step_count < 200_000, "stepper simulation did not converge")

    require(model.current == target, f"stepper final {model.current} != {target}")
    require(
        model.current - start == pulses,
        f"stepper displacement {model.current - start} != {pulses}",
    )
    return maximum_overshoot, reverse_steps


def simulate_old_profile_switch() -> Tuple[int, int]:
    model = AccelStepperModel()
    model.set_max_speed(5500.0)
    model.set_acceleration(2500.0)
    model.move(9500)
    switched = False
    maximum_overshoot = 0
    reverse_steps = 0
    requested_direction = 1

    for _ in range(100_000):
        if not switched and model.distance_to_go() <= 1500:
            model.set_max_speed(1600.0)
            model.set_acceleration(700.0)
            switched = True
        if model.speed == 0.0 and model.distance_to_go() == 0:
            break
        previous = model.current
        model.step_once()
        actual_direction = 1 if model.current > previous else -1
        if actual_direction != requested_direction:
            reverse_steps += 1
        maximum_overshoot = max(maximum_overshoot, model.current - model.target)
    else:
        raise PreflightFailure("old profile simulation did not converge")
    return maximum_overshoot, reverse_steps


def verify_motion_profiles(
    commands: Sequence[RouteCommand], values: Dict[str, float]
) -> None:
    precise_motion = False
    simulated_segments = 0
    largest_new_overshoot = 0
    new_reverse_steps = 0

    for command in commands:
        if command.command_type == "COMMAND_SET_PRECISE_MOTION":
            precise_motion = True
            continue
        if command.command_type not in TRANSLATION_COMMANDS | TURN_COMMANDS:
            continue

        if command.command_type in TRANSLATION_COMMANDS:
            remaining_mm = int(round(command.value))
            while remaining_mm > 0:
                segment_mm = min(
                    remaining_mm, int(round(values["MAX_TRANSLATION_SEGMENT_MM"]))
                )
                remaining_mm -= segment_mm
                segment_command = RouteCommand(
                    command.command_type,
                    float(segment_mm),
                    command.name,
                    command.precise_arrival,
                )
                pulses = motor_pulses_for_command(segment_command, values)[0]
                if precise_motion:
                    maximum_speed = values["FINAL_MAXIMUM_STEP_RATE"]
                    acceleration = values["FINAL_STEP_ACCELERATION"]
                elif command.precise_arrival:
                    maximum_speed = values["WORKSTATION_MAXIMUM_STEP_RATE"]
                    acceleration = values["WORKSTATION_STEP_ACCELERATION"]
                else:
                    maximum_speed = values["MAXIMUM_STEP_RATE"]
                    acceleration = values["STEP_ACCELERATION"]
                overshoot, reverse_steps = run_stepper_move(
                    pulses, maximum_speed, acceleration
                )
                largest_new_overshoot = max(largest_new_overshoot, overshoot)
                new_reverse_steps += reverse_steps
                simulated_segments += 1
        else:
            pulses = motor_pulses_for_command(command, values)[0]
            overshoot, reverse_steps = run_stepper_move(
                pulses,
                values["TURN_MAXIMUM_STEP_RATE"],
                values["TURN_STEP_ACCELERATION"],
            )
            largest_new_overshoot = max(largest_new_overshoot, overshoot)
            new_reverse_steps += reverse_steps
            simulated_segments += 1

    require(largest_new_overshoot == 0, f"new profile overshoot={largest_new_overshoot}")
    require(new_reverse_steps == 0, f"new profile reverse steps={new_reverse_steps}")

    old_overshoot, old_reverse_steps = simulate_old_profile_switch()
    require(old_overshoot > 0, "old profile-switch regression was not reproduced")
    require(old_reverse_steps > 0, "old profile-switch reversal was not reproduced")

    print(
        "PASS AccelStepper profiles:"
        f" {simulated_segments} segments, new overshoot=0 pulse,"
        " new reverse=0"
    )
    print(
        "PASS overshoot regression:"
        f" old mid-motion switch overshoot={old_overshoot} pulses"
        f" and reverse={old_reverse_steps} steps"
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


def verify_imu_unwrap() -> None:
    require(near(unwrap_sequence((170.0, 179.0, -179.0, -170.0)), 20.0), "CCW wrap")
    require(near(unwrap_sequence((-170.0, -179.0, 179.0, 170.0)), -20.0), "CW wrap")

    positive_turn = [0.0]
    negative_turn = [0.0]
    for degrees in range(10, 731, 10):
        positive_turn.append(((degrees + 180.0) % 360.0) - 180.0)
        negative_turn.append(((-degrees + 180.0) % 360.0) - 180.0)
    require(near(unwrap_sequence(positive_turn), 730.0), "multi-turn CCW unwrap")
    require(near(unwrap_sequence(negative_turn), -730.0), "multi-turn CW unwrap")
    print("PASS IMU unwrap: +/-180 crossings and +/-730 deg multi-turn sequences")


def verify_arm_base_sequence(
    commands: Sequence[RouteCommand],
    values: Dict[str, float],
    source: str,
) -> None:
    require(
        commands[0].command_type == "COMMAND_MOVE_SIDE_34_MM",
        "chassis does not start immediately with the direct Y move",
    )
    require(
        not any(
            command.command_type == "COMMAND_ARM_BASE_DEPLOY"
            for command in commands
        ),
        "route still waits for an arm-deploy command",
    )

    home_indices = [
        index
        for index, command in enumerate(commands)
        if command.command_type == "COMMAND_ARM_BASE_HOME"
    ]
    finish_indices = [
        index
        for index, command in enumerate(commands)
        if command.command_type == "COMMAND_FINISH"
    ]
    require(len(home_indices) == 1, f"arm home command count={len(home_indices)}")
    require(len(finish_indices) == 1, f"finish command count={len(finish_indices)}")
    require(
        home_indices[0] == finish_indices[0] - 1,
        "arm does not return home immediately before finish",
    )
    require(
        near(commands[home_indices[0]].value, 0.0),
        f"arm home angle is {commands[home_indices[0]].value}",
    )

    pulses_per_degree = values["ARM_BASE_PULSES_PER_DEGREE"]
    deploy_pulses = round(
        values["ARM_BASE_DEPLOY_ANGLE_DEGREES"] * pulses_per_degree
    )
    require(deploy_pulses == -4000, f"arm deploy pulses={deploy_pulses}")
    for pulses in (deploy_pulses, -deploy_pulses):
        overshoot, reverse_steps = run_stepper_move(
            pulses,
            values["ARM_BASE_MAXIMUM_STEP_RATE"],
            values["ARM_BASE_STEP_ACCELERATION"],
        )
        require(overshoot == 0, f"arm base overshoot={overshoot}")
        require(reverse_steps == 0, f"arm base reverse steps={reverse_steps}")
    require(
        "startArmBaseRotationToDegrees(" in source
        and "armBaseRotationStepper.run();" in source,
        "arm base is not serviced non-blockingly",
    )
    require(
        "startArmBaseRotationToDegrees(\n"
        "        static_cast<float>(\n"
        "            ARM_BASE_DEPLOY_ANGLE_DEGREES));"
        in source,
        "PB9 click does not trigger arm deployment",
    )
    print(
        "PASS arm base sequence:"
        " 5:1, PB9 -> -90 deg (-4000 pulses) concurrent with chassis,"
        " other arm motors idle, finish -> 0 deg"
    )


def verify_static_invariants(source: str) -> None:
    forbidden_symbols = (
        "serviceWorkstationApproachProfile",
        "WORKSTATION_APPROACH_PULSE_THRESHOLD",
        "currentTranslationEndsAtPreciseArrival",
    )
    for symbol in forbidden_symbols:
        require(symbol not in source, f"obsolete mid-motion profile symbol: {symbol}")

    require(
        "const bool ENABLE_MOTION_TIMEOUTS = false;" in source,
        "motion timeout setting changed",
    )
    require(
        "const bool ENABLE_QR_RECEIVER = false;" in source,
        "QR receiver was not disabled for the bypass route",
    )
    require("motor4.move(pulses.motor4);" in source, "M4 move target is missing")
    for motor_number in range(1, 5):
        require(
            f"motor{motor_number}.run();" in source,
            f"motor{motor_number}.run() is missing",
        )
    require("Serial.begin(" not in source, "default Serial would conflict with M4 PA1")
    require(
        "workstationApproachEnabled =\n      translationPreciseArrivalEnabled;"
        in source,
        "workstation low-speed profile is not selected before move",
    )
    require(
        source.count("setAcceleration(") == 2,
        "unexpected acceleration configuration count",
    )
    print(
        "PASS static invariants:"
        " no mid-motion profile switch, all four run()/move() paths present,"
        " no default Serial conflict"
    )


def main() -> int:
    require(SOURCE_PATH.is_file(), f"source file does not exist: {SOURCE_PATH}")
    source = SOURCE_PATH.read_text(encoding="utf-8")
    source_without_comments = strip_cpp_comments(source)
    values = parse_numeric_constants(source_without_comments)
    required_constants = {
        "FIELD_SIZE_MM",
        "FIELD_CENTER_MM",
        "START_ZONE_MIN_MM",
        "START_CENTER_X_MM",
        "START_CENTER_Y_MM",
        "FINAL_ZONE_CENTER_X_MM",
        "FINAL_ZONE_CENTER_Y_MM",
        "CHASSIS_FOOTPRINT_X_MM",
        "CHASSIS_FOOTPRINT_Y_MM",
        "CHASSIS_HALF_WIDTH_MM",
        "ARM_CENTER_BEYOND_NEAR_WHEEL_MM",
        "ARM_CENTER_TO_FARTHEST_WHEEL_MM",
        "RAW_CENTER_Y_MM",
        "PROCESS_CENTER_Y_MM",
        "STORAGE_CENTER_X_MM",
        "WORKSTATION_APPROACH_MM",
        "QR_PASS_CENTER_X_MM",
        "QR_PASS_CENTER_Y_MM",
        "MAX_TRANSLATION_SEGMENT_MM",
        "FORWARD_PULSES_PER_METER",
        "LATERAL_PULSES_PER_METER",
        "PULSES_PER_WHEEL_REVOLUTION",
        "WHEELBASE_MM",
        "TRACK_WIDTH_MM",
        "WHEEL_DIAMETER_MM",
        "COUNTERCLOCKWISE_ROTATION_PULSE_SCALE",
        "CLOCKWISE_ROTATION_PULSE_SCALE",
        "MAXIMUM_STEP_RATE",
        "STEP_ACCELERATION",
        "TURN_MAXIMUM_STEP_RATE",
        "TURN_STEP_ACCELERATION",
        "WORKSTATION_MAXIMUM_STEP_RATE",
        "WORKSTATION_STEP_ACCELERATION",
        "FINAL_MAXIMUM_STEP_RATE",
        "FINAL_STEP_ACCELERATION",
        "ARM_BASE_PULSES_PER_DEGREE",
        "ARM_BASE_DEPLOY_ANGLE_DEGREES",
        "ARM_BASE_MAXIMUM_STEP_RATE",
        "ARM_BASE_STEP_ACCELERATION",
    }
    missing = sorted(required_constants - values.keys())
    require(not missing, f"unparsed constants: {missing}")
    require(
        "ARM_CENTER_OFFSET_MM" in values
        or "TOOL_OUTWARD_REACH_MM" in values,
        "neither arm-center offset nor tool outward reach was parsed",
    )
    if "ARM_CENTER_OFFSET_MM" in values:
        ring_constants = {
            "RAW_RING_CENTER_Y_MM",
            "PROCESS_RING_CENTER_Y_MM",
            "STORAGE_RING_CENTER_X_MM",
        }
        missing_ring_constants = sorted(
            ring_constants - values.keys()
        )
        require(
            not missing_ring_constants,
            f"unparsed ring constants: {missing_ring_constants}",
        )

    commands = parse_route(source_without_comments, values)
    movements, events = simulate_route(commands, values)
    verify_route_geometry(commands, movements, events, values)
    verify_wheel_pulses(commands, values)
    verify_motion_profiles(commands, values)
    verify_imu_unwrap()
    verify_arm_base_sequence(commands, values, source)
    verify_static_invariants(source)
    print(f"ALL PREFLIGHT CHECKS PASSED: {SOURCE_PATH}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (PreflightFailure, StopIteration) as error:
        print(f"PREFLIGHT FAILED: {error}", file=sys.stderr)
        sys.exit(1)
