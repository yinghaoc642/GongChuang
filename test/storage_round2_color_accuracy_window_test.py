#!/usr/bin/env python3

"""Contract for the 500 ms stopped-pose STORAGE2 survey window."""

from __future__ import annotations

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
    opening = code.index("{", start)
    depth = 1
    for index in range(opening + 1, len(code)):
        if code[index] == "{":
            depth += 1
        elif code[index] == "}":
            depth -= 1
            if depth == 0:
                return code[opening + 1 : index]
    raise AssertionError(f"unterminated function: {marker}")


source = compact(MAIN.read_text(encoding="utf-8"))

config = compact((ROOT / "lib/RobotConfig/src/RobotConfig.h").read_text(encoding="utf-8"))
assert "STORAGE2_SCAN_DWELL_MS=500UL;" in config
assert "STORAGE2_TARGET_COLOR_ROI_RADIUS_PIXELS=100U;" in config
assert "storageColorAccuracyWindowStartMs=nowMs;" in source

search = function_body(source, "voidbeginStorageColorSearch(")
for reset in (
    "storageColorAccuracyWindowStartMs=0UL;",
    "storageColorAccuracySumX=storageColorAccuracySumY=0;",
    "storageColorAccuracySampleCount=0U;",
):
    assert reset in search

service = function_body(source, "voidserviceCompetitionAction()")
phase = service.split("caseWORK_PHASE_STORAGE_COLOR_WAIT_COORDINATE:")[1].split(
    "caseWORK_PHASE_CIRCLE_WAIT_ARM_LOWER:"
)[0]
roi_index = phase.index("storage2CoordinateWithinTargetRoi(x,y)")
sample_index = phase.index("++storageColorAccuracySampleCount;", roi_index)
window_index = phase.index(
    "nowMs-storageColorAccuracyWindowStartMs<"
    "arm_config::STORAGE2_SCAN_DWELL_MS",
    sample_index,
)
repeat_index = phase.index("beginStorageColorVision();", window_index)
mean_x_index = phase.index("x=static_cast<int16_t>(", repeat_index)
mean_y_index = phase.index("y=static_cast<int16_t>(", mean_x_index)
map_index = phase.index("storageColorTargetPose(", mean_y_index)
assert roi_index < sample_index < window_index < repeat_index < mean_x_index < mean_y_index < map_index

print(
    "PASS STORAGE2 color accuracy: each of three targets gets a 500 ms "
    "targeted-color, center-ROI multi-request coordinate average"
)
