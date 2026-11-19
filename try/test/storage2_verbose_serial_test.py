#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
DEBUG = (ROOT / "src" / "debug_workstation.cpp").read_text(encoding="utf-8")


assert re.search(
    r"#ifndef GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG\s+"
    r"#define GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG 0",
    MAIN,
)
state3 = DEBUG.split("#elif GONGCHUANG_DEBUG_WORKSTATION_STATE == 3", 1)[1]
state3 = state3.split("#else", 1)[0]
assert "#define GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG 1" in state3

# The diagnostic build must make each decision boundary visible without
# changing the production scan state machine.
for marker in (
    "[S2DBG ENABLED]",
    "[S2DBG VISIT]",
    "[S2DBG POSE]",
    "[S2DBG REQUEST]",
    "[S2DBG FRAME]",
    "[S2DBG ROI] reject",
    "target-coordinate-sampled",
    "target-rejected-outside-roi",
    "result=miss-rescan",
    "result=coordinate",
    "[S2DBG NEXT]",
    "[S2DBG COMPLETE]",
    "[S2DBG TARGET]",
    "[S2DBG RESULT]",
    "MISS_NO_VALID_RESPONSE",
    "MISS_TARGET_OUTSIDE_ROI",
    "MISS_WRONG_COLOR",
    "ACCEPTED_TARGET_COORDINATE",
    "[S2DBG EXPECT]",
    "[S2DBG PROTOCOL]",
    "[S2DBG FIXTURE]",
    "[S2DBG FAULT]",
    "[S2DBG ARM]",
    "[S2DBG CAM]",
    "[S2DBG MAPSTATE]",
):
    assert marker in MAIN, marker

assert "printStorage2VerboseFaultSnapshot(reason);" in MAIN
for field in (
    "latest.metric",
    "latest.confidence",
    "latest.requestSequence",
    "maixCam.requestedMode()",
    "storageColorAccuracySampleCount",
    "storageColorAccuracyWindowStartMs",
    "storage2ItemOrder",
    "measuredRingPoseValid",
    "storage2WindowFreshFrameCount",
    "storage2WindowOutsideRoiCount",
    "storage2WindowWrongColorCount",
):
    assert field in MAIN, field

print(
    "PASS STORAGE2 verbose serial: debug-only enable, per-frame decisions, "
    "rescan reasons, map progress and pre-stop fault snapshot"
)
