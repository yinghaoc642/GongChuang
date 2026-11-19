#!/usr/bin/env python3

"""Lock the debug-only STORAGE2 scan pose and coordinate-transform contract."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
DEBUG = (ROOT / "src" / "debug_workstation.cpp").read_text(encoding="utf-8")
CONFIG = (ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h").read_text(
    encoding="utf-8"
)


def compact(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"\s+", "", text)


def function_body(source: str, name: str) -> str:
    match = re.search(
        r"^\w[\w :*&<>]*\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{",
        source,
        flags=re.MULTILINE,
    )
    assert match, name
    opening = source.index("{", match.start())
    depth = 1
    for index in range(opening + 1, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return compact(source[opening + 1 : index])
    raise AssertionError(f"unterminated function: {name}")


compact_main = compact(MAIN)
compact_debug = compact(DEBUG)
compact_config = compact(CONFIG)

# The proven state-3 scan-pose calibration is now also the production default.
assert (
    "#ifndefGONGCHUANG_STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES"
    "#defineGONGCHUANG_STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES-5.0f"
    in compact_main
)
assert (
    "#ifndefGONGCHUANG_STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES"
    "#defineGONGCHUANG_STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES5.0f"
    in compact_main
)
assert (
    "#ifndefGONGCHUANG_STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM"
    "#defineGONGCHUANG_STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM30.0f"
    in compact_main
)

state3 = compact_debug.split(
    "#elifGONGCHUANG_DEBUG_WORKSTATION_STATE==3"
)[1].split("#else")[0]
assert (
    "#defineGONGCHUANG_STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES-5.0f"
    in state3
)
assert (
    "#defineGONGCHUANG_STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES5.0f"
    in state3
)
assert (
    "#defineGONGCHUANG_STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM30.0f"
    in state3
)

angle = function_body(MAIN, "storageColorSearchAngleForRing")
assert (
    "if(ringPosition==1U){angleDegrees+="
    "STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES;}"
    in angle
)
assert (
    "elseif(ringPosition==3U){angleDegrees+="
    "STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES;}"
    in angle
)

extension = function_body(MAIN, "storageColorSearchExtensionForRing")
assert (
    "storage2ScanExtensionMm+((ringPosition==1U||ringPosition==3U)?"
    "STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM:0.0f)"
    in extension
)

search = function_body(MAIN, "beginStorageColorSearch")
assert (
    "correctedScanExtensionMm="
    "storageColorSearchExtensionForRing(storageColorTargetRing)" in search
)
assert (
    "storageColorScanPose=ArmPose("
    "storageColorSearchAngleForRing(storageColorTargetRing),"
    "correctedScanExtensionMm,M7_STANDARD_HEIGHT_MM)" in search
)
assert (
    "startExtensionToMmWithProfile(storageColorScanPose.extensionMm," in search
)
assert (
    "startArmBaseStandardFrameDegrees("
    "storageColorScanPose.standardFrameAngleDegrees)" in search
)

service = function_body(MAIN, "serviceCompetitionAction")
wait_extension = service.split(
    "caseWORK_PHASE_STORAGE_COLOR_WAIT_EXTENSION:"
)[1].split("caseWORK_PHASE_STORAGE_COLOR_WAIT_COORDINATE:")[0]
assert (
    "fabsf(extensionAxis.currentMm-storageColorScanPose.extensionMm)" in wait_extension
)
assert (
    "storageColorScanPose=ArmPose(currentArmBaseStandardFrameDegrees(),"
    "extensionAxis.currentMm,liftAxis.currentMm)" in wait_extension
)

wait_coordinate = service.split(
    "caseWORK_PHASE_STORAGE_COLOR_WAIT_COORDINATE:"
)[1].split("caseWORK_PHASE_CIRCLE_WAIT_ARM_LOWER:")[0]
assert (
    "storageColorTargetPose(storageColorScanPose,storageColorTargetRing,x,y,targetPose)"
    in wait_coordinate
)

target_pose = function_body(MAIN, "storageColorTargetPose")
assert "STORAGE2_PIXEL_TO_MM_CORRECTION_SCALE=1.30f;" in compact_config
assert (
    "constfloatbaseMmPerPixel=rawMillimetersPerPixel(pixelRadius);"
    in target_pose
)
assert (
    "constfloatcorrectedMmPerPixel=baseMmPerPixel*"
    "arm_config::STORAGE2_PIXEL_TO_MM_CORRECTION_SCALE;" in target_pose
)
assert (
    "STORAGE2_PIXEL_TO_MM_CORRECTION_SCALE"
    not in function_body(MAIN, "rawMillimetersPerPixel")
)
assert (
    "STORAGE2_PIXEL_TO_MM_CORRECTION_SCALE"
    not in function_body(MAIN, "rawTargetPose")
)
assert (
    "STORAGE2_PIXEL_TO_MM_CORRECTION_SCALE"
    not in function_body(MAIN, "endpointVisionToPlanarPoint")
)
assert (
    "lroundf(RING_PHYSICAL_RADIUS_MM/baseMmPerPixel)" in target_pose
)
assert (
    "endpointVisionToPlanarPoint(scanPose,imageX,imageY,equivalentRadiusPixels,"
    "measuredPoint,correctedMmPerPixel)" in target_pose
)

# Restored correction directions in the standard M5 frame: 60/0/-60 becomes
# 55/0/-55; base M6 remains base+30/base/base+30 for rings 1/2/3.
base_angles = (60.0, 0.0, -60.0)
corrected_angles = (
    base_angles[0] - 5.0,
    base_angles[1],
    base_angles[2] + 5.0,
)
assert corrected_angles == (55.0, 0.0, -55.0)
base_extension = 40.0
assert (
    base_extension + 30.0,
    base_extension,
    base_extension + 30.0,
) == (70.0, 40.0, 70.0)

print(
    "PASS STORAGE2 debug scan pose: ring1 M5 -5deg, ring3 M5 +5deg, M6 +30mm, "
    "measured pose feeds coordinate conversion, and only STORAGE2 uses 1.30x scale"
)
