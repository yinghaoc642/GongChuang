#!/usr/bin/env python3

"""Check STORAGE2 correction order and run the actual C++ placement limits."""

from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.cpp"
CONFIG = ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h"


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


raw_source = MAIN.read_text(encoding="utf-8")
raw_config = CONFIG.read_text(encoding="utf-8")
source = compact(raw_source)
config = compact(raw_config)

assert "STORAGE2_RING1_TARGET_PLACE_EXTENSION_REDUCTION_MM=0.0f;" in config
assert "STORAGE2_MIDDLE_TARGET_PLACE_EXTENSION_REDUCTION_MM=10.0f;" in config
assert "STORAGE2_RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM=2.0f;" in config
assert (
    "STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE=0.90f;"
    in config
)
assert (
    "STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM=5.0f;"
    in config
)
assert "STORAGE2_FINAL_PLACE_M6_EXTENSION_SCALE" not in config

unloading = function_body(source, "voidbeginUnloadingTransfer()")
mapped_index = unloading.index("if(!ringPose(ringPosition,lowerMm,destination))")
storage2_index = unloading.index("secondStorageColorWorkflowActive()", mapped_index)
lowering_index = unloading.index("applyTargetPlacementExtraLower(", storage2_index)
assert mapped_index < storage2_index < lowering_index

compensation = unloading[storage2_index:lowering_index]
assert (
    "ringPosition==2U?"
    "arm_config::STORAGE2_MIDDLE_TARGET_PLACE_EXTENSION_REDUCTION_MM:"
    "arm_config::STORAGE2_RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM"
    in compensation
)
assert (
    "applyTargetPlacementExtraLower(destination,"
    "mappedRingPlacementExtensionReductionMm,ringPosition,"
    "secondStorageColorWorkflowActive())" in unloading
)
placement_correction_index = unloading.index(
    "applyRingPlacementPoseCorrection(destination,ringPosition)"
)
transfer_index = unloading.index("beginArmTransfer(")
assert (
    lowering_index
    < placement_correction_index
    < transfer_index
)
final_correction = function_body(
    source, "boolapplyRingPlacementPoseCorrection("
)
assert (
    "pose.extensionMm*=arm_config::"
    "STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE;" in final_correction
)
assert (
    "pose.extensionMm+=WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM["
    "ringPosition-1U];" in final_correction
)
assert final_correction.index(
    "STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE"
) < final_correction.index("WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM")
assert final_correction.index(
    "WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM"
) < final_correction.index(
    "STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM"
)


def declaration(code: str, name: str) -> str:
    match = re.search(
        rf"constexpr\s+float\s+{re.escape(name)}\s*(?:\[[^]]+\])?\s*=[^;]+;",
        code,
    )
    assert match, f"missing constant {name}"
    return match.group()


def cpp_function(name: str) -> str:
    match = re.search(rf"\bbool\s+{name}\s*\([^;{{]*\)\s*{{", raw_source)
    assert match, f"missing function {name}"
    body = function_body(raw_source, match.group())
    return match.group() + body + "}"


# Compile the real correction functions, rather than duplicating their math in
# Python. Only hardware I/O is replaced; constants come from current sources.
config_names = (
    "M6_STARTUP_WORKING_ZERO_OFFSET_MM", "M7_STARTUP_WORKING_ZERO_OFFSET_MM",
    "TARGET_PLACE_EXTRA_LOWER_MM", "STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE",
    "STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM",
    "STORAGE2_RING1_TARGET_PLACE_EXTENSION_REDUCTION_MM",
    "STORAGE2_MIDDLE_TARGET_PLACE_EXTENSION_REDUCTION_MM",
    "STORAGE2_RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM",
)
main_names = (
    "M6_STARTUP_WORKING_ZERO_OFFSET_MM", "M7_STARTUP_WORKING_ZERO_OFFSET_MM",
    "M6_STANDARD_EXTENSION_MM", "M6_RING2_MINIMUM_EXTENSION_MM",
    "M6_MAXIMUM_PHYSICAL_EXTENSION_MM", "M6_MAXIMUM_EXTENSION_MM",
    "M7_STANDARD_HEIGHT_MM", "M7_MINIMUM_PHYSICAL_HEIGHT_MM", "M7_MINIMUM_HEIGHT_MM",
    "ARM_AXIS_POSITION_TOLERANCE_MM", "RING3_EXTENSION_CORRECTION_MM",
    "PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES",
    "STORAGE2_OUTER_RING_PLACE_M5_DIFFERENCE_CCW_DEGREES",
    "WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM",
    "STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES",
    "STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM",
)
default_ring3 = re.search(
    r"^#define GONGCHUANG_RING3_EXTENSION_CORRECTION_MM (\d+(?:\.\d+)?f)$",
    raw_source, re.MULTILINE,
)
assert default_ring3
harness = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <iostream>
struct ArmPose { float standardFrameAngleDegrees, extensionMm, heightMm; };
struct DebugSink {
  template<class... T> void print(T...) {}
  template<class... T> void println(T...) {}
} SerialDebug;
enum { WORK_ACTION_PROCESS, WORK_ACTION_STORAGE };
int activeWorkAction = WORK_ACTION_STORAGE;
uint8_t workRoundIndex = 1;
bool fault = false;
void routeFault(const char*) { fault = true; }
'''
harness += (
    "\n#define GONGCHUANG_RING3_EXTENSION_CORRECTION_MM " + default_ring3.group(1)
    + "\nnamespace gongchuang::config::arm_hardware {\n"
    + "\n".join(declaration(raw_config, name) for name in config_names[:2])
    + "\n}\nnamespace arm_config {\n"
    + "\n".join(declaration(raw_config, name) for name in config_names[2:])
    + "\n}\n"
    + "\n".join(declaration(raw_source, name) for name in main_names)
    + "\nvoid logTargetPlacementM6Limit(uint8_t, float, float) {}\n"
    + "\n".join(cpp_function(name) for name in (
        "secondStorageColorWorkflowActive", "processOrStorage1RingNetCorrectionApplies",
        "storage1Ring1PlacementExtraCorrectionApplies", "workstationRingNetCorrectionApplies",
        "applyTargetPlacementExtraLower", "applyRing3ExtensionCorrection",
        "applyRingPlacementPoseCorrection", "armPoseIsValid",
    ))
)
harness += r'''
float reduction(uint8_t ring) {
  return ring == 1 ? arm_config::STORAGE2_RING1_TARGET_PLACE_EXTENSION_REDUCTION_MM
      : ring == 2 ? arm_config::STORAGE2_MIDDLE_TARGET_PLACE_EXTENSION_REDUCTION_MM
                  : arm_config::STORAGE2_RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM;
}
ArmPose place(uint8_t ring, float mapped) {
  fault = false;
  ArmPose pose{30.0f, mapped, -80.0f};
  assert(applyTargetPlacementExtraLower(pose, reduction(ring), ring, true));
  assert(applyRingPlacementPoseCorrection(pose, ring));
  assert(!fault);
  assert(armPoseIsValid(pose, M6_RING2_MINIMUM_EXTENSION_MM));
  assert(std::fabs(pose.heightMm + 85.0f) < 0.001f);
  return pose;
}
void expect(uint8_t ring, float mapped, float target) {
  assert(std::fabs(place(ring, mapped).extensionMm - target) < 0.001f);
}
int main() {
  // Logged input with current net: 4.45 - 10 - 13.5 = -19.05 mm clamps to -9.5.
  expect(2, 4.45f, -9.5f);
  // Regression: all valid, unclipped targets retain the current correction chain.
  expect(1, 102.01f, 79.309f);
  expect(2, 44.69f, 21.19f);
  expect(3, 104.91f, 85.119f);
  // Ring3 intermediate +5 exceeds 160, but final corrected target is in range.
  expect(3, 158.0f, 132.9f);
  // Values below the minimum in intermediate steps must reach the final clamp.
  for (uint8_t ring = 1; ring <= 3; ++ring) {
    expect(ring, -20.0f, -9.5f);
    expect(ring, 250.0f, 160.0f);
    // Exercise each exact final boundary, and either side of it, after all offsets.
    for (float boundary : {-9.5f, 160.0f}) {
      for (float offset : {-0.1f, 0.0f, 0.1f}) {
        const float desired = boundary + offset;
        float mapped = desired - WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[ring-1];
        if (ring != 2)
          mapped = (mapped - arm_config::STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM)
              / arm_config::STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE;
        if (ring == 3) mapped -= RING3_EXTENSION_CORRECTION_MM;
        mapped += reduction(ring);
        expect(ring, mapped, desired < -9.5f ? -9.5f : desired > 160.0f ? 160.0f : desired);
      }
    }
  }
  // M7 invalid travel and non-finite M6 values still fail the normal safety gate.
  fault = false;
  ArmPose badHeight{30.0f, 30.0f, -151.0f};
  assert(!applyTargetPlacementExtraLower(badHeight, 0.0f, 1, true) && fault);
  ArmPose badNumber{30.0f, std::numeric_limits<float>::quiet_NaN(), -85.0f};
  assert(applyRingPlacementPoseCorrection(badNumber, 1));
  assert(!armPoseIsValid(badNumber, M6_RING2_MINIMUM_EXTENSION_MM));
  // Both PROCESS rounds and STORAGE1 retain their prior rejection behavior.
  for (int action : {WORK_ACTION_PROCESS, WORK_ACTION_STORAGE}) {
    for (uint8_t round = 0; round <= 1; ++round) {
      if (action == WORK_ACTION_STORAGE && round == 1) continue;
      activeWorkAction = action;
      workRoundIndex = round;
      fault = false;
      ArmPose tooShort{30.0f, -20.0f, -80.0f};
      assert(!applyTargetPlacementExtraLower(tooShort, 0.0f, 1) && fault);
      fault = false;
      ArmPose tooLong{30.0f, 158.0f, -85.0f};
      assert(!applyRing3ExtensionCorrection(tooLong, 3) && fault);
      ArmPose finalTooShort{30.0f, 0.0f, -85.0f};
      assert(applyRingPlacementPoseCorrection(finalTooShort, 1));
      assert(!armPoseIsValid(finalTooShort, M6_RING2_MINIMUM_EXTENSION_MM));
    }
  }
  std::cout << "PASS actual C++: STORAGE2 final M6 limits, correction order, "
               "boundary cases, unchanged other stations and M7 guards\n";
}
'''
compiler = shutil.which("g++")
assert compiler, "host g++ is required for the actual C++ placement test"
with tempfile.TemporaryDirectory(prefix="gongchuang_storage2_limits_") as temporary:
    test_cpp = Path(temporary) / "placement_limits.cpp"
    executable = Path(temporary) / "placement_limits.exe"
    test_cpp.write_text(harness, encoding="utf-8")
    subprocess.run(
        [compiler, "-std=c++17", "-O0", "-static", str(test_cpp), "-o", str(executable)],
        check=True,
    )
    subprocess.run([str(executable)], check=True)

print(
    "PASS STORAGE2 integration: production unloading applies all M6 corrections "
    "before transfer, using the shared main.cpp in debug firmware"
)
