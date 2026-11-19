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
    "STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM=0.0f;"
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
    "applyRingPlacementPoseCorrection(destination,ringPosition,"
    "&activePlacementRing2AngleCorrectionApplied)"
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
assert "LOCAL_WORKSTATION_SIMULATION_MODE" not in final_correction
assert "LOCAL_WORKSTATION_SIMULATION_MODE" not in function_body(
    source, "voidremoveRingPositionPoseCorrectionForSavedBase("
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
    match = re.search(rf"\b(?:bool|void)\s+{name}\s*\([^;{{]*\)\s*{{", raw_source)
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
    "WORKSTATION_PLACE_EXTRA_M5_CLOCKWISE_DEGREES",
    "WORKSTATION_PLACE_EXTRA_M6_RETRACTION_MM",
    "STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES",
    "STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM",
    "WORKSTATION_PLACE_EXTRA_M6_EXTENSION_MM",
    "WORKSTATION_RING2_PLACE_ANGLE_THRESHOLD_DEGREES",
    "WORKSTATION_RING2_PLACE_EXTRA_M6_RETRACTION_MM",
    "WORKSTATION_RING2_PLACE_EXTRA_M5_CLOCKWISE_DEGREES",
    "WORKSTATION_RING_RETURN_M5_NET_CCW_DEGREES",
    "WORKSTATION_RING_RETURN_M6_NET_EXTENSION_MM",
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
bool LOCAL_WORKSTATION_SIMULATION_MODE = false;
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
        "ring2PlacementAngleCorrectionApplies", "applyRingPlacementPoseCorrection", "armPoseIsValid",
        "applyRingPositionPoseCorrection", "removeRingPositionPoseCorrectionForSavedBase",
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
  // Final angle 31.75 deg triggers another 2 mm retraction, then the final clamp.
  expect(2, 4.45f, -9.5f);
  // STORAGE2 outer placements retract 5 mm from their previous targets.
  expect(1, 102.01f, 84.309f);
  expect(2, 44.69f, 29.19f);
  expect(3, 104.91f, 90.119f);
  // Ring3 intermediate +5 exceeds 160, but final corrected target is in range.
  expect(3, 158.0f, 137.9f);
  // Values below the minimum in intermediate steps must reach the final clamp.
  for (uint8_t ring = 1; ring <= 3; ++ring) {
    expect(ring, -20.0f, -9.5f);
    expect(ring, 250.0f, 160.0f);
    // Exercise each exact final boundary, and either side of it, after all offsets.
    for (float boundary : {-9.5f, 160.0f}) {
      for (float offset : {-0.1f, 0.0f, 0.1f}) {
        const float desired = boundary + offset;
        float mapped = desired + WORKSTATION_PLACE_EXTRA_M6_RETRACTION_MM
            - WORKSTATION_PLACE_EXTRA_M6_EXTENSION_MM
            - WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[ring-1];
        if (ring != 2)
          mapped = (mapped - arm_config::STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM)
              / arm_config::STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE;
        if (ring == 3) mapped -= RING3_EXTENSION_CORRECTION_MM;
        if (ring == 2) mapped += WORKSTATION_RING2_PLACE_EXTRA_M6_RETRACTION_MM;
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
      ArmPose finalTooShort{30.0f, -5.0f, -85.0f};
      assert(applyRingPlacementPoseCorrection(finalTooShort, 1));
      assert(!armPoseIsValid(finalTooShort, M6_RING2_MINIMUM_EXTENSION_MM));
    }
  }
  // Debug and production share the +15 mm correction after the outer-ring
  // scale, before final limits; mode selection must not change target poses.
  activeWorkAction = WORK_ACTION_STORAGE;
  workRoundIndex = 1;
  LOCAL_WORKSTATION_SIMULATION_MODE = true;
  expect(1, 102.01f, 84.309f);
  expect(2, 44.69f, 29.19f);
  expect(3, 104.91f, 90.119f);
  expect(2, 4.45f, -9.5f);
  for (uint8_t ring = 1; ring <= 3; ++ring) {
    expect(ring, -100.0f, -9.5f);
    expect(ring, 250.0f, 160.0f);
    for (float boundary : {-9.5f, 160.0f}) {
      for (float offset : {-0.1f, 0.0f, 0.1f}) {
        const float desired = boundary + offset;
        float mapped = desired - WORKSTATION_PLACE_EXTRA_M6_EXTENSION_MM
            + WORKSTATION_PLACE_EXTRA_M6_RETRACTION_MM
            - WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[ring-1];
        if (ring != 2)
          mapped = (mapped - arm_config::STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM)
              / arm_config::STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE;
        if (ring == 3) mapped -= RING3_EXTENSION_CORRECTION_MM;
        if (ring == 2) mapped += WORKSTATION_RING2_PLACE_EXTRA_M6_RETRACTION_MM;
        mapped += reduction(ring);
        expect(ring, mapped, desired < -9.5f ? -9.5f : desired > 160.0f ? 160.0f : desired);
      }
    }
  }
  // All stations/rings: preserve vision bases and prevent accumulated offsets
  // when a completed placement is saved and another target is generated.
  for (int action : {WORK_ACTION_PROCESS, WORK_ACTION_STORAGE}) {
    for (uint8_t round = 0; round <= 1; ++round) {
      activeWorkAction = action;
      workRoundIndex = round;
      for (uint8_t ring = 1; ring <= 3; ++ring) {
        ArmPose production{30.0f, 100.0f, -85.0f};
        ArmPose debug = production;
        bool productionExtraApplied = false;
        bool debugExtraApplied = false;
        for (int repeat = 0; repeat < 3; ++repeat) {
          LOCAL_WORKSTATION_SIMULATION_MODE = false;
          assert(applyRingPlacementPoseCorrection(production, ring, &productionExtraApplied));
          if (repeat == 0) {
            // Fixed outputs for a 30 deg / 100 mm base: global CW 1 deg and
            // retraction 5 mm plus extension 15 mm reach every station visit.
            const float processAngles[] = {30.92f, 31.45f, 30.62f};
            const float processExtensions[] = {92.5f, 94.5f, 93.0f};
            const float storage1Angles[] = {30.72f, 31.45f, 30.62f};
            const float storage1Extensions[] = {93.5f, 94.5f, 93.0f};
            // STORAGE2 only: ring1 CW 0.5 deg, ring3 CCW 0.5 deg; both -5 mm.
            const float storage2Angles[] = {29.92f, 31.45f, 30.62f};
            const float storage2Extensions[] = {82.5f, 94.5f, 87.5f};
            const float* angles = action == WORK_ACTION_PROCESS ? processAngles
                : round == 0 ? storage1Angles : storage2Angles;
            const float* extensions = action == WORK_ACTION_PROCESS ? processExtensions
                : round == 0 ? storage1Extensions : storage2Extensions;
            assert(std::fabs(production.standardFrameAngleDegrees - angles[ring-1]) < 0.001f);
            assert(std::fabs(production.extensionMm - extensions[ring-1]) < 0.001f);
          }
          LOCAL_WORKSTATION_SIMULATION_MODE = true;
          assert(applyRingPlacementPoseCorrection(debug, ring, &debugExtraApplied));
          assert(productionExtraApplied == (ring == 2));
          assert(debugExtraApplied == productionExtraApplied);
          assert(std::fabs(debug.extensionMm - production.extensionMm) < 0.001f);
          assert(debug.standardFrameAngleDegrees == production.standardFrameAngleDegrees);
          assert(debug.heightMm == production.heightMm);
          assert(armPoseIsValid(debug, M6_RING2_MINIMUM_EXTENSION_MM));
          LOCAL_WORKSTATION_SIMULATION_MODE = false;
          removeRingPositionPoseCorrectionForSavedBase(production, ring, productionExtraApplied);
          LOCAL_WORKSTATION_SIMULATION_MODE = true;
          removeRingPositionPoseCorrectionForSavedBase(debug, ring, debugExtraApplied);
          assert(std::fabs(debug.extensionMm - production.extensionMm) < 0.001f);
          assert(debug.standardFrameAngleDegrees == production.standardFrameAngleDegrees);
          assert(std::fabs(production.standardFrameAngleDegrees - 30.0f) < 0.001f);
          if (action == WORK_ACTION_STORAGE && round == 1 && repeat == 0) {
            const float savedExtensions[] = {90.0f, 100.0f, 89.5f};
            assert(std::fabs(production.extensionMm - savedExtensions[ring-1]) < 0.001f);
          }
          if (action == WORK_ACTION_PROCESS || round == 0)
            assert(std::fabs(production.extensionMm - 100.0f) < 0.001f);
        }
        ArmPose productionReturn = production;
        ArmPose debugReturn = debug;
        LOCAL_WORKSTATION_SIMULATION_MODE = false;
        assert(applyRingPositionPoseCorrection(productionReturn, ring));
        LOCAL_WORKSTATION_SIMULATION_MODE = true;
        assert(applyRingPositionPoseCorrection(debugReturn, ring));
        assert(std::fabs(debugReturn.extensionMm - productionReturn.extensionMm) < 0.001f);
        assert(debugReturn.standardFrameAngleDegrees == productionReturn.standardFrameAngleDegrees);
        // Placement-only changes must not alter pickups from a fixed vision base.
        ArmPose fixedReturn{30.0f, 100.0f, -85.0f};
        assert(applyRingPositionPoseCorrection(fixedReturn, ring));
        const float returnAngles[] = {32.0f, 32.5f, 32.0f};
        const float returnExtensions[] = {83.0f, 87.5f, 84.0f};
        const float ring3Extra = action == WORK_ACTION_STORAGE && round == 1 && ring == 3
            ? 5.0f : 0.0f;
        assert(std::fabs(fixedReturn.standardFrameAngleDegrees - returnAngles[ring-1]) < 0.001f);
        assert(std::fabs(fixedReturn.extensionMm - returnExtensions[ring-1] - ring3Extra) < 0.001f);
      }
    }
  }
  // Judge after existing +1.75 deg net, before the new CW0.3/-2 mm pair.
  // Preserve the application flag when 3.01 -> 2.71 deg crosses the threshold.
  struct AngleCase {float finalAngle; bool retract;};
  const AngleCase angleCases[] = {
      {-4.0f,true}, {-3.01f,true}, {-3.0f,false}, {-2.99f,false},
      {0.0f,false}, {2.99f,false}, {3.0f,false}, {3.01f,true}, {4.0f,true}};
  for (int action : {WORK_ACTION_PROCESS, WORK_ACTION_STORAGE}) {
    for (uint8_t round = 0; round <= 1; ++round) {
      activeWorkAction = action; workRoundIndex = round;
      for (bool debug : {false, true}) {
        LOCAL_WORKSTATION_SIMULATION_MODE = debug;
        for (const AngleCase& value : angleCases) {
          const float baseAngle = value.finalAngle - 1.75f;
          ArmPose pose{baseAngle, 100.0f, -85.0f};
          for (int repeat = 0; repeat < 3; ++repeat) {
            bool extraApplied = !value.retract; // verify stale flags are overwritten
            assert(applyRingPlacementPoseCorrection(pose, 2, &extraApplied));
            assert(extraApplied == value.retract);
            const float expectedAngle = value.finalAngle - (value.retract ? 0.3f : 0.0f);
            assert(std::fabs(pose.standardFrameAngleDegrees-expectedAngle)<0.001f);
            assert(std::fabs(pose.extensionMm-(value.retract ? 94.5f : 96.5f))<0.001f);
            assert(pose.heightMm == -85.0f);
            removeRingPositionPoseCorrectionForSavedBase(pose, 2, extraApplied);
            assert(std::fabs(pose.standardFrameAngleDegrees-baseAngle)<0.001f);
            assert(std::fabs(pose.extensionMm-100.0f)<0.001f);
          }
          assert(applyRingPositionPoseCorrection(pose, 2));
          assert(std::fabs(pose.extensionMm-87.5f)<0.001f);
          assert(std::fabs(pose.standardFrameAngleDegrees-(baseAngle+2.5f))<0.001f);
        }
        // Existing travel handling remains in force after the new retraction.
        ArmPose nearLimit{2.25f, -5.0f, -85.0f}; // final 4 deg: -5-3.5-2=-10.5
        assert(applyRingPlacementPoseCorrection(nearLimit, 2));
        if (action == WORK_ACTION_STORAGE && round == 1) {
          assert(nearLimit.extensionMm == -9.5f);
          assert(armPoseIsValid(nearLimit, M6_RING2_MINIMUM_EXTENSION_MM));
        } else {
          assert(nearLimit.extensionMm == -10.5f);
          assert(!armPoseIsValid(nearLimit, M6_RING2_MINIMUM_EXTENSION_MM));
        }
      }
    }
  }
  std::cout << "PASS actual C++ ring2: corrected angle outside [-3,+3] adds -2 mm / CW0.3 deg; "
               "inclusive endpoints, both rounds/stations/modes, saved bases and pickups preserved\n";
  std::cout << "PASS actual C++: shared CW 1 deg / net extension 10 mm in all four visits, identical production/debug, final limits, "
               "STORAGE2 ring1 CW0.5/ring3 CCW0.5 and both -5 mm, saved-base normalization, unchanged return poses and M7 guards\n";
}
'''
compiler = shutil.which("g++")
assert compiler, "host g++ is required for the actual C++ placement test"
with tempfile.TemporaryDirectory(prefix="gongchuang_storage2_limits_") as temporary:
    test_cpp = Path(temporary) / "placement_limits.cpp"
    executable = Path(temporary) / "placement_limits.exe"
    test_cpp.write_text(harness, encoding="utf-8")
    subprocess.run(
        [compiler, "-std=c++17", "-O0", str(test_cpp), "-o", str(executable)],
        check=True,
    )
    subprocess.run([str(executable)], check=True)

print(
    "PASS STORAGE2 integration: production unloading applies all M6 corrections "
    "before transfer, using the shared main.cpp in debug firmware"
)
