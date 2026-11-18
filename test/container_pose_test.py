"""Compile the real tray pose planner and verify shared planar calibration."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PLANNER = ROOT / "lib/ArmTransferPlanner/src"
CONFIG = ROOT / "lib/RobotConfig/src"

harness = r'''
#include <ArmTransferPlanner.h>
#include <cassert>
#include <cmath>
#include <iostream>
using gongchuang::ArmPose;
namespace at = gongchuang::arm_transfer;

bool close(float actual, float expected) {
  return std::fabs(actual - expected) < 0.0001f;
}
void check(const ArmPose &pose, float height) {
  assert(close(pose.standardFrameAngleDegrees, -104.0f));
  assert(close(pose.extensionMm, 4.0f));
  assert(close(pose.heightMm, height));
}
int main() {
  // Repeated calls and interleaved operations must never accumulate offsets.
  for(int repetition=0; repetition<20; ++repetition) {
    check(at::containerPlacePose(), -30.0f);
    for(uint8_t item=0; item<3; ++item) {
      check(at::containerPickPose(item), item==1 ? -36.0f : -33.0f);
      check(at::containerReturnPlacePose(), -32.0f);
    }
    ArmPose callerCopy=at::containerPickPose(0);
    callerCopy.standardFrameAngleDegrees=0.0f;
    callerCopy.extensionMm=100.0f;
    check(at::containerPickPose(0), -33.0f);
    check(at::containerPlacePose(), -30.0f);
  }
  std::cout << "PASS actual tray planner: all picks/placements M5=-104 deg, M6=4 mm; "
               "M7 heights and second-pick depth preserved, no accumulated correction\n";
}
'''
compiler = shutil.which("g++")
assert compiler, "host C++ compiler is required"
with tempfile.TemporaryDirectory(prefix="gc_container_pose_") as temporary:
    source = Path(temporary) / "container_pose.cpp"
    executable = Path(temporary) / "container_pose_test"
    source.write_text(harness, encoding="utf-8")
    subprocess.run([
        compiler, "-std=c++17", "-O0",
        "-I", str(PLANNER), "-I", str(CONFIG),
        str(source), str(PLANNER / "ArmTransferPlanner.cpp"),
        "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
