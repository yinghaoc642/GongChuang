#!/usr/bin/env python3

"""Check payload-gated arm speed/acceleration profiles in the real main entry."""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
DEBUG = (ROOT / "src" / "debug_workstation.cpp").read_text(encoding="utf-8")


def compact(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    source = re.sub(r"//[^\n]*", "", source)
    return re.sub(r"\s+", "", source)


code = compact(SOURCE)

for fragment in (
    "EMPTY_GRIPPER_SPEED_SCALE=1.30f;",
    "EMPTY_GRIPPER_ACCELERATION_SCALE=1.30f;",
    "M5_GLOBAL_RAMP_TIME_SCALE=1.15f;",
    "M5_GLOBAL_ACCELERATION_SCALE=1.0f/M5_GLOBAL_RAMP_TIME_SCALE;",
    "boolarmPayloadPresent=false;",
    "emptyGripperBoostedSpeedRpm(702U)==913U",
    "emptyGripperBoostedEmmAcceleration(199U)==212U",
    "emptyGripperBoostedSpeedRpm(6500U)==8450U",
    "emptyGripperBoostedEmmAcceleration(255U)==255U",
    "if(!armPayloadPresent){maximumStepRate*=EMPTY_GRIPPER_SPEED_SCALE;"
    "stepAcceleration*=EMPTY_GRIPPER_ACCELERATION_SCALE;}"
    "stepAcceleration*=M5_GLOBAL_ACCELERATION_SCALE;",
    "if(allowEmptyGripperBoost&&!recoveryRetry&&!armPayloadPresent){"
    "speedRpm=emptyGripperBoostedSpeedRpm(speedRpm);"
    "acceleration=emptyGripperBoostedEmmAcceleration(acceleration);}",
):
    assert fragment in code, f"empty-gripper profile contract missing: {fragment}"

# Payload becomes true only after source-close settling, and becomes false only
# after destination-open settling. Profile changes cannot race either servo.
close_case = code.index("caseARM_TRANSFER_WAIT_GRIP_CLOSE:")
close_latch = code.index("armPayloadPresent=true;", close_case)
loaded_lift = code.index("startArmTransferLiftToHeightMm(", close_latch)
assert close_case < close_latch < loaded_lift

open_case = code.index("caseARM_TRANSFER_WAIT_DESTINATION_OPEN:")
open_latch = code.index("armPayloadPresent=false;", open_case)
empty_lift = code.index("startArmTransferLiftToHeightMm(", open_latch)
assert open_case < open_latch < empty_lift

# Final contact and zero soft-landing commands explicitly opt out of the boost.
assert code.count("M7_PROFILE_APPLY_EXPERIMENT,false)") >= 3
assert "finalM7ProfilePolicy,false)" in code
assert "m7ProfilePolicy,false);" in code

# The workstation debug firmware includes the production state machine; it
# must not carry a second copy of payload-profile values.
assert '#include "main.cpp"' in DEBUG
assert "EMPTY_GRIPPER_SPEED_SCALE" not in DEBUG

print(
    "PASS empty-gripper arm profile: M5/M6 +30% speed/physical acceleration, "
    "then global M5 symmetric ramp time +15%; "
    "M7 +30% speed with acceleration saturated at 255; loaded and soft-landing "
    "motions retain their profiles"
)
