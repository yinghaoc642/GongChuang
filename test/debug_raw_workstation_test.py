#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PLATFORMIO = (ROOT / "platformio.ini").read_text(encoding="utf-8")
ENTRY = (ROOT / "src" / "debug_raw_workstation.cpp").read_text(
    encoding="utf-8"
)
ENTRY_341 = (ROOT / "src" / "debug_raw_workstation_341.cpp").read_text(
    encoding="utf-8"
)
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")


section_match = re.search(
    r"\[env:debug_raw_workstation\](.*?)(?=\n\[env:|\Z)",
    PLATFORMIO,
    flags=re.DOTALL,
)
assert section_match is not None
section = section_match.group(1)
assert "extends = env:genericSTM32H750VB" in section
assert "-DGONGCHUANG_MAIX_COORDINATE_TELEMETRY=1" in section
assert "-DGONGCHUANG_RAW_VERBOSE_VISION_CALIBRATION=1" in section
assert "build_src_filter = -<*> +<debug_raw_workstation.cpp>" in section

assert "#define GONGCHUANG_DEBUG_RAW_ROUND 1" in ENTRY
assert "#define GONGCHUANG_RAW_PICKUP_LOGIC_MODE 0" in ENTRY
assert "#define GONGCHUANG_RAW_PICKUP_LOGIC_MODE 0" in MAIN
assert "#define GONGCHUANG_DEBUG_RAW 1" in ENTRY
assert '#include "main.cpp"' in ENTRY
assert '#include "debug_workstation.cpp"' not in ENTRY

assert "RAW_WORKSTATION_SIMULATION_MODE" in MAIN
assert "RAW_SIMULATION_TASK_CODE[]" in MAIN
assert '"123+132+123+132"' in MAIN
assert "GONGCHUANG_DEBUG_RAW_TASK_CODE" in MAIN
assert '#define GONGCHUANG_DEBUG_RAW_TASK_CODE "341+123+413+231"' in ENTRY_341
assert "#define GONGCHUANG_RAW_PICKUP_LOGIC_MODE 0" in ENTRY_341
assert "#define GONGCHUANG_DEBUG_RAW 1" in ENTRY_341
assert '#include "main.cpp"' in ENTRY_341
assert "local legacy fixed-A pickup started" in MAIN
assert "chassis motion blocked" in MAIN
assert (
    "if (LOCAL_WORKSTATION_SIMULATION_MODE &&\n"
    "      !(RAW_WORKSTATION_SIMULATION_MODE &&\n"
    "        GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1))" in MAIN
)
assert "[RAW SEARCH MOVE] from/to/M5/M6/chassis-left-mm=" in MAIN
assert "beginWorkAction(\n          WORK_ACTION_RAW" in MAIN
assert "serviceRawWorkstationSimulation();" in MAIN
assert "rawSimulationParkingDeadlineMs =\n        workStorageServoDeadlineMs" in MAIN
assert "deadlineReached(rawSimulationParkingDeadlineMs)" in MAIN
assert "!rawChassisAtB" in MAIN
assert "motorPulsesAreZero(visualCorrectionAccumulator)" in MAIN
assert "fabsf(visualCorrectionLeftMm) <= 0.01f" in MAIN
assert 'hmiSetRunStatus("RAWDONE")' in MAIN
assert "if (LOCAL_WORKSTATION_SIMULATION_MODE ||" in MAIN
assert '#include "debug_raw_workstation.cpp"' not in MAIN

print(
    "PASS dedicated RAW workstation debug: fixed task, selectable round, "
    "legacy fixed-A logic selected by default, route disabled and chassis "
    "motion blocked; archived A/B implementation retained behind mode 1"
)
