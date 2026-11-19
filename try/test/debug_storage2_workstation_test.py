#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PLATFORMIO = (ROOT / "platformio.ini").read_text(encoding="utf-8")
ENTRY = (ROOT / "src" / "debug_storage2_workstation.cpp").read_text(
    encoding="utf-8"
)
SHARED = (ROOT / "src" / "debug_workstation.cpp").read_text(encoding="utf-8")
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")


storage2_section_match = re.search(
    r"\[env:debug_storage2_workstation\](.*?)(?=\n\[env:|\Z)",
    PLATFORMIO,
    flags=re.DOTALL,
)
assert storage2_section_match is not None
storage2_section = storage2_section_match.group(1)
assert "extends = env:genericSTM32H750VB" in storage2_section
assert "-DGONGCHUANG_MAIX_COORDINATE_TELEMETRY=1" in storage2_section
assert (
    "build_src_filter = -<*> +<debug_storage2_workstation.cpp>"
    in storage2_section
)
assert "#define GONGCHUANG_DEBUG_WORKSTATION_STATE 3" in ENTRY
assert '#include "debug_workstation.cpp"' in ENTRY
assert '#include "main.cpp"' not in ENTRY

assert "#ifndef GONGCHUANG_DEBUG_WORKSTATION_STATE" in SHARED
assert "#define GONGCHUANG_DEBUG_WORKSTATION_STATE 2" in SHARED
assert "#elif GONGCHUANG_DEBUG_WORKSTATION_STATE == 3" in SHARED
assert (
    "#define GONGCHUANG_STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES -5.0f"
    in MAIN
)
assert (
    "#define GONGCHUANG_STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES 5.0f"
    in MAIN
)
assert (
    "#define GONGCHUANG_STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM 30.0f"
    in MAIN
)
assert "#define GONGCHUANG_STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES" not in SHARED
assert "#define GONGCHUANG_STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES" not in SHARED
assert "#define GONGCHUANG_STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM" not in SHARED
assert "#define GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG 1" in SHARED
assert "#define GONGCHUANG_RUN_MODE 2" in SHARED
assert '#include "main.cpp"' in SHARED

# Every local station inherits all physical parameters from production. Only
# station selection and detailed logging may be configured by the wrapper.
assert set(re.findall(r"^\s*#define\s+(\w+)", SHARED, re.MULTILINE)) == {
    "GONGCHUANG_DEBUG_WORKSTATION_STATE",
    "GONGCHUANG_RUN_MODE",
    "GONGCHUANG_DEBUG_STORAGE1",
    "GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG",
}

print(
    "PASS dedicated STORAGE2 workstation debug: fixed state 3, isolated build "
    "entry, shared production arm/vision state machine, M5 -5/+5deg, M6 +30mm "
    "and detailed scan serial telemetry"
)
