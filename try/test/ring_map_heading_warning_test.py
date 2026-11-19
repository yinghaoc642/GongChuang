#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.cpp"


def compact(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"\s+", "", text)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    code = compact(MAIN.read_text(encoding="utf-8"))
    start = code.index("boolringMapHeadingStillValid(){")
    end = code.index("boolstartAccumulatedWorkstationMove(", start)
    guard = code[start:end]

    require(
        'if(!imuIsFresh()){routeFault('
        '"IMUstalewhileusingendpointmap");returnfalse;}' in guard,
        "stale-IMU protection was removed with the drift stop",
    )
    require(
        "RING_MAP_MAXIMUM_HEADING_DRIFT_DEGREES" in guard
        and "[RINGMAP]headingdriftwarning;continue=" in guard,
        "post-map heading drift is no longer measured and logged",
    )
    require(
        "Chassismovedafterendpointmapping" not in code
        and "invalidatedbyheadingdrift" not in guard,
        "post-map heading drift can still trigger the deleted hard fault",
    )
    require(
        guard.endswith("returntrue;}"),
        "fresh post-map heading drift does not continue execution",
    )
    require(
        code.count("ringMapHeadingStillValid()") >= 6,
        "mapped-transfer IMU checks were accidentally removed",
    )

    print(
        "PASS ring-map heading policy: fresh drift warns and continues; "
        "stale IMU still faults"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, UnicodeError, ValueError) as error:
        print(f"RING MAP HEADING CHECK FAILED: {error}")
        raise SystemExit(1)
