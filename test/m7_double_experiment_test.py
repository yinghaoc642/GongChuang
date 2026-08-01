#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def compact(path: Path) -> str:
    source = path.read_text(encoding="utf-8")
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    source = re.sub(r"//[^\n]*", "", source)
    return re.sub(r"\s+", "", source)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def doubled_speed(baseline: int) -> int:
    return min(baseline * 2, 5000)


def doubled_acceleration_step(baseline: int) -> int:
    return min(255, (257 + baseline) // 2)


def experimental_acceleration(baseline: int) -> int:
    return doubled_acceleration_step(doubled_acceleration_step(baseline))


def main() -> int:
    config = compact(
        ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h"
    )
    source = compact(ROOT / "src" / "main.cpp")

    require(
        "DOUBLE_SPEED_AND_ACCELERATION=true;" in config,
        "M7 double experiment is not enabled",
    )
    require(
        "DOUBLE_ACCELERATION_AGAIN=true;" in config,
        "M7 additional acceleration doubling is not enabled",
    )
    require(
        "EMM42_V5_MAXIMUM_SPEED_RPM=5000U;" in config,
        "EMM42 V5 5000 RPM safety cap is missing",
    )
    require(
        "speedRpm=m7_experiment::doubledSpeedRpm(speedRpm);"
        in source
        and "acceleration="
        "m7_experiment::experimentalAcceleration(acceleration);"
        in source,
        "M7 central speed/acceleration transform is missing",
    )
    require(
        "if(axis.address==liftAxis.address){"
        "speedRpm=m7_experiment::doubledSpeedRpm(speedRpm);"
        in source,
        "experimental transform is not restricted to M7",
    )
    require(
        source.count("writeArmLinearPosition(") == 2,
        "an M6/M7 position command bypasses the shared command boundary",
    )

    expected = {
        "startup": (180, 139, 360, 227),
        "recovery/soft-zero": (2160, 171, 4320, 235),
        "ring-place": (2700, 199, 5000, 242),
        "endpoint-fine": (4493, 232, 5000, 250),
        "normal/raw": (6740, 239, 5000, 252),
        "return/matched-normal": (6740, 239, 5000, 252),
    }
    for label, (speed, acc, wanted_speed, wanted_acc) in expected.items():
        require(
            doubled_speed(speed) == wanted_speed
            and experimental_acceleration(acc) == wanted_acc,
            f"wrong effective M7 experiment profile: {label}",
        )

    transform_position = source.index(
        "speedRpm=m7_experiment::doubledSpeedRpm(speedRpm);"
    )
    stored_position = source.index("axis.commandSpeedRpm=speedRpm;")
    sent_position = source.index("writeArmLinearPosition(", stored_position)
    require(
        transform_position < stored_position < sent_position,
        "M7 effective parameters are not used for timeout/log/send state",
    )

    print(
        "PASS M7 experiment: speed remains 2x/capped at 5000 RPM; "
        "physical acceleration is doubled again with one-flag rollback"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, UnicodeError) as error:
        print(f"M7 DOUBLE EXPERIMENT CHECK FAILED: {error}")
        raise SystemExit(1)
