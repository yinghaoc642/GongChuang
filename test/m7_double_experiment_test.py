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


def experimental_speed(baseline: int) -> int:
    previous_effective = min(baseline * 2, 5000)
    return min(previous_effective * 13 // 10, 6500)


def doubled_acceleration_step(baseline: int) -> int:
    return min(255, (257 + baseline) // 2)


def experimental_acceleration(baseline: int) -> int:
    previous_effective = doubled_acceleration_step(
        doubled_acceleration_step(baseline)
    )
    boosted_gap = ((256 - previous_effective) * 10 + 6) // 13
    return 256 - boosted_gap


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
        "BOOST_EFFECTIVE_PROFILE_BY_30_PERCENT=true;" in config
        and "PREVIOUS_M7_SPEED_LIMIT_RPM=5000U;" in config
        and "M7_EXPERIMENTAL_SPEED_LIMIT_RPM=6500U;" in config,
        "M7 30-percent speed trial or 6500 RPM ceiling is missing",
    )
    require(
        "speedRpm=m7_experiment::experimentalSpeedRpm(speedRpm);"
        in source
        and "acceleration="
        "m7_experiment::experimentalAcceleration(acceleration);"
        in source,
        "M7 central speed/acceleration transform is missing",
    )
    require(
        "if(axis.address==liftAxis.address&&"
        "m7ProfilePolicy==M7_PROFILE_APPLY_EXPERIMENT){"
        "speedRpm=m7_experiment::experimentalSpeedRpm(speedRpm);"
        in source,
        "experimental transform is not restricted to M7",
    )
    require(
        "roundNumber==2U?startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)"
        in source
        and "STORAGE2_M7_HEAD_START_MS=300UL;" in source,
        "round-2 STORAGE does not use normal high-speed M7 with 300 ms head start",
    )
    require(
        "axis.commandM7ProfilePolicy=m7ProfilePolicy;" in source
        and "retryM7ProfilePolicy="
        "axis.address==liftAxis.address?"
        "M7_PROFILE_USE_REQUESTED:axis.commandM7ProfilePolicy;"
        in source
        and "finalM7ProfilePolicy=m7SoftLandingProfilePolicy;" in source,
        "M7 recovery does not bypass the experiment transform or soft "
        "landing lost its scoped profile",
    )
    require(
        source.count("writeArmLinearPosition(") == 2,
        "an M6/M7 position command bypasses the shared command boundary",
    )

    expected = {
        "startup": (180, 139, 468, 234),
        "soft-zero": (2160, 171, 5616, 240),
        "ring-place": (2700, 199, 6500, 245),
        "endpoint-fine": (4493, 232, 6500, 251),
        "normal/raw": (6740, 249, 6500, 255),
        "return/matched-normal": (6740, 249, 6500, 255),
    }
    for label, (speed, acc, wanted_speed, wanted_acc) in expected.items():
        require(
            experimental_speed(speed) == wanted_speed
            and experimental_acceleration(acc) == wanted_acc,
            f"wrong effective M7 experiment profile: {label}",
        )

    require(
        experimental_speed(6740) == 6500
        and experimental_acceleration(249) == 255,
        "round-2 STORAGE does not match normal pickup/place M7 profile",
    )
    require(
        "M7_RECOVERY_SPEED_RPM=2160U;" in source
        and "M7_RECOVERY_ACCELERATION=171U;" in source
        and "?M7_PROFILE_USE_REQUESTED" in source,
        "M7 recovery no longer transmits the requested 2160/171 profile",
    )

    transform_position = source.index(
        "speedRpm=m7_experiment::experimentalSpeedRpm(speedRpm);"
    )
    stored_position = source.index("axis.commandSpeedRpm=speedRpm;")
    sent_position = source.index("writeArmLinearPosition(", stored_position)
    require(
        transform_position < stored_position < sent_position,
        "M7 effective parameters are not used for timeout/log/send state",
    )

    print(
        "PASS M7 experiment: prior effective speed/physical acceleration "
        "boosted 30 percent, with round-2 STORAGE using the same effective "
        "6500 RPM / acceleration 255 as normal pickup/place"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, UnicodeError) as error:
        print(f"M7 DOUBLE EXPERIMENT CHECK FAILED: {error}")
        raise SystemExit(1)
