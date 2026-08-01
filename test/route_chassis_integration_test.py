#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN_PATH = ROOT / "src" / "main.cpp"
ROUTE_PATH = ROOT / "src" / "route_chassis_main.inc"
PLATFORMIO_PATH = ROOT / "platformio.ini"
EXPECTED_ROUTE_SHA256 = (
    "9283b87ecd396873d2c0ec2d6c7b3197aa9890e0dc2d1907ca4b5c9253bb768f"
)


class IntegrationFailure(AssertionError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise IntegrationFailure(message)


def normalize_newlines(source: str) -> str:
    return source.replace("\r\n", "\n").replace("\r", "\n")


def compact(source: str) -> str:
    return re.sub(r"\s+", "", source)


def extract_function(source: str, name: str) -> str:
    match = re.search(
        rf"\b(?:void|bool|float|IntegratedWorkPause)\s+"
        rf"{re.escape(name)}\s*\([^)]*\)\s*\{{",
        source,
    )
    require(match is not None, f"function is missing: {name}")
    opening = source.find("{", match.start())
    depth = 0
    quote = ""
    index = opening
    while index < len(source):
        character = source[index]
        if quote:
            if character == "\\":
                index += 2
                continue
            if character == quote:
                quote = ""
        elif character in {'"', "'"}:
            quote = character
        elif source.startswith("//", index):
            newline = source.find("\n", index + 2)
            index = len(source) if newline < 0 else newline
            continue
        elif source.startswith("/*", index):
            closing = source.find("*/", index + 2)
            require(closing >= 0, "unterminated block comment")
            index = closing + 2
            continue
        elif character == "{":
            depth += 1
        elif character == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
        index += 1
    raise IntegrationFailure(f"unterminated function: {name}")


def require_in_order(source: str, fragments: tuple[str, ...], label: str) -> None:
    cursor = 0
    for fragment in fragments:
        position = source.find(fragment, cursor)
        require(position >= 0, f"{label} is missing or reordered: {fragment}")
        cursor = position + len(fragment)


def verify_platformio(platformio_source: str) -> None:
    environments = re.findall(
        r"^\s*\[env:([^\]]+)\]\s*$",
        platformio_source,
        flags=re.MULTILINE,
    )
    require(
        environments == ["genericSTM32H750VB"],
        f"expected one VB environment, found {environments}",
    )
    require(
        re.search(
            r"^\s*default_envs\s*=\s*genericSTM32H750VB\s*$",
            platformio_source,
            flags=re.MULTILINE,
        ) is not None,
        "VB environment is not the default",
    )
    require(
        re.search(
            r"^\s*build_src_filter\s*=\s*-<\*>\s+\+<main\.cpp>\s*$",
            platformio_source,
            flags=re.MULTILINE,
        ) is not None,
        "the build must compile only main.cpp; the .inc is included once",
    )


def verify_route_copy(route_source: str) -> None:
    route_hash = hashlib.sha256(
        normalize_newlines(route_source).encode("utf-8")
    ).hexdigest()
    require(
        route_hash == EXPECTED_ROUTE_SHA256,
        "imported route chassis source changed without updating its "
        "provenance hash",
    )
    route_initializer = re.search(
        r"const\s+RouteCommand\s+route\s*\[\s*\]\s*=\s*\{"
        r"(?P<body>.*?)\n\};",
        route_source,
        flags=re.DOTALL,
    )
    require(route_initializer is not None, "route table is missing")
    steps = [
        int(value)
        for value in re.findall(
            r"\{\s*(\d+)U\s*,", route_initializer.group("body")
        )
    ]
    require(steps == list(range(1, 22)), f"route steps changed: {steps}")
    route_setup = compact(extract_function(route_source, "setup"))
    require_in_order(
        route_setup,
        (
            "SerialQr.begin(QR_BAUDRATE);",
            "initializeMotorOutputs();",
            "resetQrReceiver();",
        ),
        "route setup",
    )
    stop_body = compact(
        extract_function(route_source, "stopAllMotorsImmediately")
    )
    for fragment in (
        "driveDecelerationActive=false;",
        "translationHeadingControlActive=false;",
        "integratedTurnControlActive=false;",
    ):
        require(fragment in stop_body, f"route stop misses {fragment}")


def verify_main_bridge(main_source: str) -> None:
    compact_main = compact(main_source)
    require(
        'namespaceroute_chassis{#include"route_chassis_main.inc"}'
        in compact_main,
        "route chassis is not included inside its ownership namespace",
    )
    for alias in (
        "HardwareSerial&SerialDebug=route_chassis::SerialDebug;",
        "HardwareSerial&SerialHmi=route_chassis::SerialHmi;",
        "HardwareSerial&SerialImu=route_chassis::SerialImu;",
        "HardwareSerial&SerialQr=route_chassis::SerialQr;",
        "OneButton&startButton=route_chassis::startButton;",
        "MecanumKinematics&geometry=route_chassis::geometry;",
        "AccelStepper&motor1=route_chassis::motor1;",
        "AccelStepper&motor2=route_chassis::motor2;",
        "AccelStepper&motor3=route_chassis::motor3;",
        "AccelStepper&motor4=route_chassis::motor4;",
    ):
        require(alias in compact_main, f"shared chassis alias is missing: {alias}")

    setup = compact(extract_function(main_source, "setup"))
    require_in_order(
        setup,
        (
            "route_chassis::setup();",
            "initializeManipulationHardware();",
            "initializeMotorOutputs();",
            "establishArmLinearSafeWorkingZeros();",
            "resetQrReceiver();",
        ),
        "integrated setup",
    )

    loop = compact(extract_function(main_source, "loop"))
    require_in_order(
        loop,
        (
            "route_chassis::runAllMotors();",
            "armMotors.serviceM5();",
            "route_chassis::startButton.tick();",
            "route_chassis::receiveImuData();",
            "serviceArmLinearAxes();",
            "serviceMaixcam();",
            "receiveQrData();",
            "synchronizeTaskCodeToRouteChassis();",
            "serviceCompetitionAction();",
            "serviceIntegratedRoute();",
        ),
        "integrated loop",
    )
    require(
        "route_chassis::loop();" not in loop,
        "route loop must not be called in addition to the integrated loop",
    )
    require(
        loop.count("route_chassis::runAllMotors();") == 1
        and loop.count("route_chassis::updateRoute();") == 0,
        "chassis motor/route services have duplicate loop ownership",
    )

    qr_bridge = compact(
        extract_function(main_source, "synchronizeTaskCodeToRouteChassis")
    )
    require_in_order(
        qr_bridge,
        (
            "if(!scanFlag||route_chassis::scanFlag){return;}",
            "strncpy(route_chassis::qrData,qrData,",
            "route_chassis::scanFlag=true;",
        ),
        "QR bridge",
    )

    pause_map = compact(
        extract_function(main_source, "workPauseAfterRouteStep")
    )
    expected_pause_cases = {
        6: "INTEGRATED_WORK_RAW_1",
        8: "INTEGRATED_WORK_PROCESS_1",
        11: "INTEGRATED_WORK_STORAGE_1",
        14: "INTEGRATED_WORK_RAW_2",
        16: "INTEGRATED_WORK_PROCESS_2",
        19: "INTEGRATED_WORK_STORAGE_2",
    }
    actual_pause_cases = {
        int(step): pause
        for step, pause in re.findall(
            r"case(\d+)U:return(INTEGRATED_WORK_[A-Z0-9_]+);",
            pause_map,
        )
    }
    require(
        actual_pause_cases == expected_pause_cases,
        f"workstation pause mapping changed: {actual_pause_cases}",
    )

    begin_pause = compact(
        extract_function(main_source, "beginIntegratedWorkPause")
    )
    require_in_order(
        begin_pause,
        (
            "route_chassis::stopAllMotorsImmediately();",
            "suspendRouteChassisProfileForWorkstation();",
            "activeCompetitionRound=roundNumber;",
            "beginWorkAction(kind,roundNumber);",
        ),
        "workstation pause entry",
    )

    suspended_profile = compact(
        extract_function(
            main_source, "suspendRouteChassisProfileForWorkstation"
        )
    )
    for fragment in (
        "route_chassis::activeDriveProfileShape="
        "route_chassis::DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID;",
        "route_chassis::activeDriveDeceleration=0.0f;",
        "route_chassis::driveDecelerationActive=false;",
        "route_chassis::driveAccelerationDistanceSteps=0.0f;",
    ):
        require(
            fragment in suspended_profile,
            f"route profile suspension misses {fragment}",
        )

    begin_route = compact(extract_function(main_source, "beginRoute"))
    calibration_start = begin_route.find(
        "beginWorkAction(WORK_ACTION_PROCESS,1U);"
    )
    require(
        0 <= begin_route.rfind(
            "suspendRouteChassisProfileForWorkstation();",
            0,
            calibration_start,
        ) < calibration_start,
        "calibration mode starts PROCESS1 without suspending the route "
        "motion profile",
    )

    service_route = compact(
        extract_function(main_source, "serviceIntegratedRoute")
    )
    require_in_order(
        service_route,
        (
            "if(integratedWorkPause!=INTEGRATED_WORK_NONE)",
            "constsize_tpreviousIndex=route_chassis::routeIndex;",
            "route_chassis::updateRoute();",
            "if(route_chassis::routeIndex!=previousIndex",
            "route_chassis::route[previousIndex].specificationStep;",
            "beginIntegratedWorkPause(pause);",
        ),
        "route/workstation gate",
    )
    require(
        service_route.count("route_chassis::updateRoute();") == 1,
        "route advancement must have exactly one runtime owner",
    )

    watchdog = compact(
        extract_function(main_source, "serviceCompetitionWatchdogs")
    )
    for fragment in (
        "routeWaitingForQrCode=",
        "route_chassis::ROUTE_PHASE_WAIT_SCAN_CODE;",
        "if(routeWaitingForQrCode||",
    ):
        require(fragment in watchdog, f"QR wait watchdog bridge misses {fragment}")


def main() -> int:
    for path in (MAIN_PATH, ROUTE_PATH, PLATFORMIO_PATH):
        require(path.is_file(), f"required file is missing: {path}")
    main_source = MAIN_PATH.read_text(encoding="utf-8")
    route_source = ROUTE_PATH.read_text(encoding="utf-8")
    platformio_source = PLATFORMIO_PATH.read_text(encoding="utf-8")
    verify_platformio(platformio_source)
    verify_route_copy(route_source)
    verify_main_bridge(main_source)
    print(
        "PASS route chassis integration: exact 21-step import, one VB "
        "build, single M1-M4/PB9/IMU owner, QR handoff, six guarded "
        "workstation pauses, and QR-wait watchdog exemption"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (IntegrationFailure, OSError, UnicodeError) as error:
        print(f"ROUTE CHASSIS INTEGRATION FAILED: {error}", file=sys.stderr)
        sys.exit(1)
