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
    "6f15cfc5556d600af8bc95a13fda72eebf4dddbd8e796a58807174eee7f58299"
)


class IntegrationFailure(AssertionError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise IntegrationFailure(message)


def normalize_newlines(source: str) -> str:
    return source.replace("\r\n", "\n").replace("\r", "\n")


def normalize_route_provenance(source: str) -> str:
    source = normalize_newlines(source)
    return re.sub(
        r"(constexpr\s+float\s+STEP_\d{2}(?:_5)?_MOTION_SCALE\s*=\s*)"
        r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)f",
        r"\g<1>0.0f",
        source,
    )


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
        environments == ["genericSTM32H750VB", "debug_qr_continuous", "debug_workstation", "debug_raw_workstation", "debug_raw_workstation_341", "debug_storage2_workstation", "debug_translation", "debug_chassis_route", "debug_maix_mode8", "debug_maix_protocol_cycle"],
        f"expected production and nine isolated debug environments, found {environments}",
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
    require(
        re.search(
            r"^\s*build_src_filter\s*=\s*-<\*>\s+"
            r"\+<debug_chassis_route\.cpp>\s*$",
            platformio_source,
            flags=re.MULTILINE,
        ) is not None,
        "the full chassis debug environment must compile only its wrapper",
    )
    require(
        re.search(
            r"^\s*build_src_filter\s*=\s*-<\*>\s+"
            r"\+<debug_maix_mode8\.cpp>\s*$",
            platformio_source,
            flags=re.MULTILINE,
        ) is not None,
        "the Maix mode-8 debug environment must compile only its wrapper",
    )
    require(
        re.search(
            r"^\s*build_src_filter\s*=\s*-<\*>\s+"
            r"\+<debug_maix_protocol_cycle\.cpp>\s*$",
            platformio_source,
            flags=re.MULTILINE,
        ) is not None,
        "the Maix protocol-cycle environment must compile only its entry",
    )


def verify_route_copy(route_source: str) -> None:
    route_hash = hashlib.sha256(
        normalize_route_provenance(route_source).encode("utf-8")
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
    require(
        steps
        == [
            1,
            *range(4, 22),
        ],
        f"route steps changed: {steps}",
    )
    compact_route = compact(route_initializer.group("body"))
    require_in_order(
        compact_route,
        (
            "{1U,COMMAND_ZONE_LONGITUDINAL_FAST,"
            "START_TO_STEP_04_PRE_ENTRY_MM,"
            "STEP_01_MOTION_SCALE}",
            "{4U,COMMAND_RIGHT_FAST,STEP_04_RIGHT_DISTANCE_MM,"
            "STEP_04_MOTION_SCALE}",
        ),
        "scan-free step 1/4 route",
    )
    require(
        "{2U,COMMAND_SCAN_SLOW," not in compact_route
        and "{3U,COMMAND_ADJUST_TO_POINT_A," not in compact_route,
        "dedicated QR scan motion still exists in route[]",
    )
    require_in_order(
        compact_route,
        (
            "{7U,COMMAND_RIGHT_FAST,DISTANCE_E_MM,STEP_07_MOTION_SCALE}",
            "{8U,COMMAND_TURN_CLOCKWISE,180,STEP_08_MOTION_SCALE}",
            "{15U,COMMAND_RIGHT_FAST,DISTANCE_E_MM,STEP_15_MOTION_SCALE}",
            "{16U,COMMAND_TURN_CLOCKWISE,180,STEP_16_MOTION_SCALE}",
        ),
        "continuous step 7/15 route",
    )
    for fragment in (
        "{8U,COMMAND_TURN_CLOCKWISE,180,STEP_08_MOTION_SCALE}",
        "{16U,COMMAND_TURN_CLOCKWISE,180,STEP_16_MOTION_SCALE}",
    ):
        require(fragment in compact_route, f"route geometry misses {fragment}")
    compact_source = compact(route_source)
    require(
        "startZoneDiagonalTranslation(" not in compact_source,
        "step 1 still uses the former simultaneous diagonal motion",
    )
    for fragment in (
        "floatvalue;",
        "DISTANCE_E_MM==1835,",
        "START_TO_STEP_04_PRE_ENTRY_MM==1045,",
        "STEP_04_PRE_ENTRY_OFFSET_MM==150,",
        "STEP_04_RIGHT_DISTANCE_MM==1017,",
        "ROUTE_COMMAND_COUNT==19U",
        "STOP_AFTER_STEP=0U;",
        "GONGCHUANG_ROUTE_DETAILED_FAULT_SNAPSHOT0",
        "START_BUTTON_LONG_PRESS_MS=2000UL;",
        "STEP_01_5_MOTION_SCALE>0.0f&&",
        "STEP_21_5_MOTION_SCALE>0.0f,",
        "returnselectedStartZoneNumber==1U?1.0f:-1.0f;",
        "constfloatmotionScale=selectedStartZoneMotionScale(command);",
        "STEP_01_PRE_LATERAL_IMU_CHECK_MS=30UL;",
        "STEP_01_ENABLE_IN_MOTION_IMU_CORRECTION=false;",
        "STEP_01_HEADING_CORRECTION_RATE_RESERVE=0.15f;",
        "STEP_01_BASE_RATE_LIMIT_SCALE="
        "1.0f-STEP_01_HEADING_CORRECTION_RATE_RESERVE;",
        "LONG_LATERAL_ENABLE_IN_MOTION_IMU_CORRECTION=false;",
        "LONG_LATERAL_HEADING_CORRECTION_RATE_RESERVE=0.15f;",
        "LONG_LATERAL_BASE_RATE_LIMIT_SCALE="
        "1.0f-LONG_LATERAL_HEADING_CORRECTION_RATE_RESERVE;",
        "CHASSIS_GLOBAL_SPEED_INCREASE_SCALE=1.32f;",
        "CHASSIS_GLOBAL_ACCELERATION_INCREASE_SCALE=1.32f;",
        "returnactiveStep==7U||activeStep==15U;",
        "constboolinMotionHeadingCorrectionEnabled="
        "activeTranslationInMotionHeadingCorrectionEnabled();",
        "activeDriveMaximumStepRate*activeTranslationBaseRateLimitScale();",
        "floatactiveProfileBrakingLimitedStepRate(floatremainingSteps)",
        "STEP_01_PRE_LATERAL_IMU_CHECK_MS>0UL&&",
        "STEP_01_PRE_LATERAL_IMU_CHECK_MS<=30UL&&",
        "constboolstep01PreLateralImuCheckSatisfied="
        "!activeCommandIsStep01||"
        "(translationTerminalCorrectionStartValid&&"
        "millis()-translationTerminalCorrectionStartMs>="
        "STEP_01_PRE_LATERAL_IMU_CHECK_MS);",
        "!step01PreLateralImuCheckSatisfied||"
        "!step04TerminalImuMinimumTimeSatisfied",
        "route[routeIndex].specificationStep==4U&&"
        "zone1ReorderedRoutePhase!="
        "ZONE1_REORDERED_INITIAL_FORWARD_150;",
        '"[STEP1->4IMUCHECK]start,windowms="',
        '"[STEP1->4IMUCHECK]done,elapsed/target/actual/error="',
        "completedFinalSubstep=routeIndex+1U>=ROUTE_COMMAND_COUNT||",
        "completedStep==STOP_AFTER_STEP&&completedFinalSubstep",
        "command.specificationStep==7U&&startingFirstSubstep",
        "STEP_05_MOTION_SCALE==1.00f&&"
        "STEP_08_MOTION_SCALE==1.00f&&"
        "STEP_10_MOTION_SCALE==1.00f&&"
        "STEP_13_MOTION_SCALE==1.00f&&"
        "STEP_16_MOTION_SCALE==1.00f&&"
        "STEP_18_MOTION_SCALE==1.00f,",
        "TURN_POST_STOP_STABILITY_TIME_MS=30UL;",
        "TURN_POST_STOP_MINIMUM_NEW_FRAMES=3U;",
        "imuValidFrameSequence==integratedTurnStabilityLastFrameSequence",
        "stabilityElapsedMs<TURN_POST_STOP_STABILITY_TIME_MS||"
        "integratedTurnStabilityFrameCount<"
        "TURN_POST_STOP_MINIMUM_NEW_FRAMES",
        "GONGCHUANG_ZONE1_ROUTE_MODE1",
        "DEFAULT_START_ZONE_SELECTION=1U;",
        "ZONE1_INITIAL_FORWARD_BEFORE_STEP04_MM=150.0f;",
        "ZONE1_STEP04_BEFORE_TURN_RIGHT_MM=200.0f;",
        "ZONE1_STEP04_COUNTERCLOCKWISE_TURN_DEGREES=360.0f;",
        "ZONE1_STEP04_TURN_MAXIMUM_SPEED_SCALE=0.50f;",
        "ZONE1_QR_SWEEP_MAXIMUM_SPEED_SCALE=0.35f;",
        "ZONE1_QR_SWEEP_ACCELERATION_SCALE=0.50f;",
        "ZONE1_QR_RETURN_PROFILE_SCALE=0.50f;",
        "ZONE1_QR_SWEEP_BRAKING_TARGET_MM=80.0f;",
        "ZONE1_RETURN_DEFERRED_LONGITUDINAL_MM=150.0f;",
        "ZONE1_RETURN_LATERAL_FORWARD_CORRECTION_MM=170.0f;",
        "TURN_RESIDUAL_CORRECTION_PROFILE_SCALE=0.50f;",
        "STEP_04_MOTION_SCALE=1.05f;",
        "returnselectedStartZoneNumber==1U&&"
        "GONGCHUANG_ZONE1_ROUTE_MODE=="
        "ZONE1_ROUTE_MODE_REORDERED_QR_SWEEP;",
    ):
        require(
            fragment in compact_source,
            f"continuous route control misses {fragment}",
        )

    begin_route = compact(extract_function(route_source, "beginRoute"))
    for fragment in (
        "zone1ReorderedRoutePhase=zone1ReorderedRouteEnabled()?"
        "ZONE1_REORDERED_INITIAL_FORWARD_150:ZONE1_REORDERED_INACTIVE;",
        "routeIndex=zone1ReorderedRouteEnabled()?1U:0U;",
        "zone1FreshQrResetRequestPending=false;",
        "zone1FreshQrResetAcknowledged=false;",
    ):
        require(
            fragment in begin_route,
            f"Zone-1 reordered route startup misses {fragment}",
        )

    advance_route = compact(extract_function(route_source, "advanceRoute"))
    require_in_order(
        advance_route,
        (
            "completedStep==20U&&",
            "ZONE1_REORDERED_RETURN_FINAL_LONGITUDINAL)",
            "startZone1ReturnFinalLongitudinal();",
            "printCurrentCommandCompletion();",
        ),
        "Zone-1 final 150mm must finish before step20 completion is reported",
    )
    require_in_order(
        advance_route,
        (
            "ZONE1_REORDERED_INITIAL_FORWARD_150",
            "startZone1Step04First200();",
            "ZONE1_REORDERED_STEP04_FIRST_200",
            "startZone1Step04Turn360();",
            "ZONE1_REORDERED_STEP04_TURN_360",
            "startZone1Step04Remainder();",
        ),
        "Zone-1 150mm-forward/step-4 200mm/CCW360/remainder state machine",
    )
    for fragment in (
        "completedStep==4U&&routeIndex==1U){"
        "zone1ReorderedRoutePhase=ZONE1_REORDERED_INACTIVE;"
        "routeIndex=0U;",
        "completedStep==1U&&routeIndex==0U){"
        "requestZone1FreshQrSweep();return;",
        "completedStep==19U&&routeIndex==16U){routeIndex=18U;",
        "completedStep==21U&&routeIndex==18U){routeIndex=17U;",
        "completedStep==20U&&routeIndex==17U){"
        "finishProgram(false,completedStep);return;",
        "routeIndex=2U;",
    ):
        require(
            fragment in advance_route,
            f"Zone-1 reordered execution sequence misses {fragment}",
        )

    step4_first = compact(
        extract_function(route_source, "startZone1Step04First200")
    )
    step4_turn = compact(
        extract_function(route_source, "startZone1Step04Turn360")
    )
    step4_remainder = compact(
        extract_function(route_source, "startZone1Step04Remainder")
    )
    require(
        "configuredStep04ForwardCorrectionMm()*firstFraction" in step4_first
        and "ZONE1_STEP04_BEFORE_TURN_RIGHT_MM/motionScale" in step4_first,
        "Zone-1 first 200mm must preserve its proportional step-4 correction",
    )
    require(
        "startTurn(ZONE1_STEP04_COUNTERCLOCKWISE_TURN_DEGREES,1.0f,"
        "ZONE1_STEP04_TURN_MAXIMUM_SPEED_SCALE);"
        in step4_turn,
        "Zone-1 intermediate turn must remain CCW 360 degrees at half max speed",
    )
    require(
        "configuredStep04ForwardCorrectionMm()*remainingFraction"
        in step4_remainder
        and "remainingRightMm/motionScale" in step4_remainder,
        "Zone-1 step-4 remainder must preserve the final calibrated endpoint",
    )

    start_current = compact(
        extract_function(route_source, "startCurrentCommand")
    )
    for fragment in (
        "ZONE1_REORDERED_INITIAL_FORWARD_150",
        "startFastLongitudinalTranslation("
        "ZONE1_INITIAL_FORWARD_BEFORE_STEP04_MM,1.0f);",
        "nominalLongitudinalMm-=ZONE1_INITIAL_FORWARD_BEFORE_STEP04_MM/"
        "motionScale;",
    ):
        require(
            fragment in start_current,
            f"Zone-1 initial 150mm/step-1 compensation misses {fragment}",
        )

    qr_reset = compact(
        extract_function(route_source, "requestZone1FreshQrSweep")
    )
    qr_left = compact(
        extract_function(route_source, "startZone1QrSweepLeft")
    )
    qr_capture = compact(
        extract_function(route_source, "captureZone1QrSweepTravel")
    )
    qr_return = compact(
        extract_function(route_source, "startZone1QrReturnRight")
    )
    qr_service = compact(
        extract_function(route_source, "serviceZone1QrSweepLeft")
    )
    translation_rate = compact(
        extract_function(route_source, "updateTranslationBaseRate")
    )
    for fragment in (
        "resetQrReceiver(true);",
        "zone1FreshQrResetRequestPending=true;",
        "ZONE1_REORDERED_QR_RESET_PENDING;",
    ):
        require(fragment in qr_reset, f"fresh center QR reset misses {fragment}")
    for fragment in (
        "-static_cast<float>(STEP_04_RIGHT_DISTANCE_MM)",
        "STEP_04_MOTION_SCALE",
        "NON_07_15_LINEAR_PROFILE_INCREASE_SCALE*"
        "ZONE1_QR_SWEEP_MAXIMUM_SPEED_SCALE",
        "NON_07_15_LINEAR_PROFILE_INCREASE_SCALE*"
        "LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL*"
        "ZONE1_QR_SWEEP_ACCELERATION_SCALE",
    ):
        require(fragment in qr_left, f"reduced-profile left QR sweep misses {fragment}")
    require(
        "zone1QrSweepMaximumMm*completedFraction" in qr_capture,
        "QR sweep must record its actual completed travel",
    )
    require_in_order(
        qr_service,
        (
            "if(scanFlag&&!zone1QrSweepBraking)",
            "zone1QrSweepBraking=true;",
            "if(!serviceTranslationHeadingControl())",
            "captureZone1QrSweepTravel();",
        ),
        "QR lock must brake first and capture travel only after software stop",
    )
    for forbidden in (
        "translationBaseStepRate=0.0f;",
        "translationBaseDisplacementComplete=true;",
        "applyTranslationWheelSpeedMix();",
    ):
        require(
            forbidden not in qr_service.split(
                "if(!serviceTranslationHeadingControl())", 1
            )[0],
            f"QR lock still stops/captures immediately before braking: {forbidden}",
        )
    for fragment in (
        "if(zone1QrSweepBraking){",
        "translationBaseStepRate-="
        "ZONE1_QR_SWEEP_BRAKE_DECELERATION*dtSeconds;",
        "translationBaseDisplacementComplete=true;",
    ):
        require(
            fragment in translation_rate,
            f"controlled QR braking misses {fragment}",
        )
    for fragment in (
        "ZONE1_QR_SWEEP_ACTIVE_MAXIMUM_STEP_RATE*"
        "ZONE1_QR_SWEEP_ACTIVE_MAXIMUM_STEP_RATE/"
        "(2.0f*ZONE1_QR_SWEEP_BRAKE_TARGET_PULSES)",
        "ZONE1_QR_SWEEP_BRAKING_TARGET_MM*"
        "LATERAL_PULSES_PER_METER/1000.0f",
    ):
        require(
            fragment in compact_source,
            f"compile-time QR stopping-distance contract misses {fragment}",
        )
    require(
        "startFastLateralTranslation(zone1QrSweepTravelMm,1.0f,"
        "NON_07_15_LINEAR_PROFILE_INCREASE_SCALE*"
        "ZONE1_QR_RETURN_PROFILE_SCALE,"
        "NON_07_15_LINEAR_PROFILE_INCREASE_SCALE*"
        "LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL*"
        "ZONE1_QR_RETURN_PROFILE_SCALE" in qr_return,
        "QR return must retain the original half-speed/half-acceleration profile",
    )
    long_lateral_second_half = compact(
        extract_function(
            route_source, "serviceLongLateralSecondHalfForwardCorrection"
        )
    )
    for fragment in (
        "activeStep==7U&&"
        "GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED!=0",
        "activeStep==15U&&"
        "GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED!=0",
        "activeStep==7U?"
        "GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM:"
        "GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM",
        "completedFraction<"
        "LONG_LATERAL_SECOND_HALF_CORRECTION_TRIGGER_FRACTION",
        "translationProfileTraveledStepsBeforeOrigin+=",
        "translationMotionOrigin=current;",
        "translationExecutedCorrectionDegreesBeforeOrigin="
        "correctionDegreesAlreadyExecuted;",
        "translationRelativeMotorPulses=remainingVector;",
        "longLateralSecondHalfForwardCorrectionApplied=true;",
    ):
        require(
            fragment in long_lateral_second_half,
            f"step 7/15 second-half correction misses {fragment}",
        )
    for forbidden in (
        "stopAllMotorsImmediately(",
        "startTranslationDisplacement(",
        "startRelativeMotorMove(",
        "advanceRoute(",
    ):
        require(
            forbidden not in long_lateral_second_half,
            f"step 7/15 second-half correction must remain continuous: {forbidden}",
        )
    ordinary_service = compact(
        extract_function(route_source, "serviceOrdinaryCommand")
    )
    require(
        ordinary_service.index(
            "serviceLongLateralSecondHalfForwardCorrection();"
        )
        < ordinary_service.index("serviceTranslationHeadingControl()"),
        "step 7/15 remaining target must update before the next speed calculation",
    )
    translation_service = compact(
        extract_function(route_source, "serviceTranslationHeadingControl")
    )
    require_in_order(
        translation_service,
        (
            "updateTranslationBaseRate();",
            "applyTranslationWheelSpeedMix();",
        ),
        "continuous translation speed update",
    )
    for former_stop in (
        "serviceLongTranslationPredictiveMidpoint",
        "longTranslationMidpointRecoveryActive",
        "midpointRemainingSteps",
        "LONG_LATERAL_MIDPOINT_MAXIMUM_HOLD_MS",
    ):
        require(
            former_stop not in route_source,
            f"step 7/15 still has a midpoint stop: {former_stop}",
        )
    for name in (
        "FAST_MAXIMUM_STEP_RATE",
        "SCAN_MAXIMUM_STEP_RATE",
        "TURN_MAXIMUM_STEP_RATE",
        "TURN_RESIDUAL_MAXIMUM_STEP_RATE",
        "TRANSLATION_HEADING_MAXIMUM_RATE_DPS",
    ):
        initializer = re.search(
            rf"constexpr\s+float\s+{name}\s*=\s*(.*?);",
            route_source,
            flags=re.DOTALL,
        )
        require(
            initializer is not None
            and initializer.group(1).count(
                "CHASSIS_GLOBAL_SPEED_INCREASE_SCALE"
            ) == 1,
            f"{name} must apply the cumulative 1.32 speed scale exactly once",
        )
    for name in (
        "FAST_STEP_ACCELERATION",
        "SCAN_STEP_ACCELERATION",
        "TURN_STEP_ACCELERATION",
        "TURN_RESIDUAL_STEP_ACCELERATION",
        "TRANSLATION_HEADING_RATE_SLEW_DPS2",
    ):
        initializer = re.search(
            rf"constexpr\s+float\s+{name}\s*=\s*(.*?);",
            route_source,
            flags=re.DOTALL,
        )
        require(
            initializer is not None
            and initializer.group(1).count(
                "CHASSIS_GLOBAL_ACCELERATION_INCREASE_SCALE"
            ) == 1,
            f"{name} must apply the cumulative 1.32 acceleration scale exactly once",
        )
    for fragment in (
        "LATEST_ANGULAR_MAXIMUM_SPEED_SCALE=0.85f;",
        "LATEST_ANGULAR_ACCELERATION_SCALE=0.85f;",
        "LATEST_GLOBAL_MAXIMUM_SPEED_SCALE*CHASSIS_GLOBAL_SPEED_INCREASE_SCALE*LATEST_ANGULAR_MAXIMUM_SPEED_SCALE;",
        "LATEST_GLOBAL_ACCELERATION_SCALE*CHASSIS_GLOBAL_ACCELERATION_INCREASE_SCALE*LATEST_ANGULAR_ACCELERATION_SCALE;",
    ):
        require(fragment in compact_source, f"angular reduction misses {fragment}")
    route_setup = compact(extract_function(route_source, "setup"))
    require(
        '"[CHASSISREV]GC6-20261007no-midpoint-7-15v-a-x1.32"'
        in route_setup,
        "serial firmware revision marker is missing from chassis setup",
    )
    require_in_order(
        route_setup,
        (
            "SerialQr.begin(QR_BAUDRATE);",
            "initializeMotorOutputs();",
            "startButton.setPressMs(START_BUTTON_LONG_PRESS_MS);",
            "startButton.attachClick(onStartButtonClick);",
            "startButton.attachLongPressStart(onStartButtonLongPress);",
            "resetQrReceiver();",
        ),
        "route setup",
    )
    zone_scale = compact(
        extract_function(route_source, "selectedStartZoneMotionScale")
    )
    require_in_order(
        zone_scale,
        (
            "if(selectedStartZoneNumber==2U)",
            "if(command.specificationStep==1U)",
            "returnSTEP_01_5_MOTION_SCALE;",
            "if(command.specificationStep==21U)",
            "returnSTEP_21_5_MOTION_SCALE;",
            "returncommand.motionScale;",
        ),
        "zone-2 step 1.5/21.5 calibration selection",
    )
    button_click = compact(
        extract_function(route_source, "onStartButtonClick")
    )
    require_in_order(
        button_click,
        (
            "if(programState!=PROGRAM_WAITING||startRequested)",
            "abortRequested=false;",
            "startRequested=true;",
        ),
        "PB9 single-click start",
    )
    button_long = compact(
        extract_function(route_source, "onStartButtonLongPress")
    )
    for fragment in (
        "if(programState==PROGRAM_RUNNING||startRequested)",
        "abortRequested=true;",
    ):
        require(fragment in button_long, f"running hold-stop logic misses {fragment}")
    hmi_start = compact(
        extract_function(route_source, "receiveHmiStartCommand")
    )
    for fragment in (
        "while(SerialHmi.available())",
        "command!=0x01U&&command!=0x02U",
        "selectedStartZoneNumber=command;",
        'command==0x01U?"READY1":"READY2"',
    ):
        require(fragment in hmi_start, f"HMI direct-start logic misses {fragment}")
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
    require(
        compact_main.count(
            "route_chassis::selectedStartZoneNumber==1U"
        ) >= 2,
        "main bridge must synchronize the runtime PB9 start-zone selection",
    )
    for fragment in (
        "FINAL_RETURN_M5_TARGET_DEGREES=static_cast<float>(ARM_BASE_HOME_ANGLE_DEGREES);",
        "FINAL_RETURN_M5_TOLERANCE_DEGREES=0.10f;",
        "boolfinalReturnM5Commanded=false;",
    ):
        require(fragment in compact_main, f"final M5 return misses {fragment}")
    pause_finish = compact(
        extract_function(main_source, "finishIntegratedWorkPause")
    )
    require("startArmBaseRotationToDegrees(" not in pause_finish,
            "M5 must hold while final return is travelling")
    finish_program = compact(extract_function(main_source, "finishProgram"))
    require_in_order(
        finish_program,
        (
            "if(!finalReturnM5Commanded)",
            "if(!extensionMoveFinished()||!liftMoveFinished())",
            "startArmBaseRotationToDegrees(FINAL_RETURN_M5_TARGET_DEGREES);",
            "finalReturnM5Commanded=true;",
            "if(armMotors.isM5Running()||armBaseMotionWatchdogActive)",
            "armMotors.m5CurrentAngleDegrees()-"
            "FINAL_RETURN_M5_TARGET_DEGREES",
            "disableArmBaseMotor();",
        ),
        "endpoint-only M5 startup-angle return",
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
    workstation_profile = compact(
        extract_function(main_source, "setDriveMotionProfile")
    )
    for factor in (
        "route_chassis::CHASSIS_GLOBAL_SPEED_INCREASE_SCALE",
        "route_chassis::CHASSIS_GLOBAL_ACCELERATION_INCREASE_SCALE",
    ):
        require(
            workstation_profile.count(factor) == 1,
            f"workstation M1-M4 profile must apply {factor} exactly once",
        )

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
            "route_chassis::consumeZone1FreshQrResetRequest()",
            "resetQrReceiver(false);",
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
    # Guard future edits: the included chassis file is the only route owner.
    require(
        re.search(r"\bRouteCommand\s+route\s*\[",
                  main_source) is None,
        "main.cpp reintroduced a duplicate route[] table",
    )
    require(
        re.search(r"\b(?:void|bool)\s+"
                  r"(?:updateRoute|startCurrentCommand|runAllMotors)\s*\(",
                  main_source) is None,
        "main.cpp reintroduced a duplicate chassis route or motor runner",
    )
    verify_platformio(platformio_source)
    verify_route_copy(route_source)
    verify_main_bridge(main_source)
    print(
        "PASS route chassis integration: 19 active specification steps/19 motion "
        "commands, one VB "
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
