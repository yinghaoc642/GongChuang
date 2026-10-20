"""Verify the isolated full-route chassis firmware boundary.

This is a source/build-contract check. Hardware, serial devices and motors are
not opened or driven.
"""
from pathlib import Path
import configparser
import re

from route_chassis_integration_test import compact, extract_function

ROOT = Path(__file__).resolve().parents[1]
debug = (ROOT / "src/debug_chassis_route.cpp").read_text(encoding="utf-8")
main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
route = (ROOT / "src/route_chassis_main.inc").read_text(encoding="utf-8")

assert '#include "route_chassis_main.inc"' in debug
assert '#include "main.cpp"' not in debug
assert "const RouteCommand route[]" not in debug
assert "workPauseAfterRouteStep" not in debug
assert "serviceIntegratedRoute" not in debug

for forbidden in (
    "ArmMotorController",
    "ArmTransferPlanner",
    "FashionStar_UartServo",
    "MaixCamClient",
    "SerialArmLinear",
    "SerialServo",
    "initializeArm",
    "initializeServos",
    "beginWorkAction",
):
    assert forbidden not in debug, f"chassis-only entry leaked workstation symbol: {forbidden}"

corrections = (
    ("GONGCHUANG_STEP04_FORWARD_CORRECTION_MM", "20.0f"),
    ("GONGCHUANG_STEP06_FORWARD_CORRECTION_MM", "10.0f"),
    ("GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_MM", "70.0f"),
    ("GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM", "15.0f"),
    ("GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM", "40.0f"),
)
for name, value in corrections:
    definition = f"#define {name} {value}"
    assert definition in debug
    assert definition in main

assert "STEP06_REAR_CORRECTION" not in debug
assert "STEP06_REAR_CORRECTION" not in main

setup = compact(extract_function(debug, "setup"))
assert "rc::setup();" in setup
assert "[DEBUGCHASSISROUTE]full19-commandroute;noworkstationactions" in setup

loop = compact(extract_function(debug, "loop"))
for fragment in (
    "rc::runAllMotors();",
    "rc::startButton.tick();",
    "rc::receiveHmiStartCommand();",
    "rc::receiveImuData();",
    "rc::consumeZone1FreshQrResetRequest();",
    "rc::receiveQrData();",
    "rc::updateHmiYaw();",
    "rc::serviceBatteryVoltage();",
    "millis()-rc::startRequestAcceptedMs)>="
    "CHASSIS_ROUTE_START_MINIMUM_MS",
    "rc::beginRoute();",
    "rc::updateRoute();",
    "rc::serviceMotionTelemetry();",
):
    assert fragment in loop, f"chassis-only scheduler misses {fragment}"
assert loop.index("rc::runAllMotors();") < loop.index("rc::updateRoute();")
assert loop.index("CHASSIS_ROUTE_START_MINIMUM_MS") < loop.index("rc::beginRoute();")
assert "delay(" not in loop and "runToPosition(" not in loop
assert "constexpruint32_tCHASSIS_ROUTE_START_MINIMUM_MS=500UL;" in compact(debug)

route_table = re.search(r"const RouteCommand route\[\] = \{.*?\};", route, re.S)
assert route_table
assert len(re.findall(r"\{\d+U,", route_table.group())) == 19
advance = compact(extract_function(route, "advanceRoute"))
assert "++routeIndex;" in advance
for fragment in (
    "routeIndex=0U;",
    "routeIndex=18U;",
    "routeIndex=17U;",
    "routeIndex=2U;",
):
    assert fragment in advance, f"reordered Zone-1 route misses {fragment}"
assert "delay(" not in advance
assert "workPauseAfterRouteStep" not in route

ini = configparser.ConfigParser()
ini.read(ROOT / "platformio.ini", encoding="utf-8")
assert ini["platformio"]["default_envs"] == "genericSTM32H750VB"
assert ini["env:genericSTM32H750VB"]["build_src_filter"] == "-<*> +<main.cpp>"
assert (
    ini["env:debug_chassis_route"]["build_src_filter"]
    == "-<*> +<debug_chassis_route.cpp>"
)

print(
    "PASS: isolated 19-command chassis route, production corrections, "
    "500ms startup, Zone-1 route-mode state machine, "
    "no workstation/arm/vision entrypoints"
)
