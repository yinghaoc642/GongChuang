"""Execute the actual single-translation wrapper and route completion on host.

Hardware/startup boundaries are fake; the real route table, heading switches,
button callbacks, completion functions and debug scheduling are compiled.
No serial device is opened or flashed.
"""
from pathlib import Path
import configparser
import re
import shutil
import subprocess
import tempfile

from route_chassis_integration_test import compact, extract_function

ROOT = Path(__file__).resolve().parents[1]
debug = (ROOT / "src/debug_translation.cpp").read_text(encoding="utf-8")
route = (ROOT / "src/route_chassis_main.inc").read_text(encoding="utf-8")
main_source = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
assert "#define GONGCHUANG_DEBUG_TRANSLATION_STEP 4" in debug
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_STEP04_FORWARD_CORRECTION_ENABLED 1"
    in debug
)
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_STEP04_FORWARD_CORRECTION_MM 20.0f"
    in debug
)
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_STEP06_FORWARD_CORRECTION_ENABLED 1"
    in debug
)
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_STEP06_FORWARD_CORRECTION_MM 10.0f"
    in debug
)
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_LONG_LATERAL_FORWARD_CORRECTION_ENABLED 1"
    in debug
)
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_LONG_LATERAL_FORWARD_CORRECTION_MM 70.0f"
    in debug
)
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1"
    in debug
)
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM 15.0f"
    in debug
)
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1"
    in debug
)
assert (
    "#define GONGCHUANG_DEBUG_TRANSLATION_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM 40.0f"
    in debug
)
assert "#define GONGCHUANG_STEP04_FORWARD_CORRECTION_ENABLED 1" in main_source
assert "#define GONGCHUANG_STEP04_FORWARD_CORRECTION_MM 20.0f" in main_source
assert "#define GONGCHUANG_STEP06_FORWARD_CORRECTION_ENABLED 1" in main_source
assert "#define GONGCHUANG_STEP06_FORWARD_CORRECTION_MM 10.0f" in main_source
assert "STEP06_REAR_CORRECTION" not in main_source
assert "#define GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_ENABLED 1" in main_source
assert "#define GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_MM 70.0f" in main_source
assert (
    "#define GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1"
    in main_source
)
assert (
    "#define GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM 15.0f"
    in main_source
)
assert (
    "#define GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1"
    in main_source
)
assert (
    "#define GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM 40.0f"
    in main_source
)
assert '#include "route_chassis_main.inc"' in debug
assert '#include "main.cpp"' not in debug
assert "const RouteCommand route[]" not in debug
assert "rc::loop();" not in debug and "rc::receiveQrData();" in debug
assert "rc::scanFlag = true;" not in debug
controller = compact(extract_function(route, "serviceTranslationHeadingControl"))
assert "terminalHeadingCorrectionActive||inMotionHeadingCorrectionEnabled" in controller
assert "if(!imuIsFresh())" in controller
lateral_start = compact(extract_function(route, "startFastLateralTranslation"))
assert "commandedRightMm=nominalRightMm*motionScale" in lateral_start
assert "startTranslationDisplacement(forwardCorrectionMm,commandedRightMm)" in lateral_start
assert "forwardCorrectionMm*motionScale" not in lateral_start
current_start = compact(extract_function(route, "startCurrentCommand"))
assert "command.specificationStep==4U" in current_start
assert (
    "forwardCorrectionMm="
    "configuredStep04ForwardCorrectionMm()"
    in current_start
)
step04_correction = compact(
    extract_function(route, "configuredStep04ForwardCorrectionMm")
)
assert "returnGONGCHUANG_STEP04_FORWARD_CORRECTION_MM;" in step04_correction
assert "command.specificationStep==6U" in current_start
assert (
    "forwardCorrectionMm="
    "GONGCHUANG_STEP06_FORWARD_CORRECTION_MM"
    in current_start
)
assert (
    "command.specificationStep==7U||command.specificationStep==15U"
    in current_start
)
assert (
    "forwardCorrectionMm="
    "GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_MM"
    in current_start
)
assert "profileShape,forwardCorrectionMm" in current_start
assert (
    "#define GONGCHUANG_STEP04_FORWARD_CORRECTION_ENABLED 0"
    in route
)
assert (
    "#define GONGCHUANG_STEP04_FORWARD_CORRECTION_MM 0.0f"
    in route
)
assert "GONGCHUANG_DEBUG_TRANSLATION_STEP04_FORWARD_CORRECTION_MM" in route
assert (
    "#define GONGCHUANG_STEP06_FORWARD_CORRECTION_ENABLED 0"
    in route
)
assert (
    "#define GONGCHUANG_STEP06_FORWARD_CORRECTION_MM 0.0f"
    in route
)
assert "GONGCHUANG_DEBUG_TRANSLATION_STEP06_FORWARD_CORRECTION_MM" in route
assert "STEP06_REAR_CORRECTION" not in route
assert (
    "#define GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_ENABLED 0"
    in route
)
assert (
    "#define GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_MM 0.0f"
    in route
)
assert "GONGCHUANG_DEBUG_TRANSLATION_LONG_LATERAL_FORWARD_CORRECTION_MM" in route
assert (
    "#define GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED 0"
    in route
)
assert (
    "#define GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM 0.0f"
    in route
)
assert "GONGCHUANG_DEBUG_TRANSLATION_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM" in route
assert (
    "#define GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED 0"
    in route
)
assert (
    "#define GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM 0.0f"
    in route
)
assert "GONGCHUANG_DEBUG_TRANSLATION_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM" in route

second_half = compact(
    extract_function(route, "serviceLongLateralSecondHalfForwardCorrection")
)
for fragment in (
    "activeStep=route[routeIndex].specificationStep",
    "activeStep==7U&&GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED!=0",
    "activeStep==15U&&GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED!=0",
    "activeStep==7U?GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM:"
    "GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM",
    "completedFraction<LONG_LATERAL_SECOND_HALF_CORRECTION_TRIGGER_FRACTION",
    "remainingFraction=1.0f-clampedCompletedFraction",
    "vehicleDisplacementToMotorPulses("
    "forwardCorrectionMm,0.0f,0.0f)",
    "translationMotionOrigin=current;",
    "translationExecutedCorrectionDegreesBeforeOrigin="
    "correctionDegreesAlreadyExecuted;",
    "translationRelativeMotorPulses=remainingVector;",
    "activeTranslationForwardMm+=forwardCorrectionMm;",
    "longLateralSecondHalfForwardCorrectionApplied=true;",
):
    assert fragment in second_half
for forbidden in (
    "startTranslationDisplacement(",
    "stopAllMotorsImmediately(",
    "startRelativeMotorMove(",
    "advanceRoute(",
):
    assert forbidden not in second_half
ordinary_service = compact(extract_function(route, "serviceOrdinaryCommand"))
assert (
    ordinary_service.index("serviceLongLateralSecondHalfForwardCorrection();")
    < ordinary_service.index("serviceTranslationHeadingControl()")
)
translation_start = compact(extract_function(route, "startTranslationDisplacement"))
assert "longLateralSecondHalfForwardCorrectionApplied=false;" in translation_start
assert "translationProfileTraveledStepsBeforeOrigin=0.0f;" in translation_start
assert "translationExecutedCorrectionDegreesBeforeOrigin=0.0f;" in translation_start
actual_correction = compact(
    extract_function(route, "actualTranslationCorrectionDegrees")
)
assert (
    "returntranslationExecutedCorrectionDegreesBeforeOrigin+"
    "averageRotationPulses/pulsesPerDegree;"
    in actual_correction
)
translation_rate = compact(extract_function(route, "updateTranslationBaseRate"))
assert (
    "translationProfileTraveledStepsBeforeOrigin+"
    "fmaxf(translationMaximumBasePulses-remainingSteps,0.0f)"
    in translation_rate
)

ini = configparser.ConfigParser()
ini.read(ROOT / "platformio.ini", encoding="utf-8")
assert ini["platformio"]["default_envs"] == "genericSTM32H750VB"
assert ini["env:genericSTM32H750VB"]["build_src_filter"] == "-<*> +<main.cpp>"
assert ini["env:debug_translation"]["build_src_filter"] == "-<*> +<debug_translation.cpp>"
assert (
    ini["env:debug_chassis_route"]["build_src_filter"]
    == "-<*> +<debug_chassis_route.cpp>"
)

table = re.search(r"const RouteCommand route\[\] = \{.*?\};", route, re.S).group()
assert re.search(r"\{7U,\s*COMMAND_RIGHT_FAST,\s*DISTANCE_E_MM,\s*STEP_07_MOTION_SCALE\}", table)
assert len(re.findall(r"\{7U,", table)) == 1
assert "midpoint" not in current_start.lower() and "中点" not in current_start
assert re.search(r"\{15U,\s*COMMAND_RIGHT_FAST,\s*DISTANCE_E_MM,\s*STEP_15_MOTION_SCALE\}", table)
step07_scale = re.search(
    r"constexpr\s+float\s+STEP_07_MOTION_SCALE\s*=\s*([0-9]+(?:\.[0-9]*)?|\.[0-9]+)f",
    route,
)
step15_scale = re.search(
    r"constexpr\s+float\s+STEP_15_MOTION_SCALE\s*=\s*([0-9]+(?:\.[0-9]*)?|\.[0-9]+)f",
    route,
)
assert step07_scale and float(step07_scale.group(1)) > 0.0
assert step15_scale and float(step15_scale.group(1)) > 0.0
assert re.search(r"static_assert\(DISTANCE_E_MM\s*==\s*1835", route)
types = re.search(r"enum CommandType \{.*?\n\};", route, re.S).group()
command = re.search(r"struct RouteCommand \{.*?\n\};", route, re.S).group()
constants = {
    match.group(1): match.group(0)
    for match in re.finditer(
        r"constexpr\s+\w+\s+(\w+)\s*=\s*[^;]+;", route
    )
}
emitted = set()
constant_lines = []


def emit_constant(name):
    if name in emitted:
        return
    assert name in constants, f"missing real constant {name}"
    statement = constants[name]
    expression = statement.split("=", 1)[1]
    for dependency in re.findall(r"\b[A-Z][A-Z0-9_]+\b", expression):
        if dependency in constants:
            emit_constant(dependency)
    emitted.add(name)
    constant_lines.append(statement)


for name in re.findall(r"\b[A-Z][A-Z0-9_]+\b", table):
    if name in constants:
        emit_constant(name)
for name in (
    "STOP_AFTER_STEP", "ENABLE_IN_MOTION_IMU_CORRECTION",
    "STEP_01_ENABLE_IN_MOTION_IMU_CORRECTION", "STEP_01_BASE_RATE_LIMIT_SCALE",
    "LONG_LATERAL_ENABLE_IN_MOTION_IMU_CORRECTION", "LONG_LATERAL_BASE_RATE_LIMIT_SCALE",
):
    emit_constant(name)


def function(source, signature, name):
    return signature + " {\n" + extract_function(source, name) + "\n}\n"


harness = r'''
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>
uint32_t nowMs=0;
uint32_t millis(){return nowMs;}
struct FakePort {
 template<class T> void print(T, int=10){}
 template<class T> void println(T, int=10){}
};
namespace rc {
enum ProgramState {PROGRAM_WAITING,PROGRAM_RUNNING,PROGRAM_FINISHED,PROGRAM_ABORTED,PROGRAM_FAULT};
ProgramState programState=PROGRAM_WAITING;
enum {ROUTE_PHASE_IDLE};
int routePhase=ROUTE_PHASE_IDLE;
bool startRequested=false,abortRequested=false,commandStarted=false,fresh=false,enabled=false,completeNext=false;
bool scanFlag=false,step7RawAreaHeadingReferenceValid=false,step15HeadingMatchAppliedAtSecondRawArea=false;
float targetCounterClockwiseHeadingDegrees=0,step7RawAreaHeadingReferenceDegrees=0,scanDistanceBmm=0;
uint32_t startRequestAcceptedMs=0,routeRunStartMs=0;
uint8_t selectedStartZoneNumber=1;
size_t routeIndex=0;
int starts=0,stops=0,qrServices=0;
std::vector<uint8_t> startedSteps,completedSteps;
FakePort SerialDebug;
struct FakeButton {void tick(){}} startButton;
void hmiSetRunStatus(const char*){}
void disableDriveMotors(){enabled=false;++stops;}
void stopAllMotorsImmediately(){}
 enum Zone1ReorderedRoutePhase {
  ZONE1_REORDERED_INACTIVE,ZONE1_REORDERED_INITIAL_FORWARD_150,
  ZONE1_REORDERED_STEP04_FIRST_200,
  ZONE1_REORDERED_STEP04_TURN_360,ZONE1_REORDERED_STEP04_REMAINDER,
  ZONE1_REORDERED_QR_RESET_PENDING,ZONE1_REORDERED_QR_SCAN_LEFT,
  ZONE1_REORDERED_QR_WAIT_AT_LIMIT,ZONE1_REORDERED_QR_RETURN_RIGHT,
  ZONE1_REORDERED_RETURN_FINAL_LONGITUDINAL
 };
 Zone1ReorderedRoutePhase zone1ReorderedRoutePhase=ZONE1_REORDERED_INACTIVE;
 bool zone1ReorderedRouteEnabled(){return false;}
 bool zone1QrAuxiliaryMotionActive(){return false;}
 void startZone1Step04First200(){}
 void startZone1Step04Turn360(){}
 void startZone1Step04Remainder(){}
 void requestZone1FreshQrSweep(){}
 void startZone1QrReturnRight(){}
 void startZone1ReturnFinalLongitudinal(){}
'''
harness += types + "\n" + command + "\n" + "\n".join(constant_lines) + "\n" + table
harness += r'''
constexpr size_t ROUTE_COMMAND_COUNT=sizeof(route)/sizeof(route[0]);
void printCurrentCommandCompletion(){completedSteps.push_back(route[routeIndex].specificationStep);}
'''
for name in ("activeTranslationCommandIsStep01", "activeTranslationCommandIsLongLateral",
             "activeTranslationInMotionHeadingCorrectionEnabled"):
    harness += function(route, f"bool {name}()", name)
harness += function(route, "float activeTranslationBaseRateLimitScale()", "activeTranslationBaseRateLimitScale")
harness += function(route, "void finishProgram(bool stoppedByConfiguredStep, uint8_t completedStep)", "finishProgram")
harness += function(route, "void advanceRoute()", "advanceRoute")
for name in ("onStartButtonClick", "onStartButtonLongPress"):
    harness += function(route, f"void {name}()", name)
harness += r'''
void setup(){}
void receiveHmiStartCommand(){}
void receiveImuData(){}
void receiveQrData(){++qrServices;}
void updateHmiYaw(){}
void serviceBatteryVoltage(){}
void serviceMotionTelemetry(){}
void runAllMotors(){}
void routeFault(const char*){programState=PROGRAM_FAULT;enabled=false;startRequested=false;}
void abortRoute(){programState=PROGRAM_ABORTED;enabled=false;startRequested=false;abortRequested=false;}
void beginRoute(){
 if(programState!=PROGRAM_WAITING){startRequested=false;return;}
 if(!fresh)return;
 routeIndex=0;commandStarted=false;targetCounterClockwiseHeadingDegrees=0;
 step7RawAreaHeadingReferenceValid=false;step15HeadingMatchAppliedAtSecondRawArea=false;
 scanFlag=false;programState=PROGRAM_RUNNING;startRequested=false;enabled=true;routeRunStartMs=millis();
}
void startCurrentCommand(){
 assert(programState==PROGRAM_RUNNING);commandStarted=true;
 ++starts;startedSteps.push_back(route[routeIndex].specificationStep);
}
void updateRoute(){
 if(!commandStarted){startCurrentCommand();return;}
 if(completeNext){completeNext=false;advanceRoute();}
}
}
uint8_t DEBUG_TRANSLATION_STEP=4;
constexpr uint32_t DEBUG_START_MINIMUM_MS=500UL;
size_t selectedTranslationIndex=rc::ROUTE_COMMAND_COUNT;
bool debugTranslationStarted=false,debugTranslationCompletionReported=false;
'''
for name, signature in (
    ("isDebugTranslationType", "bool isDebugTranslationType(rc::CommandType type)"),
    ("findDebugTranslation", "bool findDebugTranslation()"),
    ("startDebugTranslation", "void startDebugTranslation()"),
    ("serviceDebugTranslation", "void serviceDebugTranslation()"),
    ("setup", "void setup()"),
    ("loop", "void loop()"),
):
    harness += function(debug, signature, name)
harness += r'''
void reset(uint8_t step){
 DEBUG_TRANSLATION_STEP=step;nowMs=0;selectedTranslationIndex=rc::ROUTE_COMMAND_COUNT;
 debugTranslationStarted=false;debugTranslationCompletionReported=false;
 rc::programState=rc::PROGRAM_WAITING;rc::startRequested=false;rc::abortRequested=false;
 rc::commandStarted=false;rc::fresh=false;rc::enabled=false;rc::completeNext=false;
 rc::routeIndex=0;rc::starts=0;rc::stops=0;rc::qrServices=0;rc::scanFlag=false;
 rc::startedSteps.clear();rc::completedSteps.clear();
 rc::step7RawAreaHeadingReferenceValid=false;rc::step15HeadingMatchAppliedAtSecondRawArea=false;
 setup();
}
int main(){
 // All translation feedback is disabled; existing step 1/7/15 base caps remain.
 for(size_t i=0;i<rc::ROUTE_COMMAND_COUNT;++i){
  rc::routeIndex=i;auto step=rc::route[i].specificationStep;
  assert(!rc::activeTranslationInMotionHeadingCorrectionEnabled());
  assert(std::fabs(rc::activeTranslationBaseRateLimitScale()-
          ((step==1||step==7||step==15)?0.85f:1.0f))<0.00001f);
 }
 // Every real translation runs once, including special steps 4, 14, 15 and 21.
 for(const auto& command:rc::route){
  reset(command.specificationStep);
  bool valid=isDebugTranslationType(command.type);
  if(!valid){
   assert(rc::programState==rc::PROGRAM_FAULT);
   rc::onStartButtonClick();rc::fresh=true;nowMs=2000;loop();assert(rc::starts==0);continue;
  }
  nowMs=100;rc::onStartButtonClick();rc::fresh=true;
  nowMs=599;loop();assert(rc::starts==0&&!rc::enabled);
  nowMs=600;loop();assert(rc::starts==1&&rc::enabled&&!rc::scanFlag&&rc::qrServices==0);
  assert(rc::startedSteps[0]==command.specificationStep);
  if(command.specificationStep==14||command.specificationStep==15){
   assert(rc::step7RawAreaHeadingReferenceValid);
   assert(rc::step7RawAreaHeadingReferenceDegrees==rc::targetCounterClockwiseHeadingDegrees);
   assert(rc::step15HeadingMatchAppliedAtSecondRawArea==(command.specificationStep==15));
  }
  if(command.specificationStep==4){rc::scanFlag=true;}
  rc::completeNext=true;nowMs=700;loop();
  assert(rc::qrServices==1);
  assert(rc::programState==rc::PROGRAM_FINISHED&&!rc::enabled);
  assert(rc::completedSteps.size()==1&&rc::completedSteps[0]==command.specificationStep);
  rc::onStartButtonClick();
  for(int i=0;i<1000;++i)loop();
  assert(rc::starts==1&&debugTranslationCompletionReported);
 }
 for(uint8_t missing:{2,3}){reset(missing);assert(rc::programState==rc::PROGRAM_FAULT);}
 // No movement until fresh IMU arrives; hold can cancel before or during motion.
 reset(7);nowMs=100;rc::onStartButtonClick();nowMs=600;loop();
 assert(rc::starts==0&&rc::startRequested&&!rc::enabled);
 rc::fresh=true;loop();assert(rc::starts==1);
 rc::onStartButtonLongPress();loop();assert(rc::programState==rc::PROGRAM_ABORTED);
 for(int i=0;i<1000;++i)loop();assert(rc::starts==1&&!rc::enabled);
 reset(7);rc::onStartButtonClick();rc::onStartButtonLongPress();nowMs=600;rc::fresh=true;loop();
 assert(rc::starts==0&&rc::programState==rc::PROGRAM_ABORTED);
 reset(7);rc::fresh=true;rc::onStartButtonClick();nowMs=500;loop();
 rc::routeFault("test fault");for(int i=0;i<1000;++i)loop();assert(rc::starts==1&&!rc::enabled);
 // Millis wraparound keeps the same minimum start duration.
 reset(7);nowMs=UINT32_MAX-100;rc::onStartButtonClick();rc::fresh=true;
 nowMs=398;loop();assert(rc::starts==0);nowMs=399;loop();assert(rc::starts==1);
}
'''
compiler = shutil.which("g++")
assert compiler, "Native g++ is required"
with tempfile.TemporaryDirectory(prefix="gc6_translation_") as temp:
    source = Path(temp) / "translation_test.cpp"
    executable = Path(temp) / "translation_test.exe"
    source.write_text(harness, encoding="utf-8")
    subprocess.run([compiler, "-std=c++17", "-O0", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
print("PASS: step4 QR, step6 forward10 with no rear correction, step7/15 +70mm, continuous second-half step7 +15mm/step15 +40mm, one-command routing and startup guards")
