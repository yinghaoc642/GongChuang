"""Run the production return state machine and displacement entries on the host.

Motor output, IMU, UART and unrelated startup actions are replaced. The actual
route, calibration constants, command selection and completion functions execute.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from route_chassis_integration_test import extract_function

ROOT = Path(__file__).resolve().parents[1]
route = (ROOT / "src/route_chassis_main.inc").read_text(encoding="utf-8")


def block(pattern: str) -> str:
    match = re.search(pattern, route, re.S)
    assert match, pattern
    return match.group()


signatures = (
    ("float selectedStartZoneDirection()", "selectedStartZoneDirection"),
    ("float selectedStartZoneMotionScale(const RouteCommand &command)", "selectedStartZoneMotionScale"),
    ("bool zone1ReorderedRouteEnabled()", "zone1ReorderedRouteEnabled"),
    ("void startFastLongitudinalTranslation(float nominalForwardMm, float motionScale, "
     "float maximumSpeedProfileScale=1.0f, float accelerationProfileScale=1.0f, "
     "float rightCorrectionMm=0.0f)",
     "startFastLongitudinalTranslation"),
    ("void startFastLateralTranslation(float nominalRightMm, float motionScale, "
     "float maximumSpeedProfileScale, float accelerationProfileScale, "
     "DriveProfileShape profileShape, float forwardCorrectionMm)", "startFastLateralTranslation"),
    ("void startZone1ReturnFinalLongitudinal()", "startZone1ReturnFinalLongitudinal"),
    ("void startCurrentCommand()", "startCurrentCommand"),
    ("void finishProgram(bool stoppedByConfiguredStep, uint8_t completedStep)", "finishProgram"),
    ("void advanceRoute()", "advanceRoute"),
)
functions = "\n".join(
    signature + " {\n" + extract_function(route, name) + "\n}\n"
    for signature, name in signatures
)
table = block(r"const RouteCommand route\[\] = \{.*?\};")
constants = {
    match.group(1): match.group()
    for match in re.finditer(r"constexpr\s+\w+\s+(\w+)\s*=\s*[^;]+;", route)
}
constant_lines = []
emitted = set()


def emit_constant(name: str) -> None:
    if name in emitted or name not in constants:
        return
    expression = constants[name].split("=", 1)[1]
    for dependency in re.findall(r"\b[A-Z][A-Z0-9_]+\b", expression):
        emit_constant(dependency)
    emitted.add(name)
    constant_lines.append(constants[name])


for name in re.findall(r"\b[A-Z][A-Z0-9_]+\b", table + functions):
    if name != "ROUTE_COMMAND_COUNT":
        emit_constant(name)

harness = r'''
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>
int testZone1Mode=1;
#define GONGCHUANG_ZONE1_ROUTE_MODE testZone1Mode
struct FakePort {
  template<class... T> void print(T...) {}
  template<class... T> void println(T...) {}
} SerialDebug;
'''
harness += "\n".join(constant_lines) + "\n"
for pattern in (
    r"enum CommandType \{.*?\n\};",
    r"struct RouteCommand \{.*?\n\};",
    r"enum DriveProfileShape \{.*?\n\};",
    r"enum ProgramState \{.*?\n\};",
    r"enum RoutePhase \{.*?\n\};",
    r"enum Zone1ReorderedRoutePhase \{.*?\n\};",
):
    harness += block(pattern) + "\n"
harness += table + r'''
constexpr size_t ROUTE_COMMAND_COUNT=sizeof(route)/sizeof(route[0]);
uint8_t selectedStartZoneNumber=1;
size_t routeIndex=18;
ProgramState programState=PROGRAM_RUNNING;
RoutePhase routePhase=ROUTE_PHASE_IDLE;
Zone1ReorderedRoutePhase zone1ReorderedRoutePhase=ZONE1_REORDERED_INACTIVE;
bool commandStarted=false,startRequested=false,abortRequested=false;
bool step7RawAreaHeadingReferenceValid=false,step15HeadingMatchAppliedAtSecondRawArea=false;
bool scanFlag=true;
float targetCounterClockwiseHeadingDegrees=0,step7RawAreaHeadingReferenceDegrees=0,scanDistanceBmm=0;
uint32_t commandStartMs=0,routeRunStartMs=0;
uint32_t millis(){return 1000;}
struct Motion {float forward,right;};
std::vector<Motion> motions;
std::vector<uint8_t> completed;
bool disabled=false;
void hmiSetRunStatus(const char*){}
void disableDriveMotors(){disabled=true;}
void stopAllMotorsImmediately(){}
void routeFault(const char*){programState=PROGRAM_FAULT;}
void printCurrentCommand(){}
void printCurrentCommandCompletion(){completed.push_back(route[routeIndex].specificationStep);}
void startTranslationDisplacement(float forward,float right){motions.push_back({forward,right});}
void setDriveMotionProfile(float,float,DriveProfileShape=DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID){}
void startZone1Step04First200(){}
void startZone1Step04Turn360(){}
void startZone1Step04Remainder(){}
void requestZone1FreshQrSweep(){}
void startZone1QrReturnRight(){}
void beginStep04QrGate(){}
void startSlowScan(float){}
void startAdjustToPointA(float){}
void startTurn(float,float){}
float wrapDeltaDegrees(float value){return std::remainder(value,360.0f);}
'''
harness += functions + r'''
bool close(float actual,float expected){return std::fabs(actual-expected)<0.002f;}
void reset(uint8_t zone,int mode){
  selectedStartZoneNumber=zone;testZone1Mode=mode;routeIndex=18;
  programState=PROGRAM_RUNNING;routePhase=ROUTE_PHASE_IDLE;
  zone1ReorderedRoutePhase=ZONE1_REORDERED_INACTIVE;
  commandStarted=false;disabled=false;motions.clear();completed.clear();
}
int main(){
  // E->F is one combined move on both visits in either start zone/route mode.
  // The actual right correction must remain 20 mm after distance calibration.
  for(uint8_t zone:{1,2}) for(int mode:{0,1}){
    for(size_t index:{8U,16U}){
      reset(zone,mode);routeIndex=index;
      startCurrentCommand();
      assert(motions.size()==1 && commandStarted && !disabled);
      assert(programState==PROGRAM_RUNNING && routePhase==ROUTE_PHASE_MOTION);
      assert(close(motions[0].forward,index==8U ? 845.175f : 867.3f));
      assert(close(motions[0].right,20.0f));
    }
    // Other forward segments must not inherit E->F's lateral correction.
    for(size_t index:{6U,9U,11U,14U}){
      reset(zone,mode);routeIndex=index;
      startCurrentCommand();
      assert(motions.size()==1 && close(motions[0].right,0.0f));
    }
  }
  std::cout << "PASS actual C++ E->F: steps11/19 forward845.175/867.3 + right20 mm; "
               "one move, both zones/modes, other forward segments unchanged\n";
  // Current zone1: calibrated 21 minus ACTUAL 150, corrected lateral, ACTUAL 150.
  reset(1,1);
  startCurrentCommand();
  assert(motions.size()==1 && close(motions[0].forward,908.4f));
  assert(close(motions[0].right,0));
  advanceRoute();
  assert(routeIndex==17 && !commandStarted && !disabled);
  assert((completed==std::vector<uint8_t>{21}));
  startCurrentCommand();
  assert(motions.size()==2);
  assert(close(motions[1].right,1944.38f) && close(motions[1].forward,170.0f));
  advanceRoute();
  assert(motions.size()==3 && close(motions[2].forward,150.0f));
  assert(close(motions[2].right,0));
  assert(routeIndex==17 && commandStarted && !disabled && programState==PROGRAM_RUNNING);
  assert(zone1ReorderedRoutePhase==ZONE1_REORDERED_RETURN_FINAL_LONGITUDINAL);
  assert((completed==std::vector<uint8_t>{21})); // no early completion of step20
  assert(close(motions[0].forward+motions[2].forward,1058.4f));
  advanceRoute();
  assert(motions.size()==3 && disabled && programState==PROGRAM_FINISHED);
  assert((completed==std::vector<uint8_t>{21,20}));

  // Zone1 legacy and either mode of zone2 retain their old full 20 -> 21 path.
  for(uint8_t zone:{1,2}) for(int mode:{0,1}){
    if(zone==1 && mode==1) continue;
    reset(zone,mode);routeIndex=17;
    startCurrentCommand();
    assert(motions.size()==1 && close(motions[0].right,1944.38f));
    assert(close(motions[0].forward,0));
    advanceRoute();assert(routeIndex==18 && !disabled);
    startCurrentCommand();
    assert(motions.size()==2 && close(motions[1].right,0));
    assert(close(motions[1].forward,zone==1 ? 1058.4f : -1048.95f));
    advanceRoute();
    assert(disabled && programState==PROGRAM_FINISHED && motions.size()==2);
    assert((completed==std::vector<uint8_t>{20,21}));
  }
  // Shortening the return must not change the initial 150mm/step1 balance.
  reset(1,1);routeIndex=0;
  startCurrentCommand();
  assert(motions.size()==1 && close(motions[0].forward,915.9f));
  std::cout << "PASS actual C++ return: zone1 908.4 -> (right1944.38, forward170) "
               "-> 150 mm; completion gates, old modes and outbound preserved\n";
}
'''
compiler = shutil.which("g++")
assert compiler, "host g++ is required for the return state-machine test"
with tempfile.TemporaryDirectory(prefix="gc6_zone1_return_") as temporary:
    source = Path(temporary) / "return_test.cpp"
    executable = Path(temporary) / "return_test.exe"
    source.write_text(harness, encoding="utf-8")
    subprocess.run([compiler, "-std=c++17", "-O0", "-static", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
