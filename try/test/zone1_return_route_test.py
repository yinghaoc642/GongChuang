"""Run production translation profiles and the return state machine on the host.

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
main_source = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
correction_definitions = re.findall(
    r"^#define GONGCHUANG_(?:STEP06|LONG_LATERAL)_FORWARD_CORRECTION_(?:ENABLED|MM) .+$",
    main_source, re.M,
)
assert len(correction_definitions) == 4, "production lateral correction config is missing"


def block(pattern: str) -> str:
    match = re.search(pattern, route, re.S)
    assert match, pattern
    return match.group()


signatures = (
    ("float selectedStartZoneDirection()", "selectedStartZoneDirection"),
    ("float selectedStartZoneMotionScale(const RouteCommand &command)", "selectedStartZoneMotionScale"),
    ("bool zone1ReorderedRouteEnabled()", "zone1ReorderedRouteEnabled"),
    ("bool zone1ReorderedReturnEnabled()", "zone1ReorderedReturnEnabled"),
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
harness += "\n".join(correction_definitions) + "\n"
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
struct Profile {float maximumSpeed,acceleration;};
std::vector<Motion> motions;
std::vector<Profile> profiles;
std::vector<uint8_t> completed;
bool disabled=false;
int qrSweepRequests=0,qrGateStarts=0;
void hmiSetRunStatus(const char*){}
void disableDriveMotors(){disabled=true;}
void stopAllMotorsImmediately(){}
void routeFault(const char*){programState=PROGRAM_FAULT;}
void printCurrentCommand(){}
void printCurrentCommandCompletion(){completed.push_back(route[routeIndex].specificationStep);}
void startTranslationDisplacement(float forward,float right){motions.push_back({forward,right});}
void setDriveMotionProfile(float speed,float acceleration,
                          DriveProfileShape=DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID){
  profiles.push_back({speed,acceleration});
}
void startZone1Step04First200(){}
void startZone1Step04Turn360(){}
void startZone1Step04Remainder(){}
void requestZone1FreshQrSweep(){++qrSweepRequests;}
void startZone1QrReturnRight(){}
void beginStep04QrGate(){++qrGateStarts;}
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
  commandStarted=false;disabled=false;motions.clear();profiles.clear();completed.clear();
  qrSweepRequests=0;qrGateStarts=0;
}
int main(){
  // Only zone1's first center-to-RAW move is extended; retain the existing
  // +10 mm forward correction and the exact speed/acceleration of zone2.
  for(int mode:{0,1}){
    reset(2,mode);routeIndex=3;
    startCurrentCommand();
    assert(motions.size()==1 && profiles.size()==1 && commandStarted);
    assert(close(motions[0].right,-925.3f) && close(motions[0].forward,10.0f));
    const Profile original=profiles[0];
    reset(1,mode);routeIndex=3;
    startCurrentCommand();
    assert(motions.size()==1 && profiles.size()==1 && commandStarted);
    assert(programState==PROGRAM_RUNNING && routePhase==ROUTE_PHASE_MOTION);
    assert(close(motions[0].right,-948.4325f) && close(motions[0].forward,10.0f));
    assert(close(profiles[0].maximumSpeed,original.maximumSpeed));
    assert(close(profiles[0].acceleration,original.acceleration));
  }
  std::cout << "PASS actual C++ step6: zone1 left948.4325 mm, zone2 left925.3 mm; "
               "both modes, forward10 mm and speed/acceleration preserved\n";
  // Both C->D visits combine the extra 10 mm with the existing 70 mm. The
  // resulting 80 mm must not be scaled by either segment's distance calibration.
  for(uint8_t zone:{1,2}) for(int mode:{0,1}) for(size_t index:{4U,12U}){
    reset(zone,mode);routeIndex=index;
    step7RawAreaHeadingReferenceValid=true;
    step15HeadingMatchAppliedAtSecondRawArea=true;
    targetCounterClockwiseHeadingDegrees=step7RawAreaHeadingReferenceDegrees=0.0f;
    startCurrentCommand();
    assert(motions.size()==1 && profiles.size()==1 && commandStarted);
    assert(programState==PROGRAM_RUNNING && routePhase==ROUTE_PHASE_MOTION);
    assert(close(motions[0].forward,80.0f));
    assert(close(motions[0].right,index==4U ? 1861.81965f : 1857.02f));
    const float motionScale=index==4U ? 0.993f : 1.012f;
    assert(close(profiles[0].maximumSpeed/(FAST_MAXIMUM_STEP_RATE*motionScale),0.8775f));
    assert(close(profiles[0].acceleration/(FAST_STEP_ACCELERATION*motionScale),0.72f));
  }
  std::cout << "PASS actual C++ steps7/15: lateral1861.81965/1857.02 mm, forward80 mm; "
               "step7 distance+3% then -15 mm, both zones/modes, speed/acceleration preserved\n";
  // E->F is one combined move on both visits in either start zone/route mode.
  // The actual right correction must remain 20 mm after distance calibration.
  for(uint8_t zone:{1,2}) for(int mode:{0,1}){
    for(size_t index:{8U,16U}){
      reset(zone,mode);routeIndex=index;
      startCurrentCommand();
      assert(motions.size()==1 && commandStarted && !disabled);
      assert(programState==PROGRAM_RUNNING && routePhase==ROUTE_PHASE_MOTION);
      assert(close(motions[0].forward,index==8U ? 840.175f : 867.3f));
      assert(close(motions[0].right,20.0f));
    }
    // First storage arrival adds 15 mm to its previous -20 mm correction.
    // Step12 retains its +20 mm departure correction.
    // Preserve step14's distance scale and steps9/11/17's speed reductions.
    // Check all forward segments to catch propagation to the second visit.
    struct ExpectedForward {size_t index;float forward,right,motionScale,speedScale;};
    const ExpectedForward expected[] = {
      {6U,876.926f,0.0f,0.982f,0.90f}, // step9: first processing departure
      {8U,840.175f,20.0f,0.955f,0.90f}, // step11: first storage arrival, net -5 mm
      {9U,936.75f,0.0f,0.965f,1.0f},  // step12: storage departure, +20 mm
      {11U,822.8995f,0.0f,0.970f,1.0f}, // step14: 866.21 * 0.95
      {14U,875.14f,0.0f,0.980f,0.90f}, // step17: unchanged distance, speed -10%
      {16U,867.3f,20.0f,0.98f,1.0f},
    };
    for(const ExpectedForward& segment:expected){
      reset(zone,mode);routeIndex=segment.index;
      startCurrentCommand();
      assert(motions.size()==1 && profiles.size()==1);
      assert(close(motions[0].forward,segment.forward));
      assert(close(motions[0].right,segment.right));
      const float originalSpeed=FAST_MAXIMUM_STEP_RATE * segment.motionScale *
                                NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;
      const float originalAcceleration=FAST_STEP_ACCELERATION * segment.motionScale *
                                       NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;
      assert(close(profiles[0].maximumSpeed/originalSpeed,segment.speedScale));
      assert(close(profiles[0].acceleration/originalAcceleration,1.0f));
    }
    // The added 15 mm at F also increases combined E->G travel by 15 mm.
    reset(zone,mode);routeIndex=8;
    startCurrentCommand();
    advanceRoute();
    assert(route[routeIndex].specificationStep==12U && !commandStarted);
    assert((completed==std::vector<uint8_t>{11}));
    startCurrentCommand();
    assert(motions.size()==2);
    assert(close(motions[0].forward+motions[1].forward,1776.925f));
    assert(close(motions[0].right+motions[1].right,20.0f));
  }
  std::cout << "PASS actual C++ E->F: steps11/19 forward840.175/867.3 + right20 mm; "
               "one move, both zones/modes\n";
  std::cout << "PASS actual C++ first E->F->G: step11 net -5 mm, step12 +20 mm; "
               "840.175/936.75 mm, combined travel1776.925 mm and second visit preserved\n";
  std::cout << "PASS actual C++ forward profiles: step14 distance822.8995 mm, speed unchanged; "
               "steps9/11/17 speed90%; acceleration and other segments preserved\n";
  // Both zone1 outbound modes use the same return, entered after storage2.
  for(int mode:{0,1}){
    reset(1,mode);routeIndex=16;
    advanceRoute();
    assert(routeIndex==18 && !commandStarted && !disabled);
    assert((completed==std::vector<uint8_t>{19})); // preserves storage2 handoff
    startCurrentCommand();
    assert(motions.size()==1 && close(motions[0].forward,908.4f));
    assert(close(motions[0].right,0));
    advanceRoute();
    assert(routeIndex==17 && !commandStarted && !disabled);
    assert((completed==std::vector<uint8_t>{19,21}));
    startCurrentCommand();
    assert(motions.size()==2);
    assert(close(motions[1].right,1944.38f) && close(motions[1].forward,170.0f));
    advanceRoute();
    assert(motions.size()==3 && close(motions[2].forward,150.0f));
    assert(close(motions[2].right,0));
    assert(routeIndex==17 && commandStarted && !disabled && programState==PROGRAM_RUNNING);
    assert(zone1ReorderedRoutePhase==ZONE1_REORDERED_RETURN_FINAL_LONGITUDINAL);
    assert((completed==std::vector<uint8_t>{19,21})); // no early completion of step20
    assert(close(motions[0].forward+motions[2].forward,1058.4f));
    assert(close(profiles[0].maximumSpeed,profiles[2].maximumSpeed));
    assert(close(profiles[0].acceleration,profiles[2].acceleration));
    advanceRoute();
    assert(motions.size()==3 && disabled && programState==PROGRAM_FINISHED);
    assert((completed==std::vector<uint8_t>{19,21,20}));
    assert(qrSweepRequests==0);
  }

  // Zone2 retains its old full 20 -> 21 path in either outbound mode.
  for(int mode:{0,1}){
    reset(2,mode);routeIndex=16;
    advanceRoute();assert(routeIndex==17 && !disabled);
    startCurrentCommand();
    assert(motions.size()==1 && close(motions[0].right,1944.38f));
    assert(close(motions[0].forward,0));
    advanceRoute();assert(routeIndex==18 && !disabled);
    startCurrentCommand();
    assert(motions.size()==2 && close(motions[1].right,0));
    assert(close(motions[1].forward,-1048.95f));
    advanceRoute();
    assert(disabled && programState==PROGRAM_FINISHED && motions.size()==2);
    assert((completed==std::vector<uint8_t>{19,20,21}));
  }
  // Default mode0 still starts with a full step1, then the ordinary step4 QR gate.
  reset(1,0);routeIndex=0;
  assert(!zone1ReorderedRouteEnabled() && zone1ReorderedReturnEnabled());
  startCurrentCommand();
  assert(motions.size()==1 && close(motions[0].forward,1055.9f));
  advanceRoute();assert(routeIndex==1 && qrSweepRequests==0);
  startCurrentCommand();
  assert(motions.size()==2 && close(motions[1].right,1067.85f));
  assert(qrGateStarts==1);
  advanceRoute();assert(routeIndex==2 && qrSweepRequests==0);
  // Shortening the return must not change the initial 150mm/step1 balance.
  reset(1,1);routeIndex=0;
  startCurrentCommand();
  assert(motions.size()==1 && close(motions[0].forward,905.9f));
  // The 10 mm trim is actual distance, before applying each zone's direction.
  for(uint8_t zone:{1,2}) for(int mode:{0,1}){
    reset(zone,mode);routeIndex=0;
    startCurrentCommand();
    const float expected=zone==2 ? -1079.099f : (mode==1 ? 905.9f : 1055.9f);
    assert(motions.size()==1 && profiles.size()==1 && close(motions[0].forward,expected));
    const float originalScale=zone==1 ? 1.02f : 1.0422f;
    assert(close(profiles[0].maximumSpeed /
        (FAST_MAXIMUM_STEP_RATE*originalScale*NON_07_15_LINEAR_PROFILE_INCREASE_SCALE),1.0f));
    assert(close(profiles[0].acceleration /
        (FAST_STEP_ACCELERATION*originalScale*NON_07_15_LINEAR_PROFILE_INCREASE_SCALE),1.0f));
  }
  std::cout << "PASS actual C++ step1: actual distance -10 mm, both zones/modes, speed/acceleration preserved\n";
  std::cout << "PASS actual C++ return: zone1 908.4 -> (right1944.38, forward170) "
               "-> 150 mm in both modes; completion gates, zone2 and ordinary outbound preserved\n";
}
'''
compiler = shutil.which("g++")
assert compiler, "host g++ is required for the return state-machine test"
with tempfile.TemporaryDirectory(prefix="gc6_zone1_return_") as temporary:
    source = Path(temporary) / "return_test.cpp"
    executable = Path(temporary) / "return_test.exe"
    source.write_text(harness, encoding="utf-8")
    subprocess.run([compiler, "-std=c++17", "-O0", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
