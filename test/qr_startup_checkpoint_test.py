"""Run actual QR, startup-hold and checkpoint functions with fake UART/motors.

Uses the real CompetitionCore parser. No serial device is opened or flashed.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from route_chassis_integration_test import compact, extract_function

ROOT = Path(__file__).resolve().parents[1]
main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
route = (ROOT / "src/route_chassis_main.inc").read_text(encoding="utf-8")
debug = (ROOT / "src/debug_qr_continuous.cpp").read_text(encoding="utf-8")


def function(source, signature, name):
    return signature + " {\n" + extract_function(source, name) + "\n}\n"


# Verify scheduling/ownership as well as individual functions below.
loop = compact(extract_function(main, "loop"))
for fragment in (
    "route_chassis::consumeZone1FreshQrResetRequest()",
    "resetQrReceiver(false);",
    "receiveQrData();",
    "synchronizeTaskCodeToRouteChassis();",
):
    assert fragment in loop
assert loop.index("consumeZone1FreshQrResetRequest") < loop.index("receiveQrData();")
assert loop.index("receiveQrData();") < loop.index("serviceIntegratedRoute();")
assert loop.index("abortRoute();") < loop.index("serviceIntegratedRoute();")
begin = compact(extract_function(main, "beginRoute"))
assert begin.count("resetQrReceiver();") == 1
assert begin.index("if(LOCAL_WORKSTATION_SIMULATION_MODE)") < begin.index("route_chassis::beginRoute();")
assert "chassisStartupHoldStartMs=route_chassis::startRequestAcceptedMs;" in begin
assert "chassisStartupHoldStartMs=millis();" not in begin
assert begin.index("route_chassis::beginRoute();") < begin.index("chassisStartupHoldStartMs=route_chassis::startRequestAcceptedMs;")
click = compact(extract_function(route, "onStartButtonClick"))
assert click.index("startRequestAcceptedMs=millis();") < click.index("startRequested=true;")
service = compact(extract_function(main, "serviceIntegratedRoute"))
assert service.index("if(serviceChassisStartupHold())") < service.index("route_chassis::updateRoute();")
for name in ("routeFault", "abortRoute"):
    assert "chassisStartupHoldActive=false;" in compact(extract_function(main, name))
assert "startCurrentCommand();" not in extract_function(route, "beginRoute")
assert "resetQrReceiver(false);" in extract_function(route, "beginRoute")
start = compact(extract_function(route, "startCurrentCommand"))
assert (
    "if(command.specificationStep==4U&&programState==PROGRAM_RUNNING&&"
    "!zone1ReorderedRouteEnabled()){beginStep04QrGate();}" in start
)
update = compact(extract_function(route, "updateRoute"))
assert update.index("if(serviceStep04QrGate())") < update.rindex("serviceOrdinaryCommand();")
assert "ZONE1_REORDERED_QR_RETURN_RIGHT){serviceOrdinaryCommand();return;}" in update
run = extract_function(route, "runAllMotors")
assert run.index("if (stopStep04AtQrCheckpoint())") < run.index("motor1.runSpeed();")
assert run.count("stopStep04AtQrCheckpoint()") == 9
assert "SerialDebug" not in run and "hmiSetRunStatus" not in run
assert "remainingSteps - step04QrReservedBasePulses" in extract_function(route, "maximumTranslationComponentDistanceToGo")
assert "moveTo(" in extract_function(route, "beginStep04QrGate")
assert "translationRelativeMotorPulses.motor1" in extract_function(route, "beginStep04QrGate")
assert "STEP_04_QR_WAIT_REMAINING_MM /" in extract_function(route, "beginStep04QrGate")
cutoff = extract_function(route, "stopStep04AtQrCheckpoint")
assert "setCurrentPosition(motors[i]->currentPosition())" in cutoff
assert "SerialDebug" not in cutoff and "hmiSetRunStatus" not in cutoff
assert "constexpr uint32_t CHASSIS_STARTUP_HOLD_MS = 500UL;" in main
assert "constexpr float STEP_04_QR_WAIT_REMAINING_MM = 190.0f;" in route
assert re.search(r"QR_REQUIRED_MATCHING_FRAMES\s*=\s*1U", main)

harness = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <CompetitionCore.h>
uint32_t nowMs=1000;
uint32_t millis(){return nowMs;}
uint32_t micros(){return nowMs*1000U;}
struct FakePort {
 std::string rx; size_t index=0; int lastValue=-1;
 int available(){return int(rx.size()-index);}
 int read(){return available()?static_cast<unsigned char>(rx[index++]):-1;}
 void begin(unsigned long){}
 void inject(const std::string &s){rx+=s;}
 void clear(){rx.clear();index=0;lastValue=-1;}
 template<class T> void print(T){}
 template<class T> void println(T){}
 void println(int value){lastValue=value;}
};
namespace mission {
FakePort SerialQr,SerialDebug;
constexpr size_t QR_CODE_LENGTH=competition::TASK_CODE_LENGTH,QR_BUFFER_SIZE=32;
constexpr uint8_t QR_REQUIRED_MATCHING_FRAMES=1U;
char qrData[QR_BUFFER_SIZE]={},qrCandidate[QR_CODE_LENGTH+1U]={};
size_t qrDataIndex=0;
bool qrOverflow=false,scanFlag=false,taskCodeDecoded=false;
uint8_t qrMatchingFrameCount=0;
competition::TaskPlan taskPlan;
int progress=0;
void markMissionProgress(){++progress;}
void hmiSetText(const char*,const char*){}
void hmiShowTaskCode(const char*){}
'''
harness += function(
    main,
    "void resetQrReceiver(bool discardPendingBytes=true)",
    "resetQrReceiver",
)
for name in ("finishQrFrame", "receiveQrData"):
    harness += function(main, f"void {name}()", name)
harness += r'''
}
namespace bench {
FakePort qrPort,debugPort;
struct Button{void tick(){}} startButton;
constexpr size_t kFrameCapacity=32;
char frame[kFrameCapacity]={};size_t frameLength=0;
bool frameOverflow=false,scanning=false;uint32_t frameCount=0;
'''
for name in ("startScanning", "finishFrame", "loop"):
    harness += function(debug, f"void {name}()", name)
harness += r'''
}
namespace startup {
FakePort SerialDebug;
constexpr uint32_t CHASSIS_STARTUP_HOLD_MS=500UL;
bool chassisStartupHoldActive=false;
uint32_t chassisStartupHoldStartMs=0;
int progress=0;
void markMissionProgress(){++progress;}
void hmiSetRunStatus(const char*){}
'''
harness += function(main, "bool serviceChassisStartupHold()", "serviceChassisStartupHold")
harness += r'''
}
namespace route_chassis {
enum {PROGRAM_RUNNING,PROGRAM_FAULT,ROUTE_PHASE_MOTION,ROUTE_PHASE_WAIT_SCAN_CODE};
struct MotorPulses{
 long motor1,motor2,motor3,motor4;
 MotorPulses(long m1=0,long m2=0,long m3=0,long m4=0):motor1(m1),motor2(m2),motor3(m3),motor4(m4){}
};
struct FakeMotor {
 long pos=0,target=0;float speed=0,acceleration=0;int resets=0,runs=0;
 long currentPosition(){return pos;}long distanceToGo(){return target-pos;}
 void moveTo(long x){target=x;}
 void setCurrentPosition(long x){pos=target=x;speed=0;++resets;}
 void setAcceleration(float x){acceleration=x;}
 void runSpeed(){if(speed!=0 && target!=pos){pos+=target>pos?1:-1;++runs;}}
 void run(){runSpeed();}
} motor1,motor2,motor3,motor4;
FakeMotor* motors[]={&motor1,&motor2,&motor3,&motor4};
MotorPulses translationMotionOrigin={},translationRelativeMotorPulses={};
bool scanFlag=false,step04QrGateActive=false,step04QrWaiting=false;
bool step04QrWaitNoticePending=false;
bool step04QrBraking=false;
float step04QrBrakeDeceleration=0,step04QrBrakeReferencePulses=0;
bool translationBaseDisplacementComplete=false,translationBaseForcedComplete=false;
bool zone1QrSweepBraking=false;
bool translationTerminalCorrectionStartValid=false,driveDecelerationActive=false;
bool translationHeadingControlActive=true;
float translationMaximumBasePulses=0,step04QrReservedBasePulses=0;
 float activeTranslationForwardMm=0,activeTranslationRightMm=0,activeDriveAcceleration=300;
float activeDriveMaximumStepRate=3000,activeDriveDeceleration=100;
enum {DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID,DRIVE_PROFILE_ASYMMETRIC_S_CURVE};
int activeDriveProfileShape=DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID;
constexpr float DECELERATION_SWITCH_MARGIN_STEPS=8,PI_F=3.14159265f;
constexpr float S_CURVE_DECELERATION_GENTLENESS=1,S_CURVE_MINIMUM_SPEED_SCALE=0.05f;
constexpr float TRANSLATION_HEADING_MAXIMUM_CONTROL_DT_SECONDS=0.05f;
constexpr float TRANSLATION_TERMINAL_MINIMUM_STEP_RATE=80.0f;
float driveAccelerationDistanceSteps=0;
float baseRateLimitScale=1.0f;
float activeTranslationBaseRateLimitScale(){return baseRateLimitScale;}
float translationBaseStepRate=0,translationAppliedBaseStepRate=0;
float translationProfileTraveledStepsBeforeOrigin=0;
float translationHeadingCorrectionRateDps=0,translationAppliedHeadingCorrectionRateDps=0;
constexpr float STEP_04_QR_WAIT_REMAINING_MM=190.0f;
constexpr float STEP_04_QR_BRAKE_REFERENCE_MM=100.0f;
constexpr float TRANSLATION_POSITION_TOLERANCE_STEPS=0.75f;
uint32_t commandStartMs=100,translationVelocityProfileLastUpdateUs=0;
uint32_t translationLastControlMs=0,translationLastImuFrameSequence=0,imuValidFrameSequence=7;
uint32_t translationTerminalCorrectionStartMs=0;
int programState=PROGRAM_RUNNING,routePhase=ROUTE_PHASE_MOTION;
size_t routeIndex=1;constexpr size_t ROUTE_COMMAND_COUNT=19;
int mixes=0;
FakePort SerialDebug;
void hmiSetRunStatus(const char*){}
void routeFault(const char*){programState=PROGRAM_FAULT;}
float absolutePulseCount(long value){return std::fabs(float(value));}
long roundedPulseCount(float pulses){return static_cast<long>(pulses>=0.0f?pulses+0.5f:pulses-0.5f);}
MotorPulses currentMotorPositions(){return {motor1.pos,motor2.pos,motor3.pos,motor4.pos};}
// Deterministic encoder scale, while the actual firmware owns all transitions,
// four-wheel translation projection, target preservation and distance limits.
MotorPulses vehicleDisplacementToMotorPulses(float forward,float right,float){
 long f=std::lround(forward*10),r=std::lround(right*10);
 return {-r+f,r-f,r+f,-r-f};
}
bool allMotorsArrived(){for(auto motor:motors)if(motor->distanceToGo())return false;return true;}
void applyTranslationWheelSpeedMix(){++mixes;}
void serviceDriveMotionProfile(){}
'''
for name in ("translationProgressFraction", "maximumTranslationComponentDistanceToGo"):
    harness += function(route, f"float {name}()", name)
for name in ("effectiveSCurvePeakDeceleration", "sCurveStoppingSteps",
             "trapezoidStoppingSteps", "activeProfileStoppingSteps"):
    signature = f"float {name}()" if name == "effectiveSCurvePeakDeceleration" else f"float {name}(float stepRate)"
    harness += function(route, signature, name)
harness += function(route, "float activeProfileBrakingLimitedStepRate(float remainingSteps)", "activeProfileBrakingLimitedStepRate")
harness += function(route, "float step04QrCheckpointBrakingRateLimit(float remainingSteps)", "step04QrCheckpointBrakingRateLimit")
harness += function(route, "float moveTowardWithMaximumDelta(float current, float target, float maximumDelta)", "moveTowardWithMaximumDelta")
harness += function(route, "void updateTranslationBaseRate()", "updateTranslationBaseRate")
harness += function(route, "void beginStep04QrGate()", "beginStep04QrGate")
harness += function(route, "bool stopStep04AtQrCheckpoint()", "stopStep04AtQrCheckpoint")
harness += function(route, "void runAllMotors()", "runAllMotors")
harness += function(route, "bool serviceStep04QrGate()", "serviceStep04QrGate")
harness += r'''
void reset(bool acquired=false){
 scanFlag=acquired;step04QrGateActive=step04QrWaiting=step04QrWaitNoticePending=false;
 step04QrReservedBasePulses=0;
 step04QrBrakeReferencePulses=0;step04QrBraking=false;step04QrBrakeDeceleration=0;
 translationBaseDisplacementComplete=translationBaseForcedComplete=false;
 zone1QrSweepBraking=false;
 translationTerminalCorrectionStartValid=driveDecelerationActive=false;
 translationHeadingControlActive=true;
 activeTranslationForwardMm=20.0f;
 activeTranslationRightMm=1017.0f*1.03f; // Existing full endpoint calibration.
 translationRelativeMotorPulses=vehicleDisplacementToMotorPulses(activeTranslationForwardMm,activeTranslationRightMm,0);
 translationMotionOrigin={11,22,33,44};
 translationMaximumBasePulses=std::fmax(
     std::fmax(absolutePulseCount(translationRelativeMotorPulses.motor1),absolutePulseCount(translationRelativeMotorPulses.motor2)),
     std::fmax(absolutePulseCount(translationRelativeMotorPulses.motor3),absolutePulseCount(translationRelativeMotorPulses.motor4)));
 long origin[]={11,22,33,44};
 long delta[]={translationRelativeMotorPulses.motor1,translationRelativeMotorPulses.motor2,
               translationRelativeMotorPulses.motor3,translationRelativeMotorPulses.motor4};
 for(int i=0;i<4;++i){*motors[i]=FakeMotor();motors[i]->pos=origin[i];motors[i]->target=origin[i]+delta[i];motors[i]->speed=80;}
 translationBaseStepRate=translationAppliedBaseStepRate=80;
 activeDriveMaximumStepRate=3000;activeDriveDeceleration=100;baseRateLimitScale=1;
 translationVelocityProfileLastUpdateUs=micros();
 programState=PROGRAM_RUNNING;routePhase=ROUTE_PHASE_MOTION;commandStartMs=100;
 routeIndex=1;mixes=0;
}
void positionAt(long progress,long rotation=0){
 long origin[]={11,22,33,44};
 long delta[]={translationRelativeMotorPulses.motor1,translationRelativeMotorPulses.motor2,
               translationRelativeMotorPulses.motor3,translationRelativeMotorPulses.motor4};
 float fraction=float(progress)/translationMaximumBasePulses;
 for(int i=0;i<4;++i)motors[i]->pos=origin[i]+std::lround(float(delta[i])*fraction)+rotation;
}
void positionAtCurrentTargets(){for(auto motor:motors)motor->pos=motor->target;}
void checkFullTargets(){
 assert(motor1.target==11+translationRelativeMotorPulses.motor1);
 assert(motor2.target==22+translationRelativeMotorPulses.motor2);
 assert(motor3.target==33+translationRelativeMotorPulses.motor3);
 assert(motor4.target==44+translationRelativeMotorPulses.motor4);
}
}
enum {PROGRAM_RUNNING,WORK_ACTION_NONE,WORK_ACTION_RAW,WORK_PHASE_IDLE,WORK_PHASE_RAW_WAIT_RESULT};
bool LOCAL_WORKSTATION_SIMULATION_MODE=false;
int programState=PROGRAM_RUNNING,activeWorkAction=WORK_ACTION_NONE,workActionPhase=WORK_PHASE_IDLE;
uint32_t lastMissionProgressMs=0;constexpr uint32_t MISSION_PROGRESS_TIMEOUT_MS=70000;
int missionFaults=0;void routeFault(const char*){++missionFaults;}
'''
harness += function(main, "void serviceCompetitionWatchdogs()", "serviceCompetitionWatchdogs")
harness += r'''
int main(){
 const char* first="123+132+123+132";const char* second="134+123+314+231";
 // Same real parser and UART framing for valid and invalid frames in both entries.
 const char* samples[]={first,second,"123-132+123+132","113+132+123+132","123+132+124+132","short"};
 for(auto sample:samples){
  competition::TaskPlan parsed;bool valid=competition::parseTaskCode(sample,parsed)==competition::TASK_CODE_OK;
  mission::resetQrReceiver();mission::SerialQr.inject(std::string(sample)+"\r\n");mission::receiveQrData();
  bench::qrPort.clear();bench::scanning=false;bench::startScanning();auto count=bench::frameCount;
  bench::qrPort.inject(std::string(sample)+"\r\n");bench::loop();
  assert(mission::scanFlag==valid);assert(bench::frameCount==count+1);
  assert(bench::debugPort.lastValue==int(valid));
 }
 // Partial packets survive separate calls. One valid frame is sufficient.
 mission::resetQrReceiver();mission::SerialQr.inject("123+132");mission::receiveQrData();assert(!mission::scanFlag);
 mission::SerialQr.inject("+123+132\n");mission::receiveQrData();assert(mission::scanFlag);
 // Mission must retain the first task even with a second frame/partial data queued.
 mission::resetQrReceiver();mission::SerialQr.inject(std::string(first)+"\r\n"+second+"\r\nnoise");mission::receiveQrData();
 assert(mission::scanFlag && mission::taskCodeDecoded && !std::strcmp(mission::qrData,first));
 mission::receiveQrData();assert(!std::strcmp(mission::qrData,first));assert(mission::SerialQr.available()==0);
 competition::TaskPlan expected;competition::parseTaskCode(first,expected);
 assert(!std::memcmp(mission::taskPlan.colors,expected.colors,sizeof(expected.colors)));
 assert(!std::memcmp(mission::taskPlan.positions,expected.positions,sizeof(expected.positions)));
 // Oversized junk rejects and the next frame recovers; debug continues all frames.
 mission::resetQrReceiver();mission::SerialQr.inject(std::string(50,'x')+"\r"+first+"\n");mission::receiveQrData();assert(mission::scanFlag);
 bench::qrPort.inject(std::string(first)+"\r"+second+"\n");auto count=bench::frameCount;bench::loop();assert(bench::frameCount==count+2);
 // Actual nonblocking startup timer: 0/499 held, 500 released once, wrap safe.
 startup::chassisStartupHoldActive=true;startup::chassisStartupHoldStartMs=1000;nowMs=1000;
 assert(startup::serviceChassisStartupHold());nowMs=1499;assert(startup::serviceChassisStartupHold());
 nowMs=1500;assert(!startup::serviceChassisStartupHold());assert(startup::progress==1);
 assert(!startup::serviceChassisStartupHold());assert(startup::progress==1);
 // Checks consuming part/all of the minimum wait: only fill the remainder.
 startup::chassisStartupHoldActive=true;startup::chassisStartupHoldStartMs=1000;nowMs=1200;
 assert(startup::serviceChassisStartupHold());nowMs=1500;assert(!startup::serviceChassisStartupHold());
 startup::chassisStartupHoldActive=true;startup::chassisStartupHoldStartMs=1000;nowMs=1750;
 assert(!startup::serviceChassisStartupHold());
 startup::chassisStartupHoldActive=true;startup::chassisStartupHoldStartMs=UINT32_MAX-100;
 nowMs=398;assert(startup::serviceChassisStartupHold());nowMs=399;assert(!startup::serviceChassisStartupHold());
 using namespace route_chassis;
 // Before step starts: valid QR means the original full segment, no checkpoint.
 reset(true);beginStep04QrGate();assert(!step04QrGateActive);assert(!serviceStep04QrGate());checkFullTargets();
 // During motion: extend the target without cancelling speed or creating a new origin.
 reset();beginStep04QrGate();positionAt(4000);route_chassis::scanFlag=true;
 assert(!serviceStep04QrGate());assert(!step04QrWaiting && translationBaseStepRate==80);
 checkFullTargets();assert(translationMotionOrigin.motor1==11 && commandStartMs==100);
 for(auto motor:motors)assert(motor->resets==0 && motor->speed==80);
 // Missing QR: braking uses the shorter checkpoint while preserving the full vector.
 reset();beginStep04QrGate();const long full=long(translationMaximumBasePulses);
 const long reserved=std::lround(step04QrReservedBasePulses);
 const long brakeReference=std::lround(step04QrBrakeReferencePulses);
 assert(reserved==std::lround(translationMaximumBasePulses*190.0f/activeTranslationRightMm));
 assert(brakeReference==std::lround(translationMaximumBasePulses*100.0f/activeTranslationRightMm));
 assert(std::fabs(maximumTranslationComponentDistanceToGo()-(translationMaximumBasePulses-reserved))<1);
 positionAt(full-reserved-20);
 assert(!serviceStep04QrGate());
 #if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  positionAt(full-reserved,73); // Common IMU rotation does not shift lateral progress.
 #else
  positionAtCurrentTargets();
 #endif
 assert(serviceStep04QrGate());assert(step04QrWaiting && routePhase==ROUTE_PHASE_WAIT_SCAN_CODE);
 assert(!translationHeadingControlActive && translationBaseStepRate==0);
 for(auto motor:motors)assert(motor->target==motor->pos && motor->speed==0);
 for(int i=0;i<100;++i)runAllMotors();
 for(auto motor:motors)assert(motor->runs==0);
 // Long waiting doesn't consume route indices or trigger mission watchdog.
 nowMs=600000;serviceCompetitionWatchdogs();assert(missionFaults==0);
 assert(serviceStep04QrGate() && routeIndex==1);
 // After QR: absolute ORIGINAL endpoint, not a new segment or twice-scaled distance.
 route_chassis::scanFlag=true;assert(!serviceStep04QrGate());checkFullTargets();
 assert(commandStartMs==nowMs && routeIndex==1 && routePhase==ROUTE_PHASE_MOTION);
 assert(!step04QrGateActive && !step04QrWaiting);
 assert(std::fabs(maximumTranslationComponentDistanceToGo()-reserved)<1);
 positionAt(full);assert(maximumTranslationComponentDistanceToGo()<=3);
 // Non-checkpoint motion still has the normal watchdog protection.
 serviceCompetitionWatchdogs();assert(missionFaults==1);
 // Actual braking helper: start near the 100mm reference at low speed.
 reset();beginStep04QrGate();translationBaseStepRate=200;
 assert(step04QrCheckpointBrakingRateLimit(1500)==3000 && !step04QrBraking);
 assert(std::fabs(step04QrCheckpointBrakingRateLimit(1000)-200)<0.1f);
 assert(step04QrBraking && std::fabs(step04QrBrakeDeceleration-20)<0.01f);
 assert(std::fabs(step04QrCheckpointBrakingRateLimit(250)-100)<0.1f);
 // Faster movement needs 500mm: brake earlier, without increasing deceleration.
 reset();beginStep04QrGate();translationBaseStepRate=1000;
 assert(step04QrCheckpointBrakingRateLimit(6000)==3000 && !step04QrBraking);
 assert(std::fabs(step04QrCheckpointBrakingRateLimit(5000)-1000)<0.1f);
 assert(step04QrBraking && step04QrBrakeDeceleration<=activeDriveDeceleration);
 // A QR during braking releases the cap and keeps the original endpoint.
 route_chassis::scanFlag=true;assert(!serviceStep04QrGate());checkFullTargets();
 assert(!step04QrBraking && step04QrCheckpointBrakingRateLimit(200)==3000);
 #if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
 // Other route profiles retain their own base-rate caps; QR logic is scoped.
 reset(true);activeDriveMaximumStepRate=300;baseRateLimitScale=1.2f;translationBaseStepRate=350;
 nowMs+=10;updateTranslationBaseRate();assert(translationBaseStepRate>350);
 // Execute the actual soft velocity planner during approach, not just its helper.
 reset();beginStep04QrGate();translationBaseStepRate=200;
 float remaining=1000;positionAt(full-reserved-1000);
 int iterations=0;float previous=200;
 while(remaining>1.0f && iterations++<10000){
  nowMs+=10;updateTranslationBaseRate();
  assert(translationBaseStepRate<=previous+0.01f);
  assert(previous-translationBaseStepRate<=activeDriveDeceleration*0.010f+0.01f);
  previous=translationBaseStepRate;
  remaining-=translationBaseStepRate*0.010f;
   positionAt(full-reserved-std::lround(std::fmax(remaining,0.0f)));
 }
 assert(iterations<10000 && step04QrBraking && previous<=81);
 positionAt(full-reserved);runAllMotors();assert(step04QrWaiting);
 #endif
 // At the point after braking: zero extra pulses and enabled holding targets.
 reset();beginStep04QrGate();
 #if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
 positionAt(full-reserved);
 #else
 positionAtCurrentTargets();
 #endif
 runAllMotors();assert(step04QrWaiting);
 for(auto motor:motors)assert(motor->runs==0);
 // Valid QR arriving just before the point lets normal movement continue.
 reset();beginStep04QrGate();
 #if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
 positionAt(full-reserved);
 #else
 positionAtCurrentTargets();
 #endif
 route_chassis::scanFlag=true;
 assert(!serviceStep04QrGate());runAllMotors();assert(!step04QrWaiting);
 for(auto motor:motors)assert(motor->runs==1);
}
'''

compiler = shutil.which("g++")
if not compiler:
    raise SystemExit("Native g++ is required")
with tempfile.TemporaryDirectory(prefix="qr_startup_checkpoint_") as directory:
    cpp = Path(directory) / "test.cpp"
    cpp.write_text(harness, encoding="utf-8")
    for mode in (1, 0):
        exe = Path(directory) / f"test_mode{mode}.exe"
        result = subprocess.run(
            [compiler, "-std=c++11", "-O0", "-fmax-errors=5",
             f"-DGONGCHUANG_TRANSLATION_CONTROL_MODE={mode}",
             "-I", str(ROOT / "lib/CompetitionCore/src"), str(cpp),
             str(ROOT / "lib/CompetitionCore/src/CompetitionCore.cpp"), "-o", str(exe)],
            capture_output=True, text=True,
        )
        if result.returncode:
            raise SystemExit(result.stderr[-6000:])
        subprocess.run([str(exe)], check=True)
        print(f"PASS QR/startup/checkpoint mode {mode}: real parser/framing, first-frame lock, 500ms/wrap, 100mm braking reference, 190mm stop, original endpoint and watchdog")
