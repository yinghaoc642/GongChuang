"""Execute the firmware's recovery hook and phase cases with fake hardware.

No serial port or motor is accessed. Requires a native C++ compiler on PATH.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "src/main.cpp").read_text(encoding="utf-8")

assert re.search(
    r"constexpr\s+float\s+M6_MAXIMUM_PHYSICAL_EXTENSION_MM\s*=\s*170\.0f\s*;",
    source,
)
assert re.search(
    r"constexpr\s+float\s+M6_MAXIMUM_EXTENSION_MM\s*=\s*"
    r"M6_MAXIMUM_PHYSICAL_EXTENSION_MM\s*-\s*"
    r"M6_STARTUP_WORKING_ZERO_OFFSET_MM\s*;",
    source,
)


def function(signature):
    start = source.index(signature + " {")
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


hook = function(
    "bool beginEndpointM6ReachRecovery(\n"
    "    LinearAxisMotion &axis,\n"
    "    bool cameraReachOverflow)")
cases = source[source.index("    case WORK_PHASE_ENDPOINT_REACH_DELAY:"):
               source.index("    case WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO:")]
handoff_start = source.index("      if (ring == 1U) {", source.index(
    '"[ENDPOINT SCAN] overhead center accepted '))
handoff_end = source.index("      (void)completeEndpointMapAndStartTransfers();", handoff_start)
handoff = source[handoff_start:handoff_end] + "completeEndpointMapAndStartTransfers();"

# Integration: only after two real position samples and a successful driver
# reset ACK may the hook replace the original push-again recovery.
recovery = function("void serviceLinearAxisRecovery(LinearAxisMotion &axis)")
assert recovery.index("secondPositionMm") < recovery.index("if (!resetAcknowledged)")
assert recovery.index("if (!resetAcknowledged)") < recovery.index("beginEndpointM6ReachRecovery(axis, false)")
assert recovery.index("beginEndpointM6ReachRecovery(axis, false)") < recovery.index("const float retryTargetMm")
assert "endpointReachRecoveryMoves = 0U;" in function(
    "void beginWorkAction(\n    WorkActionKind kind,\n    uint8_t roundNumber)")
assert "negateMotorPulses" in function("void startVisualCorrectionRestore()")
assert "visualCorrectionAccumulator" in function("void startVisualCorrectionRestore()")
move = function("bool startAccumulatedWorkstationMove(\n    float forwardMm,\n    float leftMm,\n    const char *reason,\n    bool restoreOnDeparture = true)")
restore = function("void startVisualCorrectionRestore()")

harness = r'''
#include <stdint.h>
#include <stdio.h>
extern "C" void abort();
#define assert(condition) ((condition) ? (void)0 : (fprintf(stderr,"assert line %d\n",__LINE__),abort()))
float fabsf(float value){return value < 0 ? -value : value;}
float hypotf(float x,float y){return __builtin_sqrtf(x*x+y*y);}
#include <RobotConfig.h>
using namespace gongchuang::config::arm_hardware;
enum { ARM_AXIS_RECOVERY_STALL, ARM_AXIS_RECOVERY_ACK_TIMEOUT };
enum { PROGRAM_RUNNING, PROGRAM_FAULT };
enum { WORK_ACTION_PROCESS, WORK_ACTION_STORAGE, WORK_ACTION_RAW };
enum {
 WORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION,
 WORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION,
 WORK_PHASE_ENDPOINT_REACH_DELAY, WORK_PHASE_ENDPOINT_REACH_MOVE,
 WORK_PHASE_WAIT_TRANSFER, WORK_PHASE_ENDPOINT_WAIT_COORDINATE,
 WORK_PHASE_CIRCLE_WAIT_CHASSIS, WORK_PHASE_WAIT_RESTORE
};
struct LinearAxisMotion {
 int address=6, recoveryReason=ARM_AXIS_RECOVERY_STALL;
 float targetMm=125.91f, commandStartMm=107.65f, currentMm=123.53f;
 bool recoveryPending=true;
};
struct ArmPose {
 float standardFrameAngleDegrees, extensionMm, heightMm;
 ArmPose(float a=0,float e=0,float h=0):standardFrameAngleDegrees(a),extensionMm(e),heightMm(h){}
};
struct FakeArm { bool moving=false; bool isM5Running(){return moving;} } armMotors;
struct Debug {
 template<class T> void print(T){}
 template<class T> void print(T,int){}
 template<class T> void println(T){}
 template<class T> void println(T,int){}
} SerialDebug;
LinearAxisMotion extensionAxis, liftAxis;
bool LOCAL_WORKSTATION_SIMULATION_MODE=false, armBaseMotionWatchdogActive=false;
bool wheelsDone=true, liftDone=true, fresh=true, fault=false;
int programState=PROGRAM_RUNNING, activeWorkAction=WORK_ACTION_PROCESS;
int workActionPhase=WORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION, workRoundIndex=0;
uint8_t activeEndpointScanRing=1, endpointReachRecoveryMoves=0;
float endpointReachRecoveryForwardMm=0,endpointReachRecoveryLeftMm=0;
bool endpointReachRecoveryRestoreOnDeparture=false;
uint32_t tick=100, workVisionRequestStartMs=80, endpointLocalSettleDeadlineMs=0, commandStartMs=0;
float yaw=0, endpointMapLockedHeadingDegrees=0;
float ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM=0.6f,ARM_AXIS_POSITION_TOLERANCE_MM=0.05f;
float M6_MAXIMUM_EXTENSION_MM=160;
float RING_ENDPOINT_MAXIMUM_SCAN_HEADING_DELTA_DEGREES=0.75f;
float ENDPOINT_FINE_VISION_LOWER_MM=95;
bool endpointFineVisionActive=true;
uint8_t activeEndpointServoMoveCount=2, activeEndpointStageServoMoveCount=2;
uint8_t endpointSearchFallbackMoveCount=1, workVisionRetryCount=1;
float endpointSearchBaseAngleDegrees=45;
ArmPose activeEndpointScanPose;
bool measuredRingPointValid[4]={false,true,false,true};
int moves=0, scans=0, resets=0, maps=0, planarMoves=0;
float movedForward=0, movedLeft=0, accumulatedLeft=0;
uint32_t millis(){return tick;}
bool deadlineReached(uint32_t d){return int32_t(tick-d)>=0;}
bool allMotorsArrived(){return wheelsDone;}
bool liftMoveFinished(){return liftDone;}
bool extensionMoveFinished(){return !extensionAxis.recoveryPending;}
bool imuIsFresh(){return fresh;}
float currentArmBaseStandardFrameDegrees(){return 37.06f;}
float currentRouteCounterClockwiseHeading(){return yaw;}
float wrapDeltaDegrees(float a){return a;}
void routeFault(const char*){fault=true;programState=PROGRAM_FAULT;}
void faultLinearAxis(LinearAxisMotion&,const char* r){routeFault(r);}
void markLinearAxisArrived(LinearAxisMotion& a){a.currentMm=a.targetMm;a.recoveryPending=false;}
void stopMaixRequest(){}
void resetEndpointCenteredConfirmationWindow(){}
void markMissionProgress(){}
struct MotorPulses {
 float f,l;MotorPulses(float forward=0,float left=0):f(forward),l(left){}
};
MotorPulses visualCorrectionAccumulator;
float visualCorrectionForwardMm=0,visualCorrectionLeftMm=0;
uint8_t visualCorrectionMoveCount=0;
const float MAXIMUM_ACCUMULATED_VISUAL_CORRECTION_MM=100;
const uint8_t MAXIMUM_VISUAL_CORRECTION_MOVES=5;
const float WORKSTATION_MAXIMUM_STEP_RATE=100,WORKSTATION_STEP_ACCELERATION=100;
bool turnMotionEnabled=false,workstationApproachEnabled=false,translationPreciseArrivalEnabled=false;
bool STORAGE_ROUND2_SIMULATION_MODE=false,ROUGH_PROCESSING_CALIBRATION_MODE=false;
bool STORAGE_ROUND1_SIMULATION_MODE=false;
uint32_t headingStableStartMs=0,motorsArrivedStartMs=0;
MotorPulses bodyDisplacementToMotorPulses(float f,float l,float){return MotorPulses(f*1000,l*1000);}
MotorPulses addMotorPulses(MotorPulses a,MotorPulses b){return MotorPulses(a.f+b.f,a.l+b.l);}
MotorPulses negateMotorPulses(MotorPulses a){return MotorPulses(-a.f,-a.l);}
bool motorPulsesAreZero(MotorPulses a){return a.f==0&&a.l==0;}
void setDriveMotionProfile(float,float){}
void startRelativeMotorMove(MotorPulses p){
 ++moves;movedForward=p.f;movedLeft=p.l;accumulatedLeft+=p.l;wheelsDone=false;
}
void stopAllMotorsImmediately(){wheelsDone=true;}
namespace route_chassis {void stopAllMotorsImmediately(){::stopAllMotorsImmediately();}}
void beginStorageParkingBeforeWorkFinish(){}
void resetMeasuredRingMap(){++resets;for(auto &v:measuredRingPointValid)v=false;}
void beginEndpointVision(){++scans;workActionPhase=WORK_PHASE_ENDPOINT_WAIT_COORDINATE;}
bool nominalRingPose(uint8_t r,float h,ArmPose &p){p=ArmPose(r==1?45:-45,90,-h);return true;}
void initializeEndpointScanState(uint8_t r,const ArmPose&p){activeEndpointScanRing=r;activeEndpointScanPose=p;}
bool startEndpointSearchPlanarMove(){++planarMoves;return true;}
bool beginDirectRing3EndpointScan(){activeEndpointScanRing=3;++planarMoves;return true;}
bool completeEndpointMapAndStartTransfers(){
 assert(measuredRingPointValid[1]&&measuredRingPointValid[3]);++maps;return true;
}
'''
harness += move + "\n" + restore + "\n" + hook + "\nvoid service(){switch(workActionPhase){\n" + cases + "default:break;}}\n"
harness += "void accept(uint8_t ring){measuredRingPointValid[ring]=true;do{\n" + handoff + "}while(false);}\n"
harness += r'''
void reset(){
 extensionAxis=LinearAxisMotion();liftAxis=LinearAxisMotion();liftAxis.address=7;liftAxis.currentMm=-95;
 programState=PROGRAM_RUNNING;activeWorkAction=WORK_ACTION_PROCESS;workRoundIndex=0;
 workActionPhase=WORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION;activeEndpointScanRing=1;
 endpointReachRecoveryMoves=0;LOCAL_WORKSTATION_SIMULATION_MODE=false;
 endpointReachRecoveryForwardMm=endpointReachRecoveryLeftMm=0;
 endpointReachRecoveryRestoreOnDeparture=false;
 armMotors.moving=false;armBaseMotionWatchdogActive=false;
 tick=100;wheelsDone=liftDone=fresh=true;fault=false;yaw=0;
 moves=scans=resets=maps=planarMoves=0;accumulatedLeft=0;
 visualCorrectionAccumulator=MotorPulses();
 visualCorrectionForwardMm=visualCorrectionLeftMm=0;visualCorrectionMoveCount=0;
 measuredRingPointValid[1]=measuredRingPointValid[3]=true;
}
int main(){
 // Actual incident: cancel 125.91 target, keep measured 123.53 and M5 angle.
 reset();assert(beginEndpointM6ReachRecovery(extensionAxis,false));
 assert(!fault && extensionAxis.targetMm==123.53f);
 assert(activeEndpointScanPose.extensionMm==123.53f);
 assert(activeEndpointScanPose.standardFrameAngleDegrees==37.06f);
 tick=109;service();assert(moves==0);
 tick=110;service();assert(moves==1 && movedForward==0 && movedLeft==-20);
 assert(endpointReachRecoveryMoves==1 && accumulatedLeft==-20);
 assert(motorPulsesAreZero(visualCorrectionAccumulator));
 assert(visualCorrectionLeftMm==-20 && visualCorrectionMoveCount==1);
 service();assert(scans==0); // no camera acceptance while chassis moving
 wheelsDone=true;service();assert(scans==1 && resets==1);
 assert(!measuredRingPointValid[1]&&!measuredRingPointValid[3]);
 assert(activeEndpointScanRing==1 && planarMoves==0); // M5 never commanded
 accept(1);assert(activeEndpointScanRing==3 && maps==0);
 accept(3);assert(maps==1);
 // Departure must not undo the permanent 20 mm reach correction.
 int beforeDeparture=moves;startVisualCorrectionRestore();
 assert(moves==beforeDeparture && accumulatedLeft==-20);
 // The 170 mm physical travel leaves a 160 mm working range. A camera target
 // beyond it approaches 20 mm forward, rescans, then restores that local move.
 reset();extensionAxis.recoveryPending=false;
 workActionPhase=WORK_PHASE_ENDPOINT_WAIT_COORDINATE;
 assert(beginEndpointM6ReachRecovery(extensionAxis,true)&&!fault);
 tick=110;service();assert(moves==1&&movedForward==20&&movedLeft==0);
 assert(endpointReachRecoveryMoves==1&&visualCorrectionAccumulator.f==20);
 wheelsDone=true;service();assert(scans==1&&resets==1);
 startVisualCorrectionRestore();assert(movedForward==-20);
 reset();extensionAxis.recoveryPending=false;
 workActionPhase=WORK_PHASE_ENDPOINT_WAIT_COORDINATE;
 LOCAL_WORKSTATION_SIMULATION_MODE=true;
 assert(!beginEndpointM6ReachRecovery(extensionAxis,true)&&moves==0&&!fault);
 // Ordinary corrections still restore only their own displacement, even
 // when mixed with a permanent reach recovery at the same workstation.
 reset();assert(startAccumulatedWorkstationMove(0,7,"ordinary"));
 wheelsDone=true;workActionPhase=WORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION;
 assert(beginEndpointM6ReachRecovery(extensionAxis,false));tick=110;service();
 wheelsDone=true;service();assert(visualCorrectionAccumulator.l==7);
 assert(accumulatedLeft==-13 && visualCorrectionLeftMm==-13);
 startVisualCorrectionRestore();assert(movedLeft==-7 && accumulatedLeft==-20);
 // A stall at ring 3 resumes ring 3 first, then remeasures old ring 1.
 reset();activeEndpointScanRing=3;assert(beginEndpointM6ReachRecovery(extensionAxis,false));
 tick=110;service();wheelsDone=true;service();assert(activeEndpointScanRing==3);
 accept(3);assert(activeEndpointScanRing==1 && maps==0);
 accept(1);assert(maps==1);
 // Wrong phases/axis/direction/diagnosis and bench mode never shift chassis.
 reset();assert(!beginEndpointM6ReachRecovery(liftAxis,false));
 reset();extensionAxis.targetMm=90;assert(!beginEndpointM6ReachRecovery(extensionAxis,false));
 reset();extensionAxis.recoveryReason=ARM_AXIS_RECOVERY_ACK_TIMEOUT;
 assert(!beginEndpointM6ReachRecovery(extensionAxis,false));
 reset();workActionPhase=WORK_PHASE_WAIT_TRANSFER;assert(!beginEndpointM6ReachRecovery(extensionAxis,false));
 reset();activeWorkAction=WORK_ACTION_RAW;assert(!beginEndpointM6ReachRecovery(extensionAxis,false));
 reset();activeWorkAction=WORK_ACTION_STORAGE;workRoundIndex=1;
 assert(!beginEndpointM6ReachRecovery(extensionAxis,false));
 reset();LOCAL_WORKSTATION_SIMULATION_MODE=true;assert(!beginEndpointM6ReachRecovery(extensionAxis,false));
 // Repeat bounds, communication, motion, yaw and deadline guards.
 reset();endpointReachRecoveryMoves=M6_SCAN_RECOVERY_MAX_MOVES;
 assert(beginEndpointM6ReachRecovery(extensionAxis,false)&&fault&&moves==0);
 reset();armMotors.moving=true;assert(beginEndpointM6ReachRecovery(extensionAxis,false)&&fault);
 reset();fresh=false;assert(beginEndpointM6ReachRecovery(extensionAxis,false)&&fault);
 reset();assert(beginEndpointM6ReachRecovery(extensionAxis,false));tick=110;service();
 tick+=M6_SCAN_RECOVERY_MOVE_TIMEOUT_MS;service();assert(fault&&scans==0);
 reset();assert(beginEndpointM6ReachRecovery(extensionAxis,false));tick=110;service();
 wheelsDone=true;yaw=2;service();assert(fault&&scans==0);
 // Deadline wraparound still waits the full 10 ms.
 reset();tick=UINT32_MAX-5;assert(beginEndpointM6ReachRecovery(extensionAxis,false));
 tick=3;service();assert(moves==0);tick=4;service();assert(moves==1);
}
'''

compiler = shutil.which("g++")
if not compiler:
    raise SystemExit("Native g++ is required")
with tempfile.TemporaryDirectory(prefix="m6_recovery_test_") as directory:
    cpp = Path(directory) / "test.cpp"
    exe = Path(directory) / "test.exe"
    cpp.write_text(harness, encoding="utf-8")
    result = subprocess.run([compiler, "-std=c++11", "-O0", "-fmax-errors=5", "-I",
                             str(ROOT / "lib/RobotConfig/src"), str(cpp), "-o", str(exe)],
                            capture_output=True, text=True)
    if result.returncode:
        raise SystemExit(result.stderr[-6000:])
    subprocess.run([str(exe)], check=True)
print("PASS M6 endpoint recovery: stall left-20mm; camera overflow forward+20mm and restore; fresh maps, scope and bounds")
