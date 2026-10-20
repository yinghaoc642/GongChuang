"""Run actual firmware travel/color/return functions with fake hardware.

No serial connection. Covers both travel rounds, all tray/known-ring-color
permutations, targeted requests, release clearance and endpoint home.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "src/main.cpp").read_text(encoding="utf-8")


def function(name):
    match = re.search(r"^\w[\w :*&<>]*\b" + name + r"\s*\([^;{}]*\)\s*\{", source, re.M)
    assert match, name
    i, depth = match.end(), 1
    while depth:
        depth += (source[i] == "{") - (source[i] == "}")
        i += 1
    return source[match.start():i]


travel = function("serviceStorageTravelPreparation")
begin_travel = function("beginStorageTravelPreparation")
assert "startArmBase" not in travel + begin_travel
assert "setM5MotionProfile(" not in travel + begin_travel
assert "startArmBase" not in function("finishIntegratedWorkPause")
arrival = function("beginIntegratedWorkPause")
assert arrival.index("route_chassis::stopAllMotorsImmediately()") < arrival.index("beginWorkAction(")
entry = function("beginWorkAction")
assert "beginStorageColorSearch(false)" in entry
assert "beginArmEndpointPreparation(" in entry
assert "commandStorageServoPositionWithArrivalGate(" not in entry
unload = function("beginUnloadingTransfer")
assert "storageServoArrivalGate.slot != storage2ItemOrder[workItemIndex]" in unload
assert "storage2ScanIndex != 3U" in unload
assert "!serviceStorageServoArrivalGate()" in function("serviceArmTransfer")
assert "!LOCAL_WORKSTATION_SIMULATION_MODE" in unload
assert "commandStorageServoPositionWithArrivalGate(" not in function("releaseArmTransferDestination")
action = function("serviceCompetitionAction")
assert "storage2CoordinateWithinTargetRoi(x, y)" in action
assert "selectStorageColor(" not in action
assert "storage2ReturnTravelActive && secondStorageColorWorkflowActive()" in action
route = function("serviceIntegratedRoute")
assert route.index("route_chassis::PROGRAM_FINISHED") < route.index("finishProgram();")
assert "static_cast<float>(ARM_BASE_HOME_ANGLE_DEGREES)" in source

arm = function("serviceArmTransfer")
start = arm.index("        if (finalStorageDeparturePending) {")
opening = arm.index("{", start)
i, depth = opening + 1, 1
while depth:
    depth += (arm[i] == "{") - (arm[i] == "}")
    i += 1
final_release = arm[start:i]
assert "startArmBase" not in final_release
assert final_release.index("M7_STANDARD_HEIGHT_MM") < final_release.index("startExtensionToMmWithProfile(")

enum_match = re.search(r"enum StorageTravelPreparationPhase\s*\{[^}]+\};", source)
assert enum_match
harness = r'''
#include <stdint.h>
extern "C" void abort();
#define assert(c) ((c) ? (void)0 : abort())
float fabsf(float x){return x<0?-x:x;}
enum { PROGRAM_RUNNING, PROGRAM_FAULT, PROGRAM_FINISHED, ROUTE_MOTION_IDLE,
       ARM_TRANSFER_COMPLETE };
struct Debug {template<class T> void print(T,int=0){} template<class T> void println(T,int=0){} } SerialDebug;
struct Axis { float currentMm; } extensionAxis, liftAxis;
struct Motors {bool moving=false; float angle=-135;
 bool isM5Running(){return moving;} float m5CurrentAngleDegrees(){return angle;}
} armMotors;
bool armBaseMotionWatchdogActive=false, liftDone=true, extDone=true;
bool second=true, gateFails=false, finalReturnM5Commanded=false;
bool finalStorageDeparturePending=false, storage2ReturnTravelActive=false;
bool endpointInitialStorageCommanded=false, commandStarted=true;
bool activeRouteTurnCommand=true, SHOW_RESULT_PAGE_ON_FINISH=false;
int programState=PROGRAM_RUNNING, routeMotionPhase=0, armTransferPhase=0;
int m5Moves=0, trayMoves=0, m6Moves=0, finished=0, lastSlot=-1, mode=-1;
float targetM5=99, targetM6=99;
uint32_t tick=100, workStorageServoDeadlineMs=0, storageTravelM5ZeroHandoffDeadlineUs=0;
uint32_t storageTravelM5StartDeadlineMs=0, workLastMaixSequence=0, workVisionRequestStartMs=0;
uint8_t storageTravelPreparationRound=0, workItemIndex=0;
uint8_t storage2ItemOrder[3], storageColorExpected=0, storageColorTargetRing=0, storage2ScanIndex=0;
uint8_t taskColors[2][3],taskPositions[2][3];
int workActionPhase=0;
enum { WORK_PHASE_STORAGE_COLOR_WAIT_COORDINATE, MAIXCAM_RED_REQUEST=1,
       MAIXCAM_GREEN_REQUEST=4 };
const float M7_STANDARD_HEIGHT_MM=0, M6_STANDARD_EXTENSION_MM=0;
const float ARM_AXIS_POSITION_TOLERANCE_MM=1, ENDPOINT_SEARCH_SEED_EXTENSION_MM=90;
const float STORAGE_TRAVEL_M6_EXTENSION_MM=30, FINAL_RETURN_M5_TARGET_DEGREES=0;
const float FINAL_RETURN_M5_TOLERANCE_DEGREES=0.1f;
const uint32_t STORAGE2_M7_HEAD_START_MS=300, PROCESS1_M5_ZERO_HANDOFF_US=100;
const uint32_t PROCESS1_STORAGE_TRAVEL_SERVO_SETTLE_MS=500, STORAGE_SERVO_SETTLE_MS=500;
const int PROCESS_STORAGE_TRAVEL_M6_SPEED_RPM=162, PROCESS_STORAGE_TRAVEL_M6_ACCELERATION=42;
const int STORAGE_TRAVEL_M6_SPEED_RPM=810, STORAGE_TRAVEL_M6_ACCELERATION=208;
const int STORAGE_TRAVEL_M7_SPEED_RPM=2160, STORAGE_TRAVEL_M7_ACCELERATION=171;
const int PROCESS1_STORAGE_TRAVEL_SERVO_INTERVAL_MS=500;
uint32_t millis(){return tick;} uint32_t micros(){return tick*1000;}
bool deadlineReached(uint32_t t){return int32_t(tick-t)>=0;}
bool liftMoveFinished(){return liftDone;} bool extensionMoveFinished(){return extDone;}
void routeFault(const char*){programState=PROGRAM_FAULT;}
bool secondStorageColorWorkflowActive(){return second;}
void markMissionProgress(){}
bool startLiftToHeightMm(float h){liftAxis.currentMm=h;return true;}
bool startLiftToHeightMmWithProfile(float h,int,int){return startLiftToHeightMm(h);}
bool startExtensionToMmWithProfile(float e,int,int){++m6Moves;targetM6=e;extensionAxis.currentMm=e;return true;}
void commandStorageServoPosition(int slot,int){++trayMoves;lastSlot=slot;}
void commandStorageServoPositionWithArrivalGate(uint8_t slot){++trayMoves;lastSlot=slot;if(gateFails)routeFault("");}
void commandStorageServoParkingPosition(){++trayMoves;lastSlot=-1;}
struct Frame {uint32_t sequence=9;} frame;
struct Camera {Frame& latest(){return frame;}} maixCam;
void beginMaixRequest(uint8_t m){mode=m;}
void useArmBaseReturnMotionProfile(){}
void startArmBaseRotationToDegrees(float a){++m5Moves;targetM5=a;armMotors.moving=true;}
void hmiSetRunStatus(const char*){} void hmiCommand(const char*){} void hmiSetTaskCounts(){}
void resetQrScanActionState(){} void emergencyStopArmLinearAxes(){}
void cancelCompetitionAction(){storage2ReturnTravelActive=false;}
void disableDriveMotors(){} void disableArmBaseMotor(){++finished;} void commandGripperClose(){}
'''
harness += enum_match.group() + "\nStorageTravelPreparationPhase storageTravelPreparationPhase=STORAGE_TRAVEL_PREP_IDLE;\n"
for name in ("storageRingForItem", "storage2ExpectedColorAndSlotForRing",
             "beginStorageColorVision",
             "storageTravelPreparationReady", "beginStorageTravelPreparation",
             "serviceStorageTravelPreparation", "finishProgram"):
    harness += function(name) + "\n"
harness += "void releaseAfterClearance(){if(liftMoveFinished()){do {" + final_release + "}while(false);}}\n"
harness += r'''
void reset(){
 programState=PROGRAM_RUNNING;armMotors.moving=false;armMotors.angle=-135;
 armBaseMotionWatchdogActive=false;liftDone=extDone=true;gateFails=false;second=true;
 finalReturnM5Commanded=finalStorageDeparturePending=storage2ReturnTravelActive=false;
 extensionAxis.currentMm=liftAxis.currentMm=0;tick=100;m5Moves=m6Moves=trayMoves=finished=0;
 storageTravelPreparationPhase=STORAGE_TRAVEL_PREP_IDLE;workItemIndex=0;
 storageColorExpected=0;lastSlot=-1;mode=-1;
 for(int i=0;i<3;i++)storage2ItemOrder[i]=255;
}
int main(){
 // Both travel state machines finish M7/M6/tray preparation without M5.
 for(int round=1;round<=2;round++){
  reset();beginStorageTravelPreparation(round);
  for(int i=0;i<12;i++){serviceStorageTravelPreparation();tick+=500;}
  assert(programState==PROGRAM_RUNNING&&storageTravelPreparationReady(round));
  assert(m5Moves==0&&armMotors.angle==-135);
  assert(extensionAxis.currentMm==(round==1?90:30));
 }
 reset();armMotors.moving=true;beginStorageTravelPreparation(1);
 assert(programState==PROGRAM_FAULT&&m6Moves==0);
 // Exhaust all six second-round tray permutations against all six known
 // first-round ring color layouts. Vision is targeted from that known map.
 const uint8_t perms[6][3]={{1,3,4},{1,4,3},{3,1,4},{3,4,1},{4,1,3},{4,3,1}};
 for(int q=0;q<6;q++)for(int layout=0;layout<6;layout++){
  reset();for(int i=0;i<3;i++){
   taskColors[0][i]=perms[layout][i];taskPositions[0][i]=i+1;
   taskColors[1][i]=perms[q][i];
  }
  for(int i=0;i<3;i++){
   workItemIndex=i;storage2ScanIndex=i;storageColorTargetRing=i+1;
   uint8_t expected=0,slot=255;
   assert(storage2ExpectedColorAndSlotForRing(i+1,expected,slot));
   assert(expected==perms[layout][i]&&taskColors[1][slot]==expected);
   storageColorExpected=expected;storage2ItemOrder[i]=slot;
   beginStorageColorVision();assert(mode==expected&&trayMoves==0);
   assert(storageColorTargetRing==i+1);
   assert(m5Moves==0);
  }
  assert(storage2ItemOrder[0]!=storage2ItemOrder[1]&&storage2ItemOrder[1]!=storage2ItemOrder[2]&&storage2ItemOrder[0]!=storage2ItemOrder[2]);
 }
 reset();uint8_t badColor=0,badSlot=0;
 assert(!storage2ExpectedColorAndSlotForRing(0,badColor,badSlot));
 assert(!storage2ExpectedColorAndSlotForRing(4,badColor,badSlot));
 assert(storageRingForItem(1,3)==0);
 // Final release cannot start retraction/departure before M7 is physically clear.
 reset();finalStorageDeparturePending=true;liftDone=false;releaseAfterClearance();
 assert(!storage2ReturnTravelActive&&trayMoves==0&&m6Moves==0);
 liftDone=true;liftAxis.currentMm=-10;releaseAfterClearance();
 assert(programState==PROGRAM_FAULT&&trayMoves==0&&m6Moves==0);
 reset();finalStorageDeparturePending=true;releaseAfterClearance();
 assert(storage2ReturnTravelActive&&!finalStorageDeparturePending);
 assert(armTransferPhase==ARM_TRANSFER_COMPLETE&&m6Moves==1&&trayMoves==1&&m5Moves==0);
 // At route endpoint, M5 still waits for M6/M7, then returns to startup 0.
 extDone=false;finishProgram();assert(m5Moves==0&&finished==0);
 extDone=true;finishProgram();assert(m5Moves==1&&targetM5==0&&finished==0);
 finishProgram();assert(m5Moves==1&&finished==0);
 armMotors.moving=false;armMotors.angle=0;finishProgram();
 assert(programState==PROGRAM_FINISHED&&finished==1&&trayMoves==1);
 reset();extensionAxis.currentMm=10;finishProgram();
 assert(programState==PROGRAM_FAULT&&m5Moves==0);
}
'''
compiler = shutil.which("g++")
assert compiler, "Native g++ required"
with tempfile.TemporaryDirectory(prefix="storage_arrival_") as directory:
    cpp, exe = Path(directory) / "test.cpp", Path(directory) / "test.exe"
    cpp.write_text(harness, encoding="utf-8")
    result = subprocess.run([compiler, "-std=c++11", "-O0", "-fmax-errors=5", str(cpp), "-o", str(exe)], capture_output=True, text=True)
    if result.returncode:
        raise SystemExit(result.stderr[-6000:])
    subprocess.run([str(exe)], check=True)
print("PASS actual firmware: M5 held in both travel rounds; 36 QR-known color/slot permutations; targeted requests; 1/2/3 order; safe parallel return; endpoint-only M5 home")
