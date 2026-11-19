"""Execute actual survey and transfer branches; fake hardware, no serial IO."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
config = (ROOT / "lib/RobotConfig/src/RobotConfig.h").read_text(encoding="utf-8")


def function(name):
    m = re.search(r"^\w[\w :*&<>]*\b" + name + r"\s*\([^;{}]*\)\s*\{", source, re.M)
    assert m, name
    i, depth = m.end(), 1
    while depth:
        depth += (source[i] == "{") - (source[i] == "}")
        i += 1
    return source[m.start():i]


def phase(body, first, after):
    return body[body.index("    case " + first + ":"):body.index("    case " + after + ":")]


unload = function("beginUnloadingTransfer")
assert "storage2ScanIndex != 3U" in unload
assert "storageServoArrivalGate.slot != storage2ItemOrder[workItemIndex]" in unload
assert "nextStorageSlot = storage2ItemOrder[workItemIndex + 1U]" in unload
assert "const bool prepareNextContainerPickup = workItemIndex < 2U;" in unload
assert "!serviceStorageServoArrivalGate()" not in unload
prepared = function("beginArmTransfer").split("if (armTransferSourceAlreadyPrepared) {")[1]
assert prepared.index("ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE") < prepared.index("startArmTransferLiftToSourceHeightMm(")
assert "!armTransferNextStorageCommanded" in function("releaseArmTransferDestination")
complete = function("completeTransferAndRotateStorage")
assert "beginStorageColorSearch(" not in complete
assert "? WORK_PHASE_START_UNLOAD : WORK_PHASE_WAIT_STORAGE_SERVO" in complete
assert "storage2ReturnTravelActive && secondStorageColorWorkflowActive()" in complete
assert "armTransferPrepareStorage2NextVision" not in source

harness = r'''
#include <stdint.h>
extern "C" void abort();
#define assert(c) ((c) ? (void)0 : abort())
float fabsf(float v){return v<0?-v:v;}
enum { PROGRAM_RUNNING, PROGRAM_FAULT, WORK_PHASE_STORAGE_COLOR_WAIT_EXTENSION,
 WORK_PHASE_STORAGE_COLOR_WAIT_COORDINATE, WORK_PHASE_START_UNLOAD,
 ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE, ARM_TRANSFER_WAIT_SOURCE_LOWER,
 ARM_TRANSFER_WAIT_LOADED_CLEARANCE, ARM_TRANSFER_WAIT_DESTINATION_ROTATION };
struct ArmPose {float standardFrameAngleDegrees,extensionMm,heightMm;
 ArmPose(float a=0,float e=0,float h=0):standardFrameAngleDegrees(a),extensionMm(e),heightMm(h){} };
struct Axis {float currentMm=0;} extensionAxis,liftAxis;
struct Motors {bool moving=false;float angle=0;bool isM5Running(){return moving;}} armMotors;
struct Debug {template<class T>void print(T,int=0){} template<class T>void println(T,int=0){} } SerialDebug;
struct Frame{uint32_t sequence=0;uint8_t confidence=99;};
struct Camera{Frame f;Frame& latest(){return f;}} maixCam;
int programState=PROGRAM_RUNNING,workActionPhase=0,armTransferPhase=0;
 bool armBaseMotionWatchdogActive=false,armBaseMotionLockedUntilReset=false,extensionDone=true,liftDone=true,chassisDone=true;
bool frameReady=false,geometryValid=true,extensionAccept=true,planarAccept=true,gateReady=false;
bool storage2ScanExtensionCaptured=false;
uint8_t storage2ScanIndex=0,storageColorTargetRing=0,storageColorExpected=0,workItemIndex=0;
uint8_t storage2ItemOrder[3],taskColors[2][3],taskPositions[2][3],storageColorAccuracySampleCount=0,incomingColor=0;
int16_t incomingX=0,incomingY=0;
uint32_t tick=100,workVisionRequestStartMs=0,workLastMaixSequence=0;
uint32_t storageColorAccuracyWindowStartMs=0,workStorageServoDeadlineMs=0;
int32_t storageColorAccuracySumX=0,storageColorAccuracySumY=0;
float storage2ScanExtensionMm=0,targetM5=0,targetM6=0;
ArmPose storageColorScanPose,measuredRingPoses[4];
bool measuredRingPoseValid[4],endpointInitialStorageCommanded=false;
uint8_t measuredRingConfidence[4];float measuredRingHeadingsDegrees[4];
int m5Moves=0,m6Moves=0,trayMoves=0,lastSlot=-1,requestMode=0,requestCount=0,lowerMoves=0,gateQueries=0,fastProfiles=0;
int commandOrder=0,m6StartOrder=0,m5StartOrder=0;
bool armTransferStorage2Batch=true,armTransferPrepareNextSource=true,armTransferNextStorageCommanded=false;
bool armTransferDestinationM5FinalBufferPending=false,armTransferMapDestination=true,armTransferMapSource=false;
bool armTransferDestinationRotationStarted=false;
uint8_t armTransferRing1TrayClearanceMode=0;
int8_t armTransferNextStorageSlot=2;
uint32_t armTransferDeadlineMs=0,armTransferNextStorageDeadlineMs=0;
ArmPose armTransferDestinationPose,armTransferSourcePose;
constexpr float M7_STANDARD_HEIGHT_MM=0,M6_STANDARD_EXTENSION_MM=0,M6_MAXIMUM_EXTENSION_MM=140;
constexpr float ARM_AXIS_POSITION_TOLERANCE_MM=1,PROCESS_FIRST_RING_M5_FINAL_BUFFER_DEGREES=1;
constexpr float STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES=0.0f;
constexpr float STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES=0.0f;
constexpr float STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM=0.0f;
constexpr uint32_t STORAGE_SERVO_SETTLE_MS=500;
constexpr int16_t IMAGE_CENTER_X=160,IMAGE_CENTER_Y=120;
constexpr int STORAGE_TRAVEL_M6_SPEED_RPM=810,STORAGE_TRAVEL_M6_ACCELERATION=208;
constexpr uint8_t MAIXCAM_RED_REQUEST=1,MAIXCAM_GREEN_REQUEST=4,MAIXCAM_ALL_COLORS_REQUEST=8;
uint32_t millis(){return tick;}bool deadlineReached(uint32_t d){return int32_t(tick-d)>=0;}
bool extensionMoveFinished(){return extensionDone;}bool liftMoveFinished(){return liftDone;}
bool allMotorsArrived(){return chassisDone;}bool secondStorageColorWorkflowActive(){return true;}
void routeFault(const char*){programState=PROGRAM_FAULT;}
void stopMaixRequest(){}void markMissionProgress(){}
void beginMaixRequest(uint8_t mode){requestMode=mode;++requestCount;}
void useArmBaseTransferMotionProfile(){++fastProfiles;}
void startArmBaseStandardFrameDegrees(float a){targetM5=a;++m5Moves;m5StartOrder=++commandOrder;armMotors.moving=true;}
float currentArmBaseStandardFrameDegrees(){return armMotors.angle;}
float currentRouteCounterClockwiseHeading(){return 0;}
bool startExtensionToMmWithProfile(float e,int,int){targetM6=e;++m6Moves;m6StartOrder=++commandOrder;extensionDone=false;return extensionAccept;}
void commandStorageServoPositionWithArrivalGate(uint8_t slot){++trayMoves;lastSlot=slot;}
bool readNewMaixCoordinate(uint32_t&,uint8_t& c,int16_t& x,int16_t& y){
 if(!frameReady)return false;frameReady=false;c=incomingColor;x=incomingX;y=incomingY;return true;
}
bool storageColorTargetPose(const ArmPose& p,uint8_t,int16_t x,int16_t y,ArmPose& target){
 target=ArmPose(p.standardFrameAngleDegrees,x,y);return geometryValid;
}
 bool ringMapHeadingStillValid(){return true;}bool armTransferUsesRawProfile(){return false;}bool armTransferUsesReturnProfile(){return false;}
bool serviceStorageServoArrivalGate(){++gateQueries;return gateReady;}
void commandStorageServoParkingPosition(){assert(false);}
void commandRing1TrayClearancePosition(){assert(false);}
void restoreArmTransferRing1Tray(){assert(false);}
bool startArmTransferLiftToSourceHeightMm(float){++lowerMoves;return true;}
bool startArmTransferPlanarMove(const ArmPose&,bool,bool,bool,bool& started,const char*,bool){
 if(planarAccept){armMotors.moving=true;extensionDone=false;started=true;}return planarAccept;
}
'''
harness += "namespace arm_config {\n"
for name in ("STORAGE2_SCAN_USER_ANGLES_DEGREES", "STORAGE2_SCAN_EXTENSION_ADVANCE_MM", "STORAGE2_SCAN_DWELL_MS", "STORAGE2_TARGET_COLOR_ROI_RADIUS_PIXELS"):
    m = re.search(r"constexpr [^;\n]*\b" + name + r"[^;]*;", config)
    assert m, name
    harness += m.group() + "\n"
harness += "}\n"
for name in ("storageColorSearchAngleForRing", "storageColorSearchExtensionForRing", "storage2ExpectedColorAndSlotForRing", "storage2CoordinateWithinTargetRoi", "beginStorageColorVision", "beginStorageColorSearch", "advanceStorage2Scan"):
    harness += function(name) + "\n"
harness += "void survey(){const uint32_t nowMs=millis();switch(workActionPhase){\n"
harness += phase(function("serviceCompetitionAction"), "WORK_PHASE_STORAGE_COLOR_WAIT_EXTENSION", "WORK_PHASE_CIRCLE_WAIT_ARM_LOWER") + "}}\n"
harness += "void transferStep(){switch(armTransferPhase){\n"
for first, after in (("ARM_TRANSFER_WAIT_LOADED_CLEARANCE", "ARM_TRANSFER_WAIT_DESTINATION_ROTATION"), ("ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE", "ARM_TRANSFER_WAIT_SOURCE_LOWER")):
    harness += phase(function("serviceArmTransfer"), first, after)
harness += "}}\n"
harness += r'''
void reset(){
 programState=PROGRAM_RUNNING;storage2ScanIndex=workItemIndex=storageColorExpected=0;
 storage2ScanExtensionCaptured=false;requestCount=0;
 extensionDone=liftDone=chassisDone=geometryValid=extensionAccept=planarAccept=true;
 armMotors.moving=armBaseMotionWatchdogActive=frameReady=gateReady=false;
 armTransferStorage2Batch=armTransferPrepareNextSource=true;armTransferNextStorageCommanded=false;
 armTransferDestinationM5FinalBufferPending=false;armTransferMapSource=false;
 armTransferRing1TrayClearanceMode=0;
 extensionAxis.currentMm=30;liftAxis.currentMm=0;armMotors.angle=-102;
 tick=100;m5Moves=m6Moves=trayMoves=lowerMoves=gateQueries=fastProfiles=0;lastSlot=-1;
  commandOrder=m6StartOrder=m5StartOrder=0;armBaseMotionLockedUntilReset=false;
 for(int i=0;i<3;i++)storage2ItemOrder[i]=255;
 taskColors[0][0]=1;taskColors[0][1]=3;taskColors[0][2]=4;
 taskColors[1][0]=1;taskColors[1][1]=3;taskColors[1][2]=4;
 for(int i=0;i<3;i++)taskPositions[0][i]=i+1;
 for(int i=0;i<4;i++)measuredRingPoseValid[i]=false;
}
void arrive(){armMotors.moving=false;armMotors.angle=targetM5;extensionDone=true;extensionAxis.currentMm=targetM6;survey();}
void frame(uint8_t c,int16_t x,int16_t y){incomingColor=c;incomingX=x;incomingY=y;frameReady=true;survey();}
int main(){
 assert(storage2CoordinateWithinTargetRoi(160,120));
 assert(storage2CoordinateWithinTargetRoi(260,120));
 assert(!storage2CoordinateWithinTargetRoi(261,120));
 const uint8_t p[6][3]={{1,3,4},{1,4,3},{3,1,4},{3,4,1},{4,1,3},{4,3,1}};
 for(int q=0;q<6;q++)for(int layout=0;layout<6;layout++){
  reset();
  for(int i=0;i<3;i++){
   taskColors[0][i]=p[layout][i];
   taskPositions[0][i]=i+1;
   taskColors[1][i]=p[q][i];
  }
  beginStorageColorSearch(false);
  for(int i=0;i<3;i++){
   int ring=3-i;assert(storageColorTargetRing==ring);
   uint8_t expected=p[layout][ring-1],expectedSlot=255;
   for(int slot=0;slot<3;slot++)if(p[q][slot]==expected)expectedSlot=slot;
   assert(expectedSlot<3&&storageColorExpected==expected);
   assert(storage2ItemOrder[ring-1]==expectedSlot);
   assert(targetM5==-60+60*i&&targetM6==40&&m5Moves==i+1&&m6Moves==i+1);
   assert(m6StartOrder>0&&m6StartOrder<m5StartOrder);
   assert(armMotors.moving&&!extensionDone&&trayMoves==0);
   survey();assert(workActionPhase==WORK_PHASE_STORAGE_COLOR_WAIT_EXTENSION);
   arrive();assert(requestMode==expected);uint32_t start=tick;
   tick=start+200;survey();assert(requestMode==expected);int pending=requestCount;
   tick=start+300;survey();assert(requestCount==pending);
   uint8_t wrong=expected==1?3:1;
   tick=start+325;frame(wrong,160,120); // another visible ring is ignored
   assert(storageColorAccuracySampleCount==0&&requestMode==expected);
   tick=start+350;frame(expected,280,120); // target color outside the center ROI is ignored
   assert(storageColorAccuracySampleCount==0&&requestMode==expected);
   tick=start+400;frame(expected,160+ring,120+ring);
   tick=start+450;frame(expected,162+ring,122+ring);
   tick=start+499;survey();assert(storage2ScanIndex==i&&trayMoves==0);
   tick=start+500;survey();assert(storage2ScanIndex==i+1&&programState==PROGRAM_RUNNING);
   assert(measuredRingPoseValid[ring]&&measuredRingPoses[ring].extensionMm==161+ring);
   assert(measuredRingPoses[ring].heightMm==121+ring);
   assert(taskColors[1][storage2ItemOrder[ring-1]]==expected);
  }
  assert(trayMoves==1&&lastSlot==storage2ItemOrder[0]);
  assert(workActionPhase==WORK_PHASE_START_UNLOAD&&workItemIndex==0&&fastProfiles==3);
 }
 // No frame advances at 500ms and revisits only the missing pose. The known
 // QR color and tray slot stay fixed; a miss never changes ownership.
 reset();beginStorageColorSearch(false);
 for(int i=0;i<6;i++){
  int ring=3-(i%3),expected=taskColors[0][ring-1];
  arrive();assert(requestMode==expected);int requests=requestCount;uint32_t start=tick;
  tick=start+200;survey();assert(requestCount==requests);
  tick=start+400;survey();assert(requestCount==requests);
  tick=start+500;survey();
  assert(programState==PROGRAM_RUNNING&&trayMoves==0);
  assert(storage2ScanIndex==(i+1)%3&&targetM6==40);
  assert(targetM5==-60+60*((i+1)%3));
 }
 // A target seen only outside the ROI remains missing. Valid inner samples for
 // rings 2/1 are retained, so the next lap skips them and revisits ring 3.
 reset();beginStorageColorSearch(false);arrive();uint32_t start=tick;
 tick=start+350;frame(4,280,120);tick=start+500;survey();
 assert(storage2ScanIndex==1&&storage2ItemOrder[2]==2&&!measuredRingPoseValid[3]);
 arrive();assert(requestMode==3);start=tick;
 tick=start+400;frame(3,160,120);tick=start+500;survey();
 assert(storage2ScanIndex==2&&measuredRingPoseValid[2]);
 arrive();assert(requestMode==1);start=tick;
 tick=start+400;frame(1,160,120);tick=start+500;survey();
 assert(storage2ScanIndex==0&&measuredRingPoseValid[1]&&!measuredRingPoseValid[3]);
 arrive();assert(requestMode==4&&targetM6==40);start=tick;
 tick=start+400;frame(4,160,120);tick=start+500;survey();
 assert(storage2ScanIndex==3&&trayMoves==1&&lastSlot==0&&programState==PROGRAM_RUNNING);
 // A malformed QR plan cannot assign one tray slot to two physical rings.
 reset();taskColors[0][0]=1;taskColors[0][1]=1;taskColors[0][2]=4;
 storage2ScanIndex=2;storage2ItemOrder[1]=0;beginStorageColorSearch(false);
 assert(programState==PROGRAM_FAULT&&m5Moves==0&&m6Moves==0);
 // A missing first-round ring position is rejected before any arm motion.
 reset();taskPositions[0][0]=2;taskPositions[0][1]=2;taskPositions[0][2]=3;
 storage2ScanIndex=2;beginStorageColorSearch(false);
 assert(programState==PROGRAM_FAULT&&m5Moves==0&&m6Moves==0);
 reset();liftAxis.currentMm=-10;beginStorageColorSearch(false);
 assert(programState==PROGRAM_FAULT&&m5Moves==0&&m6Moves==0);
 reset();extensionAxis.currentMm=135;beginStorageColorSearch(false);
 assert(programState==PROGRAM_FAULT&&m5Moves==0);
  reset();extensionAccept=false;beginStorageColorSearch(false);
  assert(programState==PROGRAM_FAULT&&m5Moves==0);
 reset();armTransferPhase=ARM_TRANSFER_WAIT_LOADED_CLEARANCE;liftDone=false;
 transferStep();assert(trayMoves==0);
 liftDone=true;planarAccept=false;transferStep();assert(trayMoves==0);
 planarAccept=true;transferStep();assert(trayMoves==1&&lastSlot==2&&armMotors.moving);
 assert(armTransferNextStorageCommanded&&armTransferPhase==ARM_TRANSFER_WAIT_DESTINATION_ROTATION);
 reset();armTransferPhase=ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE;
 armTransferDeadlineMs=tick;armMotors.moving=true;transferStep();assert(gateQueries==0&&lowerMoves==0);
 armMotors.moving=false;extensionDone=false;transferStep();assert(gateQueries==0);
 extensionDone=true;transferStep();assert(gateQueries==1&&lowerMoves==0);
 gateReady=true;transferStep();assert(lowerMoves==1&&armTransferPhase==ARM_TRANSFER_WAIT_SOURCE_LOWER);
}
'''
compiler = shutil.which("g++")
assert compiler, "Native g++ required"
with tempfile.TemporaryDirectory(prefix="s2_survey_") as directory:
    cpp, exe = Path(directory) / "test.cpp", Path(directory) / "test.exe"
    cpp.write_text(harness, encoding="utf-8")
    p = subprocess.run([compiler, "-std=c++11", "-O0", "-fmax-errors=5", str(cpp), "-o", str(exe)], capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-6000:]
    subprocess.run([str(exe)], check=True)
print("PASS firmware: 36 QR-known color/slot permutations, targeted requests, center ROI, 500ms miss polling, early tray and late pickup gate")
