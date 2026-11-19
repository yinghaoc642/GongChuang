#!/usr/bin/env python3
"""Exercise production coarse switch with event stubs and production geometry.
Hardware events are simulated, not STM32/driver/camera emulation.
"""
from pathlib import Path
import subprocess, tempfile, sys, types, ast, hashlib, json
ROOT=Path(__file__).resolve().parents[1]
s=(ROOT/'src/main.cpp').read_text()
start=s.index('    case WORK_PHASE_COARSE_PREPARE:',s.index('void serviceCompetitionAction()'))
end=s.index('    case WORK_PHASE_ENDPOINT_REACH_DELAY:',start)
cases=s[start:end]
harness=r'''
#include "workstation_coarse.h"
#include <cassert>
#include <cmath>
#include <vector>
#include <cstdint>
using namespace workstation_coarse;
State coarseState;
enum {WORK_PHASE_COARSE_PREPARE,WORK_PHASE_COARSE_SETTLE,WORK_PHASE_COARSE_VISION,WORK_PHASE_COARSE_MOVE,WORK_PHASE_COARSE_RETRACT,WORK_PHASE_PREPARE};
int workActionPhase,programState=1,PROGRAM_RUNNING=1,armStandardPhase,ARM_STANDARD_IDLE=0;
unsigned clockMs=10;unsigned millis(){return clockMs;}
bool stdReady=true,headingReady=true,LOCAL_WORKSTATION_SIMULATION_MODE=false;
bool rawTravelM5ZeroPending=true;unsigned processM5ZeroSettleDeadlineUs=1;
unsigned commandStartMs=0,headingStableStartMs=0,motorsArrivedStartMs=0,workLastMaixSequence=0,workVisionRequestStartMs=0;
unsigned visualCorrectionMoveCount=0,MAXIMUM_VISUAL_CORRECTION_MOVES=6;
float visualCorrectionForwardMm=0,visualCorrectionLeftMm=0,MAXIMUM_ACCUMULATED_VISUAL_CORRECTION_MM=220,M6_STANDARD_EXTENSION_MM=0;
int MOTION_TIMEOUT_MS=5000,VISION_POST_MOTION_SETTLE_TIME_MS=10,VISION_HEADING_STABLE_TIME_MS=10;
int moves=0,requests=0,fine=0,retracts=0;bool allowMove=true;
struct Debug{template<class T>void print(T){}template<class T>void println(T){}} SerialDebug;
struct Frame{unsigned metric=40,confidence=800,sequence=0;};
struct {Frame frame;const Frame&latest(){return frame;}} maixCam;
namespace gongchuang{struct MaixCamClient{enum{COARSE_RING2_REQUEST=11};};}
struct Event{bool present;int x,y;unsigned radius,confidence;uint8_t target;};
std::vector<Event> events;unsigned cursor=0;
bool readNewMaixCoordinate(unsigned&,uint8_t &target,int16_t&x,int16_t&y){
 if(cursor>=events.size()||!events[cursor].present)return false;
 auto e=events[cursor++];target=e.target;x=e.x;y=e.y;maixCam.frame.metric=e.radius;maixCam.frame.confidence=e.confidence;return true;
}
bool serviceArmStandardization(){return stdReady;}
void stopAllMotorsImmediately(){}
void stopMaixRequest(){}
bool updateHeadingLock(int,int,int){return headingReady;}
void beginMaixRequest(int mode){assert(mode==11);requests++;}
void beginArmStandardization(float angle,bool keep,float extension){assert(angle==0&&!keep&&extension==0);retracts++;}
void beginArmEndpointPreparation(bool pre){assert(!pre);fine++;}
bool startAccumulatedWorkstationMove(float f,float l,const char*){
 if(!allowMove){programState=0;return false;}
 assert(hypotf(f,l)<=60.001);moves++;visualCorrectionForwardMm+=f;visualCorrectionLeftMm+=l;visualCorrectionMoveCount++;return true;
}
void step(){if(programState!=PROGRAM_RUNNING)return;switch(workActionPhase){
'''+cases+r'''
 default:break;}}
void reset(std::vector<Event> e){
 coarseState.reset(true);events=e;cursor=0;clockMs=10;workActionPhase=WORK_PHASE_COARSE_PREPARE;
 moves=requests=fine=retracts=0;visualCorrectionMoveCount=0;visualCorrectionForwardMm=visualCorrectionLeftMm=0;
 programState=1;allowMove=stdReady=headingReady=true;LOCAL_WORKSTATION_SIMULATION_MODE=false;
}
void finish(){for(int i=0;i<40&&fine==0&&programState==1;i++){step();clockMs+=2600;}assert(fine==1&&retracts==1&&coarseState.rounds==2&&requests==2);}
int main(){
 reset({});finish();assert(moves==0); // two timeouts still fine scan
 reset({{true,160,120,40,900,2},{true,160,120,40,900,2}});finish();assert(moves==0);
 reset({{true,180,140,40,900,2},{true,160,120,40,900,2}});finish();assert(moves==1);
 reset({{true,280,120,40,900,2},{true,280,120,40,900,2}});finish();assert(moves==2&&coarseState.travelled<=100.001); // residual >15 still continue
 reset({{true,180,140,1,900,2},{true,180,140,40,100,2}});finish();assert(moves==0);
 reset({{true,180,140,40,900,1},{true,180,140,40,900,1}});finish();assert(moves==0);
 reset({{true,180,140,40,900,2},{true,180,140,40,900,2}});LOCAL_WORKSTATION_SIMULATION_MODE=true;finish();assert(moves==0);
 reset({{true,180,140,40,900,2},{true,180,140,40,900,2}});visualCorrectionMoveCount=6;finish();assert(moves==0);
 reset({});stdReady=false;for(int i=0;i<8;i++)step();assert(requests==0);stdReady=true;step();headingReady=false;for(int i=0;i<8;i++)step();assert(requests==0);headingReady=true;finish();
 reset({{true,180,140,40,900,2}});allowMove=false;for(int i=0;i<8;i++)step();assert(programState==0&&fine==0); // driver/safety rejection remains fault
 // Independent known pixel scale and body direction checks.
 State st;float f,l,a,e;st.reset(true);assert(st.observe(180,140,40,900,f,l));assert(fabs(f+20*41.75/40)<.001&&fabs(l-20*41.75/40)<.001);
 st.moved(f,l);assert(fabs(st.outward-TARGET_RADIAL_MM)<.001&&fabs(st.along)<.001);
 st.seed(1,a,e);assert(a>38&&a<39&&e>105&&e<107);st.seed(3,a,e);assert(a<-38&&a>-39);
 // Exhaustive residuals for +/-60mm independent physical coordinate oracle.
 int scenarios=0;
 for(int df=-60;df<=60;df+=3)for(int dl=-60;dl<=60;dl+=3)for(int rad: {20,40,70}){
   st.reset(true);float rf=df,rl=dl;
   for(int n=0;n<2;n++){
     const int x=lroundf(160-rf*rad/41.75f),y=lroundf(120+rl*rad/41.75f);
     if(st.observe(x,y,rad,900,f,l)){st.moved(f,l);rf-=f;rl-=l;}
   }
   assert(hypotf(rf,rl)<=16.6f);assert(st.travelled<=100.01f);
   for(auto ring: {1,3}){st.seed(ring,a,e);assert(isfinite(a)&&isfinite(e)&&fabs(a)<=75&&e>=0&&e<=160);}
   scenarios++;
 }
 assert(scenarios==5043);
}
'''
with tempfile.TemporaryDirectory(prefix='try_coarse_') as tmp:
 p=Path(tmp)/'test.cpp';p.write_text(harness);exe=Path(tmp)/'test'
 subprocess.run(['clang++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'src'),str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
# Confirm coarse eligibility and excluded branch in the actual entry function.
assert 'coarseState.reset(kind == WORK_ACTION_PROCESS ||\n                    (kind == WORK_ACTION_STORAGE && roundNumber == 1U));' in s
contract=json.loads((ROOT/'test/fixtures/coarse_original_contract.json').read_text())
block='  if (kind == WORK_ACTION_STORAGE &&\n      workRoundIndex == 1U) {'
def storage2(src):
 a=src.index(block,src.index('void beginWorkAction(')); b=src.index('\n  beginArmEndpointPreparation(',a);return src[a:b]
assert hashlib.sha256(storage2(s).encode()).hexdigest()==contract['storage2_entry_sha256'],'storage2 entry changed'
# Original detection functions remain byte-for-byte identical for modes1..10.
def functions(src):
 t=ast.parse(src);return {n.name:ast.get_source_segment(src,n) for n in t.body if isinstance(n,ast.FunctionDef)}
new=functions((ROOT/'vision/yanyanview5.py').read_text())
for name,digest in contract['vision_function_sha256'].items():
 assert hashlib.sha256(new[name].encode()).hexdigest()==digest,name
sys.path.insert(0,str(ROOT/'vision'))
from ring2_coarse import triple_candidate,box_candidate,Ring2Detector
class Circle:
 def __init__(self,x,y,r):self.v=(x,y,r)
 def x(self):return self.v[0]
 def y(self):return self.v[1]
 def r(self):return self.v[2]
assert triple_candidate([Circle(40,120,30),Circle(148,120,30),Circle(256,120,30)])==(148,120,30,800)
assert triple_candidate([Circle(160,120,30)]) is None
assert triple_candidate([Circle(40,120,30),Circle(148,160,30),Circle(256,120,30)]) is None
assert triple_candidate([Circle(40,120,30),Circle(148,120,10),Circle(256,120,30)]) is None
obj=lambda **kw:types.SimpleNamespace(**kw)
box=obj(class_id=0,score=.9,x=100,y=60,w=80,h=80)
assert box_candidate([box],['2r'])==(.9,140,100,40)
assert box_candidate([box],['other']) is None
assert box_candidate([box,box],['2r']) is None
# Model APIs are stubbed, not NN inference.
d=Ring2Detector();d.model=obj(labels=['2r'],detect=lambda *a,**kw:[box])
img=obj(find_circles=lambda **kw:[Circle(140,100,40)])
assert d.detect(img) is None
assert d.detect(img)==(140,100,40,900)
d.reset();assert d.detect(img) is None
print('PASS: actual coarse states, 5043 geometric scenarios, two timeout/invalid/tolerance/residual rounds, driver fault and local no-wheel guard, storage2 exclusion, legacy vision functions, YOLO identity/stability.')
