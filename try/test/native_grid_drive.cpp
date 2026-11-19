#include <GridNavigator.h>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <vector>
static uint32_t clockMs=100;
uint32_t millis(){return clockMs;}
constexpr int PC10=10,PC11=11;
namespace gongchuang {
struct Stm32HardwareSerial {
 int rx,tx,space=63;unsigned baud=0;std::deque<uint8_t> incoming;std::vector<uint8_t> sent;
 Stm32HardwareSerial(int r,int t):rx(r),tx(t){}
 void begin(unsigned b){baud=b;}int available(){return incoming.size();}
 int availableForWrite(){return space;}
 int read(){auto b=incoming.front();incoming.pop_front();return b;}
 size_t write(const uint8_t*b,size_t n){sent.insert(sent.end(),b,b+n);return n;}
};
}
#include "radar_navigation_config.h"
namespace route_chassis {
struct Debug {template<class T>void print(T,int=0){}template<class T>void println(T,int=0){}} SerialDebug;
enum{PROGRAM_RUNNING,PROGRAM_FAULT};int programState=PROGRAM_RUNNING;
uint32_t commandStartMs=0,imuValidFrameSequence=1,lastImuReceiveMs=100;
constexpr uint32_t MOTION_TIMEOUT_MS=30000;
constexpr float FORWARD_PULSES_PER_METER=10000;
float targetCounterClockwiseHeadingDegrees=0,yaw=0,turnCommand=0,turnSpeed=0;
float forwardCommand=0,rightCommand=0,rotationCommand=0,profileSpeed=0,profileAccel=0;
bool fresh=true,arrived=false,integratedTurnControlActive=false;
unsigned turns=0,turnPolls=0;
void routeFault(const char*){programState=PROGRAM_FAULT;arrived=true;integratedTurnControlActive=false;}
void stopAllMotorsImmediately(){arrived=true;integratedTurnControlActive=false;}
void startTurn(float d,float,float s){++turns;turnCommand=d;turnSpeed=s;targetCounterClockwiseHeadingDegrees+=d;integratedTurnControlActive=true;}
void setDriveMotionProfile(float s,float a){profileSpeed=s;profileAccel=a;}
void startVehicleDisplacement(float f,float r,float d){forwardCommand=f;rightCommand=r;rotationCommand=d;arrived=false;}
bool imuIsFresh(){return fresh;}float headingErrorDegrees(){return targetCounterClockwiseHeadingDegrees-yaw;}
bool allMotorsArrived(){return arrived;}
void serviceIntegratedTurnCommand(){++turnPolls;integratedTurnControlActive=false;}
}
#include "radar_navigation.inc"
namespace rn=radar_navigation;namespace rc=route_chassis;
int main(){
 rn::setup();assert(rn::serial.rx==11&&rn::serial.tx==10&&rn::serial.baud==115200);
 uint8_t byte=1;rn::serial.space=0;assert(!rn::write(nullptr,&byte,1)&&rn::serial.sent.empty());rn::serial.space=63;
 qiming::NavAction a;a.kind=qiming::NavAction::TURN;a.degrees=0;
 rn::startMotion(a);rc::arrived=true;assert(!rn::turning&&rc::turns==0);
 for(unsigned i=0;i<3;++i){clockMs+=20;++rc::imuValidFrameSequence;rc::lastImuReceiveMs=clockMs;
  assert(rn::serviceMotion()==(i==2));}
 a.degrees=360;
 rn::startMotion(a);assert(rc::turnCommand==360&&rc::targetCounterClockwiseHeadingDegrees==360&&rc::turnSpeed==.5f);
 assert(rn::serviceMotion()&&rc::turnPolls==1&&!rn::motionActive);
 // Zone1 CCW/restore and zone2 CW/restore retain their signs in the IMU path.
 for(float step:{60.f,60.f,-120.f,-60.f,-60.f,120.f}){
  a.degrees=step;rn::startMotion(a);
  assert(rc::turnCommand==step&&rc::targetCounterClockwiseHeadingDegrees==step&&rc::turnSpeed==.5f);
  assert(rn::serviceMotion()&&!rn::motionActive);
 }
 // Translation consumes mm directly, without legacy per-segment corrections.
 a.kind=qiming::NavAction::MOVE;a.forward=-240;a.right=240;
 rn::startMotion(a);assert(rc::forwardCommand==-240&&rc::rightCommand==240&&rc::rotationCommand==0);
 assert(rc::profileSpeed==2500&&rc::profileAccel==2000);
 assert(!rn::serviceMotion());rc::arrived=true;
 assert(!rn::serviceMotion()&&rn::turning&&rc::turnCommand==0);assert(rn::serviceMotion());
 // Edge terminal lock uses three fresh samples; no unsafe in-place rotation.
 a.forward=0;a.right=150;rn::startMotion(a);rc::arrived=true;unsigned previousTurns=rc::turns;
 for(unsigned i=0;i<3;++i){clockMs+=20;++rc::imuValidFrameSequence;rc::lastImuReceiveMs=clockMs;
  bool done=rn::serviceMotion();assert(done==(i==2));}
 assert(rc::turns==previousTurns);
 // IMU loss, excessive yaw and stalled motion all refuse completion.
 rn::startMotion(a);rc::fresh=false;assert(!rn::serviceMotion()&&rc::programState==rc::PROGRAM_FAULT);rc::fresh=true;
 rc::programState=rc::PROGRAM_RUNNING;rn::startMotion(a);rc::yaw=2;assert(!rn::serviceMotion()&&rc::programState==rc::PROGRAM_FAULT);rc::yaw=0;
 rc::programState=rc::PROGRAM_RUNNING;rn::startMotion(a);clockMs+=rc::MOTION_TIMEOUT_MS;assert(!rn::serviceMotion()&&rc::programState==rc::PROGRAM_FAULT);
 rn::begin(1);assert(rn::active);rn::stop();assert(!rn::active&&!rn::motionActive);
 puts("Grid UART wiring, nonblocking TX, mirrored +/-60/120 and accumulated 360 turns, terminal IMU locks and motion faults PASS");
 return 0;
}
