"""Run actual HMI/button selection and departure pulse hook with the mode FSM."""
from pathlib import Path
import re
import subprocess
import tempfile
import hashlib
root=Path(__file__).resolve().parents[1]
route=(root/'src/route_chassis_main.inc').read_text()
def function(name,source=route):
    m=re.search(r'^\w[\w :*&<>]*\b'+name+r'\s*\([^;{}]*\)\s*\{',source,re.M)
    assert m,name
    i,depth=m.end(),1
    while depth:
        depth+=(source[i]=='{')-(source[i]=='}');i+=1
    return source[m.start():i]
# The actual 19-command calibrated table is unchanged from the saved baseline.
baseline=subprocess.run(['git','show','e1a285cef19e081a0f6cece95cf8ff5ab72cd12f:src/route_chassis_main.inc'],cwd=root,capture_output=True,text=True,check=True).stdout
def route_table(text):return re.search(r'const RouteCommand route\[\] = \{.*?\n\};',text,re.S).group()
assert route_table(route)==route_table(baseline)
harness=r'''
#include <cassert>
#include <deque>
#include <string>
#include "radar_navigation_config.h"
struct Port{std::deque<int> rx;
 int available(){return rx.size();}int read(){int b=rx.front();rx.pop_front();return b;}
 template<class T>void print(T){}template<class T>void println(T){}
}SerialHmi,SerialDebug;
enum{PROGRAM_WAITING,PROGRAM_RUNNING};int programState=PROGRAM_WAITING;
bool startRequested=false,abortRequested=false;uint8_t selectedStartZoneNumber=1;
std::string status;void hmiSetRunStatus(const char*s){status=s;}
struct Motor{int commands=0;void move(long){++commands;}}motor1,motor2,motor3,motor4;
struct MotorPulses{long motor1=0,motor2=0,motor3=0,motor4=0;};
'''
for name in ('receiveHmiStartCommand','onNavigationModeDoubleClick','startRelativeMotorMove'):
    harness+=function(name)+'\n'
harness+=r'''
int parking=0;void park(){++parking;}
int main(){
 using M=qiming::NavigationMode;
 assert(navigation::mode.selected()==M::RADAR);
 onNavigationModeDoubleClick();assert(navigation::mode.selected()==M::LEGACY&&status=="LEGACY");
 onNavigationModeDoubleClick();assert(navigation::mode.selected()==M::RADAR&&status=="RADAR");
 SerialHmi.rx={2,4};receiveHmiStartCommand();
 assert(selectedStartZoneNumber==2&&navigation::mode.selected()==M::LEGACY);
 startRequested=true;SerialHmi.rx={1,3};receiveHmiStartCommand();onNavigationModeDoubleClick();
 assert(selectedStartZoneNumber==2&&navigation::mode.selected()==M::LEGACY);
 startRequested=false;SerialHmi.rx={3};receiveHmiStartCommand();assert(navigation::mode.selected()==M::RADAR);
 for(uint8_t zone=1;zone<=2;++zone){
  M legacy;assert(legacy.select(M::LEGACY)&&legacy.start(zone));
  assert(legacy.state()==M::FIXED_LEGACY&&!legacy.radarRun()&&legacy.zone()==zone);
  assert(!legacy.select(M::RADAR)&&!legacy.mapAccepted(false));
  M clear;assert(clear.start(zone)&&clear.state()==M::SCANNING&&clear.radarRun());
  assert(!clear.select(M::LEGACY)&&clear.mapAccepted(false)&&clear.state()==M::FIXED_CLEAR);
  M grid;assert(grid.start(zone)&&grid.mapAccepted(true)&&grid.state()==M::GRID);
  grid.fail();assert(grid.state()==M::FAULT&&!grid.mapAccepted(false)&&!grid.select(M::LEGACY));
 }
 assert(!navigation::mode.start(3));assert(navigation::mode.start(2));
 programState=PROGRAM_RUNNING;SerialHmi.rx={1,4};receiveHmiStartCommand();onNavigationModeDoubleClick();
 assert(selectedStartZoneNumber==2&&navigation::mode.selected()==M::RADAR);
 navigation::onDeparture=park;navigation::departureTrayPending=true;
 MotorPulses p;startRelativeMotorMove(p);assert(parking==0&&navigation::departureTrayPending);
 p.motor1=10;startRelativeMotorMove(p);assert(parking==1&&!navigation::departureTrayPending);
 startRelativeMotorMove(p);assert(parking==1);
 navigation::mode.finish();assert(navigation::mode.state()==M::FINISHED);
}
'''
with tempfile.TemporaryDirectory(prefix='gc_mode_') as tmp:
    cpp=Path(tmp)/'mode.cpp';cpp.write_text(harness);exe=Path(tmp)/'mode'
    subprocess.run(['clang++','-std=c++17','-Wall','-Wextra','-Werror','-DGONGCHUANG_GRID_NAVIGATION=1','-I',str(root/'src'),'-I',str(root/'lib/QimingGrid/src'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS runtime modes: default radar; actual PB9 double-click/HMI3/4; zone1/2; lock after start; scan failure cannot become clear; original Nov02 table; first nonzero departure pulse parks once')

# Execute the production dispatcher: staging motion must retain SCANNING,
# and a clear handoff after motion must still enter FIXED_CLEAR.
main_source=(root/'src/main.cpp').read_text()
dispatch=r"""
#include <cassert>
#include <GridNavigator.h>
#include "radar_navigation_config.h"
struct Debug{template<class T>void print(T){}template<class T>void println(T){}}SerialDebug;
namespace route_chassis {enum{PROGRAM_RUNNING,PROGRAM_FAULT,PROGRAM_FINISHED};int programState=PROGRAM_RUNNING;}
namespace radar_navigation {
struct Mission {
 qiming::NavAction next;bool selected=false;
 qiming::GridMission::Stage stage(){return qiming::GridMission::CONNECT;}
 qiming::NavAction tick(unsigned,bool,bool,bool){return next;}
 bool gridRouteSelected(){return selected;}int error(){return 0;}
}mission;
struct Client{int error(){return 0;}}client;
int starts=0,stops=0;bool serviceMotion(){return true;}void stop(){++stops;}
void startMotion(const qiming::NavAction&){++starts;}
}
bool taskCodeDecoded=false,scanFlag=false;unsigned millis(){return 100;}
void finishProgram(){}void routeFault(const char*){navigation::mode.fail();}
void markMissionProgress(){}void resetQrReceiver(){}void hmiSetRunStatus(const char*){}
namespace route_chassis {void resetQrReceiver(){}}
int workPauseAfterRouteStep(int n){return n;}void beginIntegratedWorkPause(int){}
"""
dispatch+=function('serviceGridMission',main_source)
dispatch+=r"""
int main(){using M=qiming::NavigationMode;using A=qiming::NavAction;namespace rn=radar_navigation;
 for(bool obstacles:{false,true}){
  navigation::mode=M();navigation::gridMotionOwned=false;
  assert(navigation::mode.start(1));rn::mission.selected=false;rn::mission.next.kind=A::MOVE;
  serviceGridMission();assert(navigation::gridMotionOwned&&navigation::mode.state()==M::SCANNING);
  rn::mission.next.kind=A::TURN;serviceGridMission();assert(navigation::mode.state()==M::SCANNING);
  if(obstacles){rn::mission.selected=true;rn::mission.next.kind=A::MOVE;serviceGridMission();assert(navigation::mode.state()==M::GRID);}
  else{rn::mission.next.kind=A::CLEAR_ROUTE;serviceGridMission();assert(!navigation::gridMotionOwned&&navigation::mode.state()==M::FIXED_CLEAR);}
 }
}
"""
# The initializer-list loop is kept local to the host harness.
dispatch='#include <initializer_list>\n'+dispatch
with tempfile.TemporaryDirectory(prefix='gc_dispatch_') as tmp:
 cpp=Path(tmp)/'dispatch.cpp';cpp.write_text(dispatch);exe=Path(tmp)/'dispatch'
 subprocess.run(['clang++','-std=c++17','-Wall','-Wextra','-Werror','-DGONGCHUANG_GRID_NAVIGATION=1','-I',str(root/'src'),'-I',str(root/'lib/QimingGrid/src'),str(cpp),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('PASS actual production dispatcher: preparation keeps SCANNING; delayed clear handoff; accepted obstacle map enters GRID')
