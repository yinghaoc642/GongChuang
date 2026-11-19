"""Run the actual station pulse conversion/guard against physical directions.

The chassis front is M3/M4, opposite MecanumKinematics library forward. This
catches the sign difference between legacy station helpers and grid commands.
"""
from pathlib import Path
import re
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
source=(root/'src/main.cpp').read_text()
def function(name):
    match=re.search(r'^\w[\w :*&<>]*\b'+name+r'\s*\([^;{}]*\)\s*\{',source,re.M)
    assert match,name
    end,depth=match.end(),1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[match.start():end]
harness=r'''
#include <MecanumKinematics.h>
#include <cmath>
#include <cassert>
#include <cstdint>
using namespace mecanum;
constexpr float PI_F=3.14159265358979323846f,TWO_PI_F=2*PI_F;
constexpr float FORWARD_PULSES_PER_METER=10000,LATERAL_PULSES_PER_METER=10000;
constexpr float PULSES_PER_WHEEL_REVOLUTION=3200;
constexpr float COUNTERCLOCKWISE_ROTATION_PULSE_SCALE=1,CLOCKWISE_ROTATION_PULSE_SCALE=1;
const WheelDirections MOTOR_DIRECTIONS(-1,1,-1,1);
MecanumKinematics geometry=MecanumKinematics::fromMillimeters(187.5f,195,100);
struct MotorPulses{long motor1,motor2,motor3,motor4;
 MotorPulses(long a=0,long b=0,long c=0,long d=0):motor1(a),motor2(b),motor3(c),motor4(d){}};
struct Motor{long position=0;void move(long p){position+=p;}}motor1,motor2,motor3,motor4;
struct Debug{template<class T>void print(T){}template<class T>void println(T){}}SerialDebug;
constexpr bool LOCAL_WORKSTATION_SIMULATION_MODE=false,RAW_WORKSTATION_SIMULATION_MODE=false;
bool fresh=true,fault=false;bool imuIsFresh(){return fresh;}
void routeFault(const char*){fault=true;}
namespace route_chassis{float currentRouteHeadingDegrees(){return 90;}}
namespace navigation{bool gridMotionOwned=true;}
namespace radar_navigation{
 bool active=true;
 struct Mission{float forward=0,right=0,degrees=0;bool safe=true;
 bool alignWorkstationHeading(float h){return h==90;}
 bool trackWorkstationMove(float f,float r,float d){forward=f;right=r;degrees=d;return safe;}
 }mission;
}
'''
for name in ('motorPulsesAreZero','roundedPulseCount','bodyDisplacementToMotorPulses','absolutePulseCount','rotationPulsesPerDegree','startRelativeMotorMove'):
    harness+=function(name)+'\n'
harness+=r'''
int main(){
 // physical forward100/right40 -> library forward -0.1m/left +0.04m.
 auto pulses=bodyDisplacementToMotorPulses(-.1f,.04f,0);
 startRelativeMotorMove(pulses);
 assert(!fault&&fabsf(radar_navigation::mission.forward-100)<.1f&&fabsf(radar_navigation::mission.right-40)<.1f);
 assert(fabsf(radar_navigation::mission.degrees)<.01f);
 pulses=bodyDisplacementToMotorPulses(0,0,10*PI_F/180);startRelativeMotorMove(pulses);
 assert(fabsf(radar_navigation::mission.degrees-10)<.1f);
 pulses=bodyDisplacementToMotorPulses(0,0,-10*PI_F/180);startRelativeMotorMove(pulses);
 assert(fabsf(radar_navigation::mission.degrees+10)<.1f);
 long old=motor1.position;radar_navigation::mission.safe=false;startRelativeMotorMove(pulses);
 assert(fault&&motor1.position==old);
 fault=false;radar_navigation::mission.safe=true;fresh=false;startRelativeMotorMove(pulses);
 assert(fault&&motor1.position==old);
}
'''
with tempfile.TemporaryDirectory(prefix='gc_station_grid_') as tmp:
    cpp=Path(tmp)/'test.cpp';cpp.write_text(harness);exe=Path(tmp)/'test'
    lib=root/'lib/MecanumKinematics/src'
    subprocess.run(['clang++','-std=c++17','-Wall','-Werror','-DGONGCHUANG_GRID_NAVIGATION=1','-DGONGCHUANG_RAW_PICKUP_LOGIC_MODE=0','-I',str(lib),str(cpp),str(lib/'MecanumKinematics.cpp'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS actual station pulse guard: M3/M4 physical front; user right; CCW/CW signs; unsafe/IMU-lost commands blocked before motor output')
