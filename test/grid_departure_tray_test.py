"""Execute the real deferred-parking helper in both firmware modes.

Also check dispatch ordering so a later storage preparation cannot overwrite
launch-angle parking during travel. Six departures are exercised by the mission
simulation in qiming_grid_test.py.
"""
from pathlib import Path
import re
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
source = (root / 'src/main.cpp').read_text()
def function(name):
    match = re.search(r'^\w[\w :*&<>]*\b' + name + r'\s*\([^;{}]*\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]
helper = function('prepareDepartureTrayParking')
route = (root / 'src/route_chassis_main.inc').read_text()
assert 'navigation::departureTrayPending = radarDepartureEnabled();' in function('finishIntegratedWorkPause')
assert 'if (navigation::onDeparture) navigation::onDeparture();' in route
assert route.index('if (navigation::onDeparture) navigation::onDeparture();') < route.index('motor1.move(pulses.motor1);')
assert 'useStorage1TravelPreparation && !radarDepartureEnabled()' in source
travel = function('serviceStorageTravelPreparation')
assert 'if (radarDepartureEnabled()) workStorageServoDeadlineMs = 0UL;' in travel
for name in ('startRawTravelParkingAtSafeLinearZero', 'beginStorageParkingBeforeWorkFinish', 'completeTransferAndRotateStorage'):
    assert 'prepareDepartureTrayParking(' in function(name)
assert 'prepareDepartureTrayParking(' in function('serviceArmTransfer')
assert 'prepareDepartureTrayParking(' in function('serviceCompetitionAction')
assert 'commandStorageServoParkingPosition(' not in function('finishIntegratedWorkPause')
# Abort/fault must STOP the radar session as well as disable chassis motion.
assert 'radar_navigation::stop();' in function('routeFault')
assert 'radar_navigation::stop();' in function('abortRoute')
assert 'GRID departure heading' in function('finishIntegratedWorkPause')
harness = r'''
#include <stdint.h>
#include <cassert>
uint32_t workStorageServoDeadlineMs=999;
[[maybe_unused]] const uint32_t STORAGE_SERVO_SETTLE_MS=510;
unsigned calls=0;uint16_t lastInterval=0;
uint32_t millis(){return 100;}
bool radar=false;bool radarDepartureEnabled(){return radar;}
void commandStorageServoParkingPosition(uint16_t interval){++calls;lastInterval=interval;}
'''+helper+r'''
int main(){
 // Runtime legacy mode must issue the original command without changing the
 // caller-owned settling deadline. Runtime RADAR defers until actual motion.
 prepareDepartureTrayParking(1023);
 assert(calls==1&&lastInterval==1023&&workStorageServoDeadlineMs==999);
 radar=true;prepareDepartureTrayParking(1023);
 assert(calls==1&&workStorageServoDeadlineMs==0);
 commandStorageServoParkingPosition(1023);
 assert(calls==2&&lastInterval==1023);
}
'''
with tempfile.TemporaryDirectory(prefix='gc_tray_') as tmp:
    cpp=Path(tmp)/'tray.cpp';cpp.write_text(harness)
    for mode in (0,1):
        exe=Path(tmp)/f'tray{mode}'
        subprocess.run(['clang++','-std=c++17','-Wall','-Werror',f'-DGONGCHUANG_GRID_NAVIGATION={mode}',str(cpp),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
print('PASS departure tray: deferred until chassis dispatch; no in-travel slot-0 overwrite; bench/legacy settling preserved; radar STOP on abort/fault')
