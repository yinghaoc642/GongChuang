// Dedicated RAW-area reproduction for the observed 3 -> 4 -> 1 sequence.
//
// It directly includes the production main.cpp implementation.  The chassis
// route never starts and legacy fixed-A pickup is forced, so M1-M4 receive no
// motion command.  The arm, gripper and storage tray perform the real RAW
// workstation action after PB9 is pressed.

#define GONGCHUANG_DEBUG_RAW_ROUND 1
#define GONGCHUANG_DEBUG_RAW_TASK_CODE "341+123+413+231"
#define GONGCHUANG_RAW_PICKUP_LOGIC_MODE 0
#define GONGCHUANG_DEBUG_RAW 1
#include "main.cpp"
