// Dedicated second-storage workstation debug firmware.
// Build/upload only through the debug_storage2_workstation PlatformIO env.
// This selects state 3 in the shared workstation wrapper, which then includes
// the production main.cpp state machine. Keep all motion and vision parameters
// in those shared sources so this entry cannot drift from the formal workflow.
#define GONGCHUANG_DEBUG_WORKSTATION_STATE 3
#include "debug_workstation.cpp"
