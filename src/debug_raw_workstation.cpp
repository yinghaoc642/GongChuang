// Dedicated RAW-area workstation debug firmware.
// Build/upload only through the debug_raw_workstation PlatformIO environment.
// It includes the production state machine directly: no RAW motion, vision,
// tracking or tray parameter is copied here.

// Select which QR color row is collected. Both rounds use the fixed bench task
// 123+132+123+132, so the physical target order is red/yellow/blue either way.
// Keep round 1 as the ordinary default; set this macro to 2 for RAW round 2.
#ifndef GONGCHUANG_DEBUG_RAW_ROUND
#define GONGCHUANG_DEBUG_RAW_ROUND 1
#endif

// 0 runs the retained legacy fixed-A pickup logic and is the current default.
// Set to 1 only when restoring the archived A/B search and descent tracker.
#ifndef GONGCHUANG_RAW_PICKUP_LOGIC_MODE
#define GONGCHUANG_RAW_PICKUP_LOGIC_MODE 0
#endif

#define GONGCHUANG_DEBUG_RAW 1
#include "main.cpp"
