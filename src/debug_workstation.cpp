// Independent workstation firmware. Build ONLY with the debug_workstation env.
// Set the single macro below to 1 PROCESS, 2 STORAGE1, or 3 STORAGE2.
// A dedicated wrapper may define it before including this file; the ordinary
// debug_workstation environment currently defaults to STORAGE1/state 2.
// Reuse the current production arm/vision state machines; never copy their
// motion parameters into a second, drifting implementation.
#ifndef GONGCHUANG_DEBUG_WORKSTATION_STATE
#define GONGCHUANG_DEBUG_WORKSTATION_STATE 2
#endif

// 本入口只选择工位和诊断输出。夹取/放置、回收、M5/M6/M7运动曲线、
// 夹爪/料盘角度、视觉换算和净修正全部继承当前main.cpp及RobotConfig.h。
// 不在此处定义运动参数覆盖值；包括3号位伸长量和1号位回料盘缓冲，
// 均使用正式默认值，防止主程序更新后调试入口仍沿用旧参数。

#if GONGCHUANG_DEBUG_WORKSTATION_STATE == 1
#define GONGCHUANG_RUN_MODE 1
#elif GONGCHUANG_DEBUG_WORKSTATION_STATE == 2
#define GONGCHUANG_DEBUG_STORAGE1 1
#elif GONGCHUANG_DEBUG_WORKSTATION_STATE == 3
// All STORAGE2 scan-pose, coordinate-scale and final-placement corrections
// come directly from the included production main.cpp/RobotConfig.h. Keep no
// debug-only copies here, otherwise bench and production values can drift.
// State 3 is the dedicated diagnosis build. Compile detailed per-visit,
// per-frame, rescan-reason and pre-fault serial telemetry only here.
#define GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG 1
#define GONGCHUANG_RUN_MODE 2
#else
#error "GONGCHUANG_DEBUG_WORKSTATION_STATE must be 1, 2 or 3"
#endif

#include "main.cpp"
