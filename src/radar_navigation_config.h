#pragma once

// 1: grid mission (production default); 0: retained 19-command legacy route.
// Workstation fixtures retain their independent arm/vision behavior.
#ifndef GONGCHUANG_GRID_NAVIGATION
#if GONGCHUANG_RUN_MODE != 0 || defined(GONGCHUANG_DEBUG_STORAGE1) || defined(GONGCHUANG_DEBUG_RAW)
#define GONGCHUANG_GRID_NAVIGATION 0
#else
#define GONGCHUANG_GRID_NAVIGATION 1
#endif
#endif
#if GONGCHUANG_GRID_NAVIGATION != 0 && GONGCHUANG_GRID_NAVIGATION != 1
#error "GONGCHUANG_GRID_NAVIGATION must be 0 or 1"
#endif

#ifndef GONGCHUANG_VERBOSE_TRACE
#if GONGCHUANG_GRID_NAVIGATION
#define GONGCHUANG_VERBOSE_TRACE 0
#else
#define GONGCHUANG_VERBOSE_TRACE 1
#endif
#endif

#if GONGCHUANG_GRID_NAVIGATION
#include <GridNavigator.h>
#include <NavigationMode.h>
namespace navigation {
static qiming::NavigationMode mode;
static bool gridMotionOwned = false;
static bool departureTrayPending = false;
static void (*onDeparture)() = nullptr;
}
namespace radar_navigation_config {
// All geometry is in mm. Grid bounds MUST equal RadarViewer's saved layout.
// GridConfig also contains the four dock centers; edit them here when measured.
inline qiming::GridConfig geometry() { return qiming::GridConfig(); }
constexpr uint8_t NODE_ID = 1;
constexpr uint32_t BAUD = 115200;
constexpr float MOVE_MAX_MM_S = 250.0f;
constexpr float MOVE_ACCEL_MM_S2 = 200.0f;
constexpr float MOVING_YAW_LIMIT_DEGREES = 1.0f;
constexpr float STOPPED_YAW_LIMIT_DEGREES = 0.5f;
}
#endif
