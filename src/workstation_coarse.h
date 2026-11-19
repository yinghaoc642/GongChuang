#pragma once
#include <math.h>
#include <stdint.h>

// All values are estimates from the existing project, not new measurements.
// Board 150 .. island edge 550 => drivable lane centre 350 mm from boundary.
// Ring centre: drawing 75..85 => 80 mm. Effective pivot offset is inferred
// from the current 320 mm dock and the former 45deg/90mm endpoint seed:
// 320 - 80 - (135.74 + 90)*cos(45deg) = 80.3757 mm.
// The old 225mm route annotation cannot be a physical pivot offset here:
// it would put ring2 behind the pivot at the current dock.
namespace workstation_coarse {
constexpr float LANE_CENTER_MM = (150.0f + 550.0f) / 2.0f;
constexpr float RING_BOUNDARY_MM = 80.0f;
constexpr float ESTIMATED_PIVOT_OFFSET_MM = 80.37570f;
constexpr float CAMERA_ZERO_RADIUS_MM = 135.74f;
constexpr float TARGET_RADIAL_MM = LANE_CENTER_MM - RING_BOUNDARY_MM - ESTIMATED_PIVOT_OFFSET_MM;
constexpr float VIEW_EXTENSION_MM = TARGET_RADIAL_MM - CAMERA_ZERO_RADIUS_MM;
constexpr float RING_SPACING_MM = 150.0f;
constexpr float TOLERANCE_MM = 15.0f;
constexpr uint32_t VISION_TIMEOUT_MS = 2500U;
constexpr uint8_t MAX_ROUNDS = 2U;
constexpr float MAX_STEP_MM = 60.0f;
constexpr float MAX_TOTAL_MM = 100.0f;
struct State {
  bool enabled = false;
  uint8_t rounds = 0;
  float outward = TARGET_RADIAL_MM;
  float along = 0;
  float travelled = 0;
  void reset(bool active) { enabled=active;rounds=0;outward=TARGET_RADIAL_MM;along=0;travelled=0; }
  // Returned library frame: +forward = arm-plane +along, +left = -outward.
  // metric is the physical OUTER circle radius, not the numeral box radius.
  bool observe(int x, int y, unsigned radius, unsigned confidence, float &forward, float &left) {
    forward=left=0;
    if (rounds >= MAX_ROUNDS) return false;
    ++rounds;
    if (x<0 || x>319 || y<0 || y>239 || radius<16 || radius>100 || confidence<700) return false;
    const float scale=41.75f / radius;
    outward=TARGET_RADIAL_MM-(y-120)*scale;
    along=-(x-160)*scale;
    forward=along;left=TARGET_RADIAL_MM-outward;
    const float error=hypotf(forward,left);
    if (error <= TOLERANCE_MM) { forward=left=0;return false; }
    const float available=fminf(MAX_STEP_MM, MAX_TOTAL_MM-travelled);
    if (available<=0) { forward=left=0;return false; }
    const float gain=fminf(1.0f, available/error);
    forward*=gain;left*=gain;
    return true;
  }
  void moved(float forward,float left) { outward+=left;along-=forward;travelled+=hypotf(forward,left); }
  void seed(uint8_t ring,float &degrees,float &extension) const {
    const float side=along+(ring==1 ? RING_SPACING_MM : -RING_SPACING_MM);
    degrees=atan2f(side,outward)*57.2957795131f;
    // Bad/partial coarse observations must never create an unreachable seed.
    degrees=fmaxf(-75.0f,fminf(75.0f,degrees));
    extension=fmaxf(0.0f,fminf(160.0f,hypotf(outward,side)-CAMERA_ZERO_RADIUS_MM));
  }
};
}
