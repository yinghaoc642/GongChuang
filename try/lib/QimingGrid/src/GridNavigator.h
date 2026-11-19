#pragma once
#include "QimingGridClient.h"
namespace qiming {
struct Point { float x, y; };
struct GridConfig {
  float x[6] = {0,480,960,1440,1920,2400};
  float y[6] = {0,480,960,1440,1920,2400};
  float halfForward = 145, halfRight = 117, margin = 5;
  float sensorForward = 0, sensorRight = 84; // 234/2 - 66/2, side-mounted inside body.
  // Map +X is mirrored. Mathematical XY heading decreases on a body CCW turn.
  // Vendor docs do not specify heading handedness: verify this sign on hardware.
  int8_t radarHeadingPerCcw = -1;
  Point qr{150,1200}, raw{1200,2080}, process{1200,320}, storage{2080,1200};
};
class GridGeometry {
 public:
  explicit GridGeometry(const GridConfig &config = GridConfig()) : config(config) {}
  GridConfig config;
  static constexpr uint32_t permanent = (1UL<<6)|(1UL<<8)|(1UL<<16)|(1UL<<18);
  uint32_t blocked = permanent;
  static uint8_t id(uint8_t column, uint8_t row);
  static uint8_t column(uint8_t cell);
  static uint8_t row(uint8_t cell) { return cell/5; }
  bool valid() const;
  uint8_t cell(Point p) const;
  Point center(uint8_t cell) const;
  bool translationSafe(Point from, Point to, float heading) const;
  bool rotationSafe(Point at) const;
  bool pathValid(const GridPath &path, uint32_t map) const;
  bool plan(uint8_t start, uint8_t goal, float heading, GridPath &out) const;
 private:
  bool rectangleFree(float left, float bottom, float right, float top) const;
};

// World origin is the bottom-right corner, +X toward STORAGE, +Y toward RAW.
// Body yaw 0: forward=-Y, right=+X; positive yaw is counterclockwise.
void bodyToWorld(float forward, float right, float heading, float &dx, float &dy);
void worldToBody(float dx, float dy, float heading, float &forward, float &right);
float shortTurn(float degrees);

struct NavAction {
  enum Kind : uint8_t { NONE, MOVE, TURN, QR_RESET, WORK, FINISH, FAULT, CLEAR_ROUTE } kind = NONE;
  float forward = 0, right = 0, degrees = 0;
  uint8_t workStep = 0;
  bool departure = false;
};
class GridMission {
 public:
  enum Stage : uint8_t { IDLE, CONNECT, SCAN1, OUT_FORWARD, OUT_RIGHT, SCAN_POSE,
    POSE_UPDATE, SCAN_VIEW, RESTORE_HEADING, RESTORE_POSE,
    CLEAR_LATERAL, CLEAR_HOME, CLEAR_POSE, ENTER_GRID, WAIT_PATH, FOLLOW, DOCK_TURN, DOCK,
    QR_WAIT, QR_SEARCH, QR_RETURN, AT_WORK, EGRESS, HOME, DONE, FAILED, CLEAR };
  explicit GridMission(GridClient &client, const GridConfig &config = GridConfig())
      : geometry(config), client_(client) {}
  GridGeometry geometry;
  void begin(uint8_t zone, bool originalWhenClear = false);
  NavAction tick(uint32_t now, bool motionFinished, bool qrReady, bool workFinished);
  bool trackWorkstationMove(float forward, float right, float degrees);
  bool alignWorkstationHeading(float heading);
  Stage stage() const { return stage_; }
  Point position() const { return position_; }
  float heading() const { return heading_; }
  bool awaitingMotion() const { return moving_; }
  bool gridRouteSelected() const { return gridSelected_; }
  uint8_t error() const { return error_; }
 private:
  GridClient &client_;
  Stage stage_ = IDLE;
  Point position_{}, endpoint_{}, home_{}, qrOrigin_{};
  float heading_ = 0, endHeading_ = 0;
  uint8_t zone_ = 1, visit_ = 0, pathIndex_ = 0, qrLeg_ = 0, error_ = 0, scanView_ = 0;
  bool moving_ = false, departure_ = false;
  bool originalWhenClear_ = false, gridSelected_ = false;
  uint32_t latestMap_ = 0, qrStart_ = 0;
  GridPath path_{};
  NavAction fault(uint8_t code);
  NavAction move(Point p, Stage next);
  NavAction turn(float degrees, Stage next);
  NavAction updatePose(uint32_t now, Stage next);
  NavAction requestPath(uint32_t now);
  Point destination() const;
  float destinationHeading() const;
};
} // namespace qiming
