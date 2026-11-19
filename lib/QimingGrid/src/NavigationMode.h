#pragma once
#include <stdint.h>
namespace qiming {
// Selection is mutable only before start. A failed scan never means clear.
class NavigationMode {
 public:
  enum Mode : uint8_t { LEGACY, RADAR };
  enum State : uint8_t { SELECTING, SCANNING, FIXED_LEGACY, FIXED_CLEAR,
                         GRID, FINISHED, FAULT };
  bool select(Mode mode) {
    if (state_ != SELECTING) return false;
    selected_ = mode; return true;
  }
  bool start(uint8_t zone) {
    if (state_ != SELECTING || (zone != 1 && zone != 2)) return false;
    zone_ = zone; state_ = selected_ == RADAR ? SCANNING : FIXED_LEGACY;
    return true;
  }
  bool mapAccepted(bool obstacles) {
    if (state_ != SCANNING) return false;
    state_ = obstacles ? GRID : FIXED_CLEAR; return true;
  }
  void fail() { state_ = FAULT; }
  void finish() { state_ = FINISHED; }
  Mode selected() const { return selected_; }
  State state() const { return state_; }
  uint8_t zone() const { return zone_; }
  bool radarRun() const { return selected_ == RADAR && state_ != SELECTING; }
 private:
  Mode selected_ = RADAR;
  State state_ = SELECTING;
  uint8_t zone_ = 1;
};
} // namespace qiming
