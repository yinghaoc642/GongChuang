#pragma once
#include "link_protocol.h"
#include <stddef.h>
#include <stdint.h>

namespace qiming {
struct RadarPose { int16_t x, y; uint16_t headingQ6; uint8_t zone; };
struct GridPath { uint8_t status, start, goal, count, cells[25]; };

// Public protocol v6 only. No point clouds, private layout commands or HAL.
// Call poll/receive frequently, including during arm operations and motion.
class GridClient {
 public:
  using Writer = bool (*)(void *, const uint8_t *, size_t);
  enum Error : uint8_t { OK, FRAME, VERSION, DEVICE, TIMEOUT, LINK_LOST,
                        TX, STATE, SCAN_OVERFLOW, SAVED_GRID };
  void begin(Writer writer, void *context, uint32_t now, uint8_t zone, uint8_t node = 1);
  void receive(const uint8_t *bytes, size_t count, uint32_t now);
  void poll(uint32_t now);
  void stop();
  bool scan(uint32_t now);
  bool setPose(const RadarPose &pose, uint32_t now);
  bool plan(uint8_t start, uint8_t goal, uint32_t now);
  bool takeGrid(uint32_t &mask);
  bool takePath(GridPath &path);
  bool takePoseDone();
  bool ready() const { return ready_; }
  bool busy() const { return pending_ != NONE; }
  Error error() const { return error_; }
  const RadarPose &savedPose() const { return savedPose_; }
 private:
  enum Pending : uint8_t { NONE, HELLO, SELECT, READ_START, SCAN, READ_SCAN, POSE, VERIFY_POSE, PLAN };
  Writer writer_ = nullptr;
  void *context_ = nullptr;
  link_decoder_t decoder_{};
  Pending pending_ = NONE;
  Error error_ = OK;
  RadarPose savedPose_{}, expectedPose_{};
  GridPath path_{};
  uint32_t now_ = 0, started_ = 0, lastRx_ = 0, heartbeat_ = 0, health_ = 0;
  uint32_t progress_ = 0, grid_ = 0, persistedGrid_ = 0, lastScan_ = 0, record_ = 0;
  uint16_t sequence_ = 0, requestSequence_ = 0;
  uint8_t node_ = 1, zone_ = 1, command_ = 0, frames_ = 0;
  bool active_ = false, ready_ = false, connected_ = false;
  bool ack_ = false, result_ = false, locked_ = false, scanning_ = false;
  bool gridReady_ = false, pathReady_ = false, poseDone_ = false, haveScan_ = false;
  bool send(uint8_t command, const uint8_t *payload = nullptr, uint16_t length = 0);
  bool request(Pending op, uint8_t command, uint32_t now,
               const uint8_t *payload = nullptr, uint16_t length = 0);
  void fail(Error error);
  void finishIfComplete();
  void message(const link_message_t &m);
  static void decoded(const link_message_t *m, void *context);
};
} // namespace qiming
