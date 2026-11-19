#include "QimingGridClient.h"
#include <string.h>
namespace qiming {
namespace {
uint16_t u16(const uint8_t *p) { return p[0] | uint16_t(p[1]) << 8; }
uint32_t u32(const uint8_t *p) { return u16(p) | uint32_t(u16(p + 2)) << 16; }
void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
bool poseValid(const RadarPose &p) {
  return p.headingQ6 < 23040 && p.x >= -200 && p.x <= 500 &&
         ((p.zone == 1 && p.y >= 1900 && p.y <= 2600) ||
          (p.zone == 2 && p.y >= -200 && p.y <= 500));
}
bool maskOf(const uint8_t *p, uint8_t count, uint32_t &mask) {
  mask = 0;
  if (count > 25) return false;
  for (uint8_t i = 0; i < count; ++i) {
    if (p[i] > 24 || (mask & (1UL << p[i]))) return false;
    mask |= 1UL << p[i];
  }
  return true;
}
}
bool GridClient::send(uint8_t command, const uint8_t *payload, uint16_t length) {
  uint8_t encoded[LINK_MAX_ENCODED_FRAME];
  const size_t n = link_encode_message(node_, command, ++sequence_, payload, length,
                                       encoded, sizeof(encoded));
  return n && writer_ && writer_(context_, encoded, n);
}
bool GridClient::request(Pending op, uint8_t command, uint32_t now,
                         const uint8_t *payload, uint16_t length) {
  if (error_ != OK || pending_ != NONE) return false;
  now_ = started_ = progress_ = now;
  pending_ = op; command_ = command;
  ack_ = result_ = locked_ = scanning_ = false; frames_ = 0;
  if (!send(command, payload, length)) { fail(TX); return false; }
  requestSequence_ = sequence_;
  return true;
}
void GridClient::begin(Writer writer, void *context, uint32_t now, uint8_t zone, uint8_t node) {
  *this = GridClient();
  writer_ = writer; context_ = context; zone_ = zone; node_ = node;
  now_ = lastRx_ = heartbeat_ = health_ = now; active_ = true;
  link_decoder_init(&decoder_);
  if ((zone != 1 && zone != 2) || node == 0 || node == 255) { fail(STATE); return; }
  request(HELLO, LINK_CMD_HELLO, now);
}
void GridClient::stop() {
  if (active_) send(LINK_CMD_STOP);
  active_ = ready_ = false; pending_ = NONE;
}
void GridClient::fail(Error e) {
  error_ = e; stop(); gridReady_ = pathReady_ = poseDone_ = false;
}
void GridClient::decoded(const link_message_t *m, void *context) {
  static_cast<GridClient *>(context)->message(*m);
}
void GridClient::receive(const uint8_t *bytes, size_t count, uint32_t now) {
  if (!active_) return;
  now_ = now;
  const uint32_t bad = decoder_.bad_frames;
  link_decoder_feed(&decoder_, bytes, count, decoded, this);
  if (decoder_.bad_frames != bad && error_ == OK) fail(FRAME);
}
void GridClient::poll(uint32_t now) {
  if (!active_ || error_ != OK) return;
  now_ = now;
  if (connected_ && uint32_t(now - lastRx_) >= 5000) { fail(LINK_LOST); return; }
  if (pending_ != NONE &&
      (uint32_t(now - started_) >= (pending_ == SCAN ? 8000U : 1500U) ||
       (pending_ == SCAN && uint32_t(now - progress_) >= 3500))) { fail(TIMEOUT); return; }
  if (uint32_t(now - heartbeat_) >= 500) {
    heartbeat_ = now;
    if (!send(LINK_CMD_HEARTBEAT)) { fail(TX); return; }
  }
  if (ready_ && pending_ == NONE && uint32_t(now - health_) >= 2000) {
    health_ = now;
    if (!send(LINK_CMD_HELLO)) fail(TX);
  }
}
bool GridClient::scan(uint32_t now) {
  if (!ready_) return false;
  gridReady_ = false;
  return request(SCAN, LINK_CMD_SINGLE_MEASURE, now);
}
bool GridClient::setPose(const RadarPose &pose, uint32_t now) {
  if (!ready_ || !poseValid(pose) || pose.zone != zone_) return false;
  uint8_t p[7] = {pose.zone};
  put16(p + 1, pose.x); put16(p + 3, pose.y); put16(p + 5, pose.headingQ6);
  expectedPose_ = pose; poseDone_ = false;
  return request(POSE, LINK_CMD_SET_POSE, now, p, sizeof(p));
}
bool GridClient::plan(uint8_t start, uint8_t goal, uint32_t now) {
  if (!ready_ || start > 24 || goal > 24) return false;
  const uint8_t p[2] = {start, goal};
  path_.start = start; path_.goal = goal; pathReady_ = false;
  return request(PLAN, LINK_CMD_PLAN_PATH, now, p, 2);
}
bool GridClient::takeGrid(uint32_t &mask) {
  if (!gridReady_) return false;
  gridReady_ = false; mask = grid_; return true;
}
bool GridClient::takePath(GridPath &path) {
  if (!pathReady_) return false;
  pathReady_ = false; path = path_; return true;
}
bool GridClient::takePoseDone() { const bool v = poseDone_; poseDone_ = false; return v; }
void GridClient::finishIfComplete() {
  if (!ack_) return;
  const Pending done = pending_;
  if (done != POSE && !result_) return;
  if (done == SCAN && !locked_) return;
  pending_ = NONE;
  switch (done) {
    case HELLO: request(SELECT, LINK_CMD_SELECT_ZONE, now_, &zone_, 1); break;
    case SELECT: request(READ_START, LINK_CMD_GET_PERSISTED_STATE, now_); break;
    case READ_START: ready_ = true; break;
    case SCAN: request(READ_SCAN, LINK_CMD_GET_PERSISTED_STATE, now_); break;
    case READ_SCAN: gridReady_ = true; break;
    case POSE: request(VERIFY_POSE, LINK_CMD_HELLO, now_); break;
    case VERIFY_POSE: poseDone_ = true; break;
    case PLAN: pathReady_ = true; break;
    default: break;
  }
}
void GridClient::message(const link_message_t &m) {
  if (!active_ || m.node_id != node_) return;
  const uint8_t *p = m.payload;
  const uint16_t n = m.payload_length;
  // Unknown/private events never change link health or transaction state.
  switch (m.type) {
    case LINK_EVT_HELLO: {
      if (n != 13 || p[2] != 6 || p[0] != 1 || p[1] < 35 || p[3] > 2 || p[10] > 1) {
        fail(VERSION); return;
      }
      connected_ = true;
      RadarPose pose{int16_t(u16(p+4)), int16_t(u16(p+6)), u16(p+8), p[3]};
      if (pending_ == HELLO) result_ = true;
      else if (pending_ == SELECT && pose.zone == zone_) { expectedPose_ = pose; result_ = true; }
      else if (pending_ == VERIFY_POSE) {
        if (pose.zone != zone_ || pose.x != expectedPose_.x || pose.y != expectedPose_.y ||
            pose.headingQ6 != expectedPose_.headingQ6) { fail(STATE); return; }
        result_ = true;
      }
      else if (ready_ && pending_ != POSE && (pose.zone != zone_ || pose.x != expectedPose_.x ||
               pose.y != expectedPose_.y || pose.headingQ6 != expectedPose_.headingQ6)) {
        fail(STATE); return;
      }
      break;
    }
    case LINK_EVT_ACK:
      if (n != 2) { fail(FRAME); return; }
      if (pending_ != NONE && p[0] == command_) {
        if (p[1]) { fail(DEVICE); return; }
        ack_ = true;
      }
      break;
    case LINK_EVT_STATUS:
      if (n != 5 || p[2] > 1 || p[4] > 1 || p[3] > 14) { fail(FRAME); return; }
      if (pending_ == SCAN && p[1] == 2) {
        if (p[4]) { fail(SCAN_OVERFLOW); return; }
        if (p[0] == 1 || p[0] == 2) scanning_ = true;
        if (scanning_ && p[3] > frames_) { frames_ = p[3]; progress_ = now_; }
        if (scanning_ && p[0] == 4 && p[3] == 14) locked_ = true;
      }
      break;
    case LINK_EVT_OBSTACLE_GRID: {
      if (pending_ != SCAN || !scanning_) return;
      uint32_t mask;
      if (n < 5 || n != 5U + p[4] || !maskOf(p+5, p[4], mask)) { fail(FRAME); return; }
      if (haveScan_ && int32_t(u32(p) - lastScan_) <= 0) return;
      grid_ = mask; lastScan_ = u32(p); haveScan_ = true; result_ = true;
      break;
    }
    case LINK_EVT_PERSISTED_STATE: {
      if (pending_ != READ_START && pending_ != READ_SCAN) return;
      uint32_t mask;
      if (n < 33 || u16(p) != 11 || p[9] > 1 || n != 33U + p[10] ||
          !maskOf(p+33, p[10], mask)) { fail(FRAME); return; }
      if (pending_ == READ_START) {
        const uint8_t *s = p + (zone_ == 1 ? 11 : 18);
        savedPose_ = {int16_t(u16(s)), int16_t(u16(s+2)), u16(s+4), zone_};
        // The seventh saved-pose byte is not interpreted by the vendor
        // client either. Zone comes from the selected slot, never this byte.
        if (!poseValid(savedPose_) || savedPose_.x != expectedPose_.x ||
            savedPose_.y != expectedPose_.y || savedPose_.headingQ6 != expectedPose_.headingQ6) {
          fail(STATE); return;
        }
        expectedPose_ = savedPose_;
      } else if (!p[9] || mask != grid_ || u32(p+2) == record_) {
        fail(SAVED_GRID); return;
      }
      persistedGrid_ = mask; record_ = u32(p+2); result_ = true;
      break;
    }
    case LINK_EVT_PATH_RESULT:
      if (n < 6) { fail(FRAME); return; }
      if (pending_ != PLAN || u16(p) != requestSequence_) return;
      if (p[2] > 6 || p[3] != path_.start || p[4] != path_.goal || p[5] > 25 || n != 6U+p[5]) {
        fail(FRAME); return;
      }
      path_.status = p[2]; path_.count = p[5];
      for (uint8_t i=0; i<path_.count; ++i) {
        if (p[6+i] > 24) { fail(FRAME); return; }
        path_.cells[i] = p[6+i];
      }
      result_ = true;
      break;
    case LINK_EVT_ERROR: fail(DEVICE); return;
    default: return;
  }
  lastRx_ = now_;
  finishIfComplete();
}
} // namespace qiming
