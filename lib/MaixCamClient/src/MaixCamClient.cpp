#include "MaixCamClient.h"

#include <VisionProtocol.h>

namespace gongchuang {

MaixCamClient::Coordinate::Coordinate()
    : targetId(0U),
      requestSequence(0U),
      mode(0U),
      x(0),
      y(0),
      metric(0U),
      confidence(0U),
      cameraTimestampMs(0UL),
      sequence(0UL),
      receivedMs(0UL) {}

MaixCamClient::MaixCamClient(
    HardwareSerial &serial,
    Print &debug,
    FaultHandler faultHandler)
    : serial_(serial),
      debug_(debug),
      faultHandler_(faultHandler),
      initialized_(false),
      requestedMode_(STOP_REQUEST),
      requestSequence_(0U),
      modeCommandSent_(false),
      modeSwitchStartMs_(0UL),
      lastRequestMs_(0UL),
      receiveLine_{0},
      receiveLength_(0U),
      receiveOverflow_(false),
      latest_(),
      telemetrySampleCount_(0U),
      telemetryFirstX_(0),
      telemetryFirstY_(0),
      telemetryPreviousX_(0),
      telemetryPreviousY_(0),
      telemetryMinimumX_(0),
      telemetryMaximumX_(0),
      telemetryMinimumY_(0),
      telemetryMaximumY_(0),
      telemetrySumX_(0),
      telemetrySumY_(0) {}

void MaixCamClient::begin(uint32_t baudrate) {
  serial_.begin(baudrate);
  initialized_ = true;
  while (serial_.available()) {
    serial_.read();
  }
  stopRequest();
}

void MaixCamClient::fault(const char *reason) {
  if (faultHandler_ != nullptr) {
    faultHandler_(reason);
  }
}

bool MaixCamClient::requestIsValid(uint8_t request) {
  const bool targetedColorRequest =
      request >= RED_REQUEST &&
      request <= GREEN_REQUEST;
  return targetedColorRequest ||
         request == ALL_COLORS_REQUEST ||
         request == HOUGH_CIRCLE_REQUEST ||
         request == ENDPOINT_CIRCLE_REQUEST;
}

void MaixCamClient::writeRequestFrame(uint8_t request) {
  uint8_t frame[vision_protocol::REQUEST_FRAME_SIZE] = {0U};
  if (!vision_protocol::buildRequest(
          requestSequence_,
          request,
          frame,
          sizeof(frame))) {
    fault("Vision request frame build failed");
    return;
  }

  const size_t queuedBytes =
      serial_.write(frame, sizeof(frame));
  debug_.print("[MAIX TX] t=");
  debug_.print(millis());
  debug_.print(" ms, v2 seq/mode=");
  debug_.print(requestSequence_);
  debug_.print("/");
  if (request < 0x10U) {
    debug_.print("0");
  }
  debug_.print(request, HEX);
  debug_.print(", crc=");
  if (frame[4] < 0x10U) {
    debug_.print("0");
  }
  debug_.print(frame[4], HEX);
  debug_.print(", queued=");
  debug_.println(
      static_cast<unsigned int>(queuedBytes));
}

void MaixCamClient::stopRequest() {
  if (initialized_) {
    writeRequestFrame(STOP_REQUEST);
  }
  requestedMode_ = STOP_REQUEST;
  modeCommandSent_ = false;
  receiveLength_ = 0U;
  receiveOverflow_ = false;
}

bool MaixCamClient::beginRequest(uint8_t request) {
  if (!requestIsValid(request)) {
    fault("Invalid MaixCAM request");
    return false;
  }

  stopRequest();
  while (serial_.available()) {
    serial_.read();
  }
  requestSequence_ =
      static_cast<uint8_t>(requestSequence_ + 1U);
  requestedMode_ = request;
  modeSwitchStartMs_ = millis();
  lastRequestMs_ = 0UL;
  modeCommandSent_ = false;
  resetCoordinateTelemetry();

  debug_.print("[MAIX PLAN] t=");
  debug_.print(millis());
  debug_.print(" ms, mode=");
  debug_.print(request);
  debug_.print(", seq=");
  debug_.print(requestSequence_);
  debug_.print(", TX after guard=");
  debug_.print(config::vision_link::MODE_SWITCH_GUARD_MS);
  debug_.println(" ms");
  return true;
}

void MaixCamClient::resetCoordinateTelemetry() {
  telemetrySampleCount_ = 0U;
  telemetryFirstX_ = 0;
  telemetryFirstY_ = 0;
  telemetryPreviousX_ = 0;
  telemetryPreviousY_ = 0;
  telemetryMinimumX_ = 0;
  telemetryMaximumX_ = 0;
  telemetryMinimumY_ = 0;
  telemetryMaximumY_ = 0;
  telemetrySumX_ = 0;
  telemetrySumY_ = 0;
}

void MaixCamClient::printCoordinateTelemetry() {
  const int16_t x = latest_.x;
  const int16_t y = latest_.y;
  const bool firstSample = telemetrySampleCount_ == 0U;
  const int16_t previousDeltaX =
      firstSample ? 0 : x - telemetryPreviousX_;
  const int16_t previousDeltaY =
      firstSample ? 0 : y - telemetryPreviousY_;

  if (firstSample) {
    telemetryFirstX_ = x;
    telemetryFirstY_ = y;
    telemetryMinimumX_ = x;
    telemetryMaximumX_ = x;
    telemetryMinimumY_ = y;
    telemetryMaximumY_ = y;
  } else {
    if (x < telemetryMinimumX_) {
      telemetryMinimumX_ = x;
    }
    if (x > telemetryMaximumX_) {
      telemetryMaximumX_ = x;
    }
    if (y < telemetryMinimumY_) {
      telemetryMinimumY_ = y;
    }
    if (y > telemetryMaximumY_) {
      telemetryMaximumY_ = y;
    }
  }
  telemetryPreviousX_ = x;
  telemetryPreviousY_ = y;
  telemetrySumX_ += x;
  telemetrySumY_ += y;
  if (telemetrySampleCount_ < 65535U) {
    ++telemetrySampleCount_;
  }

  const float meanX =
      static_cast<float>(telemetrySumX_) /
      static_cast<float>(telemetrySampleCount_);
  const float meanY =
      static_cast<float>(telemetrySumY_) /
      static_cast<float>(telemetrySampleCount_);

  debug_.print(
      "[VISION CAL] req/mode/target/n=");
  debug_.print(latest_.requestSequence);
  debug_.print("/");
  debug_.print(latest_.mode);
  debug_.print("/");
  debug_.print(latest_.targetId);
  debug_.print("/");
  debug_.print(telemetrySampleCount_);
  debug_.print(", xy=");
  debug_.print(x);
  debug_.print(",");
  debug_.print(y);
  debug_.print(", center-d=");
  debug_.print(
      x - config::vision_link::IMAGE_CENTER_X);
  debug_.print(",");
  debug_.print(
      y - config::vision_link::IMAGE_CENTER_Y);
  debug_.print(", prev-d=");
  debug_.print(previousDeltaX);
  debug_.print(",");
  debug_.print(previousDeltaY);
  debug_.print(", first-d=");
  debug_.print(x - telemetryFirstX_);
  debug_.print(",");
  debug_.print(y - telemetryFirstY_);
  debug_.print(", span=");
  debug_.print(telemetryMaximumX_ - telemetryMinimumX_);
  debug_.print(",");
  debug_.print(telemetryMaximumY_ - telemetryMinimumY_);
  debug_.print(", mean=");
  debug_.print(meanX, 2);
  debug_.print(",");
  debug_.print(meanY, 2);
  debug_.print(", metric/conf/camera-ms/rx-ms=");
  debug_.print(latest_.metric);
  debug_.print("/");
  debug_.print(latest_.confidence);
  debug_.print("/");
  debug_.print(latest_.cameraTimestampMs);
  debug_.print("/");
  debug_.println(latest_.receivedMs);
}

bool MaixCamClient::responseTargetMatchesMode(
    uint8_t target) const {
  const bool targetedColorMode =
      requestedMode_ >= RED_REQUEST &&
      requestedMode_ <= GREEN_REQUEST;
  return (targetedColorMode &&
          target == requestedMode_) ||
         (requestedMode_ == ALL_COLORS_REQUEST &&
          target >= 1U &&
          target <= 4U) ||
         (requestedMode_ == HOUGH_CIRCLE_REQUEST &&
          target == 2U) ||
         (requestedMode_ == ENDPOINT_CIRCLE_REQUEST &&
          target == 1U);
}

void MaixCamClient::finishCoordinateLine() {
  if (receiveOverflow_) {
    debug_.println(
        "MaixCAM response discarded: line too long");
    receiveLength_ = 0U;
    receiveOverflow_ = false;
    return;
  }

  receiveLine_[receiveLength_] = '\0';
  vision_protocol::VisionResponse response;
  const vision_protocol::ParseError parseError =
      vision_protocol::parseResponse(
          receiveLine_,
          receiveLength_,
          response);
  receiveLength_ = 0U;
  if (parseError != vision_protocol::PARSE_OK) {
    debug_.print("MaixCAM response rejected: ");
    debug_.println(
        vision_protocol::parseErrorText(parseError));
    return;
  }

  if (!modeCommandSent_ ||
      requestedMode_ == STOP_REQUEST ||
      response.sequence != requestSequence_ ||
      response.mode != requestedMode_) {
    debug_.print(
        "MaixCAM stale/mismatched seq/mode received=");
    debug_.print(response.sequence);
    debug_.print("/");
    debug_.print(response.mode);
    debug_.print(", expected=");
    debug_.print(requestSequence_);
    debug_.print("/");
    debug_.println(requestedMode_);
    return;
  }

  if (response.status != vision_protocol::STATUS_OK) {
    debug_.print("MaixCAM status=");
    debug_.println(response.status);
    if (response.status ==
        vision_protocol::STATUS_CAMERA_ERROR) {
      fault("MaixCAM reported camera error");
    }
    return;
  }

  if (!responseTargetMatchesMode(response.target)) {
    debug_.println(
        "MaixCAM target identity does not match mode");
    return;
  }

  latest_.targetId = response.target;
  latest_.requestSequence = response.sequence;
  latest_.mode = response.mode;
  latest_.x = static_cast<int16_t>(response.x);
  latest_.y = static_cast<int16_t>(response.y);
  latest_.metric = response.metric;
  latest_.confidence = response.confidence;
  latest_.cameraTimestampMs = response.timestamp;
  ++latest_.sequence;
  latest_.receivedMs = millis();

  debug_.print("MaixCAM v2 seq/mode/target=");
  debug_.print(response.sequence);
  debug_.print("/");
  debug_.print(response.mode);
  debug_.print("/");
  debug_.print(response.target);
  debug_.print(", xy=");
  debug_.print(response.x);
  debug_.print(",");
  debug_.print(response.y);
  debug_.print(", metric/confidence=");
  debug_.print(response.metric);
  debug_.print("/");
  debug_.println(response.confidence);
  printCoordinateTelemetry();
}

void MaixCamClient::service() {
  while (serial_.available()) {
    const char incoming =
        static_cast<char>(serial_.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      finishCoordinateLine();
      continue;
    }
    if (receiveOverflow_) {
      continue;
    }
    if (receiveLength_ < LINE_CAPACITY - 1U) {
      receiveLine_[receiveLength_++] = incoming;
    } else {
      receiveOverflow_ = true;
      receiveLength_ = 0U;
    }
  }

  if (requestedMode_ == STOP_REQUEST) {
    return;
  }

  const uint32_t nowMs = millis();
  if (!modeCommandSent_) {
    if (nowMs - modeSwitchStartMs_ <
        config::vision_link::MODE_SWITCH_GUARD_MS) {
      return;
    }
    writeRequestFrame(requestedMode_);
    modeCommandSent_ = true;
    lastRequestMs_ = nowMs;
    return;
  }

  if (nowMs - lastRequestMs_ >=
      config::vision_link::REQUEST_REPEAT_MS) {
    writeRequestFrame(requestedMode_);
    lastRequestMs_ = nowMs;
  }
}

bool MaixCamClient::readNewCoordinate(
    uint32_t &lastSequence,
    uint8_t &targetId,
    int16_t &x,
    int16_t &y) const {
  if (latest_.sequence == lastSequence ||
      millis() - latest_.receivedMs >
          config::vision_link::COORDINATE_STALE_MS) {
    return false;
  }
  lastSequence = latest_.sequence;
  targetId = latest_.targetId;
  x = latest_.x;
  y = latest_.y;
  return true;
}

const MaixCamClient::Coordinate &
MaixCamClient::latest() const {
  return latest_;
}

bool MaixCamClient::initialized() const {
  return initialized_;
}

uint8_t MaixCamClient::requestedMode() const {
  return requestedMode_;
}

}
