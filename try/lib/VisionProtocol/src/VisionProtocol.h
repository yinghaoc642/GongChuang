#ifndef GONGCHUANG_VISION_PROTOCOL_H
#define GONGCHUANG_VISION_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

namespace vision_protocol {

constexpr size_t REQUEST_FRAME_SIZE = 6U;
constexpr uint8_t REQUEST_START_BYTE = 0xAAU;
constexpr uint8_t PROTOCOL_VERSION = 0x02U;
constexpr uint8_t REQUEST_END_BYTE = 0xBBU;

enum VisionStatus : uint8_t {
  STATUS_OK = 0U,
  STATUS_NO_TARGET = 1U,
  STATUS_AMBIGUOUS = 2U,
  STATUS_UNSTABLE = 3U,
  STATUS_CAMERA_ERROR = 4U
};

struct VisionResponse {
  uint8_t sequence;
  uint8_t mode;
  uint8_t status;
  uint8_t target;
  uint16_t x;
  uint16_t y;
  uint16_t metric;
  uint16_t confidence;
  uint32_t timestamp;

  VisionResponse();
};

enum ParseError : uint8_t {
  PARSE_OK = 0U,
  PARSE_NULL_INPUT,
  PARSE_TOO_SHORT,
  PARSE_INVALID_PREFIX,
  PARSE_MISSING_CHECKSUM,
  PARSE_INVALID_CHECKSUM_FORMAT,
  PARSE_CHECKSUM_MISMATCH,
  PARSE_WRONG_FIELD_COUNT,
  PARSE_EMPTY_FIELD,
  PARSE_NON_NUMERIC,
  PARSE_NUMERIC_OVERFLOW,
  PARSE_SEQUENCE_OUT_OF_RANGE,
  PARSE_MODE_OUT_OF_RANGE,
  PARSE_STATUS_OUT_OF_RANGE,
  PARSE_TARGET_OUT_OF_RANGE,
  PARSE_X_OUT_OF_RANGE,
  PARSE_Y_OUT_OF_RANGE,
  PARSE_METRIC_OUT_OF_RANGE,
  PARSE_CONFIDENCE_OUT_OF_RANGE
};

uint8_t crc8(const uint8_t *data, size_t length);
uint8_t crc8(const char *data, size_t length);

bool buildRequest(
    uint8_t sequence,
    uint8_t mode,
    uint8_t *frame,
    size_t frameCapacity);

ParseError parseResponse(
    const char *line,
    size_t length,
    VisionResponse &response);

const char *parseErrorText(ParseError error);

}

#endif
