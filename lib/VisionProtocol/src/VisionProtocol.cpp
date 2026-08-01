#include "VisionProtocol.h"

namespace vision_protocol {
namespace {

constexpr size_t RESPONSE_FIELD_COUNT = 9U;

constexpr size_t MINIMUM_RESPONSE_LENGTH = 6U;

enum NumberParseResult : uint8_t {
  NUMBER_OK = 0U,
  NUMBER_EMPTY,
  NUMBER_NON_NUMERIC,
  NUMBER_OVERFLOW
};

uint8_t crc8Update(uint8_t crc, uint8_t value) {
  crc = static_cast<uint8_t>(crc ^ value);
  for (uint8_t bit = 0U; bit < 8U; ++bit) {
    if ((crc & 0x80U) != 0U) {
      crc = static_cast<uint8_t>((crc << 1U) ^ 0x07U);
    } else {
      crc = static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

bool decodeHex(char character, uint8_t &value) {
  if (character >= '0' && character <= '9') {
    value = static_cast<uint8_t>(character - '0');
    return true;
  }
  if (character >= 'A' && character <= 'F') {
    value = static_cast<uint8_t>(character - 'A' + 10);
    return true;
  }
  if (character >= 'a' && character <= 'f') {
    value = static_cast<uint8_t>(character - 'a' + 10);
    return true;
  }
  return false;
}

NumberParseResult parseUnsignedDecimal(
    const char *begin,
    const char *end,
    uint32_t &value) {
  if (begin == end) {
    return NUMBER_EMPTY;
  }

  const uint32_t maximum =
      static_cast<uint32_t>(~static_cast<uint32_t>(0U));
  uint32_t parsed = 0U;
  for (const char *cursor = begin; cursor != end; ++cursor) {
    if (*cursor < '0' || *cursor > '9') {
      return NUMBER_NON_NUMERIC;
    }
    const uint32_t digit =
        static_cast<uint32_t>(*cursor - '0');
    if (parsed > (maximum - digit) / 10U) {
      return NUMBER_OVERFLOW;
    }
    parsed = parsed * 10U + digit;
  }

  value = parsed;
  return NUMBER_OK;
}

ParseError numberError(NumberParseResult result) {
  if (result == NUMBER_EMPTY) {
    return PARSE_EMPTY_FIELD;
  }
  if (result == NUMBER_NON_NUMERIC) {
    return PARSE_NON_NUMERIC;
  }
  if (result == NUMBER_OVERFLOW) {
    return PARSE_NUMERIC_OVERFLOW;
  }
  return PARSE_OK;
}

}

VisionResponse::VisionResponse()
    : sequence(0U),
      mode(0U),
      status(0U),
      target(0U),
      x(0U),
      y(0U),
      metric(0U),
      confidence(0U),
      timestamp(0U) {}

uint8_t crc8(const uint8_t *data, size_t length) {
  if (data == nullptr && length != 0U) {
    return 0U;
  }

  uint8_t crc = 0U;
  for (size_t index = 0U; index < length; ++index) {
    crc = crc8Update(crc, data[index]);
  }
  return crc;
}

uint8_t crc8(const char *data, size_t length) {
  if (data == nullptr && length != 0U) {
    return 0U;
  }

  uint8_t crc = 0U;
  for (size_t index = 0U; index < length; ++index) {
    crc = crc8Update(
        crc,
        static_cast<uint8_t>(
            static_cast<unsigned char>(data[index])));
  }
  return crc;
}

bool buildRequest(
    uint8_t sequence,
    uint8_t mode,
    uint8_t *frame,
    size_t frameCapacity) {
  if (frame == nullptr || frameCapacity < REQUEST_FRAME_SIZE) {
    return false;
  }

  const uint8_t checksumInput[3] = {
      PROTOCOL_VERSION, sequence, mode};
  frame[0] = REQUEST_START_BYTE;
  frame[1] = PROTOCOL_VERSION;
  frame[2] = sequence;
  frame[3] = mode;
  frame[4] = crc8(checksumInput, 3U);
  frame[5] = REQUEST_END_BYTE;
  return true;
}

ParseError parseResponse(
    const char *line,
    size_t length,
    VisionResponse &response) {
  if (line == nullptr) {
    return PARSE_NULL_INPUT;
  }
  if (length < MINIMUM_RESPONSE_LENGTH) {
    return PARSE_TOO_SHORT;
  }

  const size_t checksumSeparator = length - 3U;
  if (line[checksumSeparator] != '*') {
    return PARSE_MISSING_CHECKSUM;
  }
  for (size_t index = 0U;
       index < checksumSeparator;
       ++index) {
    if (line[index] == '*') {
      return PARSE_INVALID_CHECKSUM_FORMAT;
    }
  }

  uint8_t highNibble = 0U;
  uint8_t lowNibble = 0U;
  if (!decodeHex(line[length - 2U], highNibble) ||
      !decodeHex(line[length - 1U], lowNibble)) {
    return PARSE_INVALID_CHECKSUM_FORMAT;
  }
  const uint8_t suppliedChecksum =
      static_cast<uint8_t>((highNibble << 4U) | lowNibble);
  if (crc8(line, checksumSeparator) != suppliedChecksum) {
    return PARSE_CHECKSUM_MISMATCH;
  }

  if (checksumSeparator < 3U ||
      line[0] != 'V' ||
      line[1] != '2' ||
      line[2] != ',') {
    return PARSE_INVALID_PREFIX;
  }

  uint32_t values[RESPONSE_FIELD_COUNT] = {};
  size_t fieldStart = 3U;
  for (size_t field = 0U;
       field < RESPONSE_FIELD_COUNT;
       ++field) {
    size_t fieldEnd = fieldStart;
    while (fieldEnd < checksumSeparator &&
           line[fieldEnd] != ',') {
      ++fieldEnd;
    }

    if (field < RESPONSE_FIELD_COUNT - 1U) {
      if (fieldEnd == checksumSeparator) {
        return PARSE_WRONG_FIELD_COUNT;
      }
    } else if (fieldEnd != checksumSeparator) {
      return PARSE_WRONG_FIELD_COUNT;
    }

    const NumberParseResult result =
        parseUnsignedDecimal(
            line + fieldStart,
            line + fieldEnd,
            values[field]);
    if (result != NUMBER_OK) {
      return numberError(result);
    }
    fieldStart = fieldEnd + 1U;
  }

  if (values[0] > 255U) {
    return PARSE_SEQUENCE_OUT_OF_RANGE;
  }
  if (values[1] > 255U) {
    return PARSE_MODE_OUT_OF_RANGE;
  }
  if (values[2] > 255U) {
    return PARSE_STATUS_OUT_OF_RANGE;
  }
  if (values[3] > 255U) {
    return PARSE_TARGET_OUT_OF_RANGE;
  }
  if (values[4] > 319U) {
    return PARSE_X_OUT_OF_RANGE;
  }
  if (values[5] > 239U) {
    return PARSE_Y_OUT_OF_RANGE;
  }
  if (values[6] > 65535U) {
    return PARSE_METRIC_OUT_OF_RANGE;
  }
  if (values[7] > 1000U) {
    return PARSE_CONFIDENCE_OUT_OF_RANGE;
  }

  VisionResponse parsed;
  parsed.sequence = static_cast<uint8_t>(values[0]);
  parsed.mode = static_cast<uint8_t>(values[1]);
  parsed.status = static_cast<uint8_t>(values[2]);
  parsed.target = static_cast<uint8_t>(values[3]);
  parsed.x = static_cast<uint16_t>(values[4]);
  parsed.y = static_cast<uint16_t>(values[5]);
  parsed.metric = static_cast<uint16_t>(values[6]);
  parsed.confidence = static_cast<uint16_t>(values[7]);
  parsed.timestamp = values[8];
  response = parsed;
  return PARSE_OK;
}

const char *parseErrorText(ParseError error) {
  switch (error) {
    case PARSE_OK:
      return "ok";
    case PARSE_NULL_INPUT:
      return "null input";
    case PARSE_TOO_SHORT:
      return "response too short";
    case PARSE_INVALID_PREFIX:
      return "expected V2 prefix";
    case PARSE_MISSING_CHECKSUM:
      return "checksum separator is not three bytes from end";
    case PARSE_INVALID_CHECKSUM_FORMAT:
      return "checksum must be one star and two hex digits";
    case PARSE_CHECKSUM_MISMATCH:
      return "checksum mismatch";
    case PARSE_WRONG_FIELD_COUNT:
      return "expected exactly nine numeric fields";
    case PARSE_EMPTY_FIELD:
      return "empty numeric field";
    case PARSE_NON_NUMERIC:
      return "numeric field contains a non-digit";
    case PARSE_NUMERIC_OVERFLOW:
      return "numeric field exceeds uint32";
    case PARSE_SEQUENCE_OUT_OF_RANGE:
      return "sequence must be 0..255";
    case PARSE_MODE_OUT_OF_RANGE:
      return "mode must be 0..255";
    case PARSE_STATUS_OUT_OF_RANGE:
      return "status must be 0..255";
    case PARSE_TARGET_OUT_OF_RANGE:
      return "target must be 0..255";
    case PARSE_X_OUT_OF_RANGE:
      return "x must be 0..319";
    case PARSE_Y_OUT_OF_RANGE:
      return "y must be 0..239";
    case PARSE_METRIC_OUT_OF_RANGE:
      return "metric must be 0..65535";
    case PARSE_CONFIDENCE_OUT_OF_RANGE:
      return "confidence must be 0..1000";
    default:
      return "unknown parse error";
  }
}

}
