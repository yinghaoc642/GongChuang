#include <VisionProtocol.h>

using namespace vision_protocol;

namespace {

#define CHECK(condition) \
  do {                   \
    if (!(condition)) {  \
      return __LINE__;   \
    }                    \
  } while (false)

size_t textLength(const char *text) {
  size_t length = 0U;
  while (text[length] != '\0') {
    ++length;
  }
  return length;
}

char hexDigit(uint8_t value, bool lowerCase) {
  if (value < 10U) {
    return static_cast<char>('0' + value);
  }
  return static_cast<char>(
      (lowerCase ? 'a' : 'A') + value - 10U);
}

size_t makeResponse(
    const char *body,
    char *line,
    size_t capacity,
    bool lowerCase = false) {
  const size_t bodyLength = textLength(body);
  if (capacity < bodyLength + 4U) {
    return 0U;
  }
  for (size_t index = 0U; index < bodyLength; ++index) {
    line[index] = body[index];
  }
  const uint8_t checksum = crc8(body, bodyLength);
  line[bodyLength] = '*';
  line[bodyLength + 1U] =
      hexDigit(static_cast<uint8_t>(checksum >> 4U), lowerCase);
  line[bodyLength + 2U] =
      hexDigit(static_cast<uint8_t>(checksum & 0x0FU), lowerCase);
  line[bodyLength + 3U] = '\0';
  return bodyLength + 3U;
}

int testCrcAndRequest() {
  const uint8_t checkVector[] = {
      '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc8(checkVector, sizeof(checkVector)) == 0xF4U);
  CHECK(crc8(static_cast<const uint8_t *>(nullptr), 0U) == 0U);

  uint8_t frame[REQUEST_FRAME_SIZE] = {};
  CHECK(buildRequest(0x34U, 0x09U, frame, sizeof(frame)));
  CHECK(frame[0] == 0xAAU);
  CHECK(frame[1] == 0x02U);
  CHECK(frame[2] == 0x34U);
  CHECK(frame[3] == 0x09U);
  CHECK(frame[4] == 0x44U);
  CHECK(frame[5] == 0xBBU);

  uint8_t untouched[REQUEST_FRAME_SIZE] = {
      1U, 2U, 3U, 4U, 5U, 6U};
  CHECK(!buildRequest(1U, 2U, untouched, 5U));
  CHECK(untouched[0] == 1U && untouched[5] == 6U);
  CHECK(!buildRequest(1U, 2U, nullptr, REQUEST_FRAME_SIZE));
  return 0;
}

int testValidResponses() {
  char line[96];
  const size_t length = makeResponse(
      "V2,17,9,0,2,319,239,65535,1000,4294967295",
      line,
      sizeof(line));
  CHECK(length != 0U);

  VisionResponse response;
  CHECK(parseResponse(line, length, response) == PARSE_OK);
  CHECK(response.sequence == 17U);
  CHECK(response.mode == 9U);
  CHECK(response.status == STATUS_OK);
  CHECK(response.target == 2U);
  CHECK(response.x == 319U);
  CHECK(response.y == 239U);
  CHECK(response.metric == 65535U);
  CHECK(response.confidence == 1000U);
  CHECK(response.timestamp == 4294967295UL);

  const size_t upperLength = makeResponse(
      "V2,1,2,4,255,0,0,0,0,12345",
      line,
      sizeof(line));
  CHECK(upperLength != 0U);
  CHECK(line[upperLength - 2U] == 'A');
  CHECK(parseResponse(line, upperLength, response) == PARSE_OK);
  CHECK(response.status == STATUS_CAMERA_ERROR);
  CHECK(response.target == 255U);

  const size_t lowerLength = makeResponse(
      "V2,1,2,4,255,0,0,0,0,12345",
      line,
      sizeof(line),
      true);
  CHECK(lowerLength != 0U);
  CHECK(line[lowerLength - 2U] == 'a');
  CHECK(parseResponse(line, lowerLength, response) == PARSE_OK);
  CHECK(response.status == STATUS_CAMERA_ERROR);
  CHECK(response.target == 255U);

  const size_t unknownStatusLength = makeResponse(
      "V2,255,255,255,255,0,0,0,0,0",
      line,
      sizeof(line));
  CHECK(parseResponse(
            line, unknownStatusLength, response) == PARSE_OK);
  CHECK(response.status == 255U);
  return 0;
}

ParseError parseBody(const char *body) {
  char line[128];
  const size_t length =
      makeResponse(body, line, sizeof(line));
  VisionResponse response;
  return parseResponse(line, length, response);
}

int testChecksumAndFramingErrors() {
  char line[96];
  size_t length = makeResponse(
      "V2,7,9,0,2,160,120,42,900,123456",
      line,
      sizeof(line));
  CHECK(length != 0U);

  line[3] = '8';
  VisionResponse response;
  CHECK(parseResponse(
            line, length, response) ==
        PARSE_CHECKSUM_MISMATCH);

  length = makeResponse(
      "V2,7,9,0,2,160,120,42,900,123456",
      line,
      sizeof(line));
  CHECK(parseResponse(
            line, length - 1U, response) ==
        PARSE_MISSING_CHECKSUM);

  line[length - 1U] = 'G';
  CHECK(parseResponse(
            line, length, response) ==
        PARSE_INVALID_CHECKSUM_FORMAT);

  length = makeResponse(
      "V2,7,9,0,2,160,120,42,900,123456",
      line,
      sizeof(line));
  line[length] = '\n';
  CHECK(parseResponse(
            line, length + 1U, response) ==
        PARSE_MISSING_CHECKSUM);

  CHECK(parseResponse("V2", 2U, response) == PARSE_TOO_SHORT);
  CHECK(parseResponse(nullptr, 25U, response) == PARSE_NULL_INPUT);
  return 0;
}

int testFieldSyntaxErrors() {
  CHECK(parseBody(
            "V2,1,2,0,3,4,5,6,7,8,9") ==
        PARSE_WRONG_FIELD_COUNT);
  CHECK(parseBody(
            "V2,1,2,0,3,4,5,6,7") ==
        PARSE_WRONG_FIELD_COUNT);
  CHECK(parseBody(
            "V2,1,2,0,3,,5,6,7,8") ==
        PARSE_EMPTY_FIELD);
  CHECK(parseBody(
            "V2,1,2,0,3,4x,5,6,7,8") ==
        PARSE_NON_NUMERIC);
  CHECK(parseBody(
            "V2,1,2,0,3,-4,5,6,7,8") ==
        PARSE_NON_NUMERIC);
  CHECK(parseBody(
            "V1,1,2,0,3,4,5,6,7,8") ==
        PARSE_INVALID_PREFIX);
  CHECK(parseBody(
            "V2,1,2,0,3,4,5,6,7,4294967296") ==
        PARSE_NUMERIC_OVERFLOW);
  return 0;
}

int testRangeErrors() {
  CHECK(parseBody(
            "V2,256,2,0,3,4,5,6,7,8") ==
        PARSE_SEQUENCE_OUT_OF_RANGE);
  CHECK(parseBody(
            "V2,1,256,0,3,4,5,6,7,8") ==
        PARSE_MODE_OUT_OF_RANGE);
  CHECK(parseBody(
            "V2,1,2,256,3,4,5,6,7,8") ==
        PARSE_STATUS_OUT_OF_RANGE);
  CHECK(parseBody(
            "V2,1,2,0,256,4,5,6,7,8") ==
        PARSE_TARGET_OUT_OF_RANGE);
  CHECK(parseBody(
            "V2,1,2,0,3,320,5,6,7,8") ==
        PARSE_X_OUT_OF_RANGE);
  CHECK(parseBody(
            "V2,1,2,0,3,4,240,6,7,8") ==
        PARSE_Y_OUT_OF_RANGE);
  CHECK(parseBody(
            "V2,1,2,0,3,4,5,65536,7,8") ==
        PARSE_METRIC_OUT_OF_RANGE);
  CHECK(parseBody(
            "V2,1,2,0,3,4,5,6,1001,8") ==
        PARSE_CONFIDENCE_OUT_OF_RANGE);
  return 0;
}

int testOutputIsTransactional() {
  VisionResponse response;
  response.sequence = 42U;
  response.timestamp = 123U;
  char line[64];
  const size_t length = makeResponse(
      "V2,1,2,0,3,320,5,6,7,8",
      line,
      sizeof(line));
  CHECK(parseResponse(
            line, length, response) == PARSE_X_OUT_OF_RANGE);
  CHECK(response.sequence == 42U);
  CHECK(response.timestamp == 123U);
  CHECK(parseErrorText(PARSE_X_OUT_OF_RANGE)[0] == 'x');
  return 0;
}

} // namespace

int main() {
  int result = testCrcAndRequest();
  if (result != 0) {
    return result;
  }
  result = testValidResponses();
  if (result != 0) {
    return result;
  }
  result = testChecksumAndFramingErrors();
  if (result != 0) {
    return result;
  }
  result = testFieldSyntaxErrors();
  if (result != 0) {
    return result;
  }
  result = testRangeErrors();
  if (result != 0) {
    return result;
  }
  return testOutputIsTransactional();
}
