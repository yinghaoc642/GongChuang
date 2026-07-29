/*
 * GongChuang STM32H750 <-> MaixCAM communication demo
 *
 * Request protocol (binary, three bytes):
 *   AA 01 BB : request red detection
 *   AA 02 BB : request yellow detection
 *   AA 03 BB : request blue detection
 *   AA 04 BB : request green detection
 *   AA 07 BB : request circle detection
 *
 * Reply protocol from vision-2.py (ASCII):
 *   x,y\n
 *
 * The board sends requests 1, 2, 3, 4 and 7 in turn, one every two seconds.
 * When a valid reply is received, LED4 flashes the request number of times.
 */

#include <Arduino.h>
#include <stdlib.h>

namespace {

// STM32 UART7: PE7/RX receives from MaixCAM A16/TX,
// and PE8/TX sends to MaixCAM A17/RX.
constexpr uint8_t MAIX_RX_PIN = PE7;
constexpr uint8_t MAIX_TX_PIN = PE8;
constexpr uint32_t MAIX_BAUDRATE = 115200UL;

// Same LED pin and active level as examples/LED.ino.
constexpr uint8_t LED4_PIN = PB4;
constexpr uint8_t LED_ON_LEVEL = HIGH;
constexpr uint8_t LED_OFF_LEVEL = LOW;

constexpr uint8_t FRAME_HEADER = 0xAA;
constexpr uint8_t FRAME_TAIL = 0xBB;
constexpr uint8_t REQUEST_SEQUENCE[] = {1U, 2U, 3U, 4U, 7U};
constexpr size_t REQUEST_COUNT =
    sizeof(REQUEST_SEQUENCE) / sizeof(REQUEST_SEQUENCE[0]);

constexpr uint32_t REQUEST_INTERVAL_MS = 2000UL;

// Seven flashes finish in about 780 ms: 13 ON/OFF transitions x 60 ms.
constexpr uint32_t BLINK_PHASE_MS = 60UL;

constexpr size_t RX_LINE_CAPACITY = 32U;
constexpr long IMAGE_MAX_X = 319L;
constexpr long IMAGE_MAX_Y = 239L;

HardwareSerial Serial_Maix(MAIX_RX_PIN, MAIX_TX_PIN);

char rxLine[RX_LINE_CAPACITY] = {};
size_t rxLength = 0U;

size_t nextRequestIndex = 0U;
uint8_t pendingRequest = 0U;
bool replyHandledForPendingRequest = false;
uint32_t nextRequestAtMs = 0UL;

struct BlinkState {
  bool active;
  bool ledOn;
  uint8_t targetFlashes;
  uint8_t completedFlashes;
  uint32_t lastTransitionMs;
};

BlinkState blink = {
    false,
    false,
    0U,
    0U,
    0UL,
};

void setLed(bool on) {
  digitalWrite(
      LED4_PIN,
      on ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

bool isSupportedRequest(uint8_t request) {
  for (size_t i = 0U; i < REQUEST_COUNT; ++i) {
    if (REQUEST_SEQUENCE[i] == request) {
      return true;
    }
  }
  return false;
}

void startBlink(uint8_t flashCount) {
  if (!isSupportedRequest(flashCount)) {
    return;
  }

  blink.active = true;
  blink.ledOn = true;
  blink.targetFlashes = flashCount;
  blink.completedFlashes = 0U;
  blink.lastTransitionMs = millis();
  setLed(true);
}

void serviceBlink() {
  if (!blink.active) {
    return;
  }

  const uint32_t now = millis();
  if (now - blink.lastTransitionMs < BLINK_PHASE_MS) {
    return;
  }

  blink.lastTransitionMs = now;

  if (blink.ledOn) {
    setLed(false);
    blink.ledOn = false;
    ++blink.completedFlashes;

    if (blink.completedFlashes >= blink.targetFlashes) {
      blink.active = false;
    }
    return;
  }

  setLed(true);
  blink.ledOn = true;
}

bool parseCoordinateLine(
    const char *line, long &x, long &y) {
  if (line == nullptr || line[0] == '\0') {
    return false;
  }

  char *xEnd = nullptr;
  x = strtol(line, &xEnd, 10);
  if (xEnd == line || xEnd == nullptr || *xEnd != ',') {
    return false;
  }

  const char *yStart = xEnd + 1;
  char *yEnd = nullptr;
  y = strtol(yStart, &yEnd, 10);
  if (yEnd == yStart || yEnd == nullptr || *yEnd != '\0') {
    return false;
  }

  return x >= 0L && x <= IMAGE_MAX_X &&
         y >= 0L && y <= IMAGE_MAX_Y;
}

void handleReplyLine(const char *line) {
  long x = 0L;
  long y = 0L;

  if (!parseCoordinateLine(line, x, y)) {
    Serial.print("Ignored invalid Maix reply: ");
    Serial.println(line);
    return;
  }

  if (pendingRequest == 0U ||
      replyHandledForPendingRequest) {
    return;
  }

  replyHandledForPendingRequest = true;
  startBlink(pendingRequest);

  Serial.print("Reply for request ");
  Serial.print(pendingRequest);
  Serial.print(": x=");
  Serial.print(x);
  Serial.print(", y=");
  Serial.println(y);
}

void serviceMaixReceiver() {
  while (Serial_Maix.available() > 0) {
    const char incoming =
        static_cast<char>(Serial_Maix.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      rxLine[rxLength] = '\0';
      handleReplyLine(rxLine);
      rxLength = 0U;
      continue;
    }

    if (rxLength < RX_LINE_CAPACITY - 1U) {
      rxLine[rxLength++] = incoming;
    } else {
      // Discard an oversized or malformed line safely.
      rxLength = 0U;
    }
  }
}

void sendNextRequest() {
  // Do not join an incomplete old reply to the next request.
  rxLength = 0U;

  const uint8_t request = REQUEST_SEQUENCE[nextRequestIndex];
  const uint8_t frame[3] = {
      FRAME_HEADER,
      request,
      FRAME_TAIL,
  };

  Serial_Maix.write(frame, sizeof(frame));
  Serial_Maix.flush();

  pendingRequest = request;
  replyHandledForPendingRequest = false;

  Serial.print("Sent request ");
  Serial.println(pendingRequest);

  ++nextRequestIndex;
  if (nextRequestIndex >= REQUEST_COUNT) {
    nextRequestIndex = 0U;
  }
}

void serviceRequestScheduler() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextRequestAtMs) < 0) {
    return;
  }

  sendNextRequest();

  // Keep a stable one-second cadence without accumulating normal loop delay.
  nextRequestAtMs += REQUEST_INTERVAL_MS;
  if (static_cast<int32_t>(now - nextRequestAtMs) >= 0) {
    nextRequestAtMs = now + REQUEST_INTERVAL_MS;
  }
}

} // namespace

void setup() {
  pinMode(LED4_PIN, OUTPUT);
  setLed(false);

  Serial.begin(115200);
  Serial_Maix.begin(MAIX_BAUDRATE);

  while (Serial_Maix.available() > 0) {
    Serial_Maix.read();
  }

  nextRequestAtMs = millis() + REQUEST_INTERVAL_MS;

  Serial.println("Maix request/LED4 demo ready.");
  Serial.println(
      "Requests 1, 2, 3, 4 and 7 will be sent once every two seconds.");
}

void loop() {
  // Receive before sending the next request so a reply that arrived near the
  // one-second boundary is still associated with the previous request.
  serviceMaixReceiver();
  serviceBlink();
  serviceRequestScheduler();
}
