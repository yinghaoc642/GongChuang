/*
 * MaixCAM formal-protocol cycle diagnostic.
 *
 * This deliberately uses the production MaixCamClient instead of writing
 * request bytes directly.  It reproduces the observed RAW request order:
 *
 *   STOP -> mode 3 (blue) -> STOP -> mode 4 (green)
 *        -> STOP -> mode 1 (red) -> repeat
 *
 * No chassis, arm, servo, QR, HMI or IMU hardware is initialized.  The test
 * starts automatically after reset; PB9 is not required.  Keep blue, green
 * and red targets visible so every stage can complete.
 */

#include <Arduino.h>
#include <MaixCamClient.h>

namespace {

HardwareSerial debugPort(PB12, PB13);  // RX, TX
HardwareSerial maixPort(PE7, PE8);      // RX, TX

constexpr uint32_t kBaudrate = 115200UL;
constexpr uint32_t kBootDelayMs = 1000UL;
constexpr uint32_t kRequestRefreshMs = 5000UL;

constexpr uint8_t kModes[] = {
    gongchuang::MaixCamClient::BLUE_REQUEST,
    gongchuang::MaixCamClient::GREEN_REQUEST,
    gongchuang::MaixCamClient::RED_REQUEST,
};

// Match the approximate quiet intervals measured in the failing formal run:
// blue->green about 4.2 s and green->red about 3.7 s.  The final pause makes
// consecutive diagnostic cycles easy to distinguish in the serial log.
constexpr uint32_t kPostResponsePauseMs[] = {
    4200UL,
    3700UL,
    1000UL,
};

static_assert(
    sizeof(kModes) / sizeof(kModes[0]) ==
        sizeof(kPostResponsePauseMs) /
            sizeof(kPostResponsePauseMs[0]),
    "Every protocol-cycle mode needs a post-response pause");

enum CyclePhase : uint8_t {
  CYCLE_BOOT_WAIT,
  CYCLE_REQUEST_ACTIVE,
  CYCLE_STOPPED_WAIT,
  CYCLE_FAULT
};

bool clientFaulted = false;
uint8_t activeModeIndex = 0U;
uint32_t acceptedCoordinateSequence = 0UL;
uint32_t phaseStartMs = 0UL;
uint32_t requestStartMs = 0UL;
uint32_t cycleCount = 0UL;
uint16_t requestRefreshCount = 0U;
CyclePhase cyclePhase = CYCLE_BOOT_WAIT;

void handleClientFault(const char *reason) {
  clientFaulted = true;
  cyclePhase = CYCLE_FAULT;
  debugPort.print("[PROTO CYCLE FAULT] ");
  debugPort.println(reason != nullptr ? reason : "unknown");
}

gongchuang::MaixCamClient maixClient(
    maixPort,
    debugPort,
    handleClientFault);

const char *modeName(uint8_t mode) {
  switch (mode) {
    case gongchuang::MaixCamClient::BLUE_REQUEST:
      return "blue";
    case gongchuang::MaixCamClient::GREEN_REQUEST:
      return "green";
    case gongchuang::MaixCamClient::RED_REQUEST:
      return "red";
    default:
      return "unknown";
  }
}

void startCurrentRequest(const char *reason) {
  const uint8_t mode = kModes[activeModeIndex];
  debugPort.print("[PROTO CYCLE REQUEST] t=");
  debugPort.print(millis());
  debugPort.print(" ms cycle/index/mode/name/reason=");
  debugPort.print(cycleCount);
  debugPort.print('/');
  debugPort.print(static_cast<unsigned int>(activeModeIndex));
  debugPort.print('/');
  debugPort.print(static_cast<unsigned int>(mode));
  debugPort.print('/');
  debugPort.print(modeName(mode));
  debugPort.print('/');
  debugPort.println(reason);

  if (!maixClient.beginRequest(mode)) {
    handleClientFault("beginRequest rejected the diagnostic mode");
    return;
  }
  requestStartMs = millis();
  cyclePhase = CYCLE_REQUEST_ACTIVE;
}

void acceptCurrentResponse(
    uint8_t targetId,
    int16_t x,
    int16_t y) {
  const uint8_t mode = kModes[activeModeIndex];
  debugPort.print("[PROTO CYCLE MATCH] t=");
  debugPort.print(millis());
  debugPort.print(" ms cycle/index/mode/target/xy/refreshes=");
  debugPort.print(cycleCount);
  debugPort.print('/');
  debugPort.print(static_cast<unsigned int>(activeModeIndex));
  debugPort.print('/');
  debugPort.print(static_cast<unsigned int>(mode));
  debugPort.print('/');
  debugPort.print(static_cast<unsigned int>(targetId));
  debugPort.print('/');
  debugPort.print(x);
  debugPort.print(',');
  debugPort.print(y);
  debugPort.print('/');
  debugPort.println(requestRefreshCount);

  // This STOP is the same production client call used after a RAW target is
  // accepted.  beginRequest() will send a second STOP before the next mode,
  // matching the formal RAW handoff sequence.
  maixClient.stopRequest();
  phaseStartMs = millis();
  requestRefreshCount = 0U;
  cyclePhase = CYCLE_STOPPED_WAIT;
}

void advanceMode() {
  ++activeModeIndex;
  if (activeModeIndex >= sizeof(kModes) / sizeof(kModes[0])) {
    activeModeIndex = 0U;
    ++cycleCount;
    debugPort.print("[PROTO CYCLE COMPLETE] cycles=");
    debugPort.println(cycleCount);
  }
  startCurrentRequest("post-STOP pause complete");
}

}  // namespace

void setup() {
  debugPort.begin(kBaudrate);
  maixClient.begin(kBaudrate);
  phaseStartMs = millis();

  debugPort.println(
      "[PROTO CYCLE] auto-start; no PB9; chassis/arm/servo/QR/HMI/IMU disabled");
  debugPort.println(
      "[PROTO CYCLE] production MaixCamClient sequence: STOP-3-STOP-4-STOP-1");
  debugPort.println(
      "[PROTO CYCLE] STM32 PE8 TX -> Maix A17 RX; Maix A16 TX -> STM32 PE7 RX");
}

void loop() {
  maixClient.service();
  if (clientFaulted || cyclePhase == CYCLE_FAULT) {
    return;
  }

  const uint32_t nowMs = millis();
  switch (cyclePhase) {
    case CYCLE_BOOT_WAIT:
      if (nowMs - phaseStartMs >= kBootDelayMs) {
        startCurrentRequest("boot delay complete");
      }
      break;

    case CYCLE_REQUEST_ACTIVE: {
      uint8_t targetId = 0U;
      int16_t x = 0;
      int16_t y = 0;
      if (maixClient.readNewCoordinate(
              acceptedCoordinateSequence,
              targetId,
              x,
              y)) {
        acceptCurrentResponse(targetId, x, y);
        break;
      }

      if (nowMs - requestStartMs >= kRequestRefreshMs) {
        if (requestRefreshCount < UINT16_MAX) {
          ++requestRefreshCount;
        }
        startCurrentRequest("5 s without matching response; new sequence");
      }
      break;
    }

    case CYCLE_STOPPED_WAIT:
      if (nowMs - phaseStartMs >=
          kPostResponsePauseMs[activeModeIndex]) {
        advanceMode();
      }
      break;

    case CYCLE_FAULT:
      break;
  }
}
