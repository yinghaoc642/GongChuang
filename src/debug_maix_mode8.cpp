/*
 * MaixCAM 模式8直连测试固件。
 *
 * 仅使用：
 *   - PB12/PB13：调试串口，115200 baud
 *   - PE7/PE8：MaixCAM RX/TX，115200 baud
 *
 * 本程序不调用 MaixCamClient，因为该客户端会先发送模式0停止帧。
 * 这里直接复用正式 VisionProtocol 构造请求，线上只会出现模式8请求：
 *   AA 02 <sequence> 08 <crc8> BB
 *
 * 同一序号每秒重发，不会反复清空 MaixCAM 的稳定计数。收到对应响应后，
 * 等待1秒再换新序号继续测试；5秒没有响应也换新序号，以避开相机仍记着
 * 上一次已完成序号的情况。程序不初始化底盘、机械臂、舵机、扫码器、
 * HMI 或 IMU。
 */

#include <Arduino.h>
#include <VisionProtocol.h>

namespace {

HardwareSerial debugPort(PB12, PB13);  // RX, TX
HardwareSerial maixPort(PE7, PE8);      // RX, TX

constexpr uint32_t kBaudrate = 115200UL;
constexpr uint8_t kMode = 0x08U;
constexpr uint32_t kRepeatIntervalMs = 1000UL;
constexpr uint32_t kSequenceTimeoutMs = 5000UL;
constexpr uint32_t kNextRequestDelayMs = 1000UL;
constexpr size_t kResponseCapacity = 192U;

uint8_t requestSequence = 1U;
uint32_t sequenceStartMs = 0UL;
uint32_t lastTransmitMs = 0UL;
uint32_t responseReceivedMs = 0UL;
uint32_t receivedByteCount = 0UL;
uint32_t lastReportedByteCount = 0UL;
uint32_t lastActivityReportMs = 0UL;
bool requestActive = false;
bool matchingResponseReceived = false;
bool receiveOverflow = false;
char responseLine[kResponseCapacity] = {};
size_t responseLength = 0U;

void printHexByte(uint8_t value) {
  if (value < 0x10U) {
    debugPort.print('0');
  }
  debugPort.print(value, HEX);
}

void sendMode8Request() {
  uint8_t frame[vision_protocol::REQUEST_FRAME_SIZE] = {};
  if (!vision_protocol::buildRequest(
          requestSequence,
          kMode,
          frame,
          sizeof(frame))) {
    debugPort.println("[MODE8 FAULT] request frame build failed");
    return;
  }

  const size_t queued = maixPort.write(frame, sizeof(frame));
  lastTransmitMs = millis();

  debugPort.print("[MODE8 TX] t=");
  debugPort.print(lastTransmitMs);
  debugPort.print(" ms seq=");
  debugPort.print(requestSequence);
  debugPort.print(" mode=8 queued=");
  debugPort.print(static_cast<unsigned int>(queued));
  debugPort.print(" frame=");
  for (size_t index = 0U; index < sizeof(frame); ++index) {
    if (index != 0U) {
      debugPort.print(' ');
    }
    printHexByte(frame[index]);
  }
  debugPort.println();
}

void startNewMode8Sequence(const char *reason) {
  if (requestActive) {
    requestSequence = static_cast<uint8_t>(requestSequence + 1U);
    if (requestSequence == 0U) {
      requestSequence = 1U;
    }
  }

  requestActive = true;
  matchingResponseReceived = false;
  responseReceivedMs = 0UL;
  sequenceStartMs = millis();
  lastTransmitMs = sequenceStartMs - kRepeatIntervalMs;

  debugPort.print("[MODE8 SEQUENCE] reason=");
  debugPort.print(reason);
  debugPort.print(" seq=");
  debugPort.println(requestSequence);
  sendMode8Request();
}

void finishResponseLine() {
  if (responseLength == 0U && !receiveOverflow) {
    return;
  }

  if (receiveOverflow) {
    debugPort.print("[MODE8 RX LINE] overflow; capacity=");
    debugPort.println(static_cast<unsigned int>(kResponseCapacity));
    responseLength = 0U;
    receiveOverflow = false;
    return;
  }

  responseLine[responseLength] = '\0';
  debugPort.print("[MODE8 RX LINE] t=");
  debugPort.print(millis());
  debugPort.print(" ms len=");
  debugPort.print(static_cast<unsigned int>(responseLength));
  debugPort.print(" raw=");
  debugPort.println(responseLine);

  vision_protocol::VisionResponse response;
  const vision_protocol::ParseError error =
      vision_protocol::parseResponse(
          responseLine,
          responseLength,
          response);
  if (error != vision_protocol::PARSE_OK) {
    debugPort.print("[MODE8 RX REJECT] ");
    debugPort.println(vision_protocol::parseErrorText(error));
  } else {
    debugPort.print(
        "[MODE8 RX PARSED] seq/mode/status/target/x/y/metric/confidence/camera_ms=");
    debugPort.print(response.sequence);
    debugPort.print('/');
    debugPort.print(response.mode);
    debugPort.print('/');
    debugPort.print(response.status);
    debugPort.print('/');
    debugPort.print(response.target);
    debugPort.print('/');
    debugPort.print(response.x);
    debugPort.print('/');
    debugPort.print(response.y);
    debugPort.print('/');
    debugPort.print(response.metric);
    debugPort.print('/');
    debugPort.print(response.confidence);
    debugPort.print('/');
    debugPort.println(response.timestamp);

    if (response.sequence == requestSequence &&
        response.mode == kMode) {
      matchingResponseReceived = true;
      responseReceivedMs = millis();
      debugPort.println(
          "[MODE8 RX MATCH] current mode8 request received successfully");
    } else {
      debugPort.print("[MODE8 RX MISMATCH] expected seq/mode=");
      debugPort.print(requestSequence);
      debugPort.println("/8");
    }
  }

  responseLength = 0U;
  receiveOverflow = false;
}

void serviceMaixReceive() {
  while (maixPort.available()) {
    const uint8_t incoming =
        static_cast<uint8_t>(maixPort.read());
    ++receivedByteCount;

    if (incoming == '\n') {
      finishResponseLine();
    } else if (incoming != '\r') {
      if (responseLength < kResponseCapacity - 1U) {
        responseLine[responseLength++] =
            static_cast<char>(incoming);
      } else {
        receiveOverflow = true;
      }
    }
  }

  const uint32_t nowMs = millis();
  if (receivedByteCount != lastReportedByteCount &&
      nowMs - lastActivityReportMs >= 500UL) {
    debugPort.print("[MODE8 RX ACTIVITY] total-bytes=");
    debugPort.print(receivedByteCount);
    debugPort.print(" partial-line-bytes=");
    debugPort.println(static_cast<unsigned int>(responseLength));
    lastReportedByteCount = receivedByteCount;
    lastActivityReportMs = nowMs;
  }
}

void serviceMode8Request() {
  const uint32_t nowMs = millis();
  if (matchingResponseReceived) {
    if (nowMs - responseReceivedMs >= kNextRequestDelayMs) {
      startNewMode8Sequence("previous response complete");
    }
    return;
  }

  if (nowMs - sequenceStartMs >= kSequenceTimeoutMs) {
    startNewMode8Sequence("5 s without matching response");
    return;
  }

  if (nowMs - lastTransmitMs >= kRepeatIntervalMs) {
    sendMode8Request();
  }
}

}  // namespace

void setup() {
  debugPort.begin(kBaudrate);
  maixPort.begin(kBaudrate);

  while (maixPort.available()) {
    maixPort.read();
  }

  debugPort.println(
      "[MODE8 DEBUG] standalone; only mode8 frames; no mode0 and no other hardware");
  debugPort.println(
      "[MODE8 DEBUG] debug=PB12/PB13, MaixCAM=PE7/PE8, baud=115200");
  startNewMode8Sequence("power-up");
}

void loop() {
  serviceMaixReceive();
  serviceMode8Request();
}
