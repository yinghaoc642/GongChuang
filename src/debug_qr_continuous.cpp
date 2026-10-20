// Independent QR bench firmware. Build ONLY with the debug_qr_continuous env.
// PB9 starts PE0/PE1 reception; no motor, servo, camera, HMI or IMU is set up.
#include <Arduino.h>
#include <CompetitionCore.h>
#include <OneButton.h>

namespace {
HardwareSerial debugPort(PB12, PB13);  // RX, TX, 115200 baud
HardwareSerial qrPort(PE0, PE1);       // RX, TX, 9600 baud
OneButton startButton(PB9, true, true); // Active low, internal pull-up
constexpr size_t kFrameCapacity = 32U;
char frame[kFrameCapacity] = {};
size_t frameLength = 0U;
bool frameOverflow = false;
bool scanning = false;
uint32_t frameCount = 0UL;

void startScanning() {
  if (scanning) {
    return;
  }
  qrPort.begin(9600UL);
  // Ignore bytes accumulated while the operator was preparing the scanner.
  while (qrPort.available()) {
    qrPort.read();
  }
  frameLength = 0U;
  frameOverflow = false;
  scanning = true;
  debugPort.println("[QR DEBUG] PB9 pressed; continuous scanning started");
}

void finishFrame() {
  if (frameLength == 0U && !frameOverflow) {
    return; // CRLF is one terminator, not two frames.
  }
  frame[frameLength] = '\0';
  competition::TaskPlan parsed;
  const competition::TaskCodeStatus status = frameOverflow
      ? competition::TASK_CODE_WRONG_LENGTH
      : competition::parseTaskCode(frame, parsed);
  debugPort.print("[QR DEBUG] frame=");
  debugPort.print(++frameCount);
  debugPort.print(" raw=");
  debugPort.print(frame);
  debugPort.print(" len=");
  debugPort.print(static_cast<unsigned int>(frameLength));
  debugPort.print(" status=");
  debugPort.print(static_cast<unsigned int>(status));
  debugPort.print(" valid=");
  debugPort.println(status == competition::TASK_CODE_OK ? 1 : 0);
  // Do not latch a valid code: every subsequent scan is reported.
  frameLength = 0U;
  frameOverflow = false;
}
} // namespace

void setup() {
  debugPort.begin(115200UL);
  startButton.reset();
  startButton.attachClick(startScanning);
  debugPort.println("[QR DEBUG] ready; click PB9 to scan continuously");
}

void loop() {
  startButton.tick();
  if (!scanning) {
    return;
  }
  while (qrPort.available()) {
    const char incoming = static_cast<char>(qrPort.read());
    if (incoming == '\r' || incoming == '\n') {
      finishFrame();
    } else if (frameLength < kFrameCapacity - 1U) {
      frame[frameLength++] = incoming;
    } else {
      frameOverflow = true;
    }
  }
}
