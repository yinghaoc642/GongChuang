// Include after route_chassis, outside its namespace. Shared by production and
// chassis-only grid debug. No arm, tray or workstation hardware is initialized.
namespace radar_navigation {
namespace rc = route_chassis;
namespace cfg = radar_navigation_config;
gongchuang::Stm32HardwareSerial serial(PC11, PC10); // RX, TX: UART4
qiming::GridClient client;
qiming::GridMission mission(client, cfg::geometry());
bool active = false, motionActive = false, turning = false;
uint32_t stoppedFrame = 0, stoppedStart = 0;
uint8_t stoppedSamples = 0;
qiming::Point motionEnd{};

bool write(void *, const uint8_t *bytes, size_t length) {
  // Public requests are <=20 bytes. Never wait for space while stepping motors.
  if (serial.availableForWrite() < static_cast<int>(length)) return false;
  return serial.write(bytes, length) == length;
}
void setup() { serial.begin(cfg::BAUD); }
void stop() {
  client.stop(); active = motionActive = turning = false;
}
void begin(uint8_t zone) {
  while (serial.available()) serial.read();
  active = true; motionActive = turning = false;
  mission.begin(zone,true);
  client.begin(write, nullptr, millis(), zone, cfg::NODE_ID);
}
bool serviceLink() {
  if (!active) return true;
  uint8_t bytes[64]; size_t count = 0;
  while (serial.available() && count < sizeof bytes) bytes[count++] = serial.read();
  if (count) client.receive(bytes, count, millis());
  client.poll(millis());
  return client.error() == qiming::GridClient::OK;
}
void startMotion(const qiming::NavAction &action) {
  rc::stopAllMotorsImmediately();
  rc::commandStartMs = millis();
  // Mission headings include the 0/60/120 scan stops and return to zero. Keep the same
  // reference for the shared IMU controller and all workstation corrections.
  rc::targetCounterClockwiseHeadingDegrees = mission.heading();
  motionActive = true; stoppedSamples = 0; stoppedStart = 0;
  motionEnd = mission.position();
  // A zero nominal turn is a heading lock. Where a full sweep does not fit,
  // use the edge stability gate instead of allowing residual turn pulses.
  turning = action.kind == qiming::NavAction::TURN &&
      (fabsf(action.degrees)>0.01f || mission.geometry.rotationSafe(motionEnd));
  rc::SerialDebug.print("[GRID] f/r/ccw=");
  rc::SerialDebug.print(action.forward,1);rc::SerialDebug.print('/');
  rc::SerialDebug.print(action.right,1);rc::SerialDebug.print('/');
  rc::SerialDebug.println(action.degrees,1);
  if (turning) {
    rc::startTurn(action.degrees, 1.0f, 0.5f);
  } else {
    float dx,dy;
    qiming::bodyToWorld(action.forward,action.right,mission.heading(),dx,dy);
    motionEnd.x+=dx;motionEnd.y+=dy;
    rc::setDriveMotionProfile(
        cfg::MOVE_MAX_MM_S * rc::FORWARD_PULSES_PER_METER / 1000.0f,
        cfg::MOVE_ACCEL_MM_S2 * rc::FORWARD_PULSES_PER_METER / 1000.0f);
    rc::startVehicleDisplacement(action.forward,action.right,0);
  }
}
bool serviceMotion() {
  if (!motionActive) return false;
  if (!rc::imuIsFresh() || millis()-rc::commandStartMs >= rc::MOTION_TIMEOUT_MS) {
    rc::routeFault("GRID motion IMU/timeout"); return false;
  }
  if (turning) {
    rc::serviceIntegratedTurnCommand();
    if (rc::programState == rc::PROGRAM_FAULT) return false;
    if (rc::integratedTurnControlActive) return false;
    motionActive = false; return true;
  }
  if (fabsf(rc::headingErrorDegrees()) > cfg::MOVING_YAW_LIMIT_DEGREES) {
    rc::routeFault("GRID translation yaw"); return false;
  }
  if (!rc::allMotorsArrived()) return false;
  if (mission.geometry.rotationSafe(motionEnd)) {
    // Reuse the production predictive IMU controller for the terminal lock.
    // A zero nominal turn retains the target and corrects only measured drift.
    turning = true;rc::startTurn(0,1.0f,0.5f);return false;
  }
  // Near a field edge, refuse to rotate. Require fresh stable IMU samples.
  if (stoppedFrame == rc::imuValidFrameSequence) return false;
  stoppedFrame = rc::imuValidFrameSequence;
  if (fabsf(rc::headingErrorDegrees()) > cfg::STOPPED_YAW_LIMIT_DEGREES) {
    rc::routeFault("GRID edge yaw");return false;
  }
  if (!stoppedSamples) stoppedStart = rc::lastImuReceiveMs;
  if (stoppedSamples < 255) ++stoppedSamples;
  if (stoppedSamples < 3 || rc::lastImuReceiveMs-stoppedStart < 30) return false;
  motionActive = false;return true;
}
} // namespace radar_navigation
