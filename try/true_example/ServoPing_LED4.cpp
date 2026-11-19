#include <Arduino.h>
#include <FashionStar_UartServo.h>

namespace {

// 与当前 main.cpp 相同的串口总线舵机接线。
constexpr uint8_t SERVO_RX_PIN = PC7;
constexpr uint8_t SERVO_TX_PIN = PC6;
constexpr uint32_t SERVO_BAUDRATE = 115200UL;

// 当前机械臂使用的两个舵机 ID。
constexpr uint8_t GRIPPER_SERVO_ID = 4U;
constexpr uint8_t STORAGE_SERVO_ID = 5U;

// LED4：PB4，高电平点亮。
constexpr uint8_t LED4_PIN = PB4;
constexpr uint8_t LED4_ON_LEVEL = HIGH;
constexpr uint8_t LED4_OFF_LEVEL = LOW;

// 与 main.cpp 相同的独立调试串口。
constexpr uint8_t DEBUG_RX_PIN = PB12;
constexpr uint8_t DEBUG_TX_PIN = PB13;
constexpr uint32_t DEBUG_BAUDRATE = 115200UL;

constexpr uint32_t SERVO_POWER_UP_DELAY_MS = 300UL;
constexpr uint32_t PING_INTERVAL_MS = 2000UL;
constexpr uint32_t LED_BLINK_HALF_PERIOD_MS = 250UL;

HardwareSerial SerialServo(SERVO_RX_PIN, SERVO_TX_PIN);
HardwareSerial SerialDebug(DEBUG_RX_PIN, DEBUG_TX_PIN);

FSUS_Protocol servoProtocol;
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &servoProtocol);
FSUS_Servo storageServo(STORAGE_SERVO_ID, &servoProtocol);

bool bothServosOnline = false;
bool led4On = false;
uint32_t lastPingMs = 0UL;
uint32_t lastLedToggleMs = 0UL;

void setLed4(bool on) {
  led4On = on;
  digitalWrite(
      LED4_PIN,
      on ? LED4_ON_LEVEL : LED4_OFF_LEVEL);
}

void pingBothServos() {
  // 不使用 && 短路，确保夹爪和容器舵机每次都会分别收到 PING。
  const bool gripperOnline = gripperServo.ping();
  const bool storageOnline = storageServo.ping();
  bothServosOnline = gripperOnline && storageOnline;

  SerialDebug.print("Servo ping gripper(ID4)/storage(ID5): ");
  SerialDebug.print(gripperOnline ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.println(storageOnline ? 1 : 0);

  if (!bothServosOnline) {
    setLed4(false);
  }
}

void serviceServoPing() {
  const uint32_t nowMs = millis();
  if (nowMs - lastPingMs < PING_INTERVAL_MS) {
    return;
  }

  lastPingMs = nowMs;
  pingBothServos();
}

void serviceLed4() {
  if (!bothServosOnline) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastLedToggleMs < LED_BLINK_HALF_PERIOD_MS) {
    return;
  }

  lastLedToggleMs = nowMs;
  setLed4(!led4On);
}

} // namespace

void setup() {
  pinMode(LED4_PIN, OUTPUT);
  setLed4(false);

  SerialDebug.begin(DEBUG_BAUDRATE);
  servoProtocol.init(&SerialServo, SERVO_BAUDRATE);

  // 等待舵机供电和总线收发器稳定后，烧录启动即自动检测。
  delay(SERVO_POWER_UP_DELAY_MS);
  pingBothServos();

  lastPingMs = millis();
  lastLedToggleMs = millis();

  SerialDebug.println(
      "Both servos online -> LED4 blinks; "
      "either servo offline -> LED4 stays off.");
}

void loop() {
  serviceServoPing();
  serviceLed4();
}
