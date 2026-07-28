#include <AccelStepper.h>
#include <Arduino.h>
#include <JY901.h>
#include <MecanumKinematics.h>
#include <OneButton.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

using namespace mecanum;

/*
 * GongChuang tmcode1：225 mm机械臂中心偏移的国赛两轮移动路径
 *
 * 场地坐标：
 *   原点在场地左下角，+X 向右（东），+Y 向上（北），单位 mm。
 *
 * 车辆定义：
 *   3、4 号轮所在端是当前车头；
 *   1、2 号轮所在端是车尾；
 *   扫码器、机械臂和向下红外都从 2、4 号轮所在侧向外伸出。
 *   初始姿态下：3、4 侧朝南，1、2 侧朝北，1、3 侧朝西，
 *   2、4 侧朝东。
 *
 * 已按本次确认采用的实车约定：
 *   1. 10000 pulse 约等于直行 1 m；
 *   2. 四个电机安装方向仍为 [-,+,-,+]；
 *   3. JY901/HWT101 逆时针旋转时角度增加。
 *
 * MecanumKinematics 的 forward 轴仍沿 1、2 侧，不能把库中的
 * forward 直接理解为新车头方向。本程序统一按“向哪一个车体侧面平移”
 * 下达命令，避免车头旋转 90° 后出现前进、后退歧义。
 *
 * 路径完整执行两轮“原料区 -> 粗加工区 -> 暂存区”。每个工作区均由
 * 2、4 侧面对工位中线。工位动作暂时使用定时占位；后续接入机械臂时，
 * 将 PATH_ONLY_TEST 改为 false，并在对应动作完成后设置反馈标志。
 */

namespace {

// ---------------------------------------------------------------------------
// 硬件引脚与串口
// ---------------------------------------------------------------------------

const uint8_t DRIVE_ENABLE_PIN = PE13; // M1～M4 公共使能，低有效

const uint8_t M1_STEP_PIN = PD4;
const uint8_t M1_DIRECTION_PIN = PD6;
const uint8_t M2_STEP_PIN = PE11;
const uint8_t M2_DIRECTION_PIN = PE9;
const uint8_t M3_STEP_PIN = PD15;
const uint8_t M3_DIRECTION_PIN = PD14;
const uint8_t M4_STEP_PIN = PA1;
const uint8_t M4_DIRECTION_PIN = PC3_C;

// ARM.ino 的底座旋转轴 M5：PE10 低电平使能。
const uint8_t M5_ENABLE_PIN = PE10;
const uint8_t M5_DIRECTION_PIN = PE15;
const uint8_t M5_STEP_PIN = PB11;
const uint8_t START_BUTTON_PIN = PB9;

// true_example/Battery*.ino 使用的独立调试串口。
// 不能使用默认 Serial：通用 H750 板型的 Serial RX 是 PA1，
// 与四号轮 STEP 引脚冲突。
const uint8_t DEBUG_RX_PIN = PB12;
const uint8_t DEBUG_TX_PIN = PB13;
const uint8_t HMI_RX_PIN = PB15;
const uint8_t HMI_TX_PIN = PB14;
const uint8_t IMU_RX_PIN = PD9;
const uint8_t IMU_TX_PIN = PD8;
const uint8_t QR_RX_PIN = PE0;
const uint8_t QR_TX_PIN = PE1;
const uint8_t BATTERY_ADC_PIN = PC1;

const uint32_t DEBUG_BAUDRATE = 115200;
const uint32_t HMI_BAUDRATE = 115200;
const uint32_t IMU_BAUDRATE = 115200;
const uint32_t QR_BAUDRATE = 9600;

HardwareSerial SerialDebug(DEBUG_RX_PIN, DEBUG_TX_PIN);
HardwareSerial SerialHmi(HMI_RX_PIN, HMI_TX_PIN);
HardwareSerial SerialImu(IMU_RX_PIN, IMU_TX_PIN);
HardwareSerial SerialQr(QR_RX_PIN, QR_TX_PIN);

OneButton startButton(START_BUTTON_PIN, true, true);

// ---------------------------------------------------------------------------
// 底盘标定与安全参数
// ---------------------------------------------------------------------------

const float PI_F = 3.14159265358979323846f;
const float TWO_PI_F = 2.0f * PI_F;

// GitHub 运动学库当前记录的实测几何尺寸。
const float WHEELBASE_MM = 187.5f;
const float TRACK_WIDTH_MM = 195.0f;
const float WHEEL_DIAMETER_MM = 100.0f;

/*
 * 本次确认先采用 yyq5 旧底盘的实测标定：10000 pulse 约为 1 m。
 * 麦轮横移通常比纵向更容易受滚子与地面影响，因此保留独立标定项；
 * 当前第一次烧录先按相同数值，之后可分别实测修正。
 */
const float FORWARD_PULSES_PER_METER = 10000.0f;
const float LATERAL_PULSES_PER_METER = 10000.0f;
const float PULSES_PER_WHEEL_REVOLUTION = 3200.0f;

// 顺、逆时针分别标定，减少粗转后的多次原地微调；IMU 仍做最终闭环。
const float COUNTERCLOCKWISE_ROTATION_PULSE_SCALE = 1.0f;
const float CLOCKWISE_ROTATION_PULSE_SCALE = 1.0f;

/*
 * ARM.ino 的底座轴参数：200步/圈、16细分；实车确认减速比为5:1。
 * 例程 rotateBase(+degrees) 的方向与本车需要的方向相反，因此按下PB9后
 * 使用-90°；比赛路线全部结束后再回到0°。
 */
const float ARM_BASE_MOTOR_STEPS_PER_REVOLUTION = 200.0f;
const float ARM_BASE_MICROSTEPS = 16.0f;
const float ARM_BASE_GEAR_RATIO = 5.0f;
const float ARM_BASE_PULSES_PER_DEGREE =
    ARM_BASE_MOTOR_STEPS_PER_REVOLUTION *
    ARM_BASE_MICROSTEPS *
    ARM_BASE_GEAR_RATIO / 360.0f;
constexpr int32_t ARM_BASE_DEPLOY_ANGLE_DEGREES = -90;
constexpr int32_t ARM_BASE_HOME_ANGLE_DEGREES = 0;
const float ARM_BASE_MAXIMUM_STEP_RATE = 1000.0f;
const float ARM_BASE_STEP_ACCELERATION = 500.0f;

/*
 * M1～M4 电机安装方向。前进时原始驱动脉冲符号为 [-,+,-,+]。
 * 注意不要混淆两层符号：
 *   运动学“物理轮”[-,+,-,+] = 车体逆时针旋转；
 *   乘安装方向后，逆时针对应的“驱动原始脉冲”是 [+,+,+,+]。
 */
const WheelDirections MOTOR_DIRECTIONS(-1, +1, -1, +1);

// 场地与车辆均按毫米建模。
constexpr uint16_t FIELD_SIZE_MM = 2400U;
constexpr uint16_t FIELD_CENTER_MM = FIELD_SIZE_MM / 2U;
constexpr uint16_t START_ZONE_SIZE_MM = 300U;
constexpr uint16_t START_ZONE_MIN_MM =
    FIELD_SIZE_MM - START_ZONE_SIZE_MM;                      // 2100

/*
 * PulseMecanumChassis 的 187.5/195/100 mm 是轮心轴距、轮距和轮径，
 * 不是车体外廓。按实车最终确认的初始摆放：
 *   2、4侧朝+X，世界X方向外廓为230 mm；
 *   3、4侧朝-Y，世界Y方向外廓为300 mm。
 */
constexpr uint16_t CHASSIS_FOOTPRINT_X_MM = 230U;
constexpr uint16_t CHASSIS_FOOTPRINT_Y_MM = 300U;

/*
 * 三个目标都是每组三个圆中的中间圆。圆心在场地内侧，距离对应边界
 * 统一为40 mm。机械臂中心相对车体几何中心沿2、4侧偏移225 mm。
 */
constexpr uint16_t RING_BOUNDARY_OFFSET_MM = 40U;
constexpr uint16_t ARM_CENTER_OFFSET_MM = 225U;
constexpr uint16_t CHASSIS_HALF_WIDTH_MM =
    CHASSIS_FOOTPRINT_X_MM / 2U;                             // 115
constexpr uint16_t ARM_CENTER_BEYOND_NEAR_WHEEL_MM =
    ARM_CENTER_OFFSET_MM - CHASSIS_HALF_WIDTH_MM;            // 110
constexpr uint16_t ARM_CENTER_TO_FARTHEST_WHEEL_MM =
    ARM_CENTER_OFFSET_MM + CHASSIS_HALF_WIDTH_MM;            // 340
static_assert(
    CHASSIS_HALF_WIDTH_MM == 115U &&
        ARM_CENTER_BEYOND_NEAR_WHEEL_MM == 110U &&
        ARM_CENTER_TO_FARTHEST_WHEEL_MM == 340U,
    "225 mm arm offset must be measured from the center of the 230 mm width");
constexpr uint16_t RAW_RING_CENTER_Y_MM =
    FIELD_SIZE_MM - RING_BOUNDARY_OFFSET_MM;                 // 2360
constexpr uint16_t PROCESS_RING_CENTER_Y_MM =
    RING_BOUNDARY_OFFSET_MM;                                 // 40
constexpr uint16_t STORAGE_RING_CENTER_X_MM =
    RING_BOUNDARY_OFFSET_MM;                                 // 40

/*
 * 所有图纸标注为1100~1300 mm的可变中心位置统一采用1200 mm。
 * 车体在启停区1贴左边和下边放置，因此起点中心为(2215,2250)。
 */
static_assert(FIELD_CENTER_MM == 1200U, "Field centerline must be 1200 mm");
constexpr uint16_t START_CENTER_X_MM =
    START_ZONE_MIN_MM + CHASSIS_FOOTPRINT_X_MM / 2U;         // 2215
constexpr uint16_t START_CENTER_Y_MM =
    START_ZONE_MIN_MM + CHASSIS_FOOTPRINT_Y_MM / 2U;         // 2250
constexpr uint16_t FINAL_ZONE_CENTER_X_MM = START_CENTER_X_MM;
constexpr uint16_t FINAL_ZONE_CENTER_Y_MM = START_CENTER_Y_MM;

/*
 * 由“目标圆心 - 机械臂中心偏移”反算车体中心。三个工作姿态都让
 * 2、4侧面对工位，所以原料区向北、粗加工区向南、暂存区向西。
 */
constexpr uint16_t RAW_CENTER_Y_MM =
    RAW_RING_CENTER_Y_MM - ARM_CENTER_OFFSET_MM;             // 2135
constexpr uint16_t PROCESS_CENTER_Y_MM =
    PROCESS_RING_CENTER_Y_MM + ARM_CENTER_OFFSET_MM;         // 265
constexpr uint16_t STORAGE_CENTER_X_MM =
    STORAGE_RING_CENTER_X_MM + ARM_CENTER_OFFSET_MM;         // 265

/*
 * 启停区1先沿3、4侧直走1050 mm到达二维码区，收到有效任务码后
 * 再继续横移。
 * 回程仍使用X=2150的安全转弯带；若在起点中心X=2215原地转180°，
 * 230x300 mm车体的外接圆会越过场地右边界约4 mm。
 */
constexpr uint16_t QR_PASS_CENTER_X_MM = START_CENTER_X_MM;  // 2215
constexpr uint16_t RETURN_LANE_X_MM = 2150U;
constexpr uint16_t UPPER_TURN_Y_MM = 2150U;

static_assert(
    RAW_CENTER_Y_MM + ARM_CENTER_OFFSET_MM ==
        RAW_RING_CENTER_Y_MM,
    "Raw arm center must hit the middle ring");
static_assert(
    PROCESS_CENTER_Y_MM - ARM_CENTER_OFFSET_MM ==
        PROCESS_RING_CENTER_Y_MM,
    "Process arm center must hit the middle ring");
static_assert(
    STORAGE_CENTER_X_MM - ARM_CENTER_OFFSET_MM ==
        STORAGE_RING_CENTER_X_MM,
    "Storage arm center must hit the middle ring");

constexpr uint16_t WORKSTATION_APPROACH_MM = 150U;
/*
 * 起步Y方向实车手调只改这一行：
 *   走过头多少毫米就减多少，没走够多少毫米就加多少。
 * QR_PASS_TO_RAW_APPROACH_MM会自动反算，原料区最终坐标不会被带偏。
 */
constexpr uint16_t START_TO_QR_PASS_MM = 1050U;
constexpr uint16_t QR_PASS_CENTER_Y_MM =
    START_CENTER_Y_MM - START_TO_QR_PASS_MM;                 // 1200
constexpr uint16_t QR_PASS_TO_FIELD_CENTER_X_MM =
    QR_PASS_CENTER_X_MM - FIELD_CENTER_MM;                   // 1015
constexpr uint16_t QR_PASS_TO_RAW_APPROACH_MM =
    RAW_CENTER_Y_MM - WORKSTATION_APPROACH_MM -
    QR_PASS_CENTER_Y_MM;                                     // 785
static_assert(
    START_TO_QR_PASS_MM >= 1000U &&
        START_TO_QR_PASS_MM <= 1100U,
    "Start-to-QR manual calibration must stay within a safe range");
constexpr uint16_t CENTER_TO_RAW_MM =
    RAW_CENTER_Y_MM - FIELD_CENTER_MM;                       // 935
constexpr uint16_t RAW_TO_PROCESS_MM =
    RAW_CENTER_Y_MM - PROCESS_CENTER_Y_MM;                   // 1870
constexpr uint16_t PROCESS_TO_CENTER_MM =
    FIELD_CENTER_MM - PROCESS_CENTER_Y_MM;                   // 935
constexpr uint16_t CENTER_TO_STORAGE_MM =
    FIELD_CENTER_MM - STORAGE_CENTER_X_MM;                   // 935
constexpr uint16_t STORAGE_TO_CENTER_MM =
    FIELD_CENTER_MM - STORAGE_CENTER_X_MM;                   // 935
constexpr uint16_t CENTER_TO_RETURN_LANE_MM =
    RETURN_LANE_X_MM - FIELD_CENTER_MM;                      // 950
constexpr uint16_t RETURN_LANE_TO_UPPER_TURN_MM =
    UPPER_TURN_Y_MM - FIELD_CENTER_MM;                       // 950
constexpr uint16_t UPPER_TURN_TO_FINAL_ZONE_Y_MM =
    FINAL_ZONE_CENTER_Y_MM - UPPER_TURN_Y_MM;                // 100
constexpr uint16_t RETURN_LANE_TO_FINAL_ZONE_X_MM =
    FINAL_ZONE_CENTER_X_MM - RETURN_LANE_X_MM;               // 65

const float MAXIMUM_STEP_RATE = 5500.0f;       // 非误差区稍微提速
const float STEP_ACCELERATION = 2500.0f;
const float TURN_MAXIMUM_STEP_RATE = 2000.0f;
const float TURN_STEP_ACCELERATION = 800.0f;
const float HEADING_CORRECTION_MAXIMUM_STEP_RATE = 1000.0f;
const float HEADING_CORRECTION_STEP_ACCELERATION = 450.0f;
const float WORKSTATION_MAXIMUM_STEP_RATE = 1600.0f;
const float WORKSTATION_STEP_ACCELERATION = 700.0f;
const float FINAL_MAXIMUM_STEP_RATE = 800.0f; // 最后进入300×300启停区
const float FINAL_STEP_ACCELERATION = 400.0f;
const uint16_t MINIMUM_STEP_WIDTH_US = 2U;

/*
 * 起步1050 mm和随后横移1015 mm都保持一次走完，不再出现二维码区前的
 * 二段短蠕动。原料区到粗加工区前的1720 mm高速段仍拆成1100+620 mm，
 * 中间回正一次。
 * 最后150 mm仍在转稳后低速进入，当前不启用全程竞赛计时限制。
 */
constexpr uint16_t MAX_TRANSLATION_SEGMENT_MM = 1100U;

const float TRANSLATION_HEADING_TOLERANCE_DEGREES = 0.40f;
const float WORKSTATION_HEADING_TOLERANCE_DEGREES = 0.20f;
const float TURN_HEADING_TOLERANCE_DEGREES = 0.15f;
const float FINAL_HEADING_TOLERANCE_DEGREES = 0.15f;
const float MAXIMUM_HEADING_CORRECTION_DEGREES = 12.0f;
const float TRANSLATION_MINIMUM_HEADING_CORRECTION_DEGREES = 0.20f;
const float WORKSTATION_MINIMUM_HEADING_CORRECTION_DEGREES = 0.10f;
const float TURN_MINIMUM_HEADING_CORRECTION_DEGREES = 0.10f;
const float FINAL_MINIMUM_HEADING_CORRECTION_DEGREES = 0.10f;
const uint32_t TRANSLATION_HEADING_STABLE_TIME_MS = 200UL;
const uint32_t WORKSTATION_HEADING_STABLE_TIME_MS = 400UL;
const uint32_t TURN_HEADING_STABLE_TIME_MS = 500UL;
const uint32_t FINAL_HEADING_STABLE_TIME_MS = 600UL;
const uint32_t IMU_POST_MOTION_SETTLE_TIME_MS = 120UL;
const uint32_t MOTION_TIMEOUT_MS = 20000UL;
const uint32_t TURN_TIMEOUT_MS = 16000UL;
const uint32_t IMU_STALE_TIMEOUT_MS = 600UL;
const bool ENABLE_MOTION_TIMEOUTS = false;

// +1：传感器逆时针角度增加；若以后更换安装方向，可改为 -1。
const int8_t IMU_COUNTERCLOCKWISE_SIGN = +1;

// “只检查路径”模式：工位动作定时完成，不调用机械臂。
const bool PATH_ONLY_TEST = true;

// 恢复二维码接收：到达扫码位置后，必须收到有效任务码才继续路线。
const bool ENABLE_QR_RECEIVER = true;
const bool REQUIRE_QR_SUCCESS = true;

const uint32_t QR_TEST_HOLD_MS = 1000UL;
const uint32_t WORKSTATION_TEST_HOLD_MS = 1500UL;
const uint32_t FINAL_HOLD_MS = 3000UL;

// 当前屏幕仍保留 x0 时显示连续航向角；若已删除 x0，可改为 false。
const bool DISPLAY_YAW_ON_X0 = true;
const bool DISPLAY_BATTERY_ON_X1 = true;
const bool SHOW_RESULT_PAGE_ON_FINISH = true;

// true_example uses a 10 kOhm + 1 kOhm divider on PC1 and sends V * 100 to x1.
// Adjust BATTERY_CALIBRATION_SCALE after comparing x1 with a multimeter.
const float BATTERY_ADC_REFERENCE_VOLTAGE = 3.3f;
const float BATTERY_DIVIDER_RATIO = 11.0f;
const float BATTERY_CALIBRATION_SCALE = 1.0f;
const float BATTERY_FILTER_ALPHA = 0.10f;
const uint32_t BATTERY_SAMPLE_INTERVAL_MS = 50UL;
const uint32_t BATTERY_HMI_INTERVAL_MS = 500UL;

MecanumKinematics geometry =
    MecanumKinematics::fromMillimeters(
        WHEELBASE_MM, TRACK_WIDTH_MM, WHEEL_DIAMETER_MM);

AccelStepper motor1(
    AccelStepper::DRIVER, M1_STEP_PIN, M1_DIRECTION_PIN);
AccelStepper motor2(
    AccelStepper::DRIVER, M2_STEP_PIN, M2_DIRECTION_PIN);
AccelStepper motor3(
    AccelStepper::DRIVER, M3_STEP_PIN, M3_DIRECTION_PIN);
AccelStepper motor4(
    AccelStepper::DRIVER, M4_STEP_PIN, M4_DIRECTION_PIN);
AccelStepper armBaseRotationStepper(
    AccelStepper::DRIVER, M5_STEP_PIN, M5_DIRECTION_PIN);

AccelStepper *const motors[4] = {
    &motor1, &motor2, &motor3, &motor4};

// ---------------------------------------------------------------------------
// HMI
// ---------------------------------------------------------------------------

uint32_t lastHmiRefreshMs = 0;
uint32_t lastBatterySampleMs = 0;
uint32_t lastBatteryHmiMs = 0;
float filteredBatteryVoltage = 0.0f;
bool batteryVoltageInitialized = false;
uint8_t correctGrabCount = 0;
uint8_t correctPlacementCount = 0;

void hmiEndCommand() {
  SerialHmi.write(0xFF);
  SerialHmi.write(0xFF);
  SerialHmi.write(0xFF);
}

void hmiCommand(const char *command) {
  SerialHmi.print(command);
  hmiEndCommand();
}

void hmiSetText(const char *objectName, const char *text) {
  SerialHmi.print(objectName);
  SerialHmi.print(".txt=\"");
  SerialHmi.print(text);
  SerialHmi.print("\"");
  hmiEndCommand();
}

void hmiSetValue(const char *objectName, int32_t value) {
  SerialHmi.print(objectName);
  SerialHmi.print(".val=");
  SerialHmi.print(value);
  hmiEndCommand();
}

void serviceBatteryVoltage() {
  if (!DISPLAY_BATTERY_ON_X1) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastBatterySampleMs >= BATTERY_SAMPLE_INTERVAL_MS) {
    lastBatterySampleMs = nowMs;

    const uint32_t rawAdc =
        static_cast<uint32_t>(analogRead(BATTERY_ADC_PIN));
    const float sampledVoltage =
        static_cast<float>(rawAdc) *
        BATTERY_ADC_REFERENCE_VOLTAGE / 4095.0f *
        BATTERY_DIVIDER_RATIO * BATTERY_CALIBRATION_SCALE;

    if (!batteryVoltageInitialized) {
      filteredBatteryVoltage = sampledVoltage;
      batteryVoltageInitialized = true;
    } else {
      filteredBatteryVoltage +=
          BATTERY_FILTER_ALPHA *
          (sampledVoltage - filteredBatteryVoltage);
    }
  }

  if (batteryVoltageInitialized &&
      nowMs - lastBatteryHmiMs >= BATTERY_HMI_INTERVAL_MS) {
    lastBatteryHmiMs = nowMs;
    const int32_t voltageTimes100 =
        static_cast<int32_t>(
            lroundf(filteredBatteryVoltage * 100.0f));
    hmiSetValue("x1", voltageTimes100);
  }
}

void hmiSetRunStatus(const char *status) {
  hmiSetText("t7", status);
}

void hmiSetTaskCounts() {
  // 短期调试记录中最终界面使用 n5=正确抓取数、n6=正确放置数。
  hmiSetValue("n5", correctGrabCount);
  hmiSetValue("n6", correctPlacementCount);
}

void hmiShowTaskCode(const char *taskCode) {
  // 完整码：156+123+516+231
  // t3：156+123+（8字符）
  // t8：516+231 （7字符）
  char firstRow[9] = {0};
  char secondRow[8] = {0};

  memcpy(firstRow, taskCode, 8);
  memcpy(secondRow, taskCode + 8, 7);

  hmiSetText("t3", firstRow);
  hmiSetText("t8", secondRow);
}

// ---------------------------------------------------------------------------
// 二维码
// ---------------------------------------------------------------------------

const size_t QR_CODE_LENGTH = 15;
const size_t QR_BUFFER_SIZE = 32;

char qrData[QR_BUFFER_SIZE] = {0};
size_t qrDataIndex = 0;
bool qrOverflow = false;
bool scanFlag = false;

bool characterInRange(char value, char minimum, char maximum) {
  return value >= minimum && value <= maximum;
}

bool isValidTaskCode(const char *code) {
  if (strlen(code) != QR_CODE_LENGTH) {
    return false;
  }

  if (code[3] != '+' || code[7] != '+' || code[11] != '+') {
    return false;
  }

  for (uint8_t i = 0; i < 3; ++i) {
    // 第1、3组为颜色号1～6；第2、4组为放置位置1～3。
    if (!characterInRange(code[i], '1', '6') ||
        !characterInRange(code[8 + i], '1', '6') ||
        !characterInRange(code[4 + i], '1', '3') ||
        !characterInRange(code[12 + i], '1', '3')) {
      return false;
    }
  }

  return true;
}

void resetQrReceiver() {
  qrDataIndex = 0;
  qrOverflow = false;
  scanFlag = false;
  qrData[0] = '\0';

  while (SerialQr.available()) {
    SerialQr.read();
  }
}

void finishQrFrame() {
  if (qrDataIndex == 0 && !qrOverflow) {
    return;
  }

  qrData[qrDataIndex] = '\0';

  if (!qrOverflow && isValidTaskCode(qrData)) {
    scanFlag = true;
    hmiSetText("t1", "QROK");
    hmiShowTaskCode(qrData);
    SerialDebug.print("QR OK: ");
    SerialDebug.println(qrData);
  } else {
    hmiSetText("t1", "QRERR");
  }

  qrDataIndex = 0;
  qrOverflow = false;
}

void receiveQrData() {
  if (scanFlag) {
    while (SerialQr.available()) {
      SerialQr.read();
    }
    return;
  }

  while (SerialQr.available()) {
    const char incomingByte =
        static_cast<char>(SerialQr.read());

    // 同时兼容 CR、LF 和 CRLF 结尾。
    if (incomingByte == '\r' || incomingByte == '\n') {
      finishQrFrame();
      continue;
    }

    if (qrDataIndex < QR_BUFFER_SIZE - 1) {
      qrData[qrDataIndex++] = incomingByte;
    } else {
      qrOverflow = true;
    }
  }
}

// ---------------------------------------------------------------------------
// IMU：把 ±180° 原始角展开为连续的“逆时针为正”角度
// ---------------------------------------------------------------------------

bool imuInitialized = false;
float imuLastSignedRawDegrees = 0.0f;
float imuCounterClockwiseDegrees = 0.0f;
uint32_t lastImuReceiveMs = 0;
uint8_t imuFrame[11] = {0};
uint8_t imuFrameIndex = 0;

float wrapDeltaDegrees(float degrees) {
  while (degrees >= 180.0f) {
    degrees -= 360.0f;
  }
  while (degrees < -180.0f) {
    degrees += 360.0f;
  }
  return degrees;
}

void updateContinuousImuHeading(int16_t rawYawValue) {
  const float rawDegrees =
      static_cast<float>(rawYawValue) /
      32768.0f * 180.0f;
  const float counterClockwiseSignedRaw =
      rawDegrees *
      static_cast<float>(IMU_COUNTERCLOCKWISE_SIGN);

  lastImuReceiveMs = millis();

  if (!imuInitialized) {
    imuInitialized = true;
    imuLastSignedRawDegrees = counterClockwiseSignedRaw;
    imuCounterClockwiseDegrees = 0.0f;
    return;
  }

  /*
   * 先对相邻帧做相位展开，再累加连续角。不能直接对原始 ±180°
   * 数值做普通均值或低通，否则 179° 与 -179° 会被错误平均到 0°。
   */
  const float continuousDeltaDegrees =
      wrapDeltaDegrees(
          counterClockwiseSignedRaw -
          imuLastSignedRawDegrees);
  imuCounterClockwiseDegrees +=
      continuousDeltaDegrees;
  imuLastSignedRawDegrees =
      counterClockwiseSignedRaw;
}

void receiveImuData() {
  while (SerialImu.available()) {
    const uint8_t incomingByte =
        static_cast<uint8_t>(SerialImu.read());
    JY901.CopeSerialData(incomingByte);

    /*
     * JY901 每帧11字节：0x55、类型、8字节数据、校验和。
     * 只有收到校验正确的0x53角度帧后，才把IMU标记为可用；
     * 这样不会在半帧或其他类型数据到达时误用初始零值。
     */
    if (imuFrameIndex == 0) {
      if (incomingByte == 0x55) {
        imuFrame[imuFrameIndex++] = incomingByte;
      }
      continue;
    }

    imuFrame[imuFrameIndex++] = incomingByte;

    if (imuFrameIndex == sizeof(imuFrame)) {
      uint8_t checksum = 0;
      for (uint8_t i = 0; i < 10; ++i) {
        checksum = static_cast<uint8_t>(
            checksum + imuFrame[i]);
      }

      if (checksum == imuFrame[10] &&
          imuFrame[0] == 0x55 &&
          imuFrame[1] == 0x53) {
        // JY901 角度帧中 yaw 是第6、7字节的小端 int16_t。
        const uint16_t yawUnsigned =
            static_cast<uint16_t>(imuFrame[6]) |
            (static_cast<uint16_t>(imuFrame[7]) << 8);
        updateContinuousImuHeading(
            static_cast<int16_t>(yawUnsigned));
      }

      // 若坏帧最后一个字节恰好是下一帧帧头，保留该帧头继续同步。
      if (checksum != imuFrame[10] &&
          imuFrame[10] == 0x55) {
        imuFrame[0] = 0x55;
        imuFrameIndex = 1;
      } else {
        imuFrameIndex = 0;
      }
    }
  }
}

bool imuIsFresh() {
  return imuInitialized &&
         millis() - lastImuReceiveMs <= IMU_STALE_TIMEOUT_MS;
}

// ---------------------------------------------------------------------------
// 运动学位移逆解
// ---------------------------------------------------------------------------

struct MotorPulses {
  long motor1;
  long motor2;
  long motor3;
  long motor4;

  MotorPulses(long m1 = 0, long m2 = 0,
              long m3 = 0, long m4 = 0)
      : motor1(m1), motor2(m2), motor3(m3), motor4(m4) {}
};

long roundedPulseCount(float pulses) {
  return static_cast<long>(
      pulses >= 0.0f ? pulses + 0.5f : pulses - 0.5f);
}

MotorPulses bodyDisplacementToMotorPulses(
    float forwardMeters,
    float leftMeters,
    float counterClockwiseRadians) {
  /*
   * MecanumKinematics 是线性模型。把 vx/vy/wz 分别替换成
   * dx/dy/dHeading，inverse() 的结果就对应各轮转角。
   *
   * 纵向与横移分别使用各自的 pulse/m 标定；
   * 旋转使用 3200 pulse/rev 与实测几何；
   * 最后再施加 -,+,-,+ 的电机安装方向修正。
   */
  const WheelSpeeds forwardWheelRadians =
      geometry.inverse(
          ChassisVelocity(forwardMeters, 0.0f, 0.0f));
  const WheelSpeeds lateralWheelRadians =
      geometry.inverse(
          ChassisVelocity(0.0f, leftMeters, 0.0f));
  const WheelSpeeds rotationWheelRadians =
      geometry.inverse(
          ChassisVelocity(0.0f, 0.0f, counterClockwiseRadians));

  const float forwardFactor =
      geometry.wheelRadiusMeters() * FORWARD_PULSES_PER_METER;
  const float lateralFactor =
      geometry.wheelRadiusMeters() * LATERAL_PULSES_PER_METER;
  const float rotationPulseScale =
      counterClockwiseRadians >= 0.0f
          ? COUNTERCLOCKWISE_ROTATION_PULSE_SCALE
          : CLOCKWISE_ROTATION_PULSE_SCALE;
  const float rotationFactor =
      PULSES_PER_WHEEL_REVOLUTION / TWO_PI_F *
      rotationPulseScale;

  const float physical1 =
      forwardWheelRadians.frontLeft * forwardFactor +
      lateralWheelRadians.frontLeft * lateralFactor +
      rotationWheelRadians.frontLeft * rotationFactor;
  const float physical2 =
      forwardWheelRadians.frontRight * forwardFactor +
      lateralWheelRadians.frontRight * lateralFactor +
      rotationWheelRadians.frontRight * rotationFactor;
  const float physical3 =
      forwardWheelRadians.rearLeft * forwardFactor +
      lateralWheelRadians.rearLeft * lateralFactor +
      rotationWheelRadians.rearLeft * rotationFactor;
  const float physical4 =
      forwardWheelRadians.rearRight * forwardFactor +
      lateralWheelRadians.rearRight * lateralFactor +
      rotationWheelRadians.rearRight * rotationFactor;

  return MotorPulses(
      roundedPulseCount(physical1 * MOTOR_DIRECTIONS.frontLeft),
      roundedPulseCount(physical2 * MOTOR_DIRECTIONS.frontRight),
      roundedPulseCount(physical3 * MOTOR_DIRECTIONS.rearLeft),
      roundedPulseCount(physical4 * MOTOR_DIRECTIONS.rearRight));
}

void startRelativeMotorMove(const MotorPulses &pulses) {
  // 调试串口可直接确认每段四号轮是否收到了非零目标脉冲。
  SerialDebug.print("Pulses M1..M4: ");
  SerialDebug.print(pulses.motor1);
  SerialDebug.print(", ");
  SerialDebug.print(pulses.motor2);
  SerialDebug.print(", ");
  SerialDebug.print(pulses.motor3);
  SerialDebug.print(", ");
  SerialDebug.println(pulses.motor4);

  motor1.move(pulses.motor1);
  motor2.move(pulses.motor2);
  motor3.move(pulses.motor3);
  motor4.move(pulses.motor4);
}

void startBodyDisplacement(
    float forwardMeters,
    float leftMeters,
    float counterClockwiseRadians) {
  startRelativeMotorMove(
      bodyDisplacementToMotorPulses(
          forwardMeters, leftMeters, counterClockwiseRadians));
}

bool allMotorsArrived() {
  return motor1.distanceToGo() == 0 &&
         motor2.distanceToGo() == 0 &&
         motor3.distanceToGo() == 0 &&
         motor4.distanceToGo() == 0;
}

void runAllMotors() {
  // 每个 run() 都必须执行，不能使用短路逻辑连接。
  motor1.run();
  motor2.run();
  motor3.run();
  motor4.run();
}

void stopAllMotorsImmediately() {
  for (uint8_t i = 0; i < 4; ++i) {
    const long currentPosition = motors[i]->currentPosition();
    motors[i]->setCurrentPosition(currentPosition);
  }
}

void enableDriveMotors() {
  digitalWrite(DRIVE_ENABLE_PIN, LOW);
}

void disableDriveMotors() {
  stopAllMotorsImmediately();
  digitalWrite(DRIVE_ENABLE_PIN, HIGH);
}

long armBaseAngleToPulses(float angleDegrees) {
  return lroundf(
      angleDegrees * ARM_BASE_PULSES_PER_DEGREE);
}

void startArmBaseRotationToDegrees(float angleDegrees) {
  digitalWrite(M5_ENABLE_PIN, LOW);
  armBaseRotationStepper.moveTo(
      armBaseAngleToPulses(angleDegrees));

  SerialDebug.print("Arm base target: ");
  SerialDebug.print(angleDegrees, 1);
  SerialDebug.print(" deg, pulses=");
  SerialDebug.println(
      armBaseRotationStepper.targetPosition());
}

void stopArmBaseImmediately() {
  const long currentPosition =
      armBaseRotationStepper.currentPosition();
  armBaseRotationStepper.setCurrentPosition(
      currentPosition);
}

void disableArmBaseMotor() {
  stopArmBaseImmediately();
  digitalWrite(M5_ENABLE_PIN, HIGH);
}

void setDriveMotionProfile(float maximumStepRate, float acceleration) {
  for (uint8_t i = 0; i < 4; ++i) {
    motors[i]->setMaxSpeed(maximumStepRate);
    motors[i]->setAcceleration(acceleration);
  }
}

// ---------------------------------------------------------------------------
// 国赛累计两轮路径
// ---------------------------------------------------------------------------

enum CommandType {
  COMMAND_ARM_BASE_HOME,
  COMMAND_MOVE_SIDE_12_MM,
  COMMAND_MOVE_SIDE_34_MM,
  COMMAND_MOVE_SIDE_13_MM,
  COMMAND_MOVE_SIDE_24_MM,
  COMMAND_TURN_COUNTERCLOCKWISE_DEGREES,
  COMMAND_TURN_CLOCKWISE_DEGREES,
  COMMAND_SET_PRECISE_MOTION,
  COMMAND_QR_ACTION,
  COMMAND_RAW_ACTION,
  COMMAND_PROCESS_ACTION,
  COMMAND_STORAGE_ACTION,
  COMMAND_FINAL_ALIGN,
  COMMAND_HOLD,
  COMMAND_FINISH
};

struct RouteCommand {
  CommandType type;
  int32_t value;
  const char *name;
  bool preciseArrival;
};

/*
 * 关键工位姿态（单位 mm，角度表示2、4侧朝向）：
 *
 *   启停区1起点：(2215, 2250)，2、4侧朝东
 *   启停区1终点：(2215, 2250)，2、4侧朝西，左边/下边贴启停区边线
 *   二维码扫码点：(2215,1200)，2、4侧朝东；有效扫码后继续
 *   原料区：  (1200, 2135)，2、4侧朝北，机械臂中心(1200,2360)
 *   粗加工区：(1200,265)，两轮均由2、4侧朝南进入
 *   暂存区：  ( 265, 1200)，2、4侧朝西
 *
 * 初始世界外廓按X=230、Y=300 mm建模；全部工位横向中心固定为1200 mm，
 * 不再在1100~1300 mm范围内取近似值。机械臂中心目标分别为
 * (1200,2360)、(1200,40)、(40,1200)，均距对应场地边界40 mm。
 * 机械臂中心从车体中心沿2、4侧偏移225 mm。
 *
 * 新车头是3、4侧。第一段向3、4侧移动就是实车“前进”，但在旧运动学
 * 坐标中等价于负 forward，所以路径命令不再使用 forward/backward 命名。
 */
const RouteCommand route[] = {
    /*
     * PB9已在后台触发机械臂底座反向转90°；底盘不等待它到位，
     * 立即执行第一段。其他机械臂电机没有初始化、没有运动命令。
     */
    // 初始3、4侧朝南；一次走1050 mm到达二维码区后执行扫码。
    {COMMAND_MOVE_SIDE_34_MM, START_TO_QR_PASS_MM,
     "Start1 -> QR area direct"},
    {COMMAND_QR_ACTION, 0, "Scan QR task code"},

    // ----------------------------- 第1轮 -----------------------------
    // 立即一次横移1015 mm；重算衔接距离，原料区最终停车点保持不变。
    {COMMAND_MOVE_SIDE_13_MM, QR_PASS_TO_FIELD_CENTER_X_MM,
     "QR area -> raw centerline"},
    {COMMAND_MOVE_SIDE_12_MM,
     QR_PASS_TO_RAW_APPROACH_MM,
     "QR Y=1200 -> raw approach round 1"},
    {COMMAND_TURN_COUNTERCLOCKWISE_DEGREES, 90,
     "Face side 2,4 north"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM,
     "Precise entry to raw round 1", true},
    {COMMAND_RAW_ACTION, 1, "Raw action round 1"},

    // 在工位前150 mm转稳，再由2、4侧低速直线进入，避免工位内窜动。
    {COMMAND_MOVE_SIDE_13_MM,
     RAW_TO_PROCESS_MM - WORKSTATION_APPROACH_MM + 20U,
     "Raw -> process approach round 1"},
    {COMMAND_TURN_COUNTERCLOCKWISE_DEGREES, 180,
     "Face side 2,4 south"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM,
     "Precise entry to process round 1", true},
    {COMMAND_PROCESS_ACTION, 1, "Process action round 1"},

    // 加工后先到暂存区前150 mm，再转向并低速进入。
    {COMMAND_MOVE_SIDE_13_MM, PROCESS_TO_CENTER_MM - 50U,
     "Process -> field center round 1"},
    {COMMAND_MOVE_SIDE_34_MM,
     CENTER_TO_STORAGE_MM - WORKSTATION_APPROACH_MM,
     "Center -> storage approach round 1"},
    {COMMAND_TURN_CLOCKWISE_DEGREES, 90,
     "Face side 2,4 west"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM,
     "Precise entry to storage round 1", true},
    {COMMAND_STORAGE_ACTION, 1, "Storage action round 1"},

    // ----------------------------- 第2轮 -----------------------------
    {COMMAND_MOVE_SIDE_13_MM, STORAGE_TO_CENTER_MM,
     "Storage -> field center round 2"},
    {COMMAND_MOVE_SIDE_34_MM,
     CENTER_TO_RAW_MM - WORKSTATION_APPROACH_MM,
     "Field center -> raw approach round 2"},
    {COMMAND_TURN_CLOCKWISE_DEGREES, 90,
     "Face side 2,4 north"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM + 40U,
     "Precise entry to raw round 2", true},
    {COMMAND_RAW_ACTION, 2, "Raw action round 2"},

    {COMMAND_MOVE_SIDE_13_MM,
     RAW_TO_PROCESS_MM - WORKSTATION_APPROACH_MM + 40U,
     "Raw -> process approach round 2"},
    {COMMAND_TURN_COUNTERCLOCKWISE_DEGREES, 180,
     "Face side 2,4 south"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM - 20U,
     "Precise entry to process round 2", true},
    {COMMAND_PROCESS_ACTION, 2, "Process action round 2"},

    {COMMAND_MOVE_SIDE_13_MM, PROCESS_TO_CENTER_MM - 40U,
     "Process -> field center round 2"},
    {COMMAND_MOVE_SIDE_34_MM,
     CENTER_TO_STORAGE_MM - WORKSTATION_APPROACH_MM,
     "Center -> storage approach round 2"},
    {COMMAND_TURN_CLOCKWISE_DEGREES, 90,
     "Face side 2,4 west"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM,
     "Precise entry to storage round 2", true},
    {COMMAND_STORAGE_ACTION, 2, "Storage action round 2"},

    // 返回启停区1：全程保持2、4侧朝西，不再做最后一次180°原地转圈。
    {COMMAND_MOVE_SIDE_13_MM, STORAGE_TO_CENTER_MM,
     "Storage -> field center final"},
    {COMMAND_MOVE_SIDE_13_MM, CENTER_TO_RETURN_LANE_MM,
     "Field center -> return lane final"},
    {COMMAND_MOVE_SIDE_34_MM, RETURN_LANE_TO_UPPER_TURN_MM,
     "QR lane -> upper turning area"},
    {COMMAND_SET_PRECISE_MOTION, 0,
     "Set low speed for precise Start1 entry"},
    {COMMAND_MOVE_SIDE_34_MM, UPPER_TURN_TO_FINAL_ZONE_Y_MM,
      "Upper turning area -> Start1 row"},
    {COMMAND_MOVE_SIDE_13_MM, RETURN_LANE_TO_FINAL_ZONE_X_MM,
      "Enter Start1"},
    {COMMAND_FINAL_ALIGN, 0, "Final infrared or vision align"},
    {COMMAND_HOLD, FINAL_HOLD_MS, "Hold at Start1"},
    {COMMAND_MOVE_SIDE_34_MM, 50U,
     "Final Y+50 before arm base home"},
    {COMMAND_MOVE_SIDE_13_MM, 40U,
     "Final X+40 before arm base home"},
    {COMMAND_ARM_BASE_HOME, ARM_BASE_HOME_ANGLE_DEGREES,
     "Return arm base to zero"},
    {COMMAND_FINISH, 0, "Finished"}};

const size_t ROUTE_COMMAND_COUNT =
    sizeof(route) / sizeof(route[0]);

enum ProgramState {
  PROGRAM_WAITING,
  PROGRAM_RUNNING,
  PROGRAM_FINISHED,
  PROGRAM_FAULT
};

ProgramState programState = PROGRAM_WAITING;
size_t routeIndex = 0;
uint8_t activeCompetitionRound = 0;
bool commandStarted = false;
uint32_t commandStartMs = 0;
uint32_t headingStableStartMs = 0;
uint32_t motorsArrivedStartMs = 0;
bool imuWaitStatusDisplayed = false;
uint16_t translationRemainingMm = 0;
bool preciseMotionEnabled = false;
bool turnMotionEnabled = false;
bool translationPreciseArrivalEnabled = false;
bool workstationApproachEnabled = false;
bool turnCoarseTelemetryPending = false;
float activeTurnStartHeadingDegrees = 0.0f;
float activeTurnCommandDegrees = 0.0f;
uint8_t activeTurnCorrectionCount = 0;

float routeImuReferenceDegrees = 0.0f;
float targetCounterClockwiseHeadingDegrees = 0.0f;

volatile bool startRequested = false;
volatile bool abortRequested = false;

// 接入真实视觉、向下红外和机械臂后，由对应模块设置这些完成标志。
volatile bool rawActionFinished = false;
volatile bool processActionFinished = false;
volatile bool storageActionFinished = false;
volatile bool finalAlignmentFinished = false;

float currentRouteCounterClockwiseHeading() {
  return imuCounterClockwiseDegrees - routeImuReferenceDegrees;
}

float headingErrorDegrees() {
  return targetCounterClockwiseHeadingDegrees -
         currentRouteCounterClockwiseHeading();
}

void printTurnLockTelemetry() {
  SerialDebug.print("Turn lock: target=");
  SerialDebug.print(targetCounterClockwiseHeadingDegrees, 2);
  SerialDebug.print(" deg, actual=");
  SerialDebug.print(currentRouteCounterClockwiseHeading(), 2);
  SerialDebug.print(" deg, error=");
  SerialDebug.print(headingErrorDegrees(), 2);
  SerialDebug.print(" deg, corrections=");
  SerialDebug.println(activeTurnCorrectionCount);
}

void printTurnCoarseTelemetry() {
  const float actualTurnDegrees =
      currentRouteCounterClockwiseHeading() -
      activeTurnStartHeadingDegrees;
  const float coarseErrorDegrees =
      activeTurnCommandDegrees - actualTurnDegrees;

  SerialDebug.print("Turn coarse: command=");
  SerialDebug.print(activeTurnCommandDegrees, 2);
  SerialDebug.print(" deg, actual=");
  SerialDebug.print(actualTurnDegrees, 2);
  SerialDebug.print(" deg, error=");
  SerialDebug.print(coarseErrorDegrees, 2);
  SerialDebug.println(" deg");
}

void printCurrentCommand() {
  SerialDebug.print("Route ");
  SerialDebug.print(static_cast<unsigned int>(routeIndex + 1));
  SerialDebug.print("/");
  SerialDebug.print(static_cast<unsigned int>(ROUTE_COMMAND_COUNT));
  SerialDebug.print(": ");
  SerialDebug.println(route[routeIndex].name);
}

void routeFault(const char *reason) {
  disableDriveMotors();
  programState = PROGRAM_FAULT;
  commandStarted = false;
  hmiSetRunStatus("FAULT");

  SerialDebug.print("FAULT: ");
  SerialDebug.println(reason);
}

void advanceRoute() {
  stopAllMotorsImmediately();
  ++routeIndex;
  commandStarted = false;
  commandStartMs = millis();
  headingStableStartMs = 0;
  motorsArrivedStartMs = 0;
}

void startHeadingCorrection(
    float errorCounterClockwiseDegrees) {
  float correction = errorCounterClockwiseDegrees;
  const float minimumCorrection =
      preciseMotionEnabled
          ? FINAL_MINIMUM_HEADING_CORRECTION_DEGREES
          : (turnMotionEnabled
                 ? TURN_MINIMUM_HEADING_CORRECTION_DEGREES
                 : (workstationApproachEnabled
                        ? WORKSTATION_MINIMUM_HEADING_CORRECTION_DEGREES
                        : TRANSLATION_MINIMUM_HEADING_CORRECTION_DEGREES));

  if (correction > MAXIMUM_HEADING_CORRECTION_DEGREES) {
    correction = MAXIMUM_HEADING_CORRECTION_DEGREES;
  } else if (correction < -MAXIMUM_HEADING_CORRECTION_DEGREES) {
    correction = -MAXIMUM_HEADING_CORRECTION_DEGREES;
  }

  if (correction > 0.0f &&
      correction < minimumCorrection) {
    correction = minimumCorrection;
  } else if (correction < 0.0f &&
             correction > -minimumCorrection) {
    correction = -minimumCorrection;
  }

  // IMU与运动学均为逆时针正，误差可以直接换算为运动学角度。
  const float correctionCounterClockwiseRadians =
      correction * PI_F / 180.0f;
  if (turnMotionEnabled) {
    ++activeTurnCorrectionCount;
  }
  setDriveMotionProfile(
      HEADING_CORRECTION_MAXIMUM_STEP_RATE,
      HEADING_CORRECTION_STEP_ACCELERATION);
  startBodyDisplacement(
      0.0f, 0.0f, correctionCounterClockwiseRadians);
}

bool updateHeadingLock(uint32_t timeoutMs) {
  if (!imuIsFresh()) {
    routeFault("IMU data timeout");
    return false;
  }

  if (!allMotorsArrived()) {
    headingStableStartMs = 0;
    motorsArrivedStartMs = 0;
    if (ENABLE_MOTION_TIMEOUTS &&
        millis() - commandStartMs >= timeoutMs) {
      routeFault("Motor motion timeout");
    }
    return false;
  }

  if (motorsArrivedStartMs == 0) {
    motorsArrivedStartMs = millis();
    return false;
  }

  if (millis() - motorsArrivedStartMs <
      IMU_POST_MOTION_SETTLE_TIME_MS) {
    return false;
  }

  if (turnMotionEnabled && turnCoarseTelemetryPending) {
    printTurnCoarseTelemetry();
    turnCoarseTelemetryPending = false;
  }

  const float tolerance =
      preciseMotionEnabled
          ? FINAL_HEADING_TOLERANCE_DEGREES
          : (turnMotionEnabled
                 ? TURN_HEADING_TOLERANCE_DEGREES
                 : (workstationApproachEnabled
                        ? WORKSTATION_HEADING_TOLERANCE_DEGREES
                        : TRANSLATION_HEADING_TOLERANCE_DEGREES));
  const uint32_t stableTimeMs =
      preciseMotionEnabled
          ? FINAL_HEADING_STABLE_TIME_MS
          : (turnMotionEnabled
                 ? TURN_HEADING_STABLE_TIME_MS
                 : (workstationApproachEnabled
                        ? WORKSTATION_HEADING_STABLE_TIME_MS
                        : TRANSLATION_HEADING_STABLE_TIME_MS));

  const float error = headingErrorDegrees();
  if (fabsf(error) <= tolerance) {
    if (headingStableStartMs == 0) {
      headingStableStartMs = millis();
    }

    if (millis() - headingStableStartMs >= stableTimeMs) {
      return true;
    }
    return false;
  }

  headingStableStartMs = 0;

  if (ENABLE_MOTION_TIMEOUTS &&
      millis() - commandStartMs >= timeoutMs) {
    routeFault("Heading correction timeout");
    return false;
  }

  startHeadingCorrection(error);
  return false;
}

bool isTranslationCommand(CommandType type) {
  return type == COMMAND_MOVE_SIDE_12_MM ||
         type == COMMAND_MOVE_SIDE_34_MM ||
         type == COMMAND_MOVE_SIDE_13_MM ||
         type == COMMAND_MOVE_SIDE_24_MM;
}

void startTranslationSegment(CommandType type) {
  const uint16_t segmentMm =
      translationRemainingMm > MAX_TRANSLATION_SEGMENT_MM
          ? MAX_TRANSLATION_SEGMENT_MM
          : translationRemainingMm;
  /*
   * 精靠段必须从静止状态就使用低速低加速度，严禁在高速运动中突然降低
   * acceleration；否则 AccelStepper 会因剩余刹车距离不足越过目标再反向。
   */
  workstationApproachEnabled =
      translationPreciseArrivalEnabled;
  translationRemainingMm -= segmentMm;

  const float distanceMeters =
      static_cast<float>(segmentMm) / 1000.0f;

  setDriveMotionProfile(
      preciseMotionEnabled
          ? FINAL_MAXIMUM_STEP_RATE
          : (workstationApproachEnabled
                 ? WORKSTATION_MAXIMUM_STEP_RATE
                 : MAXIMUM_STEP_RATE),
      preciseMotionEnabled
          ? FINAL_STEP_ACCELERATION
          : (workstationApproachEnabled
                 ? WORKSTATION_STEP_ACCELERATION
                 : STEP_ACCELERATION));

  switch (type) {
    case COMMAND_MOVE_SIDE_12_MM:
      startBodyDisplacement(distanceMeters, 0.0f, 0.0f);
      break;
    case COMMAND_MOVE_SIDE_34_MM:
      startBodyDisplacement(-distanceMeters, 0.0f, 0.0f);
      break;
    case COMMAND_MOVE_SIDE_13_MM:
      startBodyDisplacement(0.0f, distanceMeters, 0.0f);
      break;
    case COMMAND_MOVE_SIDE_24_MM:
      startBodyDisplacement(0.0f, -distanceMeters, 0.0f);
      break;
    default:
      routeFault("Invalid translation command");
      return;
  }

  commandStartMs = millis();
  headingStableStartMs = 0;
  motorsArrivedStartMs = 0;
}

void startMotionCommand(const RouteCommand &command) {
  turnMotionEnabled = false;
  workstationApproachEnabled = false;
  translationPreciseArrivalEnabled = false;

  if (isTranslationCommand(command.type)) {
    translationRemainingMm = command.value;
    translationPreciseArrivalEnabled =
        command.preciseArrival;
    startTranslationSegment(command.type);
    return;
  }

  switch (command.type) {
    case COMMAND_ARM_BASE_HOME:
      hmiSetRunStatus("ARM0");
      startArmBaseRotationToDegrees(
          static_cast<float>(command.value));
      break;

    case COMMAND_TURN_COUNTERCLOCKWISE_DEGREES:
    case COMMAND_TURN_CLOCKWISE_DEGREES: {
      turnMotionEnabled = true;
      setDriveMotionProfile(
          TURN_MAXIMUM_STEP_RATE, TURN_STEP_ACCELERATION);
      const float counterClockwiseDegrees =
          static_cast<float>(command.value) *
          (command.type ==
                   COMMAND_TURN_COUNTERCLOCKWISE_DEGREES
                ? 1.0f
                : -1.0f);
      activeTurnStartHeadingDegrees =
          currentRouteCounterClockwiseHeading();
      activeTurnCommandDegrees =
          counterClockwiseDegrees;
      activeTurnCorrectionCount = 0;
      turnCoarseTelemetryPending = true;
      targetCounterClockwiseHeadingDegrees +=
          counterClockwiseDegrees;
      startBodyDisplacement(
          0.0f, 0.0f,
          counterClockwiseDegrees * PI_F / 180.0f);
      break;
    }

    case COMMAND_SET_PRECISE_MOTION:
      preciseMotionEnabled = true;
      setDriveMotionProfile(
          FINAL_MAXIMUM_STEP_RATE, FINAL_STEP_ACCELERATION);
      hmiSetRunStatus("SLOW");
      break;

    case COMMAND_QR_ACTION:
      hmiSetRunStatus("SCAN");
      break;

    case COMMAND_RAW_ACTION:
      activeCompetitionRound =
          static_cast<uint8_t>(command.value);
      rawActionFinished = false;
      hmiSetRunStatus(
          command.value == 1 ? "RAW1" : "RAW2");
      break;

    case COMMAND_PROCESS_ACTION:
      activeCompetitionRound =
          static_cast<uint8_t>(command.value);
      processActionFinished = false;
      hmiSetRunStatus(
          command.value == 1 ? "PROCESS1" : "PROCESS2");
      break;

    case COMMAND_STORAGE_ACTION:
      activeCompetitionRound =
          static_cast<uint8_t>(command.value);
      storageActionFinished = false;
      hmiSetRunStatus(
          command.value == 1 ? "STORAGE1" : "STORAGE2");
      break;

    case COMMAND_FINAL_ALIGN:
      hmiSetRunStatus("ALIGN");
      break;

    case COMMAND_HOLD:
      hmiSetRunStatus("HOLD");
      break;

    case COMMAND_FINISH:
      break;

    case COMMAND_MOVE_SIDE_12_MM:
    case COMMAND_MOVE_SIDE_34_MM:
    case COMMAND_MOVE_SIDE_13_MM:
    case COMMAND_MOVE_SIDE_24_MM:
      // 纯平移已在switch之前分段启动。
      break;
  }
}

void startCurrentCommand() {
  if (routeIndex >= ROUTE_COMMAND_COUNT) {
    return;
  }

  commandStartMs = millis();
  headingStableStartMs = 0;
  motorsArrivedStartMs = 0;
  printCurrentCommand();
  startMotionCommand(route[routeIndex]);
  commandStarted = true;
}

bool timedActionFinished(
    bool externalFeedback,
    uint32_t testDurationMs) {
  if (PATH_ONLY_TEST) {
    return millis() - commandStartMs >= testDurationMs;
  }
  return externalFeedback;
}

void finishRawActionIfNeeded() {
  if (PATH_ONLY_TEST) {
    // 每轮原料区抓取3个。
    correctGrabCount += 3;
    hmiSetTaskCounts();
  }
}

void finishProcessActionIfNeeded() {
  if (PATH_ONLY_TEST) {
    // 每轮粗加工区先放下3个，再重新抓回3个。
    correctPlacementCount += 3;
    correctGrabCount += 3;
    hmiSetTaskCounts();
  }
}

void finishStorageActionIfNeeded() {
  if (PATH_ONLY_TEST) {
    // 每轮暂存区放置3个。
    correctPlacementCount += 3;
    hmiSetTaskCounts();
  }
}

void finishProgram() {
  disableDriveMotors();
  disableArmBaseMotor();
  programState = PROGRAM_FINISHED;
  commandStarted = false;
  hmiSetRunStatus("FINISH");

  if (SHOW_RESULT_PAGE_ON_FINISH) {
    hmiCommand("page page0");
    hmiSetTaskCounts();
  }

  SerialDebug.println("Route finished");
}

void updateRoute() {
  if (programState != PROGRAM_RUNNING ||
      routeIndex >= ROUTE_COMMAND_COUNT) {
    return;
  }

  if (!commandStarted) {
    startCurrentCommand();
  }

  const RouteCommand &command = route[routeIndex];

  switch (command.type) {
    case COMMAND_MOVE_SIDE_12_MM:
    case COMMAND_MOVE_SIDE_34_MM:
    case COMMAND_MOVE_SIDE_13_MM:
    case COMMAND_MOVE_SIDE_24_MM:
      // 最长每1100 mm回正一次；长距离移动会分段锁定航向。
      if (updateHeadingLock(MOTION_TIMEOUT_MS)) {
        if (translationRemainingMm > 0) {
          startTranslationSegment(command.type);
        } else {
          advanceRoute();
        }
      }
      break;

    case COMMAND_ARM_BASE_HOME:
      if (armBaseRotationStepper.distanceToGo() == 0) {
        SerialDebug.println("Arm base returned to zero");
        disableArmBaseMotor();
        hmiSetRunStatus("RUN");
        advanceRoute();
      }
      break;

    case COMMAND_TURN_COUNTERCLOCKWISE_DEGREES:
    case COMMAND_TURN_CLOCKWISE_DEGREES:
      if (updateHeadingLock(TURN_TIMEOUT_MS)) {
        printTurnLockTelemetry();
        advanceRoute();
      }
      break;

    case COMMAND_SET_PRECISE_MOTION:
      advanceRoute();
      break;

    case COMMAND_QR_ACTION:
      if (scanFlag ||
          (!REQUIRE_QR_SUCCESS &&
           millis() - commandStartMs >= QR_TEST_HOLD_MS)) {
        hmiSetRunStatus("RUN");
        advanceRoute();
      }
      break;

    case COMMAND_RAW_ACTION:
      if (timedActionFinished(
              rawActionFinished, WORKSTATION_TEST_HOLD_MS)) {
        finishRawActionIfNeeded();
        hmiSetRunStatus("RUN");
        advanceRoute();
      }
      break;

    case COMMAND_PROCESS_ACTION:
      if (timedActionFinished(
              processActionFinished, WORKSTATION_TEST_HOLD_MS)) {
        finishProcessActionIfNeeded();
        hmiSetRunStatus("RUN");
        advanceRoute();
      }
      break;

    case COMMAND_STORAGE_ACTION:
      if (timedActionFinished(
              storageActionFinished, WORKSTATION_TEST_HOLD_MS)) {
        finishStorageActionIfNeeded();
        hmiSetRunStatus("RUN");
        advanceRoute();
      }
      break;

    case COMMAND_FINAL_ALIGN:
      if (PATH_ONLY_TEST || finalAlignmentFinished) {
        advanceRoute();
      }
      break;

    case COMMAND_HOLD:
      if (millis() - commandStartMs >= command.value) {
        advanceRoute();
      }
      break;

    case COMMAND_FINISH:
      finishProgram();
      break;
  }
}

// ---------------------------------------------------------------------------
// 启动、停止与周期服务
// ---------------------------------------------------------------------------

void onStartButtonClick() {
  if (programState != PROGRAM_RUNNING) {
    abortRequested = false;
    startRequested = true;
    imuWaitStatusDisplayed = false;
    // PB9一按即开始展开；底盘路线不等待，二者由loop()并行服务。
    startArmBaseRotationToDegrees(
        static_cast<float>(
            ARM_BASE_DEPLOY_ANGLE_DEGREES));
  }
}

void onStartButtonLongPress() {
  abortRequested = true;
}

void beginRoute() {
  if (!imuIsFresh()) {
    if (!imuWaitStatusDisplayed) {
      hmiSetRunStatus("IMUWAIT");
      SerialDebug.println("Waiting for a valid JY901 angle frame");
      imuWaitStatusDisplayed = true;
    }
    return;
  }

  routeIndex = 0;
  activeCompetitionRound = 0;
  commandStarted = false;
  commandStartMs = millis();
  headingStableStartMs = 0;
  motorsArrivedStartMs = 0;
  translationRemainingMm = 0;
  preciseMotionEnabled = false;
  turnMotionEnabled = false;
  translationPreciseArrivalEnabled = false;
  workstationApproachEnabled = false;
  turnCoarseTelemetryPending = false;
  activeTurnStartHeadingDegrees = 0.0f;
  activeTurnCommandDegrees = 0.0f;
  activeTurnCorrectionCount = 0;

  routeImuReferenceDegrees = imuCounterClockwiseDegrees;
  targetCounterClockwiseHeadingDegrees = 0.0f;

  correctGrabCount = 0;
  correctPlacementCount = 0;
  rawActionFinished = false;
  processActionFinished = false;
  storageActionFinished = false;
  finalAlignmentFinished = false;

  if (ENABLE_QR_RECEIVER) {
    resetQrReceiver();
  }
  hmiSetTaskCounts();
  hmiSetText(
      "t1",
      ENABLE_QR_RECEIVER ? "QRWAIT" : "BYPASS");
  hmiSetText("t3", "000+000+");
  hmiSetText("t8", "000+000");
  hmiSetRunStatus("RUN");

  for (uint8_t i = 0; i < 4; ++i) {
    motors[i]->setCurrentPosition(0);
  }

  // 上一次运行结束时可能停留在最终低速档，每次启动都恢复巡航参数。
  setDriveMotionProfile(MAXIMUM_STEP_RATE, STEP_ACCELERATION);
  enableDriveMotors();
  programState = PROGRAM_RUNNING;
  startRequested = false;
  imuWaitStatusDisplayed = false;

  SerialDebug.println("Route started");
}

void abortRoute() {
  disableDriveMotors();
  disableArmBaseMotor();
  programState = PROGRAM_FAULT;
  commandStarted = false;
  abortRequested = false;
  startRequested = false;
  hmiSetRunStatus("STOP");
  SerialDebug.println("Route aborted by long press");
}

void updateHmiYaw() {
  if (!DISPLAY_YAW_ON_X0 || !imuInitialized) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastHmiRefreshMs < 100UL) {
    return;
  }

  lastHmiRefreshMs = nowMs;
  const int32_t yawTimes100 =
      static_cast<int32_t>(
          currentRouteCounterClockwiseHeading() * 100.0f);
  hmiSetValue("x0", yawTimes100);
}

void initializeMotorOutputs() {
  pinMode(DRIVE_ENABLE_PIN, OUTPUT);
  pinMode(M5_ENABLE_PIN, OUTPUT);

  // 初始化期间先保持所有步进驱动器失能。
  digitalWrite(DRIVE_ENABLE_PIN, HIGH);
  digitalWrite(M5_ENABLE_PIN, HIGH);

  for (uint8_t i = 0; i < 4; ++i) {
    // AccelStepper 的构造函数会配置引脚，但这里在全部串口初始化后
    // 再明确配置一次，防止任何外设复用覆盖 STEP/DIR GPIO。
    motors[i]->enableOutputs();
    motors[i]->setMinPulseWidth(MINIMUM_STEP_WIDTH_US);
  }

  armBaseRotationStepper.enableOutputs();
  armBaseRotationStepper.setMinPulseWidth(
      MINIMUM_STEP_WIDTH_US);
  armBaseRotationStepper.setMaxSpeed(
      ARM_BASE_MAXIMUM_STEP_RATE);
  armBaseRotationStepper.setAcceleration(
      ARM_BASE_STEP_ACCELERATION);
  // 与ARM.ino一致：上电时机械臂必须实际位于0°，本程序不含限位开关寻零。
  armBaseRotationStepper.setCurrentPosition(0);

  setDriveMotionProfile(MAXIMUM_STEP_RATE, STEP_ACCELERATION);
}

void initializeHmi() {
  delay(300);

  while (SerialHmi.available()) {
    SerialHmi.read();
  }

  hmiCommand("page main");
  delay(50);
  hmiSetText(
      "t1",
      ENABLE_QR_RECEIVER ? "QRWAIT" : "BYPASS");
  hmiSetText("t3", "000+000+");
  hmiSetText("t8", "000+000");
  hmiSetRunStatus("READY");
  hmiSetTaskCounts();

  if (DISPLAY_YAW_ON_X0) {
    hmiSetValue("x0", 0);
  }
  if (DISPLAY_BATTERY_ON_X1) {
    hmiSetValue("x1", 0);
  }
}

} // namespace

void setup() {
  SerialDebug.begin(DEBUG_BAUDRATE);
  SerialHmi.begin(HMI_BAUDRATE);
  SerialImu.begin(IMU_BAUDRATE);
  if (ENABLE_QR_RECEIVER) {
    SerialQr.begin(QR_BAUDRATE);
  }

  analogReadResolution(12);
  pinMode(BATTERY_ADC_PIN, INPUT_ANALOG);

  // 串口初始化完成后最后配置电机 GPIO，确保 PA1 保持为 M4 STEP 输出。
  initializeMotorOutputs();

  startButton.reset();
  startButton.attachClick(onStartButtonClick);
  startButton.attachLongPressStart(onStartButtonLongPress);

  initializeHmi();
  if (ENABLE_QR_RECEIVER) {
    resetQrReceiver();
  }

  SerialDebug.println("GongChuang route controller ready");
  SerialDebug.println("Click PB9 to start; long-press PB9 to stop");
}

void loop() {
  /*
   * 电机 run()、按钮、IMU、二维码和状态机都采用非阻塞服务。
   * 运行过程中不要加入 delay()、舵机 wait() 或 runToPosition()。
   */
  runAllMotors();
  armBaseRotationStepper.run();
  startButton.tick();
  receiveImuData();
  if (ENABLE_QR_RECEIVER) {
    receiveQrData();
  }
  updateHmiYaw();
  serviceBatteryVoltage();

  if (abortRequested && programState == PROGRAM_RUNNING) {
    abortRoute();
  }

  if (startRequested && programState != PROGRAM_RUNNING) {
    beginRoute();
  }

  updateRoute();
}
