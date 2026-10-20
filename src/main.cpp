#include <MobileRobotConfig.h>
#include <RobotConfig.h>

#include <AccelStepper.h>
#include <ArmMotorController.h>
#include <ArmTransferPlanner.h>
#include <Arduino.h>
#include <CompetitionCore.h>
#include <FashionStar_UartServo.h>
#include <ImuHeadingTracker.h>
#include <MaixCamClient.h>
#include <MecanumKinematics.h>
#include <OneButton.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// 原料区取料逻辑选择：0使用原A点固定等待/复用上次夹取姿态的旧逻辑；
// 1启用已存档的A/B三视角搜索与下降追踪。当前正式程序默认使用旧逻辑；
// 需要恢复新逻辑时显式定义-DGONGCHUANG_RAW_PICKUP_LOGIC_MODE=1。
#ifndef GONGCHUANG_RAW_PICKUP_LOGIC_MODE
#define GONGCHUANG_RAW_PICKUP_LOGIC_MODE 0
#endif
// 原料区视觉请求策略与机械搜索策略分开选择：
// 0=历史模式8全颜色请求，并每15秒换一次序号；
// 1=按二维码已知的下一种颜色请求模式1..4，并每5秒换一次序号。
// 当前正式程序默认0；需要试验定向单色请求时显式定义为1。
#ifndef GONGCHUANG_RAW_COLOR_REQUEST_POLICY
#define GONGCHUANG_RAW_COLOR_REQUEST_POLICY 0
#endif
#ifndef GONGCHUANG_RAW_VERBOSE_VISION_CALIBRATION
#define GONGCHUANG_RAW_VERBOSE_VISION_CALIBRATION 0
#endif
static_assert(
    GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 0 ||
        GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1,
    "GONGCHUANG_RAW_PICKUP_LOGIC_MODE must be 0 or 1");
static_assert(
    GONGCHUANG_RAW_COLOR_REQUEST_POLICY == 0 ||
        GONGCHUANG_RAW_COLOR_REQUEST_POLICY == 1,
    "GONGCHUANG_RAW_COLOR_REQUEST_POLICY must be 0 or 1");

#ifndef GONGCHUANG_RING3_EXTENSION_CORRECTION_MM
#ifdef GONGCHUANG_DEBUG_RING3_EXTENSION_CORRECTION_MM
#define GONGCHUANG_RING3_EXTENSION_CORRECTION_MM \
  GONGCHUANG_DEBUG_RING3_EXTENSION_CORRECTION_MM
#else
#define GONGCHUANG_RING3_EXTENSION_CORRECTION_MM 5.0f
#endif
#endif

// Proven STORAGE2 scan-pose corrections are formal defaults. The independent
// state-3 workstation wrapper selects the same production implementation and
// adds diagnostics; it does not provide another copy of these parameters. The
// production build does not include or call that debug entry. The corrected
// ArmPose is shared by motor commands and camera-to-arm coordinate conversion,
// so changing the camera position cannot leave the geometry on the old pose.
#ifndef GONGCHUANG_STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES
#define GONGCHUANG_STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES -5.0f
#endif
#ifndef GONGCHUANG_STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES
#define GONGCHUANG_STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES 5.0f
#endif
#ifndef GONGCHUANG_STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM
#define GONGCHUANG_STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM 30.0f
#endif

// Detailed second-storage scan telemetry is compiled only into the dedicated
// state-3 workstation debug firmware. Keep the production default disabled:
// its flash budget is tight, and these prints deliberately favor diagnosis
// over timing transparency.
#ifndef GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
#define GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG 0
#endif

#ifndef GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT
#ifdef GONGCHUANG_DEBUG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT
#define GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT \
  GONGCHUANG_DEBUG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT
#elif defined(GONGCHUANG_DEBUG_STORAGE1) || GONGCHUANG_RUN_MODE == 2
// 工位调试状态2/3没有圆环回料盘动作，保留普通100%斜坡。
#define GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT 100U
#else
#define GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT 150U
#endif
#endif

// 第4规范段的车头方向附加量。20 mm是实车修正量，不是理论横移量，
// 也不乘本段motionScale；由底盘入口合成为一次二维运动。启停区1新路线
// 将其按右移200 mm/余程的比例分配；旧路线和单段调试的190 mm等待点
// 则沿完整二维向量计算，恢复后仍到达同一个原终点。
#ifndef GONGCHUANG_STEP04_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_STEP04_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_STEP04_FORWARD_CORRECTION_MM
#define GONGCHUANG_STEP04_FORWARD_CORRECTION_MM 20.0f
#endif

// 正式路线第6段只保留车头方向+10 mm实车修正量。它不是理论横移量，
// 也不乘本段motionScale；由底盘入口融合成同一段二维运动。
#ifndef GONGCHUANG_STEP06_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_STEP06_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_STEP06_FORWARD_CORRECTION_MM
#define GONGCHUANG_STEP06_FORWARD_CORRECTION_MM 10.0f
#endif

// 正式路线第7/15段的车头方向附加量。70 mm已合并本轮新增10 mm；是实车修正量，不是1835 mm
// 理论横移量，也不乘各段横移motionScale；由底盘入口合成为一次二维运动。
#ifndef GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_MM
#define GONGCHUANG_LONG_LATERAL_FORWARD_CORRECTION_MM 70.0f
#endif

// 正式路线只在第7段越过50%后，把额外15 mm车头修正叠加到剩余运动。
// 该值是后半程实车修正量，不是理论横移量，也不乘motionScale。底盘不会
// 停车或重新启动第7段，只会连续更新剩余二维目标。
#ifndef GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM
#define GONGCHUANG_STEP07_SECOND_HALF_FORWARD_CORRECTION_MM 15.0f
#endif

// 第15段使用与第7段相同的连续剩余目标重建方式：越过50%后，在现有全段
// +70 mm车头修正基础上，再把+40 mm车头修正加入剩余运动，不停车或重启。
#ifndef GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED
#define GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_ENABLED 1
#endif
#ifndef GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM
#define GONGCHUANG_STEP15_SECOND_HALF_FORWARD_CORRECTION_MM 40.0f
#endif

using namespace mecanum;

/*
 * 维护警告：完整运行只执行下方 include 中的 route_chassis::route[] 与
 * route_chassis::updateRoute()；不要在本文件重建 route[]/updateRoute()。
 * 底盘唯一运行实现是本工程的 route_chassis_main.inc；历史导入来源
 * GongChuang_route 只用于追溯，后续调速必须修改本工程正式入口。
 *
 * route_chassis_main.inc 管理19条运动命令、M1～M4、PB9、IMU、二维码、
 * HMI、电池与底盘速度/位移控制。GC当前文件在六个工位暂停窗口
 * 运行机械臂和视觉。到站暂停映射在 workPauseAfterRouteStep()，
 * 接力入口在 serviceIntegratedRoute()。改路线时同时核对这六处触发段号。
 */
namespace route_chassis {
#include "route_chassis_main.inc"
} // namespace route_chassis

namespace {

using gongchuang::ArmPose;
namespace arm_transfer = gongchuang::arm_transfer;
namespace arm_config = gongchuang::config::arm_transfer;
namespace m7_experiment = gongchuang::config::m7_experiment;
namespace mobile_robot_config = gongchuang::mobile_robot;

const uint8_t DRIVE_ENABLE_PIN = PE13;

const uint8_t M1_STEP_PIN = PD4;
const uint8_t M1_DIRECTION_PIN = PD6;
const uint8_t M2_STEP_PIN = PE11;
const uint8_t M2_DIRECTION_PIN = PE9;
const uint8_t M3_STEP_PIN = PD15;
const uint8_t M3_DIRECTION_PIN = PD14;
const uint8_t M4_STEP_PIN = PA1;
const uint8_t M4_DIRECTION_PIN = PC3_C;

const uint8_t START_BUTTON_PIN = PB9;

const uint8_t DEBUG_RX_PIN = PB12;
const uint8_t DEBUG_TX_PIN = PB13;
const uint8_t HMI_RX_PIN = PB15;
const uint8_t HMI_TX_PIN = PB14;
const uint8_t IMU_RX_PIN = PD9;
const uint8_t IMU_TX_PIN = PD8;
const uint8_t QR_RX_PIN = PE0;
const uint8_t QR_TX_PIN = PE1;
const uint8_t BATTERY_ADC_PIN = PC1;
const uint8_t MAIXCAM_RX_PIN = PE7;
const uint8_t MAIXCAM_TX_PIN = PE8;
const uint8_t ARM_LINEAR_RX_PIN = PA3;
const uint8_t ARM_LINEAR_TX_PIN = PA2;
const uint8_t SERVO_RX_PIN = PC7;
const uint8_t SERVO_TX_PIN = PC6;

const uint32_t DEBUG_BAUDRATE = 115200;
const uint32_t HMI_BAUDRATE = 115200;
const uint32_t IMU_BAUDRATE = 115200;
const uint32_t QR_BAUDRATE = 9600;
const uint32_t MAIXCAM_BAUDRATE = 115200;
const uint32_t ARM_LINEAR_BAUDRATE = 115200;
const uint32_t SERVO_BAUDRATE = 115200;

HardwareSerial &SerialDebug = route_chassis::SerialDebug;
HardwareSerial &SerialHmi = route_chassis::SerialHmi;
HardwareSerial &SerialImu = route_chassis::SerialImu;
HardwareSerial &SerialQr = route_chassis::SerialQr;
HardwareSerial SerialMaixcam(MAIXCAM_RX_PIN, MAIXCAM_TX_PIN);
HardwareSerial SerialArmLinear(
    ARM_LINEAR_RX_PIN, ARM_LINEAR_TX_PIN);
HardwareSerial SerialServo(SERVO_RX_PIN, SERVO_TX_PIN);
gongchuang::ImuHeadingTracker imuTracker(SerialImu);

constexpr uint8_t GRIPPER_SERVO_ID = 4U;
constexpr uint8_t STORAGE_SERVO_ID = 5U;
FSUS_Protocol servoProtocol;
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &servoProtocol);
FSUS_Servo storageServo(STORAGE_SERVO_ID, &servoProtocol);

OneButton &startButton = route_chassis::startButton;

const float PI_F = 3.14159265358979323846f;
const float TWO_PI_F = 2.0f * PI_F;

const float WHEELBASE_MM = 187.5f;
const float TRACK_WIDTH_MM = 195.0f;
const float WHEEL_DIAMETER_MM = 100.0f;

const float FORWARD_PULSES_PER_METER = 10000.0f;
const float LATERAL_PULSES_PER_METER = 10000.0f;
const float PULSES_PER_WHEEL_REVOLUTION = 3200.0f;

const float COUNTERCLOCKWISE_ROTATION_PULSE_SCALE = 1.0f;
const float CLOCKWISE_ROTATION_PULSE_SCALE = 1.0f;

constexpr int32_t ARM_BASE_TRAVEL_OLD_FRAME_DEGREES = 0;
constexpr int32_t ARM_BASE_STANDARD_OLD_FRAME_DEGREES = -90;
constexpr float ARM_BASE_TRAVEL_STANDARD_FRAME_DEGREES = 90.0f;
constexpr int32_t ARM_BASE_HOME_ANGLE_DEGREES =
    ARM_BASE_TRAVEL_OLD_FRAME_DEGREES;
constexpr float FINAL_RETURN_M5_TARGET_DEGREES =
    static_cast<float>(ARM_BASE_HOME_ANGLE_DEGREES);
constexpr float FINAL_RETURN_M5_TOLERANCE_DEGREES = 0.10f;
static_assert(
    FINAL_RETURN_M5_TARGET_DEGREES ==
        static_cast<float>(ARM_BASE_HOME_ANGLE_DEGREES),
    "Final M5 target must be the startup angle");

const float ARM_BASE_MAXIMUM_STEP_RATE = 81000.0f;
const float ARM_BASE_STEP_ACCELERATION = 27000.0f;
const float ARM_BASE_ENDPOINT_COARSE_MAXIMUM_STEP_RATE = 121500.0f;
const float ARM_BASE_ENDPOINT_COARSE_STEP_ACCELERATION = 40500.0f;

const float ARM_BASE_ENDPOINT_TRAVEL_MAXIMUM_STEP_RATE = 182250.0f;
const float ARM_BASE_ENDPOINT_TRAVEL_STEP_ACCELERATION = 60750.0f;

// STORAGE2 pre-positioning now uses exactly the same high-speed M5 profile as
// the established tray-to-ring placement motion.  This remains local to the
// STORAGE2 preparation/pipeline and does not change any global arm profile.
constexpr float STORAGE_TRAVEL_M5_MAXIMUM_STEP_RATE =
    arm_config::M5_PLACE_MAXIMUM_STEP_RATE;
constexpr float STORAGE_TRAVEL_M5_STEP_ACCELERATION =
    arm_config::M5_PLACE_STEP_ACCELERATION;
// PROCESS1 -> STORAGE1: M5 keeps only 20% of its return profile so the
// in-transit zeroing cannot strike the mechanism.
constexpr float PROCESS1_STORAGE_TRAVEL_M5_MAXIMUM_STEP_RATE =
    arm_config::M5_RETURN_MAXIMUM_STEP_RATE * 0.20f;
constexpr float PROCESS1_STORAGE_TRAVEL_M5_STEP_ACCELERATION =
    arm_config::M5_RETURN_STEP_ACCELERATION * 0.20f;
// After either RAW visit, M5 returns to zero while the chassis has already
// resumed.  Retain 30% of the existing RAW profile (70% slower) locally;
// ordinary RAW pickup/placement motion is unchanged.
constexpr float RAW_EXIT_M5_MAXIMUM_STEP_RATE =
    arm_config::M5_RAW_MAXIMUM_STEP_RATE * 0.30f;
constexpr float RAW_EXIT_M5_STEP_ACCELERATION =
    arm_config::M5_RAW_STEP_ACCELERATION * 0.30f;
constexpr float STORAGE_TRAVEL_M6_EXTENSION_MM = 30.0f;
// Preserve the existing second-round travel preparation delay; M5 holds.
constexpr uint32_t STORAGE2_M7_HEAD_START_MS = 300UL;
// After PROCESS round 1 raises M7 to zero, M5 must return to the standard
// frame zero before storage travel preparation is released.  The requested
// 0.1 ms post-arrival handoff is represented in microseconds so it is not
// rounded up to a full millisecond tick.
constexpr uint32_t PROCESS1_M5_ZERO_HANDOFF_US = 100UL;

// Vision-facing M5 stabilization uses the requested 10 ms cadence.
constexpr uint32_t ARM_BASE_SETTLE_MS = 10UL;
constexpr uint32_t FIRST_ENDPOINT_M7_SETTLE_MS = 10UL;
// PROCESS entry has already verified the commanded M5 pulse target.  Keep the
// user-selected 0.1 ms handoff separate from the 10 ms vision-facing settles
// used after actual endpoint positioning.
constexpr uint32_t PROCESS_ENTRY_M5_SETTLE_US = 100UL;
// The initial endpoint-preparation gripper/M7 boundary uses the same requested
// 0.1 ms command handoff in each direction.  Gripper travel itself remains
// fully covered by GRIPPER_OPEN_SETTLE_MS.
constexpr uint32_t ENDPOINT_GRIPPER_M7_HANDOFF_US = 100UL;
static_assert(
    FIRST_ENDPOINT_M7_SETTLE_MS == ARM_BASE_SETTLE_MS,
    "Endpoint M5/M7 vision settles must remain matched");
static_assert(
    PROCESS_ENTRY_M5_SETTLE_US == 100UL &&
        ENDPOINT_GRIPPER_M7_HANDOFF_US == 100UL,
    "Requested PROCESS entry handoffs must remain exactly 0.1 ms");
// Endpoint vision moves have already been position-verified. The remaining
// M5-to-M6 handoff is reduced by exactly 80% without changing the normal arm
// base settle used by travel, recovery, or non-vision standardization.
constexpr uint32_t ENDPOINT_BASE_TO_EXTENSION_SETTLE_PREVIOUS_MS = 50UL;
constexpr uint32_t ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS = 10UL;
static_assert(
    ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS * 5UL ==
        ENDPOINT_BASE_TO_EXTENSION_SETTLE_PREVIOUS_MS,
    "Endpoint M5-to-M6 handoff must remain reduced by 80 percent");
constexpr uint32_t ARM_TRANSFER_BASE_SETTLE_MS = 5UL;
constexpr uint32_t ARM_TRANSFER_RETURN_SETTLE_MS = 0UL;
constexpr uint32_t ARM_BASE_MOTION_TIMEOUT_MS = 4500UL;

constexpr uint8_t ARM_EXTENSION_ADDRESS = 6U;
constexpr uint8_t ARM_LIFT_ADDRESS = 7U;
constexpr float FULL_STEPS_PER_REVOLUTION =
    static_cast<float>(
        gongchuang::config::arm_hardware::
            FULL_STEPS_PER_REVOLUTION);

constexpr float M6_MICROSTEPS =
    static_cast<float>(
        gongchuang::config::arm_hardware::M6_MICROSTEPS);
constexpr float M7_MICROSTEPS =
    static_cast<float>(
        gongchuang::config::arm_hardware::M7_MICROSTEPS);
const float M6_TRAVEL_PER_REVOLUTION_MM =
    PI_F *
    gongchuang::config::arm_hardware::
        M6_PINION_PITCH_DIAMETER_MM;
constexpr float M7_TRAVEL_PER_REVOLUTION_MM =
    gongchuang::config::arm_hardware::
        M7_LEAD_MM_PER_REVOLUTION;
const float M6_PULSES_PER_MM =
    FULL_STEPS_PER_REVOLUTION * M6_MICROSTEPS /
    M6_TRAVEL_PER_REVOLUTION_MM;
constexpr float M7_PULSES_PER_MM =
    FULL_STEPS_PER_REVOLUTION * M7_MICROSTEPS /
    M7_TRAVEL_PER_REVOLUTION_MM;
constexpr uint8_t M6_EXTEND_DIRECTION = 0U;
constexpr uint8_t M6_RETRACT_DIRECTION = 1U;
constexpr uint8_t M7_RAISE_DIRECTION = 1U;
constexpr uint8_t M7_LOWER_DIRECTION = 0U;

constexpr uint16_t M6_SPEED_RPM =
    arm_config::M6_STANDARD_SPEED_RPM;
constexpr uint8_t M6_ACCELERATION =
    arm_config::M6_STANDARD_ACCELERATION;
constexpr uint16_t RAW_M6_SPEED_RPM =
    arm_config::M6_RAW_SPEED_RPM;
constexpr uint8_t RAW_M6_ACCELERATION =
    arm_config::M6_RAW_ACCELERATION;
constexpr uint16_t PLACE_M6_SPEED_RPM =
    arm_config::M6_PLACE_SPEED_RPM;
constexpr uint8_t PLACE_M6_ACCELERATION =
    arm_config::M6_PLACE_ACCELERATION;
constexpr uint16_t RETURN_M6_SPEED_RPM =
    arm_config::M6_RETURN_SPEED_RPM;
constexpr uint8_t RETURN_M6_ACCELERATION =
    arm_config::M6_RETURN_ACCELERATION;
constexpr uint16_t RETURN_M7_SPEED_RPM =
    arm_config::M7_RETURN_SPEED_RPM;
constexpr uint8_t RETURN_M7_ACCELERATION =
    arm_config::M7_RETURN_ACCELERATION;

constexpr uint16_t ENDPOINT_FINE_M6_SPEED_RPM = 288U;
constexpr uint8_t ENDPOINT_FINE_M6_ACCELERATION = 149U;
constexpr uint16_t ENDPOINT_COARSE_M6_SPEED_RPM = 432U;
constexpr uint8_t ENDPOINT_COARSE_M6_ACCELERATION = 185U;
constexpr uint16_t STORAGE_TRAVEL_M6_SPEED_RPM =
    PLACE_M6_SPEED_RPM;
constexpr uint8_t STORAGE_TRAVEL_M6_ACCELERATION =
    PLACE_M6_ACCELERATION;
// Both PROCESS departures prepare M6/M7 while the chassis is travelling.
// Keep the second departure on the same proven low-impact M6 profile as the
// first; do not restore PLACE_M6_* here or step 17 will receive a much larger
// arm disturbance than step 9.
constexpr uint16_t PROCESS_STORAGE_TRAVEL_M6_SPEED_RPM = 162U;
constexpr uint8_t PROCESS_STORAGE_TRAVEL_M6_ACCELERATION = 42U;
// STORAGE1 -> RAW2 pre-positioning retains 40% of the current RAW M6
// profile (a 60% reduction) while the chassis is already travelling.
constexpr uint16_t STORAGE1_RAW2_M6_SPEED_RPM =
    static_cast<uint16_t>(
        (static_cast<uint32_t>(
             RAW_M6_SPEED_RPM) *
             40UL +
         50UL) /
        100UL);
constexpr uint8_t STORAGE1_RAW2_M6_ACCELERATION =
    static_cast<uint8_t>(
        (static_cast<uint16_t>(
             RAW_M6_ACCELERATION) *
             40U +
         50U) /
        100U);
constexpr uint16_t STORAGE_TRAVEL_M7_SPEED_RPM = 2160U;
constexpr uint8_t STORAGE_TRAVEL_M7_ACCELERATION = 171U;
constexpr uint16_t M6_RECOVERY_SPEED_RPM = 216U;
constexpr uint8_t M6_RECOVERY_ACCELERATION = 128U;
constexpr uint16_t M7_RECOVERY_SPEED_RPM = 2160U;
constexpr uint8_t M7_RECOVERY_ACCELERATION = 171U;

constexpr float M6_STARTUP_WORKING_ZERO_OFFSET_MM =
    gongchuang::config::arm_hardware::
        M6_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float M7_STARTUP_WORKING_ZERO_OFFSET_MM =
    gongchuang::config::arm_hardware::
        M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr uint16_t ARM_LINEAR_STARTUP_ZERO_SPEED_RPM = 180U;
constexpr uint8_t ARM_LINEAR_STARTUP_ZERO_ACCELERATION = 139U;
constexpr uint32_t ARM_LINEAR_POSITION_READ_TIMEOUT_MS = 120UL;

constexpr uint16_t M7_SPEED_RPM =
    gongchuang::config::arm_hardware::M7_TRAVEL_SPEED_RPM;
constexpr uint8_t M7_ACCELERATION =
    gongchuang::config::arm_hardware::M7_TRAVEL_ACCELERATION;

constexpr float M7_ZERO_SOFT_LANDING_DISTANCE_MM =
    gongchuang::config::arm_hardware::
        M7_ZERO_SOFT_LANDING_DISTANCE_MM;
constexpr uint16_t M7_ZERO_SOFT_LANDING_SPEED_RPM =
    gongchuang::config::arm_hardware::
        M7_ZERO_SOFT_LANDING_SPEED_RPM;
constexpr uint8_t M7_ZERO_SOFT_LANDING_ACCELERATION =
    gongchuang::config::arm_hardware::
        M7_ZERO_SOFT_LANDING_ACCELERATION;
constexpr float M6_CONTACT_SOFT_LANDING_DISTANCE_MM =
    gongchuang::config::arm_hardware::
        M6_CONTACT_SOFT_LANDING_DISTANCE_MM;
constexpr uint8_t M6_CONTACT_SOFT_LANDING_ACCELERATION =
    gongchuang::config::arm_hardware::
        M6_CONTACT_SOFT_LANDING_ACCELERATION;
constexpr float M7_CONTACT_SOFT_LANDING_DISTANCE_MM =
    gongchuang::config::arm_hardware::
        M7_CONTACT_SOFT_LANDING_DISTANCE_MM;
constexpr uint8_t M7_CONTACT_SOFT_LANDING_ACCELERATION =
    gongchuang::config::arm_hardware::
        M7_CONTACT_SOFT_LANDING_ACCELERATION;
constexpr uint16_t RAW_M7_SPEED_RPM = M7_SPEED_RPM;
constexpr uint8_t RAW_M7_ACCELERATION = M7_ACCELERATION;

constexpr uint16_t ENDPOINT_FINE_M7_SPEED_RPM = 4493U;
constexpr uint8_t ENDPOINT_FINE_M7_ACCELERATION = 232U;

constexpr float RING_PLACE_FINAL_DESCENT_MM =
    M7_CONTACT_SOFT_LANDING_DISTANCE_MM;
constexpr uint16_t M7_RING_PLACE_SPEED_RPM =
    arm_config::M7_RING_PLACE_SPEED_RPM;
constexpr uint8_t M7_RING_PLACE_ACCELERATION =
    arm_config::M7_RING_PLACE_ACCELERATION;
constexpr uint32_t ARM_LINEAR_POWER_ON_SETTLE_MS = 1500UL;
constexpr uint32_t ARM_AXIS_COMMAND_GUARD_MS = 30UL;

constexpr uint32_t ARM_AXIS_ENABLE_RESPONSE_WAIT_MS = 70UL;
// A normal FD position ACK arrives within a few milliseconds.  If the ACK is
// missing, first allow one fresh status frame to prove that the motor actually
// started; otherwise recover the lost command without waiting for the full
// motion timeout.  The extra grace also covers a temporarily missing status
// response on the shared M6/M7 bus.
constexpr uint32_t ARM_AXIS_POSITION_ACK_TIMEOUT_MS = 100UL;
constexpr uint32_t ARM_AXIS_POSITION_ACK_STATUS_GRACE_MS = 60UL;
constexpr uint32_t ARM_AXIS_STATUS_INTERVAL_MS = 30UL;
constexpr uint32_t ARM_AXIS_MINIMUM_ON_POSITION_MS = 120UL;
// M6 uses the same non-blocking two-sample encoder arrival proof as M7.  This
// bypasses only a delayed driver terminal flag; the rack must already be close
// to the commanded target, stable, enabled and fault-free before M7 is
// released.  Any missing/rejected query falls back to the normal status gate.
constexpr bool M6_ENCODER_FAST_ARRIVAL_ENABLED = true;
constexpr uint32_t M6_FAST_ARRIVAL_QUERY_INTERVAL_MS = 20UL;
constexpr uint32_t M6_FAST_ARRIVAL_SAMPLE_GAP_MS = 10UL;
constexpr uint32_t M6_FAST_ARRIVAL_RESPONSE_TIMEOUT_MS = 20UL;
constexpr uint32_t M6_FAST_ARRIVAL_STATUS_FRESH_MS = 45UL;
constexpr float M6_FAST_ARRIVAL_TARGET_TOLERANCE_MM = 0.35f;
constexpr float M6_FAST_ARRIVAL_STABILITY_MM = 0.15f;
// EMM42 V5 M7 gets a tighter verified-arrival cadence for workstation
// gripper handoffs. The same enabled/on-position/no-fault/command-evidence
// checks still apply; only the polling and minimum command-age are reduced.
constexpr uint32_t M7_AXIS_STATUS_INTERVAL_MS = 10UL;
constexpr uint32_t M7_AXIS_MINIMUM_ON_POSITION_MS = 100UL;
// EMM42 V5 can delay its position-complete indication after the shaft has
// physically settled. Poll the real-time encoder position and accept M7 only
// after two close, stable samples plus a fresh healthy status frame.
constexpr bool M7_ENCODER_FAST_ARRIVAL_ENABLED = true;
constexpr uint32_t M7_FAST_ARRIVAL_QUERY_INTERVAL_MS = 20UL;
constexpr uint32_t M7_FAST_ARRIVAL_SAMPLE_GAP_MS = 10UL;
constexpr uint32_t M7_FAST_ARRIVAL_RESPONSE_TIMEOUT_MS = 20UL;
constexpr uint32_t M7_FAST_ARRIVAL_STATUS_FRESH_MS = 30UL;
constexpr float M7_FAST_ARRIVAL_TARGET_TOLERANCE_MM = 0.35f;
constexpr float M7_FAST_ARRIVAL_STABILITY_MM = 0.15f;
static_assert(
    M7_AXIS_STATUS_INTERVAL_MS < ARM_AXIS_STATUS_INTERVAL_MS &&
        M7_AXIS_MINIMUM_ON_POSITION_MS <=
            ARM_AXIS_MINIMUM_ON_POSITION_MS,
    "M7 verified-arrival response must be tighter than the normal axis gate");
constexpr uint32_t ARM_AXIS_EXPECTED_COMPLETION_VERIFY_MARGIN_MS =
    250UL;
constexpr uint32_t ARM_AXIS_MINIMUM_TIMEOUT_MS = 2500UL;
constexpr uint32_t ARM_AXIS_MAXIMUM_TIMEOUT_MS = 30000UL;
constexpr uint8_t ARM_AXIS_TERMINAL_CONFIRMATION_SAMPLES = 2U;
constexpr uint8_t ARM_AXIS_TERMINAL_VERIFY_MAX_FAILURES = 2U;
constexpr float ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM = 0.60f;

constexpr uint8_t ARM_AXIS_STALL_CONFIRMATION_SAMPLES = 3U;

constexpr uint8_t ARM_AXIS_MAXIMUM_RECOVERY_ATTEMPTS = 1U;
constexpr uint32_t ARM_AXIS_RECOVERY_STOP_SETTLE_MS = 40UL;
constexpr uint32_t ARM_AXIS_RECOVERY_TOTAL_TIMEOUT_MS = 5500UL;
constexpr uint32_t ARM_AXIS_RECOVERY_SAMPLE_SETTLE_MS = 10UL;
constexpr float ARM_AXIS_RECOVERY_POSITION_STABILITY_MM = 0.80f;
constexpr float ARM_AXIS_RECOVERY_PATH_MARGIN_MM = 3.0f;
static_assert(
    M6_FAST_ARRIVAL_SAMPLE_GAP_MS <
            M6_FAST_ARRIVAL_QUERY_INTERVAL_MS &&
        M6_FAST_ARRIVAL_RESPONSE_TIMEOUT_MS <=
            ARM_LINEAR_POSITION_READ_TIMEOUT_MS &&
        M6_FAST_ARRIVAL_TARGET_TOLERANCE_MM <
            ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM &&
        M6_FAST_ARRIVAL_STABILITY_MM <
            ARM_AXIS_RECOVERY_POSITION_STABILITY_MM &&
        M7_FAST_ARRIVAL_SAMPLE_GAP_MS <
            M7_FAST_ARRIVAL_QUERY_INTERVAL_MS &&
        M7_FAST_ARRIVAL_RESPONSE_TIMEOUT_MS <=
            ARM_LINEAR_POSITION_READ_TIMEOUT_MS &&
        M7_FAST_ARRIVAL_TARGET_TOLERANCE_MM <
            ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM &&
        M7_FAST_ARRIVAL_STABILITY_MM <
            ARM_AXIS_RECOVERY_POSITION_STABILITY_MM,
    "M6/M7 encoder arrival gates must remain faster and stricter than recovery");

constexpr float ARM_LINEAR_STARTUP_PROBE_MM = 0.5f;
constexpr float ARM_LINEAR_ZERO_ANGLE_RATIO_MINIMUM = 0.80f;
constexpr float ARM_LINEAR_ZERO_ANGLE_RATIO_MAXIMUM = 1.20f;

constexpr float M6_STANDARD_EXTENSION_MM = 0.0f;

// Ring 2 and both RAW pickups may use the calibrated near-side fallback.
// With the 10 mm startup working-zero offset, -9.5 mm still retains 0.5 mm
// before the physical retraction stop.
constexpr float M6_RING2_MINIMUM_EXTENSION_MM = -9.5f;
constexpr float M6_RAW_PICK_MINIMUM_EXTENSION_MM = -9.5f;
static_assert(
    M6_STARTUP_WORKING_ZERO_OFFSET_MM +
            M6_RING2_MINIMUM_EXTENSION_MM >=
        0.5f,
    "Ring-2 M6 retraction must retain 0.5 mm hard-stop margin");
static_assert(
    M6_STARTUP_WORKING_ZERO_OFFSET_MM +
            M6_RAW_PICK_MINIMUM_EXTENSION_MM >=
        0.5f,
    "RAW M6 retraction must retain 0.5 mm hard-stop margin");
constexpr float M7_STANDARD_HEIGHT_MM = 0.0f;

// Physical verification found 170 mm of total M6 travel. The startup process
// deliberately moves 10 mm away from the retracted hard stop, leaving 160 mm
// of logical extension. Do not confuse the physical and working-zero ranges.
constexpr float M6_MAXIMUM_PHYSICAL_EXTENSION_MM = 170.0f;
constexpr float M6_MAXIMUM_EXTENSION_MM =
    M6_MAXIMUM_PHYSICAL_EXTENSION_MM -
    M6_STARTUP_WORKING_ZERO_OFFSET_MM;

constexpr float M7_MINIMUM_PHYSICAL_HEIGHT_MM = -160.0f;
constexpr float M7_MINIMUM_HEIGHT_MM =
    M7_MINIMUM_PHYSICAL_HEIGHT_MM +
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float ARM_AXIS_POSITION_TOLERANCE_MM = 0.05f;

// Both RAW visits descend another 2 mm from the previous 75.2 mm pickup.
constexpr float RAW_PICK_PHYSICAL_LOWER_MM = 77.2f;
constexpr float HOUGH_VISION_PHYSICAL_LOWER_MM = 90.0f;

constexpr float ENDPOINT_FINE_VISION_PHYSICAL_LOWER_MM =
    105.0f;
constexpr float PROCESS_PLACE_PHYSICAL_LOWER_MM = 148.0f;
constexpr float STORAGE_ROUND1_PLACE_PHYSICAL_LOWER_MM =
    148.0f;
constexpr float STORAGE_ROUND2_PLACE_PHYSICAL_LOWER_MM =
    90.0f;
constexpr float RAW_PICK_LOWER_MM =
    RAW_PICK_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float HOUGH_VISION_LOWER_MM =
    HOUGH_VISION_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float ENDPOINT_FINE_VISION_LOWER_MM =
    ENDPOINT_FINE_VISION_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float HOUGH_VISION_HEIGHT_MM =
    M7_STANDARD_HEIGHT_MM - HOUGH_VISION_LOWER_MM;
// Endpoint acquisition and final confirmation share the former fine height.
// Coarse/fine now describe only the planar servo profile and acceptance gate;
// M7 remains fixed while rings 1 and 3 are mapped.
constexpr float ENDPOINT_VISION_HEIGHT_MM =
    M7_STANDARD_HEIGHT_MM - ENDPOINT_FINE_VISION_LOWER_MM;

constexpr float PROCESS_PLACE_LOWER_MM =
    PROCESS_PLACE_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float STORAGE_ROUND1_PLACE_LOWER_MM =
    STORAGE_ROUND1_PLACE_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float STORAGE_ROUND2_PLACE_LOWER_MM =
    STORAGE_ROUND2_PLACE_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
static_assert(
    PROCESS_PLACE_LOWER_MM +
            arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM +
            arm_config::PROCESS_ROUND1_RING_RETURN_PICK_ADDITIONAL_LOWER_MM <=
        -M7_MINIMUM_HEIGHT_MM &&
    PROCESS_PLACE_LOWER_MM +
                arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM +
                arm_config::PROCESS_ROUND2_RING_RETURN_PICK_ADDITIONAL_LOWER_MM <=
            -M7_MINIMUM_HEIGHT_MM,
    "Ring-return pickup extra lower must remain inside M7 travel");
static_assert(
    PROCESS_PLACE_LOWER_MM +
            arm_config::TARGET_PLACE_EXTRA_LOWER_MM <=
        -M7_MINIMUM_HEIGHT_MM &&
        STORAGE_ROUND1_PLACE_LOWER_MM +
                arm_config::TARGET_PLACE_EXTRA_LOWER_MM <=
            -M7_MINIMUM_HEIGHT_MM,
    "Target placement extra lower must remain inside M7 travel");

constexpr float GRIPPER_CLOSE_ANGLE_DEGREES =
    gongchuang::config::gripper::CLOSE_ANGLE_DEGREES;
constexpr float GRIPPER_OPEN_ANGLE_DEGREES =
    gongchuang::config::gripper::OPEN_ANGLE_DEGREES;
// Restore RAW pickup to the original shared open-jaw angle.
constexpr float RAW_GRIPPER_OPEN_ANGLE_DEGREES =
    GRIPPER_OPEN_ANGLE_DEGREES;
constexpr float GRIPPER_MAX_OPEN_ANGLE_DEGREES =
    gongchuang::config::gripper::MAX_OPEN_ANGLE_DEGREES;
constexpr uint16_t GRIPPER_INTERVAL_MS = 60U;
// Target placement keeps the prior 40 ms release. The three user-selected
// operations (tray pickup, target pickup, tray release) use the doubled-speed
// 20 ms command interval.
constexpr uint16_t GRIPPER_TARGET_PLACE_OPEN_INTERVAL_MS = 40U;
constexpr uint16_t GRIPPER_DOUBLE_SPEED_INTERVAL_MS = 20U;
constexpr uint16_t GRIPPER_OPEN_POWER_MW = 0U;
constexpr uint16_t GRIPPER_CLOSE_POWER_MW = 2000U;

// Opening has no clamping-load uncertainty, so its gate matches the commanded
// 60 ms motion exactly; the short gripper-to-M7 handoff follows separately.
constexpr uint32_t GRIPPER_OPEN_SETTLE_MS = 60UL;
constexpr uint32_t GRIPPER_CLOSE_SETTLE_MS = 120UL;
constexpr uint32_t GRIPPER_TARGET_PLACE_OPEN_SETTLE_MS = 40UL;
constexpr uint32_t GRIPPER_TRAY_RELEASE_OPEN_SETTLE_MS = 20UL;
// Restore the proven loaded-jaw settle gates for rough-processing transfers.
// The servo command itself remains at the selected 20 ms interval, but M7 may
// not reverse until the jaw has had enough time to establish clamping force.
constexpr uint32_t GRIPPER_TARGET_PICK_CLOSE_SETTLE_MS = 120UL;
constexpr uint32_t GRIPPER_TRAY_PICK_CLOSE_SETTLE_MS = 160UL;
// Initial PROCESS/STORAGE1 endpoint preparation used to launch the storage
// turntable in the same service pass as M7.  Launch it with the already-proven
// gripper-open gate instead: M7 keeps its original absolute deadline, while
// the turntable receives a 60 ms head start (plus the existing 0.1 ms guard).
constexpr uint32_t INITIAL_STORAGE_SERVO_M7_HEAD_START_MS =
    GRIPPER_OPEN_SETTLE_MS;
static_assert(
    INITIAL_STORAGE_SERVO_M7_HEAD_START_MS == 60UL,
    "Initial storage turntable must lead M7 by the 60 ms gripper gate");
static_assert(
    GRIPPER_OPEN_SETTLE_MS >=
        static_cast<uint32_t>(GRIPPER_INTERVAL_MS),
    "Normal gripper open gate must cover the full servo command");
static_assert(
    GRIPPER_CLOSE_SETTLE_MS >=
        static_cast<uint32_t>(GRIPPER_INTERVAL_MS),
    "Normal gripper close gate must cover the full servo command");
// M7 terminal verification already confirms that the lift has stopped. Keep
// only a short command handoff in each direction; the separate settle values
// above cover the actual servo movement and clamping time.
constexpr uint32_t WORK_M7_TO_GRIPPER_GAP_MS = 10UL;
constexpr uint32_t WORK_GRIPPER_TO_M7_GAP_MS = 10UL;
// Keep every RAW/PROCESS gripper-close command on its existing schedule.
// Only the following loaded M7 rise is held back another 50 ms.
// STORAGE pickup/release timing remains independent and unchanged.
constexpr uint32_t RAW_PROCESS_PICKUP_M7_RAISE_ADDITIONAL_DELAY_MS =
    50UL;
// During both PROCESS rounds, a ring picked from the rough-processing area
// needs another 100 ms to develop clamping force before M7 begins to rise.
// This is additive to the existing close-settle and short handoff gates.
constexpr uint32_t RING_RETURN_M7_RAISE_EXTRA_DELAY_MS = 100UL;
// In each PROCESS visit, only the first ring pickup keeps the existing gripper
// close command time and delays the following loaded M7 rise by another 30 ms.
constexpr uint32_t PROCESS_FIRST_RING_PICK_M7_RAISE_EXTRA_DELAY_MS = 30UL;
// On that same first ring-to-tray transfer, M5 alone stops one degree before
// the tray angle and completes the last degree with this triangular profile.
// At 44.44 pulses/degree, 18000 pulse/s^2 makes this one-degree triangular
// final approach take approximately 0.10 s and still end at the exact target.
constexpr float PROCESS_FIRST_RING_M5_FINAL_BUFFER_DEGREES = 1.0f;
constexpr float PROCESS_FIRST_RING_M5_FINAL_BUFFER_MAXIMUM_STEP_RATE =
    1000.0f;
constexpr float PROCESS_FIRST_RING_M5_FINAL_BUFFER_STEP_ACCELERATION =
    18000.0f;
// RAW pickup requires a 0.1 ms handoff after M7 has been position-verified.
// Keep this gate in microseconds so it is not rounded up to a millisecond.
constexpr uint32_t RAW_M7_TO_GRIPPER_HANDOFF_US = 100UL;
// Target placement is the one requested 50% exception. Pickup from either
// side and release into the tray retain the common 10 ms handoff.
constexpr uint32_t WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_PREVIOUS_MS = 10UL;
constexpr uint32_t WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_MS = 5UL;
constexpr uint32_t WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS = 100UL;
constexpr uint32_t RING_RETURN_STORAGE_COMMAND_DELAY_MS = 500UL;
// During both RAW visits, the first and second items must be physically clear
// of the gripper before the tray indexes to the next slot.  Keep this delay
// independent from the arm's remembered-pickup-pose preparation; item three
// has no next-slot command and therefore retains its existing exit sequence.
constexpr uint32_t RAW_NEXT_STORAGE_COMMAND_DELAY_MS = 1000UL;
static_assert(
    WORK_M7_TO_GRIPPER_GAP_MS > 0UL &&
        WORK_GRIPPER_TO_M7_GAP_MS > 0UL &&
        WORK_M7_TO_GRIPPER_GAP_MS <=
            WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS,
    "Workstation M7/gripper command handoffs must remain nonzero");
static_assert(
    RAW_M7_TO_GRIPPER_HANDOFF_US == 100UL,
    "RAW M7-to-gripper handoff must remain exactly 0.1 ms");
static_assert(
    RAW_PROCESS_PICKUP_M7_RAISE_ADDITIONAL_DELAY_MS == 50UL,
    "RAW/PROCESS pickup M7 rise must lag the existing timeline by 50 ms");
static_assert(
    RING_RETURN_M7_RAISE_EXTRA_DELAY_MS == 100UL,
    "PROCESS ring-return M7 rise must retain the extra 0.1 s clamp delay");
static_assert(
    PROCESS_FIRST_RING_PICK_M7_RAISE_EXTRA_DELAY_MS == 30UL &&
        PROCESS_FIRST_RING_M5_FINAL_BUFFER_DEGREES == 1.0f &&
        PROCESS_FIRST_RING_M5_FINAL_BUFFER_MAXIMUM_STEP_RATE == 1000.0f &&
        PROCESS_FIRST_RING_M5_FINAL_BUFFER_STEP_ACCELERATION == 18000.0f,
    "PROCESS first-ring pickup must retain its M7 +30 ms and M5 final buffer");
static_assert(
    WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_MS * 2UL ==
            WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_PREVIOUS_MS &&
        WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_MS <=
            WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS,
    "Target-place M7-to-gripper handoff must remain 50 percent shorter");
static_assert(
    RING_RETURN_STORAGE_COMMAND_DELAY_MS >= 500UL,
    "Ring-return storage command delay must be at least 500 ms");
static_assert(
    RAW_NEXT_STORAGE_COMMAND_DELAY_MS == 1000UL,
    "RAW next-slot command must wait exactly 1 s after release completion");
static_assert(
    GRIPPER_TARGET_PLACE_OPEN_SETTLE_MS >=
        static_cast<uint32_t>(
            GRIPPER_TARGET_PLACE_OPEN_INTERVAL_MS),
    "Target-place open gate must cover the full servo command");
static_assert(
    GRIPPER_TRAY_RELEASE_OPEN_SETTLE_MS >=
        static_cast<uint32_t>(GRIPPER_DOUBLE_SPEED_INTERVAL_MS),
    "Tray-release open gate must cover the doubled-speed command");
static_assert(
    GRIPPER_TARGET_PICK_CLOSE_SETTLE_MS >=
        static_cast<uint32_t>(GRIPPER_DOUBLE_SPEED_INTERVAL_MS) &&
        GRIPPER_TRAY_PICK_CLOSE_SETTLE_MS >=
            static_cast<uint32_t>(GRIPPER_DOUBLE_SPEED_INTERVAL_MS),
    "Pickup close gates must cover the doubled-speed command");
// 30% lower turntable speed means 1 / 0.7 times the former 286 ms duration.
constexpr uint16_t STORAGE_SERVO_INTERVAL_MS = 409U;
constexpr uint32_t STORAGE_SERVO_SETTLE_MS = 510UL;
// RAW/PROCESS departure tray motion retains 30% of the present speed.  The
// FashionStar interval is motion duration, so 70% slower means divide the
// original speed by 0.30 (equivalently multiply the interval by 10/3).
constexpr uint16_t DEPARTURE_STORAGE_SERVO_INTERVAL_MS =
    static_cast<uint16_t>(
        static_cast<uint32_t>(STORAGE_SERVO_INTERVAL_MS) * 10UL / 3UL);
constexpr uint32_t DEPARTURE_STORAGE_SERVO_SETTLE_MS =
    static_cast<uint32_t>(DEPARTURE_STORAGE_SERVO_INTERVAL_MS) + 100UL;
// RAW slot changes use the requested 200 ms motion, but pickup is released by
// real-angle confirmation rather than by this nominal duration alone.
constexpr uint16_t RAW_STORAGE_SERVO_INTERVAL_MS = 200U;
constexpr uint32_t RAW_STORAGE_SERVO_SETTLE_MS = 200UL;
constexpr uint32_t RAW_STORAGE_SERVO_VERIFY_FIRST_QUERY_MS =
    RAW_STORAGE_SERVO_SETTLE_MS;
// PROCESS1 travel preparation uses the common 70%-slower departure tray
// profile while the chassis is already moving toward STORAGE1.
constexpr uint16_t PROCESS1_STORAGE_TRAVEL_SERVO_INTERVAL_MS =
    DEPARTURE_STORAGE_SERVO_INTERVAL_MS;
constexpr uint32_t PROCESS1_STORAGE_TRAVEL_SERVO_SETTLE_MS =
    DEPARTURE_STORAGE_SERVO_SETTLE_MS;
// STORAGE1 -> RAW2 reduces the current parking speed by 60%. FashionStar
// expresses speed as a duration, so retaining 40% speed multiplies the
// current 409 ms interval by 2.5.
constexpr uint16_t STORAGE1_RAW2_SERVO_INTERVAL_MS =
    static_cast<uint16_t>(
        (static_cast<uint32_t>(
             STORAGE_SERVO_INTERVAL_MS) *
             5UL +
         1UL) /
        2UL);
constexpr uint32_t STORAGE1_RAW2_SERVO_SETTLE_MS =
    static_cast<uint32_t>(
        STORAGE1_RAW2_SERVO_INTERVAL_MS) +
    100UL;
constexpr uint32_t STORAGE1_RAW2_M7_HANDOFF_US = 100UL;
static_assert(
    STORAGE_SERVO_SETTLE_MS >=
        static_cast<uint32_t>(STORAGE_SERVO_INTERVAL_MS) + 100UL,
    "Storage servo settle gate must retain at least 100 ms margin");
static_assert(
    RAW_STORAGE_SERVO_INTERVAL_MS == 200U &&
        RAW_STORAGE_SERVO_SETTLE_MS ==
            static_cast<uint32_t>(RAW_STORAGE_SERVO_INTERVAL_MS),
    "RAW tray command and first verification must remain at 200 ms");
static_assert(
    DEPARTURE_STORAGE_SERVO_INTERVAL_MS == 1363U &&
        DEPARTURE_STORAGE_SERVO_SETTLE_MS == 1463UL,
    "RAW/PROCESS departure tray speed must remain 70 percent lower");
static_assert(
    STORAGE1_RAW2_M6_SPEED_RPM == 302U &&
        STORAGE1_RAW2_M6_ACCELERATION == 82U &&
        STORAGE1_RAW2_SERVO_INTERVAL_MS == 1023U &&
        STORAGE1_RAW2_SERVO_SETTLE_MS == 1123UL &&
        STORAGE1_RAW2_M7_HANDOFF_US == 100UL,
    "STORAGE1 exit must retain 40 percent M6/tray speed after 100 us");
// A fixed time gate cannot prove that the tray actually moved.  STORAGE2
// starts checking at the commanded motion duration and requires two
// consecutive in-deadband samples.  Give the loaded tray a simple 800 ms
// observation window before one same-slot retry, rather than interrupting its
// still-normal motion at the old 510 ms boundary.
constexpr uint32_t STORAGE_SERVO_VERIFY_FIRST_QUERY_MS =
    static_cast<uint32_t>(STORAGE_SERVO_INTERVAL_MS);
constexpr uint32_t STORAGE_SERVO_VERIFY_QUERY_INTERVAL_MS = 20UL;
constexpr uint32_t STORAGE_SERVO_VERIFY_MAX_OBSERVE_MS = 800UL;
constexpr uint8_t STORAGE_SERVO_VERIFY_CONFIRMATIONS = 2U;
constexpr uint8_t STORAGE_SERVO_VERIFY_MAX_RETRIES = 1U;
constexpr float STORAGE_SERVO_VERIFY_TOLERANCE_DEGREES = 2.5f;
static_assert(
    STORAGE_SERVO_VERIFY_FIRST_QUERY_MS <=
            STORAGE_SERVO_SETTLE_MS &&
        STORAGE_SERVO_VERIFY_MAX_OBSERVE_MS >
            STORAGE_SERVO_SETTLE_MS &&
        STORAGE_SERVO_VERIFY_CONFIRMATIONS >= 2U &&
        STORAGE_SERVO_VERIFY_MAX_RETRIES >= 1U,
    "Storage servo verification must allow arrival before retry");

constexpr float STORAGE_SERVO_PARK_ANGLE_DEGREES = 165.0f;
constexpr float STORAGE_SERVO_WORK_ZERO_ANGLE_DEGREES = -5.0f;
constexpr float STORAGE_SERVO_CLOCKWISE_STEP_DEGREES = -90.0f;
constexpr float STORAGE_SERVO_POSITIONS_DEGREES[4] = {
    STORAGE_SERVO_WORK_ZERO_ANGLE_DEGREES,
    STORAGE_SERVO_WORK_ZERO_ANGLE_DEGREES +
        STORAGE_SERVO_CLOCKWISE_STEP_DEGREES,
    STORAGE_SERVO_WORK_ZERO_ANGLE_DEGREES +
        2.0f * STORAGE_SERVO_CLOCKWISE_STEP_DEGREES,
    STORAGE_SERVO_WORK_ZERO_ANGLE_DEGREES};
// Ring-1 clearance is fixed 180 degrees from the chassis-departure angle.
// It never depends on which tray slot was commanded immediately beforehand.
constexpr float RING1_TRAY_CLEARANCE_ANGLE_DEGREES =
    STORAGE_SERVO_PARK_ANGLE_DEGREES - 180.0f;

struct StorageServoArrivalGate {
  bool active;
  bool confirmed;
  uint8_t slot;
  uint8_t consecutiveConfirmations;
  uint8_t retryCount;
  uint32_t commandStartMs;
  uint32_t nextQueryMs;
  uint16_t commandIntervalMs;
  uint32_t firstQueryDelayMs;

  StorageServoArrivalGate()
      : active(false),
        confirmed(false),
        slot(0U),
        consecutiveConfirmations(0U),
        retryCount(0U),
        commandStartMs(0UL),
        nextQueryMs(0UL),
        commandIntervalMs(STORAGE_SERVO_INTERVAL_MS),
        firstQueryDelayMs(STORAGE_SERVO_VERIFY_FIRST_QUERY_MS) {}
};

StorageServoArrivalGate storageServoArrivalGate;

constexpr uint8_t MAIXCAM_STOP_REQUEST =
    gongchuang::MaixCamClient::STOP_REQUEST;
constexpr uint8_t MAIXCAM_RED_REQUEST =
    gongchuang::MaixCamClient::RED_REQUEST;
constexpr uint8_t MAIXCAM_YELLOW_REQUEST =
    gongchuang::MaixCamClient::YELLOW_REQUEST;
constexpr uint8_t MAIXCAM_BLUE_REQUEST =
    gongchuang::MaixCamClient::BLUE_REQUEST;
constexpr uint8_t MAIXCAM_GREEN_REQUEST =
    gongchuang::MaixCamClient::GREEN_REQUEST;
constexpr uint8_t MAIXCAM_ALL_COLORS_REQUEST =
    gongchuang::MaixCamClient::ALL_COLORS_REQUEST;
constexpr uint8_t MAIXCAM_HOUGH_CIRCLE_REQUEST =
    gongchuang::MaixCamClient::HOUGH_CIRCLE_REQUEST;

constexpr uint8_t MAIXCAM_ENDPOINT_CIRCLE_REQUEST =
    gongchuang::MaixCamClient::ENDPOINT_CIRCLE_REQUEST;
constexpr int16_t IMAGE_CENTER_X =
    gongchuang::config::vision_link::IMAGE_CENTER_X;
constexpr int16_t IMAGE_CENTER_Y =
    gongchuang::config::vision_link::IMAGE_CENTER_Y;
constexpr int16_t IMAGE_MAX_X = 319;
constexpr int16_t IMAGE_MAX_Y = 239;
constexpr float RAW_NEAR_MM_PER_PIXEL = 40.0f / 72.0f;
constexpr float RAW_FAR_MM_PER_PIXEL = 70.0f / 72.0f;
// The former hard switch at 50 px changed the scale from 40/72 to 70/72
// in one frame, which could jump the commanded M6 target by about 22 mm.
// Blend continuously from the center calibration to the 72 px far anchor.
// RAW_VIEW_EXTENSION_MM remains the real camera pose and is still included
// in the geometry; only the discontinuous scale selection is removed.
constexpr float RAW_PIXEL_MAPPING_BLEND_END_RADIUS_PIXELS = 72.0f;
static_assert(
    RAW_NEAR_MM_PER_PIXEL > 0.0f &&
        RAW_FAR_MM_PER_PIXEL > RAW_NEAR_MM_PER_PIXEL &&
        RAW_PIXEL_MAPPING_BLEND_END_RADIUS_PIXELS > 0.0f,
    "RAW pixel mapping blend must stay positive and ordered");

constexpr float RAW_VIEW_EXTENSION_MM = 20.0f;
// Both RAW visits share rawTargetPose(). Shorten every vision-derived pickup
// reach by 10 mm, so round 1 and round 2 receive the same M6 correction.
constexpr float RAW_PICK_EXTENSION_REDUCTION_MM = 10.0f;

#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
// A is the established 0 deg / 20 mm camera pose. B first shifts the chassis
// 40 mm toward the route's raw-material side (body-left), then scans at
// M6=150 mm from -25 deg and +25 deg. These are search poses, not pickup
// coordinates; every accepted pixel result is remapped from the actual pose.
constexpr float RAW_SEARCH_CHASSIS_LEFT_MM = 40.0f;
constexpr float RAW_SEARCH_B_EXTENSION_MM = 150.0f;
constexpr float RAW_SEARCH_B_CLOCKWISE_ANGLE_DEGREES = -25.0f;
constexpr float RAW_SEARCH_B_COUNTERCLOCKWISE_ANGLE_DEGREES = 25.0f;
constexpr uint32_t RAW_INITIAL_A_NO_TARGET_MS = 750UL;
constexpr uint32_t RAW_NORMAL_NO_TARGET_MS = 350UL;
constexpr uint32_t RAW_POST_PICK_NO_TARGET_MS = 1500UL;

// During the RAW M7 descent, image error is converted to the M5/M6 target
// that should be reached in the estimated remaining descent time. The user
// requested gain is applied once to both tracking speeds.
constexpr float RAW_PICK_TRACKING_VELOCITY_GAIN = 1.5f;
constexpr float RAW_TRACKING_MINIMUM_REMAINING_SECONDS = 0.05f;
constexpr float RAW_TRACKING_M7_EFFECTIVE_SPEED_SCALE = 0.18f;
constexpr float RAW_TRACKING_MINIMUM_M5_STEP_RATE = 300.0f;
constexpr uint16_t RAW_TRACKING_MINIMUM_M6_SPEED_RPM = 30U;
#endif

static_assert(
    RAW_VIEW_EXTENSION_MM >= M6_STANDARD_EXTENSION_MM &&
        RAW_VIEW_EXTENSION_MM <= M6_MAXIMUM_EXTENSION_MM,
    "RAW view extension must stay inside M6 travel");
static_assert(
    RAW_PICK_EXTENSION_REDUCTION_MM >= 0.0f &&
        RAW_PICK_EXTENSION_REDUCTION_MM <=
            M6_MAXIMUM_EXTENSION_MM - M6_STANDARD_EXTENSION_MM,
    "RAW pickup M6 reduction must stay inside the working range");
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
static_assert(
    RAW_SEARCH_B_EXTENSION_MM >= M6_STANDARD_EXTENSION_MM &&
        RAW_SEARCH_B_EXTENSION_MM <= M6_MAXIMUM_EXTENSION_MM &&
        RAW_INITIAL_A_NO_TARGET_MS > RAW_NORMAL_NO_TARGET_MS &&
        RAW_POST_PICK_NO_TARGET_MS > RAW_INITIAL_A_NO_TARGET_MS &&
        RAW_PICK_TRACKING_VELOCITY_GAIN == 1.5f,
    "RAW A/B search and tracking calibration changed unexpectedly");
#endif
constexpr float RING_OUTER_DIAMETER_MM = 85.0f;
constexpr float RING_INNER_DIAMETER_MM = 82.0f;
constexpr float RING_CENTERLINE_DIAMETER_MM =
    0.5f *
    (RING_OUTER_DIAMETER_MM + RING_INNER_DIAMETER_MM);
constexpr float RING_PHYSICAL_RADIUS_MM =
    0.5f * RING_CENTERLINE_DIAMETER_MM;
constexpr float CIRCLE_MM_PER_PIXEL =
    RING_PHYSICAL_RADIUS_MM / 72.0f;

constexpr float ARM_PIVOT_TO_CAMERA_FULLY_RETRACTED_MM =
    125.74f;

constexpr float ARM_PIVOT_TO_GRIPPER_FULLY_RETRACTED_MM =
    125.74f;

constexpr float ARM_PIVOT_TO_CAMERA_CENTER_MM =
    ARM_PIVOT_TO_CAMERA_FULLY_RETRACTED_MM +
    M6_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float ARM_PIVOT_TO_GRIPPER_CENTER_MM =
    ARM_PIVOT_TO_GRIPPER_FULLY_RETRACTED_MM +
    M6_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float RING_ADJACENT_SPACING_MM = 150.0f;
constexpr float RING_ENDPOINT_EXPECTED_SPAN_MM =
    2.0f * RING_ADJACENT_SPACING_MM;
constexpr float RING_ENDPOINT_MAXIMUM_SPAN_ERROR_MM = 35.0f;
constexpr float RING_ENDPOINT_WARNING_SPAN_ERROR_MM = 10.0f;
constexpr uint16_t RING_ENDPOINT_MINIMUM_RADIUS_PIXELS = 40U;
constexpr uint16_t RING_ENDPOINT_MAXIMUM_RADIUS_PIXELS = 110U;
constexpr uint16_t RING_ENDPOINT_MINIMUM_CONFIDENCE = 600U;
constexpr float RING_ENDPOINT_MAXIMUM_RADIUS_RATIO = 1.50f;

constexpr float ENDPOINT_COARSE_CENTER_TOLERANCE_PIXELS = 6.0f;

constexpr float ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS = 5.0f;
constexpr uint8_t ENDPOINT_FINAL_CENTER_CONFIRMATIONS = 2U;
// Every mode-10 response has already passed MaixCAM's multi-frame stability
// gate.  A response inside the final 5 px target with confidence >=800 has at
// most a one-pixel span, so accept it without discarding that valid window and
// starting another full Hough request. Confidence-600 boundary results retain
// the existing independent fine stage and two STM32 confirmations.
constexpr uint16_t ENDPOINT_STABLE_DIRECT_ACCEPT_MINIMUM_CONFIDENCE = 800U;
static_assert(
    ENDPOINT_STABLE_DIRECT_ACCEPT_MINIMUM_CONFIDENCE >
            RING_ENDPOINT_MINIMUM_CONFIDENCE &&
        ENDPOINT_STABLE_DIRECT_ACCEPT_MINIMUM_CONFIDENCE <= 1000U,
    "Endpoint direct acceptance must retain the strong Maix stability gate");
constexpr uint8_t ENDPOINT_MAXIMUM_SERVO_MOVES_PER_STAGE = 5U;

constexpr uint8_t ENDPOINT_MAXIMUM_TOTAL_SERVO_MOVES = 7U;
constexpr float ENDPOINT_COARSE_SERVO_GAIN = 0.85f;
constexpr float ENDPOINT_FINE_SERVO_GAIN = 0.55f;
// Endpoint Hough-circle centering still uses the existing coarse/fine gains,
// then increases the commanded M5/M6 planar correction only when the measured
// miss is large.  Keep errors below 5 mm on the original precision gain.
constexpr float ENDPOINT_SERVO_MEDIUM_DISTANCE_THRESHOLD_MM = 5.0f;
constexpr float ENDPOINT_SERVO_FAR_DISTANCE_THRESHOLD_MM = 10.0f;
constexpr float ENDPOINT_SERVO_MEDIUM_DISTANCE_MULTIPLIER = 1.20f;
constexpr float ENDPOINT_SERVO_FAR_DISTANCE_MULTIPLIER = 1.50f;
constexpr float ENDPOINT_COARSE_MEDIUM_DISTANCE_GAIN =
    ENDPOINT_COARSE_SERVO_GAIN *
    ENDPOINT_SERVO_MEDIUM_DISTANCE_MULTIPLIER;
constexpr float ENDPOINT_COARSE_FAR_DISTANCE_GAIN =
    ENDPOINT_COARSE_SERVO_GAIN *
    ENDPOINT_SERVO_FAR_DISTANCE_MULTIPLIER;
constexpr float ENDPOINT_FINE_MEDIUM_DISTANCE_GAIN =
    ENDPOINT_FINE_SERVO_GAIN *
    ENDPOINT_SERVO_MEDIUM_DISTANCE_MULTIPLIER;
constexpr float ENDPOINT_FINE_FAR_DISTANCE_GAIN =
    ENDPOINT_FINE_SERVO_GAIN *
    ENDPOINT_SERVO_FAR_DISTANCE_MULTIPLIER;
static_assert(
    ENDPOINT_SERVO_MEDIUM_DISTANCE_THRESHOLD_MM <
            ENDPOINT_SERVO_FAR_DISTANCE_THRESHOLD_MM &&
        ENDPOINT_SERVO_MEDIUM_DISTANCE_MULTIPLIER > 1.0f &&
        ENDPOINT_SERVO_FAR_DISTANCE_MULTIPLIER >
            ENDPOINT_SERVO_MEDIUM_DISTANCE_MULTIPLIER,
    "Endpoint distance gain bands must remain ordered");
constexpr float ENDPOINT_COARSE_MAXIMUM_CORRECTION_MM = 25.0f;
constexpr float ENDPOINT_FINE_MAXIMUM_CORRECTION_MM = 6.0f;
// Closed-loop endpoint vision cadence: after each M5/M6 correction reaches
// its verified target, wait 10 ms before requesting the next frame.
constexpr uint32_t ENDPOINT_LOCAL_MOVE_SETTLE_PREVIOUS_MS = 80UL;
constexpr uint32_t ENDPOINT_LOCAL_MOVE_SETTLE_MS = 10UL;
static_assert(
    ENDPOINT_LOCAL_MOVE_SETTLE_MS * 8UL ==
        ENDPOINT_LOCAL_MOVE_SETTLE_PREVIOUS_MS,
    "Endpoint re-recognition settle must remain reduced by 87.5 percent");

constexpr float ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES = 45.0f;
constexpr float ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES = -45.0f;
constexpr float ENDPOINT_SEARCH_SEED_EXTENSION_MM = 90.0f;
constexpr float ENDPOINT_RING1_SEARCH_FALLBACK_STEP_DEGREES = 15.0f;
constexpr float ENDPOINT_RING3_SEARCH_FALLBACK_STEP_DEGREES = 15.0f;
constexpr float ENDPOINT_RING1_SEARCH_MINIMUM_ANGLE_DEGREES = 30.0f;
constexpr float ENDPOINT_RING1_SEARCH_MAXIMUM_ANGLE_DEGREES = 60.0f;
constexpr float ENDPOINT_RING3_SEARCH_MINIMUM_ANGLE_DEGREES = -60.0f;
constexpr float ENDPOINT_RING3_SEARCH_MAXIMUM_ANGLE_DEGREES = -30.0f;
constexpr uint8_t ENDPOINT_SEARCH_MAXIMUM_FALLBACK_MOVES = 2U;
constexpr uint32_t ENDPOINT_VISION_RESULT_TIMEOUT_MS = 2500UL;
constexpr uint8_t ENDPOINT_VISION_MAXIMUM_RETRIES = 1U;
constexpr float RING_TARGET_MINIMUM_ANGLE_DEGREES = -75.0f;
constexpr float RING_TARGET_MAXIMUM_ANGLE_DEGREES = 75.0f;
constexpr float RING_MAP_MAXIMUM_HEADING_DRIFT_DEGREES = 1.1f;
constexpr float RING_ENDPOINT_MAXIMUM_SCAN_HEADING_DELTA_DEGREES =
    0.75f;

constexpr float RING_SCAN_MINIMUM_ANGLE_DEGREES = -80.0f;
constexpr float RING_SCAN_MAXIMUM_ANGLE_DEGREES = 80.0f;

constexpr float IMAGE_Y_TO_ARM_OUTWARD_SIGN = -1.0f;

constexpr float IMAGE_X_TO_BODY_FORWARD_SIGN = -1.0f;
constexpr float IMAGE_Y_TO_BODY_LEFT_SIGN = -1.0f;
constexpr float CIRCLE_CENTER_TOLERANCE_PIXELS = 3.0f;

constexpr uint32_t CIRCLE_CENTER_STABILITY_MS = 0UL;
constexpr uint8_t CIRCLE_STABILITY_MINIMUM_SAMPLES = 1U;
constexpr uint32_t VISION_STABILITY_MAXIMUM_SAMPLE_GAP_MS =
    1200UL;
constexpr float MAXIMUM_VISUAL_CORRECTION_MM = 150.0f;
constexpr float MAXIMUM_ACCUMULATED_VISUAL_CORRECTION_MM = 220.0f;
constexpr uint8_t MAXIMUM_VISUAL_CORRECTION_MOVES = 6U;

const WheelDirections MOTOR_DIRECTIONS(-1, +1, -1, +1);

constexpr uint16_t FIELD_SIZE_MM = 2400U;
constexpr uint16_t FIELD_CENTER_MM = FIELD_SIZE_MM / 2U;
constexpr uint16_t START_ZONE_SIZE_MM = 300U;
constexpr uint16_t START_ZONE_MIN_X_MM =
    FIELD_SIZE_MM - START_ZONE_SIZE_MM;
constexpr uint16_t START_ZONE_1_MIN_Y_MM =
    FIELD_SIZE_MM - START_ZONE_SIZE_MM;

constexpr uint16_t CHASSIS_FOOTPRINT_X_MM = 230U;
constexpr uint16_t CHASSIS_FOOTPRINT_Y_MM = 300U;

constexpr uint16_t RING_BOUNDARY_OFFSET_MM = 40U;
constexpr uint16_t ARM_CENTER_OFFSET_MM = 225U;
constexpr uint16_t CHASSIS_HALF_WIDTH_MM =
    CHASSIS_FOOTPRINT_X_MM / 2U;
constexpr uint16_t ARM_CENTER_BEYOND_NEAR_WHEEL_MM =
    ARM_CENTER_OFFSET_MM - CHASSIS_HALF_WIDTH_MM;
constexpr uint16_t ARM_CENTER_TO_FARTHEST_WHEEL_MM =
    ARM_CENTER_OFFSET_MM + CHASSIS_HALF_WIDTH_MM;
static_assert(
    CHASSIS_HALF_WIDTH_MM == 115U &&
        ARM_CENTER_BEYOND_NEAR_WHEEL_MM == 110U &&
        ARM_CENTER_TO_FARTHEST_WHEEL_MM == 340U,
    "225 mm arm offset must be measured from the center of the 230 mm width");
constexpr uint16_t RAW_RING_CENTER_Y_MM =
    FIELD_SIZE_MM - RING_BOUNDARY_OFFSET_MM;
constexpr uint16_t PROCESS_RING_CENTER_Y_MM =
    RING_BOUNDARY_OFFSET_MM;
constexpr uint16_t STORAGE_RING_CENTER_X_MM =
    RING_BOUNDARY_OFFSET_MM;

static_assert(FIELD_CENTER_MM == 1200U, "Field centerline must be 1200 mm");
constexpr uint16_t START_CENTER_X_MM =
    START_ZONE_MIN_X_MM + CHASSIS_FOOTPRINT_X_MM / 2U;
constexpr uint16_t START_ZONE_1_CENTER_Y_MM =
    START_ZONE_1_MIN_Y_MM + CHASSIS_FOOTPRINT_Y_MM / 2U;
constexpr uint16_t START_ZONE_2_CENTER_Y_MM =
    CHASSIS_FOOTPRINT_Y_MM / 2U;

constexpr uint16_t FINAL_ZONE_CENTER_X_MM =
    FIELD_SIZE_MM - START_ZONE_SIZE_MM / 2U;

constexpr uint16_t RAW_CENTER_Y_MM =
    RAW_RING_CENTER_Y_MM - ARM_CENTER_OFFSET_MM;
constexpr uint16_t PROCESS_CENTER_Y_MM =
    PROCESS_RING_CENTER_Y_MM + ARM_CENTER_OFFSET_MM;
constexpr uint16_t STORAGE_CENTER_X_MM =
    STORAGE_RING_CENTER_X_MM + ARM_CENTER_OFFSET_MM;

constexpr uint16_t QR_PASS_CENTER_X_MM = START_CENTER_X_MM;
constexpr uint16_t RETURN_LANE_X_MM = 2150U;

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

constexpr uint16_t START_TO_QR_PASS_MM = 1050U;
constexpr uint16_t QR_PASS_CENTER_Y_MM = FIELD_CENTER_MM;
static_assert(
    START_ZONE_1_CENTER_Y_MM - QR_PASS_CENTER_Y_MM ==
            START_TO_QR_PASS_MM &&
        QR_PASS_CENTER_Y_MM - START_ZONE_2_CENTER_Y_MM ==
            START_TO_QR_PASS_MM,
    "Both start zones must be symmetric around the QR row");
constexpr uint16_t QR_PASS_TO_FIELD_CENTER_X_MM =
    QR_PASS_CENTER_X_MM - FIELD_CENTER_MM;
constexpr uint16_t QR_PASS_TO_RAW_APPROACH_MM =
    RAW_CENTER_Y_MM - WORKSTATION_APPROACH_MM -
    QR_PASS_CENTER_Y_MM;
static_assert(
    START_TO_QR_PASS_MM >= 1000U &&
        START_TO_QR_PASS_MM <= 1100U,
    "Start-to-QR manual calibration must stay within a safe range");
constexpr uint16_t CENTER_TO_RAW_MM =
    RAW_CENTER_Y_MM - FIELD_CENTER_MM;
constexpr uint16_t RAW_TO_PROCESS_MM =
    RAW_CENTER_Y_MM - PROCESS_CENTER_Y_MM;
constexpr uint16_t PROCESS_TO_CENTER_MM =
    FIELD_CENTER_MM - PROCESS_CENTER_Y_MM;
constexpr uint16_t CENTER_TO_STORAGE_MM =
    FIELD_CENTER_MM - STORAGE_CENTER_X_MM;
constexpr uint16_t STORAGE_TO_CENTER_MM =
    FIELD_CENTER_MM - STORAGE_CENTER_X_MM;
constexpr uint16_t CENTER_TO_RETURN_LANE_MM =
    RETURN_LANE_X_MM - FIELD_CENTER_MM;
constexpr uint16_t RETURN_LANE_TO_FINAL_ZONE_X_MM =
    FINAL_ZONE_CENTER_X_MM - RETURN_LANE_X_MM;

constexpr uint16_t STORAGE_ROUND2_OPEN_LOOP_Y_MM =
    FIELD_CENTER_MM - 90U;
constexpr uint16_t RETURN_TO_START_ZONE_1_Y_MM =
    START_ZONE_1_CENTER_Y_MM - STORAGE_ROUND2_OPEN_LOOP_Y_MM;
constexpr uint16_t RETURN_TO_START_ZONE_2_Y_MM =
    STORAGE_ROUND2_OPEN_LOOP_Y_MM - START_ZONE_2_CENTER_Y_MM;

constexpr int32_t ROUTE_CHASSIS_LENGTH_MM = 290;
constexpr int32_t ROUTE_CHASSIS_WIDTH_MM = 234;
constexpr int32_t DISTANCE_A_MM =
    1100 - 300 + ROUTE_CHASSIS_LENGTH_MM / 2 + 8;
constexpr int32_t SCAN_START_TO_POINT_A_MM =
    900 + ROUTE_CHASSIS_LENGTH_MM / 2 - DISTANCE_A_MM;
constexpr int32_t MAXIMUM_SCAN_DISTANCE_B_MM = 200;
constexpr int32_t DISTANCE_C_MM =
    900 + ROUTE_CHASSIS_WIDTH_MM / 2;
constexpr int32_t DISTANCE_D_MM =
    1200 - 85 - ROUTE_CHASSIS_LENGTH_MM / 2 - 20;
constexpr int32_t DISTANCE_E_MM =
    DISTANCE_D_MM + 1200 - 150 -
    ROUTE_CHASSIS_LENGTH_MM / 2 - 20;
constexpr int32_t DISTANCE_F_MM =
    1200 - 150 - 40 - ROUTE_CHASSIS_WIDTH_MM / 2;
constexpr int32_t DISTANCE_G_MM =
    1200 - 150 - ROUTE_CHASSIS_LENGTH_MM / 2 - 20;
constexpr int32_t STORAGE_F_TO_SCAN_A_MM =
    DISTANCE_F_MM + DISTANCE_C_MM;
constexpr int32_t POINT_A_TO_START_ZONE_MM =
    1100 - 300 + ROUTE_CHASSIS_LENGTH_MM / 2;

static_assert(DISTANCE_A_MM == 953, "Route distance a must be 953 mm");
static_assert(
    SCAN_START_TO_POINT_A_MM == 92,
    "Scan start to point A must be 92 mm");
static_assert(DISTANCE_C_MM == 1017, "Route distance c must be 1017 mm");
static_assert(DISTANCE_D_MM == 950, "Route distance d must be 950 mm");
static_assert(DISTANCE_E_MM == 1835, "Route distance e must be 1835 mm");
static_assert(DISTANCE_F_MM == 893, "Route distance f must be 893 mm");
static_assert(DISTANCE_G_MM == 885, "Route distance g must be 885 mm");
static_assert(
    STORAGE_F_TO_SCAN_A_MM == 1910,
    "Storage F to scan A must be 1910 mm");
static_assert(
    POINT_A_TO_START_ZONE_MM == 945,
    "Point A to start zone must be 945 mm");
static_assert(
    SCAN_START_TO_POINT_A_MM < MAXIMUM_SCAN_DISTANCE_B_MM,
    "The slow scan limit must pass point A");

// 历史路线遗留的 STEP_xx 常量仅参与下方静态断言；正式路线的逐段距离、
// 速度和加速度系数都在 route_chassis_main.inc。调第7/15段不要改这一组。
constexpr float STEP_01_MOTION_SCALE = 1.1f;
constexpr float STEP_02_MOTION_SCALE = 1.00f;
constexpr float STEP_03_MOTION_SCALE = 1.00f;
constexpr float STEP_04_MOTION_SCALE = 0.94f;
constexpr float STEP_05_MOTION_SCALE = 1.00f;
constexpr float STEP_06_MOTION_SCALE = 0.5f;
constexpr float STEP_07_MOTION_SCALE = 1.1f;
constexpr float STEP_08_MOTION_SCALE = 1.00f;
constexpr float STEP_09_MOTION_SCALE = 0.993f;
constexpr float STEP_10_MOTION_SCALE = 0.99f;
constexpr float STEP_11_MOTION_SCALE = 0.968f;
constexpr float STEP_12_MOTION_SCALE = 0.97f;
constexpr float STEP_13_MOTION_SCALE = 1.00f;
constexpr float STEP_14_MOTION_SCALE = 0.98f;
constexpr float STEP_15_MOTION_SCALE = 0.995f;
constexpr float STEP_16_MOTION_SCALE = 1.00f;
constexpr float STEP_17_MOTION_SCALE = 1.03f;
constexpr float STEP_18_MOTION_SCALE = 0.99f;
constexpr float STEP_19_MOTION_SCALE = 0.95f;
constexpr float STEP_20_MOTION_SCALE = 1.015f;
constexpr float STEP_21_MOTION_SCALE = 1.077f;

constexpr float STEP_07_LATERAL_MAX_SPEED_SCALE = 0.975f;
constexpr float STEP_07_LATERAL_ACCELERATION_SCALE = 0.60f;
constexpr float STEP_15_LATERAL_MAX_SPEED_SCALE = 0.975f;
constexpr float STEP_15_LATERAL_ACCELERATION_SCALE = 0.60f;
static_assert(
    STEP_07_LATERAL_MAX_SPEED_SCALE > 0.0f &&
        STEP_07_LATERAL_ACCELERATION_SCALE > 0.0f &&
        STEP_15_LATERAL_MAX_SPEED_SCALE > 0.0f &&
        STEP_15_LATERAL_ACCELERATION_SCALE > 0.0f,
    "Step 7/15 anti-slip profile scales must be greater than zero");
constexpr float ROUTE_LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL =
    0.50f;
static_assert(
    ROUTE_LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL > 0.0f &&
        ROUTE_LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL <= 1.0f,
    "Route lateral acceleration limit must be in (0, 1]");

static_assert(
    STEP_01_MOTION_SCALE > 0.0f &&
        STEP_02_MOTION_SCALE > 0.0f &&
        STEP_03_MOTION_SCALE > 0.0f &&
        STEP_04_MOTION_SCALE > 0.0f &&
        STEP_05_MOTION_SCALE > 0.0f &&
        STEP_06_MOTION_SCALE > 0.0f &&
        STEP_07_MOTION_SCALE > 0.0f &&
        STEP_08_MOTION_SCALE > 0.0f &&
        STEP_09_MOTION_SCALE > 0.0f &&
        STEP_10_MOTION_SCALE > 0.0f &&
        STEP_11_MOTION_SCALE > 0.0f &&
        STEP_12_MOTION_SCALE > 0.0f &&
        STEP_13_MOTION_SCALE > 0.0f &&
        STEP_14_MOTION_SCALE > 0.0f &&
        STEP_15_MOTION_SCALE > 0.0f &&
        STEP_16_MOTION_SCALE > 0.0f &&
        STEP_17_MOTION_SCALE > 0.0f &&
        STEP_18_MOTION_SCALE > 0.0f &&
        STEP_19_MOTION_SCALE > 0.0f &&
        STEP_20_MOTION_SCALE > 0.0f &&
        STEP_21_MOTION_SCALE > 0.0f,
    "Every route step scale must be greater than zero");

constexpr float ROUTE_MOTION_PROFILE_INCREASE_SCALE = 1.50f;
constexpr float ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE = 1.50f;
constexpr float ROUTE_ADDITIONAL_ANGULAR_MAXIMUM_SPEED_SCALE = 1.50f;
constexpr float ROUTE_ADDITIONAL_ANGULAR_ACCELERATION_SCALE = 1.50f;
constexpr float ROUTE_FINAL_ALL_MAXIMUM_SPEED_SCALE = 2.0f / 3.0f;
constexpr float ROUTE_FINAL_ALL_ACCELERATION_SCALE = 4.00f;
constexpr float ROUTE_FAST_MAXIMUM_STEP_RATE =
    mobile_robot_config::ROUTE_FAST_BASE_MAXIMUM_STEP_RATE *
    ROUTE_MOTION_PROFILE_INCREASE_SCALE *
    ROUTE_FINAL_ALL_MAXIMUM_SPEED_SCALE;
constexpr float ROUTE_FAST_STEP_ACCELERATION =
    mobile_robot_config::ROUTE_FAST_BASE_STEP_ACCELERATION *
    ROUTE_MOTION_PROFILE_INCREASE_SCALE *
    ROUTE_FINAL_ALL_ACCELERATION_SCALE;
constexpr float ROUTE_SCAN_MAXIMUM_STEP_RATE =
    mobile_robot_config::ROUTE_SCAN_BASE_MAXIMUM_STEP_RATE *
    ROUTE_MOTION_PROFILE_INCREASE_SCALE *
    ROUTE_FINAL_ALL_MAXIMUM_SPEED_SCALE;
constexpr float ROUTE_SCAN_STEP_ACCELERATION =
    mobile_robot_config::ROUTE_SCAN_BASE_STEP_ACCELERATION *
    ROUTE_MOTION_PROFILE_INCREASE_SCALE *
    ROUTE_FINAL_ALL_ACCELERATION_SCALE;
constexpr float ROUTE_TURN_MAXIMUM_STEP_RATE =
    mobile_robot_config::ROUTE_TURN_BASE_MAXIMUM_STEP_RATE *
    ROUTE_MOTION_PROFILE_INCREASE_SCALE *
    ROUTE_ADDITIONAL_ANGULAR_MAXIMUM_SPEED_SCALE *
    ROUTE_FINAL_ALL_MAXIMUM_SPEED_SCALE;
constexpr float ROUTE_TURN_STEP_ACCELERATION =
    mobile_robot_config::ROUTE_TURN_BASE_STEP_ACCELERATION *
    ROUTE_MOTION_PROFILE_INCREASE_SCALE *
    ROUTE_ADDITIONAL_ANGULAR_ACCELERATION_SCALE *
    ROUTE_FINAL_ALL_ACCELERATION_SCALE;
constexpr float ROUTE_HEADING_CORRECTION_MAXIMUM_STEP_RATE =
    mobile_robot_config::ROUTE_HEADING_BASE_MAXIMUM_STEP_RATE *
    ROUTE_MOTION_PROFILE_INCREASE_SCALE *
    ROUTE_ADDITIONAL_ANGULAR_MAXIMUM_SPEED_SCALE *
    ROUTE_FINAL_ALL_MAXIMUM_SPEED_SCALE;
constexpr float ROUTE_HEADING_CORRECTION_STEP_ACCELERATION =
    mobile_robot_config::ROUTE_HEADING_BASE_STEP_ACCELERATION *
    ROUTE_MOTION_PROFILE_INCREASE_SCALE *
    ROUTE_ADDITIONAL_ANGULAR_ACCELERATION_SCALE *
    ROUTE_FINAL_ALL_ACCELERATION_SCALE;

constexpr float ROUTE_TRANSLATION_HEADING_TOLERANCE_DEGREES = 0.25f;
constexpr float ROUTE_TURN_HEADING_TOLERANCE_DEGREES = 0.15f;
constexpr float ROUTE_MAXIMUM_HEADING_CORRECTION_DEGREES = 12.0f;
constexpr float ROUTE_MINIMUM_HEADING_CORRECTION_DEGREES = 0.15f;
constexpr uint32_t ROUTE_MOTION_TIMEOUT_MS = 30000UL;
constexpr uint32_t ROUTE_HEADING_LOCK_TIMEOUT_MS = 20000UL;
constexpr float TURN_IMU_CONTROL_LATENCY_SECONDS = 0.015f;
constexpr float TURN_PREDICTIVE_BRAKE_MARGIN_DEGREES = 0.10f;

constexpr float MAXIMUM_STEP_RATE =
    mobile_robot_config::MAXIMUM_STEP_RATE;
constexpr float CENTRAL_CHANNEL_MAXIMUM_STEP_RATE =
    mobile_robot_config::CENTRAL_CHANNEL_MAXIMUM_STEP_RATE;

constexpr float STEP_ACCELERATION =
    mobile_robot_config::STEP_ACCELERATION;

constexpr float TURN_MAXIMUM_STEP_RATE =
    mobile_robot_config::TURN_MAXIMUM_STEP_RATE;
constexpr float TURN_STEP_ACCELERATION =
    mobile_robot_config::TURN_STEP_ACCELERATION;
constexpr float HEADING_CORRECTION_MAXIMUM_STEP_RATE =
    mobile_robot_config::HEADING_CORRECTION_MAXIMUM_STEP_RATE;
constexpr float HEADING_CORRECTION_STEP_ACCELERATION =
    mobile_robot_config::HEADING_CORRECTION_STEP_ACCELERATION;
constexpr float WORKSTATION_MAXIMUM_STEP_RATE =
    mobile_robot_config::WORKSTATION_MAXIMUM_STEP_RATE;
constexpr float WORKSTATION_STEP_ACCELERATION =
    mobile_robot_config::WORKSTATION_STEP_ACCELERATION;
constexpr float FINAL_MAXIMUM_STEP_RATE =
    mobile_robot_config::FINAL_MAXIMUM_STEP_RATE;
constexpr float FINAL_STEP_ACCELERATION =
    mobile_robot_config::FINAL_STEP_ACCELERATION;

constexpr uint16_t QR_SCAN_SWEEP_MAXIMUM_MM = 500U;
constexpr float QR_SCAN_MAXIMUM_STEP_RATE =
    mobile_robot_config::QR_SCAN_MAXIMUM_STEP_RATE;
constexpr float QR_SCAN_STEP_ACCELERATION =
    mobile_robot_config::QR_SCAN_STEP_ACCELERATION;
const uint16_t MINIMUM_STEP_WIDTH_US = 2U;

constexpr uint16_t MAX_TRANSLATION_SEGMENT_MM = 1100U;
constexpr uint16_t CENTRAL_CHANNEL_MAX_TRANSLATION_SEGMENT_MM = 2000U;

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
// Generic workstation departure waits for M5/M6/M7 and the storage servo,
// discards 120 ms, then requires 200 ms of fresh in-tolerance IMU samples.
// PROCESS2 may observe that same 320 ms window while M7/tray are moving, but it
// still needs a new in-tolerance frame after M7 zero; any doubt falls back to
// this full 1000 ms bounded correction lock.
constexpr uint32_t WORK_DEPARTURE_POST_ARM_SETTLE_MS = 120UL;
constexpr uint32_t WORK_DEPARTURE_IMU_STABLE_MS = 200UL;
constexpr uint32_t WORK_DEPARTURE_MAX_WAIT_MS = 1000UL;
constexpr uint32_t WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS = 50UL;
static_assert(
    WORK_DEPARTURE_POST_ARM_SETTLE_MS +
            WORK_DEPARTURE_IMU_STABLE_MS ==
        320UL &&
    WORK_DEPARTURE_IMU_STABLE_MS <
            WORK_DEPARTURE_MAX_WAIT_MS &&
        WORK_DEPARTURE_MAX_WAIT_MS == 1000UL,
    "Workstation departure lock timing changed unexpectedly");
// Vision action gates are deliberately capped at 10 ms. Failure timeouts and
// camera retransmission periods are not action delays and stay independent.
constexpr uint32_t VISION_ACTION_DELAY_CAP_MS = 10UL;
constexpr uint32_t VISION_POST_MOTION_SETTLE_TIME_MS = 10UL;
constexpr uint32_t VISION_HEADING_STABLE_TIME_MS = 10UL;
constexpr uint32_t ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS = 10UL;
constexpr uint32_t ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS = 10UL;
static_assert(
    FIRST_ENDPOINT_M7_SETTLE_MS +
            ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS +
            ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS +
            ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS ==
        40UL,
    "First endpoint software handoff must stay at 40 ms");
static_assert(
    gongchuang::config::vision_link::MODE_SWITCH_GUARD_MS <=
            VISION_ACTION_DELAY_CAP_MS &&
        ARM_BASE_SETTLE_MS <= VISION_ACTION_DELAY_CAP_MS &&
        FIRST_ENDPOINT_M7_SETTLE_MS <= VISION_ACTION_DELAY_CAP_MS &&
        ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS <=
            VISION_ACTION_DELAY_CAP_MS &&
        ENDPOINT_LOCAL_MOVE_SETTLE_MS <= VISION_ACTION_DELAY_CAP_MS &&
        VISION_POST_MOTION_SETTLE_TIME_MS <=
            VISION_ACTION_DELAY_CAP_MS &&
        VISION_HEADING_STABLE_TIME_MS <=
            VISION_ACTION_DELAY_CAP_MS &&
        ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS <=
            VISION_ACTION_DELAY_CAP_MS &&
        ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS <=
            VISION_ACTION_DELAY_CAP_MS,
    "Every deliberate vision action delay must be at most 10 ms");

const uint32_t MOTION_TIMEOUT_MS = 20000UL;
const uint32_t TURN_TIMEOUT_MS = 16000UL;
const bool ENABLE_MOTION_TIMEOUTS = true;
constexpr uint8_t MAXIMUM_TURN_CORRECTIONS = 12U;

constexpr uint8_t QR_REQUIRED_MATCHING_FRAMES = 1U;

constexpr bool REQUIRE_RAW_PICK_QR_ORDER = true;
static_assert(
    REQUIRE_RAW_PICK_QR_ORDER,
    "RAW pickup must remain locked to QR order");
constexpr uint32_t VISION_RESULT_TIMEOUT_MS = 12000UL;
constexpr uint8_t VISION_MAXIMUM_RETRIES = 2U;

constexpr uint32_t RAW_TARGET_REQUEST_REFRESH_MS =
    GONGCHUANG_RAW_COLOR_REQUEST_POLICY == 1
        ? 5000UL
        : 15000UL;
// The deployed yanyanview5.py supports targeted color modes 1..4. STORAGE2
// already knows the required color from the QR task, so request that one mode
// directly instead of running all four threshold groups through mode 8. The
// camera's targeted path needs only two frames and no 300 ms all-color gate.
// Keep each camera request alive for its full stopped-pose survey window.
static_assert(arm_config::STORAGE2_SCAN_DWELL_MS * 3UL == 1500UL,
              "Three stopped-pose survey windows total 1.5 seconds");
static_assert(
    arm_config::STORAGE2_PIXEL_TO_MM_CORRECTION_SCALE == 1.30f,
    "STORAGE2 pixel-to-millimeter correction must remain 1.30x");
static_assert(
    arm_config::STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE == 0.90f,
    "STORAGE2 outer-ring final placement M6 scale must remain 0.90x");
static_assert(
    arm_config::STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM ==
        5.0f,
    "STORAGE2 outer-ring final placement correction must remain +5 mm");
static_assert(
    arm_config::STORAGE2_TARGET_COLOR_ROI_RADIUS_PIXELS > 0U &&
        arm_config::STORAGE2_TARGET_COLOR_ROI_RADIUS_PIXELS <= 160U,
    "STORAGE2 targeted-color ROI must stay inside the 320-pixel image width");
constexpr uint32_t RAW_ACTION_TIMEOUT_MS = 65000UL;
constexpr uint32_t PROCESS_ACTION_TIMEOUT_MS = 65000UL;
constexpr uint32_t STORAGE_ACTION_TIMEOUT_MS = 45000UL;
constexpr uint32_t MISSION_PROGRESS_TIMEOUT_MS = 70000UL;

constexpr bool ROUGH_PROCESSING_CALIBRATION_MODE =
    GONGCHUANG_RUN_MODE == 1;
constexpr bool STORAGE_ROUND2_SIMULATION_MODE =
    GONGCHUANG_RUN_MODE == 2;
#ifndef GONGCHUANG_DEBUG_RAW
#define GONGCHUANG_DEBUG_RAW 0
#endif
#ifndef GONGCHUANG_DEBUG_RAW_ROUND
#define GONGCHUANG_DEBUG_RAW_ROUND 1
#endif
#ifndef GONGCHUANG_DEBUG_RAW_TASK_CODE
#define GONGCHUANG_DEBUG_RAW_TASK_CODE "123+132+123+132"
#endif
static_assert(
    GONGCHUANG_DEBUG_RAW == 0 ||
        GONGCHUANG_DEBUG_RAW == 1,
    "GONGCHUANG_DEBUG_RAW must be 0 or 1");
static_assert(
    GONGCHUANG_DEBUG_RAW_ROUND == 1 ||
        GONGCHUANG_DEBUG_RAW_ROUND == 2,
    "GONGCHUANG_DEBUG_RAW_ROUND must be 1 or 2");
constexpr bool RAW_WORKSTATION_SIMULATION_MODE =
    GONGCHUANG_DEBUG_RAW == 1;
#ifndef GONGCHUANG_DEBUG_STORAGE1
#define GONGCHUANG_DEBUG_STORAGE1 0
#endif
static_assert(GONGCHUANG_DEBUG_STORAGE1 == 0 ||
                  GONGCHUANG_DEBUG_STORAGE1 == 1,
              "GONGCHUANG_DEBUG_STORAGE1 must be 0 or 1");
static_assert(GONGCHUANG_DEBUG_STORAGE1 == 0 ||
                  GONGCHUANG_RUN_MODE == 0,
              "Storage1 debug must not be combined with another local mode");
constexpr bool STORAGE_ROUND1_SIMULATION_MODE =
    GONGCHUANG_DEBUG_STORAGE1 == 1;
static_assert(
    !RAW_WORKSTATION_SIMULATION_MODE ||
        (!ROUGH_PROCESSING_CALIBRATION_MODE &&
         !STORAGE_ROUND1_SIMULATION_MODE &&
         !STORAGE_ROUND2_SIMULATION_MODE &&
         GONGCHUANG_RUN_MODE == 0),
    "RAW debug must be the only local workstation mode");
constexpr bool LOCAL_WORKSTATION_SIMULATION_MODE =
    RAW_WORKSTATION_SIMULATION_MODE ||
    ROUGH_PROCESSING_CALIBRATION_MODE ||
    STORAGE_ROUND2_SIMULATION_MODE ||
    STORAGE_ROUND1_SIMULATION_MODE;
constexpr float RING3_EXTENSION_CORRECTION_MM =
    GONGCHUANG_RING3_EXTENSION_CORRECTION_MM;
static_assert(
    RING3_EXTENSION_CORRECTION_MM >= 0.0f &&
        RING3_EXTENSION_CORRECTION_MM <= 10.0f,
    "Ring-3 extension correction must remain in the 0..10 mm tuning window");
constexpr float STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES =
    GONGCHUANG_STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES;
constexpr float STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES =
    GONGCHUANG_STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES;
constexpr float STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM =
    GONGCHUANG_STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM;
static_assert(
    STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES >= -10.0f &&
        STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES <= 10.0f &&
        STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES >= -10.0f &&
        STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES <= 10.0f &&
        STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM >= 0.0f &&
        STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM <= 30.0f,
    "STORAGE2 debug scan-pose corrections exceed their tuning windows");
// 以下都是相对视觉生成 measuredRingPoses 的最终净值：M5正数=物理逆时针，
// M6正数=伸长、负数=回缩。所有增量均已合并成净值，本轮1号位再加
// 顺时针1.00度；粗加工两轮、暂存两轮及工位调试共用。
// 第二次暂存区外圈在公共值上扣除专用0.50度差值；回收和扫描使用各自的参数表。
constexpr float PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[3] = {
    1.92f, 2.75f, 1.62f};
constexpr float STORAGE2_OUTER_RING_PLACE_M5_DIFFERENCE_CCW_DEGREES =
    -0.50f;
constexpr float WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[3] = {
    -17.5f, -13.5f, -17.0f};
// 第一次暂存区1号位放置专用附加量。它叠加在上面的公共最终净值之后：
// M5正数定义为逆时针，因此“顺时针0.2度”在应用时做减法；M6正数为伸长。
// 不得把这两个量并入公共数组，否则会同时改变粗加工区和第二次暂存区。
constexpr float STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES =
    0.20f;
constexpr float STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM =
    1.0f;
constexpr float WORKSTATION_RING_RETURN_M5_NET_CCW_DEGREES[3] = {
    2.0f, 2.5f, 2.0f};
constexpr float WORKSTATION_RING_RETURN_M6_NET_EXTENSION_MM[3] = {
    -17.0f, -12.5f, -16.0f};
static_assert(
    PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[0] >= 0.0f &&
        PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[1] <= 3.0f &&
        PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[2] <= 3.0f &&
        PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[0] +
                STORAGE2_OUTER_RING_PLACE_M5_DIFFERENCE_CCW_DEGREES >=
            0.0f &&
        PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[2] +
                STORAGE2_OUTER_RING_PLACE_M5_DIFFERENCE_CCW_DEGREES >=
            0.0f &&
        WORKSTATION_RING_RETURN_M5_NET_CCW_DEGREES[0] >= 0.0f &&
        WORKSTATION_RING_RETURN_M5_NET_CCW_DEGREES[1] <= 3.0f &&
        WORKSTATION_RING_RETURN_M5_NET_CCW_DEGREES[2] <= 3.0f &&
        WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[0] >= -20.0f &&
        WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[1] <= 0.0f &&
        WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[2] >= -20.0f &&
        STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES >= 0.0f &&
        STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES <= 1.0f &&
        STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM >= 0.0f &&
        STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM <= 5.0f &&
        WORKSTATION_RING_RETURN_M6_NET_EXTENSION_MM[0] >= -20.0f &&
        WORKSTATION_RING_RETURN_M6_NET_EXTENSION_MM[1] <= 0.0f &&
        WORKSTATION_RING_RETURN_M6_NET_EXTENSION_MM[2] >= -20.0f,
    "Workstation net pose corrections exceed tuning window");
constexpr uint16_t RING1_LOADED_RETURN_RAMP_TIME_PERCENT =
    GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT;
static_assert(
    RING1_LOADED_RETURN_RAMP_TIME_PERCENT >= 100U &&
        RING1_LOADED_RETURN_RAMP_TIME_PERCENT <= 200U,
    "Ring-1 loaded-return ramp percent must remain within 100..200");
constexpr float RING1_LOADED_RETURN_M5_STEP_ACCELERATION =
    arm_config::M5_LOADED_RETURN_STEP_ACCELERATION * 100.0f /
    static_cast<float>(RING1_LOADED_RETURN_RAMP_TIME_PERCENT);
constexpr uint16_t EMM_ACCELERATION_PARAMETER_LIMIT = 256U;
constexpr uint16_t RING1_LOADED_RETURN_M6_ACCELERATION_DISTANCE =
    ((EMM_ACCELERATION_PARAMETER_LIMIT -
      static_cast<uint16_t>(arm_config::M6_LOADED_RETURN_ACCELERATION)) *
         RING1_LOADED_RETURN_RAMP_TIME_PERCENT +
     50U) /
    100U;
static_assert(
    RING1_LOADED_RETURN_M6_ACCELERATION_DISTANCE <
        EMM_ACCELERATION_PARAMETER_LIMIT,
    "Ring-1 loaded-return M6 ramp exceeds the EMM acceleration range");
constexpr uint8_t RING1_LOADED_RETURN_M6_ACCELERATION =
    static_cast<uint8_t>(
        EMM_ACCELERATION_PARAMETER_LIMIT -
        RING1_LOADED_RETURN_M6_ACCELERATION_DISTANCE);
static_assert(
    GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT != 150U ||
        (RING1_LOADED_RETURN_M5_STEP_ACCELERATION == 48000.0f &&
         RING1_LOADED_RETURN_M6_ACCELERATION == 184U),
    "Ring-1 loaded return 150-percent ramp must map M5/M6 to 48000/184");
// 所有“夹着物料进入料盘”的M5/M6平面运动使用1.5倍斜坡时间；最高速度
// 保持不变。驱动接口只有对称加减速参数，因此加速和减速时间会一起延长。
// 粗加工1号位回料盘现也使用相同的1.5倍缓冲，但仍保留独立动作门控和日志。
constexpr uint16_t TRAY_LOAD_RAMP_TIME_NUMERATOR = 3U;
constexpr uint16_t TRAY_LOAD_RAMP_TIME_DENOMINATOR = 2U;
constexpr float TRAY_LOAD_RETURN_M5_STEP_ACCELERATION =
    arm_config::M5_LOADED_RETURN_STEP_ACCELERATION *
    static_cast<float>(TRAY_LOAD_RAMP_TIME_DENOMINATOR) /
    static_cast<float>(TRAY_LOAD_RAMP_TIME_NUMERATOR);
constexpr float TRAY_LOAD_RAW_M5_STEP_ACCELERATION =
    arm_config::M5_RAW_STEP_ACCELERATION *
    static_cast<float>(TRAY_LOAD_RAMP_TIME_DENOMINATOR) /
    static_cast<float>(TRAY_LOAD_RAMP_TIME_NUMERATOR);
constexpr uint16_t TRAY_LOAD_RETURN_M6_ACCELERATION_DISTANCE =
    ((EMM_ACCELERATION_PARAMETER_LIMIT -
      static_cast<uint16_t>(arm_config::M6_LOADED_RETURN_ACCELERATION)) *
         TRAY_LOAD_RAMP_TIME_NUMERATOR +
     TRAY_LOAD_RAMP_TIME_DENOMINATOR / 2U) /
    TRAY_LOAD_RAMP_TIME_DENOMINATOR;
constexpr uint16_t TRAY_LOAD_RAW_M6_ACCELERATION_DISTANCE =
    ((EMM_ACCELERATION_PARAMETER_LIMIT -
      static_cast<uint16_t>(arm_config::M6_RAW_ACCELERATION)) *
         TRAY_LOAD_RAMP_TIME_NUMERATOR +
     TRAY_LOAD_RAMP_TIME_DENOMINATOR / 2U) /
    TRAY_LOAD_RAMP_TIME_DENOMINATOR;
static_assert(
    TRAY_LOAD_RETURN_M6_ACCELERATION_DISTANCE <
            EMM_ACCELERATION_PARAMETER_LIMIT &&
        TRAY_LOAD_RAW_M6_ACCELERATION_DISTANCE <
            EMM_ACCELERATION_PARAMETER_LIMIT,
    "Tray-load M6 ramp exceeds the EMM acceleration range");
constexpr uint8_t TRAY_LOAD_RETURN_M6_ACCELERATION =
    static_cast<uint8_t>(
        EMM_ACCELERATION_PARAMETER_LIMIT -
        TRAY_LOAD_RETURN_M6_ACCELERATION_DISTANCE);
constexpr uint8_t TRAY_LOAD_RAW_M6_ACCELERATION =
    static_cast<uint8_t>(
        EMM_ACCELERATION_PARAMETER_LIMIT -
        TRAY_LOAD_RAW_M6_ACCELERATION_DISTANCE);
static_assert(
    TRAY_LOAD_RETURN_M5_STEP_ACCELERATION == 48000.0f &&
        TRAY_LOAD_RAW_M5_STEP_ACCELERATION == 60000.0f &&
        TRAY_LOAD_RETURN_M6_ACCELERATION == 184U &&
        TRAY_LOAD_RAW_M6_ACCELERATION == 179U,
    "Tray-load x1.5 ramp must map return/raw M5/M6 to 48000/184 and 60000/179");

// 空夹爪的普通机械臂运动：速度上限和物理加速度上限各提高30%。M5使用
// 直接步频/加速度，M6/M7的EMM字节按a~1/(256-byte)换算。EMM位置命令
// 只有一套对称加减速参数，无法另设独立减速时间；M7正常加速度已饱和255。
constexpr float EMPTY_GRIPPER_SPEED_SCALE = 1.30f;
constexpr float EMPTY_GRIPPER_ACCELERATION_SCALE = 1.30f;
// M5 uses one symmetric AccelStepper acceleration value. Increasing the
// deceleration time by 15% therefore also lengthens acceleration by 15%.
// Apply the reciprocal once at the final common profile setter so every M5
// profile, including empty-gripper boosts and loaded tray cushions, receives
// the same global ramp-time change.
constexpr float M5_GLOBAL_RAMP_TIME_SCALE = 1.15f;
constexpr float M5_GLOBAL_ACCELERATION_SCALE =
    1.0f / M5_GLOBAL_RAMP_TIME_SCALE;
static_assert(
    EMPTY_GRIPPER_SPEED_SCALE == 1.30f &&
        EMPTY_GRIPPER_ACCELERATION_SCALE == 1.30f &&
        M5_GLOBAL_RAMP_TIME_SCALE == 1.15f &&
        M5_GLOBAL_ACCELERATION_SCALE > 0.0f &&
        M5_GLOBAL_ACCELERATION_SCALE < 1.0f,
    "M5 empty boost and global ramp-time scales are invalid");

const bool DISPLAY_YAW_ON_X0 = true;
const bool DISPLAY_BATTERY_ON_X1 = true;
// The deployed HMI project has no populated page0. Keep the main page visible
// after completion and update only its FINISH status/count fields.
const bool SHOW_RESULT_PAGE_ON_FINISH = false;

const float BATTERY_ADC_REFERENCE_VOLTAGE = 3.3f;
const float BATTERY_DIVIDER_RATIO = 11.0f;
const float BATTERY_CALIBRATION_SCALE = 1.0f;
const float BATTERY_FILTER_ALPHA = 0.10f;
const uint32_t BATTERY_SAMPLE_INTERVAL_MS = 50UL;
const uint32_t BATTERY_HMI_INTERVAL_MS = 500UL;

MecanumKinematics &geometry = route_chassis::geometry;

AccelStepper &motor1 = route_chassis::motor1;
AccelStepper &motor2 = route_chassis::motor2;
AccelStepper &motor3 = route_chassis::motor3;
AccelStepper &motor4 = route_chassis::motor4;
ArmMotorController armMotors;

// This latch describes the material state for motion-profile selection. It is
// raised only after a source close has fully settled and cleared only after the
// destination open has fully settled. Startup, vision and between-transfer
// preparation therefore use the empty-gripper profile.
bool armPayloadPresent = false;

constexpr uint16_t emptyGripperBoostedSpeedRpm(uint16_t speedRpm) {
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(speedRpm) * 13U + 5U) / 10U);
}

constexpr uint8_t emptyGripperBoostedEmmAcceleration(
    uint8_t acceleration) {
  return acceleration >= 255U
             ? 255U
             : static_cast<uint8_t>(
                   256U -
                   (((256U - static_cast<uint16_t>(acceleration)) * 10U +
                     6U) /
                    13U));
}

static_assert(
    emptyGripperBoostedSpeedRpm(702U) == 913U &&
        emptyGripperBoostedEmmAcceleration(199U) == 212U &&
        emptyGripperBoostedSpeedRpm(6500U) == 8450U &&
        emptyGripperBoostedEmmAcceleration(255U) == 255U,
    "Empty-gripper EMM +30-percent mapping changed unexpectedly");

bool armBaseMotionLockedUntilReset = false;
bool armBaseMotionWatchdogActive = false;
uint32_t armBaseMotionDeadlineMs = 0UL;

AccelStepper *const motors[4] = {
    &motor1, &motor2, &motor3, &motor4};

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

  hmiSetValue("n2", correctGrabCount);
  hmiSetValue("n1", correctPlacementCount);
}

void hmiShowTaskCode(const char *taskCode) {

  char firstRow[9] = {0};
  char secondRow[8] = {0};

  memcpy(firstRow, taskCode, 8);
  memcpy(secondRow, taskCode + 8, 7);

  hmiSetText("t3", firstRow);
  hmiSetText("t8", secondRow);
}

const size_t QR_CODE_LENGTH = competition::TASK_CODE_LENGTH;
const size_t QR_BUFFER_SIZE = 32;

char qrData[QR_BUFFER_SIZE] = {0};
char qrCandidate[QR_CODE_LENGTH + 1U] = {0};
size_t qrDataIndex = 0;
bool qrOverflow = false;
bool scanFlag = false;
bool taskCodeDecoded = false;
uint8_t qrMatchingFrameCount = 0U;
competition::TaskPlan taskPlan;
uint8_t (&taskColors)[2][3] = taskPlan.colors;
uint8_t (&taskPositions)[2][3] = taskPlan.positions;

void commandGripperOpen();
void markMissionProgress();

void resetQrReceiver(bool discardPendingBytes = true) {
  qrDataIndex = 0;
  qrOverflow = false;
  scanFlag = false;
  taskCodeDecoded = false;
  qrData[0] = '\0';
  qrCandidate[0] = '\0';
  qrMatchingFrameCount = 0U;
  taskPlan.clear();

  if (discardPendingBytes) {
    while (SerialQr.available()) {
      SerialQr.read();
    }
  }
}

void finishQrFrame() {
  if (qrDataIndex == 0 && !qrOverflow) {
    return;
  }

  qrData[qrDataIndex] = '\0';

  competition::TaskPlan candidatePlan;
  const competition::TaskCodeStatus taskStatus =
      qrOverflow
          ? competition::TASK_CODE_WRONG_LENGTH
          : competition::parseTaskCode(qrData, candidatePlan);
  if (taskStatus == competition::TASK_CODE_OK) {
    if (strcmp(qrCandidate, qrData) == 0) {
      if (qrMatchingFrameCount < UINT8_MAX) {
        ++qrMatchingFrameCount;
      }
    } else {
      memcpy(
          qrCandidate,
          qrData,
          QR_CODE_LENGTH + 1U);
      qrMatchingFrameCount = 1U;
    }

    SerialDebug.print("[QR CONFIRM] ");
    SerialDebug.print(qrCandidate);
    SerialDebug.print(" ");
    SerialDebug.print(qrMatchingFrameCount);
    SerialDebug.print("/");
    SerialDebug.println(QR_REQUIRED_MATCHING_FRAMES);

    if (qrMatchingFrameCount >=
        QR_REQUIRED_MATCHING_FRAMES) {
      taskPlan = candidatePlan;
      taskCodeDecoded = true;
      scanFlag = true;
      markMissionProgress();
      hmiSetText("t1", "QROK");
      hmiShowTaskCode(qrData);
      SerialDebug.print("QR locked: ");
      SerialDebug.println(qrData);
    } else {
      hmiSetText("t1", "QRCHK");
    }
  } else {
    qrCandidate[0] = '\0';
    qrMatchingFrameCount = 0U;
    hmiSetText("t1", "QRERR");
    SerialDebug.print("QR rejected, status=");
    SerialDebug.println(
        static_cast<unsigned int>(taskStatus));
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

    if (incomingByte == '\r' || incomingByte == '\n') {
      finishQrFrame();
      // Same framing/validation as debug_qr_continuous.cpp. A mission keeps
      // its first valid frame; later bytes must not overwrite that task code.
      if (scanFlag) {
        return;
      }
      continue;
    }

    if (qrDataIndex < QR_BUFFER_SIZE - 1) {
      qrData[qrDataIndex++] = incomingByte;
    } else {
      qrOverflow = true;
    }
  }
}

float wrapDeltaDegrees(float degrees) {
  return gongchuang::ImuHeadingTracker::
      wrapDeltaDegrees(degrees);
}

void receiveImuData() {
  /*
   * PD8/PD9只允许route_chassis解析一次。机械臂/视觉侧直接读取其已经
   * 校验并展开的连续航向，避免两个接收器争抢同一串口字节。
   */
}

bool imuIsFresh() {
  return route_chassis::imuIsFresh();
}

struct MotorPulses {
  long motor1;
  long motor2;
  long motor3;
  long motor4;

  MotorPulses(long m1 = 0, long m2 = 0,
              long m3 = 0, long m4 = 0)
      : motor1(m1), motor2(m2), motor3(m3), motor4(m4) {}
};

MotorPulses addMotorPulses(
    const MotorPulses &left,
    const MotorPulses &right) {
  return MotorPulses(
      left.motor1 + right.motor1,
      left.motor2 + right.motor2,
      left.motor3 + right.motor3,
      left.motor4 + right.motor4);
}

MotorPulses negateMotorPulses(const MotorPulses &pulses) {
  return MotorPulses(
      -pulses.motor1,
      -pulses.motor2,
      -pulses.motor3,
      -pulses.motor4);
}

bool motorPulsesAreZero(const MotorPulses &pulses) {
  return pulses.motor1 == 0 &&
         pulses.motor2 == 0 &&
         pulses.motor3 == 0 &&
         pulses.motor4 == 0;
}

long roundedPulseCount(float pulses) {
  return static_cast<long>(
      pulses >= 0.0f ? pulses + 0.5f : pulses - 0.5f);
}

MotorPulses bodyDisplacementToMotorPulses(
    float forwardMeters,
    float leftMeters,
    float counterClockwiseRadians) {

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

float absolutePulseCount(long pulses) {
  return static_cast<float>(
      pulses >= 0L ? pulses : -pulses);
}

float rotationPulsesPerDegree(int8_t directionSign) {

  const float calibrationDegrees =
      directionSign >= 0 ? 100.0f : -100.0f;
  const MotorPulses pulses =
      bodyDisplacementToMotorPulses(
          0.0f,
          0.0f,
          calibrationDegrees * PI_F / 180.0f);
  const float averageAbsolutePulses =
      (absolutePulseCount(pulses.motor1) +
       absolutePulseCount(pulses.motor2) +
       absolutePulseCount(pulses.motor3) +
       absolutePulseCount(pulses.motor4)) /
      4.0f;
  return averageAbsolutePulses /
         fabsf(calibrationDegrees);
}

void routeFault(const char *reason);
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
void printStorage2VerboseFaultSnapshot(const char *reason);
#endif

void startRelativeMotorMove(const MotorPulses &pulses) {
  // 工位视觉/离站航向修正仍使用这些共用M1～M4对象；电机脉冲只由
  // route_chassis::runAllMotors()统一服务，勿再添加本地runAllMotors()。
  // 只有显式启用新RAW逻辑的独立调试才允许A/B可逆40 mm横移。
  // 当前默认旧RAW逻辑与其余独立工位调试都禁止下发底盘运动。
  if (LOCAL_WORKSTATION_SIMULATION_MODE &&
      !(RAW_WORKSTATION_SIMULATION_MODE &&
        GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1)) {
    routeFault("Local workstation mode attempted chassis motion");
    return;
  }

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

void startArmBaseLibraryDegrees(float libraryDegrees) {
  if (armBaseMotionLockedUntilReset) {
    SerialDebug.println(
        "M5 command ignored until power-cycle reset");
    return;
  }
  armMotors.moveM5ToDegrees(libraryDegrees, false);
  armBaseMotionWatchdogActive = true;
  armBaseMotionDeadlineMs =
      millis() + ARM_BASE_MOTION_TIMEOUT_MS;

  SerialDebug.print("Arm base library target: ");
  SerialDebug.print(libraryDegrees, 1);
  SerialDebug.print(" deg, pulses=");
  SerialDebug.println(
      armMotors.m5PulsesForDegrees(libraryDegrees));
}

void startArmBaseRotationToDegrees(float oldFrameDegrees) {

  startArmBaseLibraryDegrees(oldFrameDegrees);
}

void startArmBaseStandardFrameDegrees(float angleDegrees) {

  const float standardLibraryDegrees =
      static_cast<float>(
          ARM_BASE_STANDARD_OLD_FRAME_DEGREES);
  startArmBaseLibraryDegrees(
      standardLibraryDegrees + angleDegrees);
}

float currentArmBaseStandardFrameDegrees() {
  return armMotors.m5CurrentAngleDegrees() -
         static_cast<float>(
             ARM_BASE_STANDARD_OLD_FRAME_DEGREES);
}

void stopArmBaseImmediately() {
  armBaseMotionWatchdogActive = false;
  armBaseMotionDeadlineMs = 0UL;
  armMotors.stopM5Immediately();
}

void disableArmBaseMotor() {
  stopArmBaseImmediately();
  armMotors.disableM5();
  armBaseMotionLockedUntilReset = true;
}

void setArmBaseMotionProfileForPayload(
    float maximumStepRate,
    float stepAcceleration) {
  if (!armPayloadPresent) {
    maximumStepRate *= EMPTY_GRIPPER_SPEED_SCALE;
    stepAcceleration *= EMPTY_GRIPPER_ACCELERATION_SCALE;
  }
  stepAcceleration *= M5_GLOBAL_ACCELERATION_SCALE;
  armMotors.setM5MotionProfile(
      maximumStepRate,
      stepAcceleration);
}

void useArmBaseEndpointMotionProfile() {
  setArmBaseMotionProfileForPayload(
      ARM_BASE_MAXIMUM_STEP_RATE,
      ARM_BASE_STEP_ACCELERATION);
}

void useArmBaseEndpointCoarseMotionProfile() {
  setArmBaseMotionProfileForPayload(
      ARM_BASE_ENDPOINT_COARSE_MAXIMUM_STEP_RATE,
      ARM_BASE_ENDPOINT_COARSE_STEP_ACCELERATION);
}

void useArmBaseEndpointTravelMotionProfile() {
  setArmBaseMotionProfileForPayload(
      ARM_BASE_ENDPOINT_TRAVEL_MAXIMUM_STEP_RATE,
      ARM_BASE_ENDPOINT_TRAVEL_STEP_ACCELERATION);
}

void useArmBaseTransferMotionProfile() {
  setArmBaseMotionProfileForPayload(
      arm_config::M5_STANDARD_MAXIMUM_STEP_RATE,
      arm_config::M5_STANDARD_STEP_ACCELERATION);
}

void useArmBasePlaceMotionProfile() {
  setArmBaseMotionProfileForPayload(
      arm_config::M5_PLACE_MAXIMUM_STEP_RATE,
      arm_config::M5_PLACE_STEP_ACCELERATION);
}

void useArmBaseReturnMotionProfile() {
  setArmBaseMotionProfileForPayload(
      arm_config::M5_RETURN_MAXIMUM_STEP_RATE,
      arm_config::M5_RETURN_STEP_ACCELERATION);
}

void useArmBaseRing1LoadedReturnCushionMotionProfile() {
  setArmBaseMotionProfileForPayload(
      arm_config::M5_LOADED_RETURN_MAXIMUM_STEP_RATE,
      RING1_LOADED_RETURN_M5_STEP_ACCELERATION);
}

void useArmBaseTrayLoadReturnCushionMotionProfile() {
  setArmBaseMotionProfileForPayload(
      arm_config::M5_LOADED_RETURN_MAXIMUM_STEP_RATE,
      TRAY_LOAD_RETURN_M5_STEP_ACCELERATION);
}

void useArmBaseRawTrayLoadCushionMotionProfile() {
  setArmBaseMotionProfileForPayload(
      arm_config::M5_RAW_MAXIMUM_STEP_RATE,
      TRAY_LOAD_RAW_M5_STEP_ACCELERATION);
}

void useArmBaseRawTransferMotionProfile() {
  setArmBaseMotionProfileForPayload(
      arm_config::M5_RAW_MAXIMUM_STEP_RATE,
      arm_config::M5_RAW_STEP_ACCELERATION);
}

void setDriveMotionProfile(float maximumStepRate, float acceleration) {
  // 正式路线在route_chassis中使用同一全局倍率；这里覆盖共用M1～M4的
  // 工位视觉挪车/航向动作。不要在调用点再乘一次，否则会重复提速。
  for (uint8_t i = 0; i < 4; ++i) {
    motors[i]->setMaxSpeed(
        maximumStepRate *
        route_chassis::CHASSIS_GLOBAL_SPEED_INCREASE_SCALE);
    motors[i]->setAcceleration(
        acceleration *
        route_chassis::CHASSIS_GLOBAL_ACCELERATION_INCREASE_SCALE);
  }
}

// 维护警告：本文件不定义路线命令；底盘路线仅在 route_chassis_main.inc。

enum ProgramState {
  PROGRAM_WAITING,
  PROGRAM_RUNNING,
  PROGRAM_FINISHED,
  PROGRAM_FAULT
};

enum StartZone {
  START_ZONE_1 = 1,
  START_ZONE_2 = 2
};

ProgramState programState = PROGRAM_WAITING;

// 正式比赛从启动请求确认到发车至少500 ms；启动检查耗时也计入这500 ms。
// 不使用delay()：扫码、IMU、机械臂服务和长按中止必须继续运行。
constexpr uint32_t CHASSIS_STARTUP_HOLD_MS = 500UL;
bool chassisStartupHoldActive = false;
uint32_t chassisStartupHoldStartMs = 0UL;

enum IntegratedWorkPause {
  INTEGRATED_WORK_NONE,
  INTEGRATED_WORK_RAW_1,
  INTEGRATED_WORK_PROCESS_1,
  INTEGRATED_WORK_STORAGE_1,
  INTEGRATED_WORK_RAW_2,
  INTEGRATED_WORK_PROCESS_2,
  INTEGRATED_WORK_STORAGE_2
};

enum StorageRound2SimulationPhase {
  STORAGE2_SIMULATION_IDLE,
  STORAGE2_SIMULATION_WAIT_GRIPPER_OPEN,
  STORAGE2_SIMULATION_WAIT_TRAVEL_PREPARATION,
  STORAGE2_SIMULATION_RUN_ACTION,
  STORAGE2_SIMULATION_COMPLETE
};

enum StorageRound1SimulationPhase {
  STORAGE1_SIMULATION_IDLE,
  STORAGE1_SIMULATION_WAIT_PREPARATION,
  STORAGE1_SIMULATION_RUN_ACTION,
  STORAGE1_SIMULATION_COMPLETE
};

IntegratedWorkPause integratedWorkPause = INTEGRATED_WORK_NONE;
uint32_t integratedWorkPauseStartMs = 0UL;
bool integratedWorkActionStartPending = false;
bool finalReturnM5Commanded = false;
bool finalStorageDeparturePending = false;
bool storage2ReturnTravelActive = false;
StorageRound2SimulationPhase storageRound2SimulationPhase =
    STORAGE2_SIMULATION_IDLE;
uint32_t storageRound2SimulationGripperDeadlineMs = 0UL;
StorageRound1SimulationPhase storageRound1SimulationPhase =
    STORAGE1_SIMULATION_IDLE;
uint32_t rawSimulationParkingDeadlineMs = 0UL;
StartZone selectedStartZone = START_ZONE_1;
uint8_t activeCompetitionRound = 0;
uint32_t commandStartMs = 0;
uint32_t lastMissionProgressMs = 0UL;
uint32_t headingStableStartMs = 0;
uint32_t motorsArrivedStartMs = 0;
uint32_t routeHeadingLockStartMs = 0UL;
bool imuWaitStatusDisplayed = false;
uint16_t translationRemainingMm = 0;
bool translationCentralChannelEnabled = false;
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

volatile bool rawActionFinished = false;
volatile bool processActionFinished = false;
volatile bool storageActionFinished = false;
volatile bool finalAlignmentFinished = false;

void markMissionProgress() {
  lastMissionProgressMs = millis();
}

float selectedStartZoneDirection() {
  return selectedStartZone == START_ZONE_1 ? 1.0f : -1.0f;
}

float currentRouteCounterClockwiseHeading() {
  return route_chassis::imuCounterClockwiseDegrees -
         routeImuReferenceDegrees;
}

float headingErrorDegrees() {
  return targetCounterClockwiseHeadingDegrees -
         currentRouteCounterClockwiseHeading();
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

void emergencyStopArmLinearAxes();
void stopMaixRequest();
void invalidateArmLinearReference();
void finishProgram();

void routeFault(const char *reason) {
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
  // Capture the live state before stop/disable calls overwrite motion flags.
  printStorage2VerboseFaultSnapshot(reason);
#endif
  chassisStartupHoldActive = false;
  invalidateArmLinearReference();
  if (route_chassis::programState ==
      route_chassis::PROGRAM_RUNNING) {
    route_chassis::routeFault(reason);
  } else {
    route_chassis::disableDriveMotors();
  }
  disableDriveMotors();
  disableArmBaseMotor();
  emergencyStopArmLinearAxes();
  stopMaixRequest();
  programState = PROGRAM_FAULT;
  hmiSetRunStatus("FAULT");

  SerialDebug.print("FAULT: ");
  SerialDebug.println(reason);
  SerialDebug.println(
      "Power off before reset: retract M6 fully; put M7 at physical top.");
}

bool deadlineReached(uint32_t deadlineMs) {
  return static_cast<int32_t>(millis() - deadlineMs) >= 0;
}

void serviceArmBaseMotionWatchdog() {
  if (!armBaseMotionWatchdogActive) {
    return;
  }
  if (!armMotors.isM5Running()) {
    armBaseMotionWatchdogActive = false;
    armBaseMotionDeadlineMs = 0UL;
    return;
  }
  if (!deadlineReached(armBaseMotionDeadlineMs)) {
    return;
  }

  armBaseMotionWatchdogActive = false;
  armBaseMotionDeadlineMs = 0UL;
  armMotors.stopM5Immediately();
  routeFault("M5 motion timeout");
}

enum LinearAxisRecoveryReason : uint8_t {
  ARM_AXIS_RECOVERY_NONE = 0U,
  ARM_AXIS_RECOVERY_STALL = 1U,
  ARM_AXIS_RECOVERY_TIMEOUT = 2U,
  ARM_AXIS_RECOVERY_COMMAND_REJECTED = 3U,
  ARM_AXIS_RECOVERY_ACK_TIMEOUT = 4U
};

enum LinearAxisTerminalVerificationReason : uint8_t {
  ARM_AXIS_VERIFY_NONE = 0U,
  ARM_AXIS_VERIFY_HEALTHY_WITHOUT_COMMAND_EVIDENCE = 1U,
  ARM_AXIS_VERIFY_LOCKED_ON_POSITION = 2U,
  ARM_AXIS_VERIFY_REJECTED_ON_POSITION = 3U,
  ARM_AXIS_VERIFY_EXPECTED_COMPLETION = 4U,
  ARM_AXIS_VERIFY_M7_FAST_ARRIVAL = 5U,
  ARM_AXIS_VERIFY_M6_FAST_ARRIVAL = 6U
};

enum M7ProfilePolicy : uint8_t {
  M7_PROFILE_APPLY_EXPERIMENT = 0U,
  M7_PROFILE_USE_REQUESTED = 1U
};

enum ArmLinearRequestKind : uint8_t {
  ARM_LINEAR_REQUEST_NONE = 0U,
  ARM_LINEAR_REQUEST_ENABLE = 1U,
  ARM_LINEAR_REQUEST_POSITION = 2U,
  ARM_LINEAR_REQUEST_STATUS = 3U,
  ARM_LINEAR_REQUEST_POSITION_QUERY = 4U
};

struct LinearAxisMotion {
  uint8_t address;
  float currentMm;
  float targetMm;
  bool active;
  bool fault;
  bool commandAcknowledged;
  bool motionObserved;
  bool positionCommandRejected;
  uint8_t terminalOnPositionSamples;
  bool terminalVerificationPending;
  bool terminalPositionRequestSent;
  uint8_t terminalStatusFlags;
  LinearAxisTerminalVerificationReason
      terminalVerificationReason;
  uint8_t terminalVerificationFailures;
  uint8_t terminalPositionSamples;
  float terminalFirstPositionMm;
  uint32_t terminalPositionRequestDeadlineMs;
  uint32_t terminalNextPositionRequestMs;
  uint8_t stallProtectionSamples;
  bool recoveryPending;
  uint8_t recoveryAttemptCount;
  LinearAxisRecoveryReason recoveryReason;
  uint32_t recoveryReadyMs;
  uint32_t recoveryDeadlineMs;
  float commandStartMm;
  float commandMinimumMm;
  float commandMaximumMm;
  float commandPulsesPerMm;
  uint8_t commandPositiveDirection;
  uint8_t commandNegativeDirection;
  uint16_t commandSpeedRpm;
  uint8_t commandAcceleration;
  M7ProfilePolicy commandM7ProfilePolicy;
  float driverWorkingZeroAngleDegrees;
  bool driverWorkingZeroAngleValid;
  uint32_t startMs;
  uint32_t lastStatusRequestMs;
  uint8_t lastStatusFlags;
  uint32_t lastStatusResponseMs;
  uint32_t lastFastArrivalQueryMs;
  ArmLinearRequestKind lastRequestKind;
  uint32_t timeoutMs;

  LinearAxisMotion(uint8_t axisAddress)
      : address(axisAddress),
        currentMm(0.0f),
        targetMm(0.0f),
        active(false),
        fault(false),
        commandAcknowledged(false),
        motionObserved(false),
        positionCommandRejected(false),
        terminalOnPositionSamples(0U),
        terminalVerificationPending(false),
        terminalPositionRequestSent(false),
        terminalStatusFlags(0U),
        terminalVerificationReason(ARM_AXIS_VERIFY_NONE),
        terminalVerificationFailures(0U),
        terminalPositionSamples(0U),
        terminalFirstPositionMm(0.0f),
        terminalPositionRequestDeadlineMs(0UL),
        terminalNextPositionRequestMs(0UL),
        stallProtectionSamples(0U),
        recoveryPending(false),
        recoveryAttemptCount(0U),
        recoveryReason(ARM_AXIS_RECOVERY_NONE),
        recoveryReadyMs(0UL),
        recoveryDeadlineMs(0UL),
        commandStartMm(0.0f),
        commandMinimumMm(0.0f),
        commandMaximumMm(0.0f),
        commandPulsesPerMm(0.0f),
        commandPositiveDirection(0U),
        commandNegativeDirection(0U),
        commandSpeedRpm(0U),
        commandAcceleration(0U),
        commandM7ProfilePolicy(
            M7_PROFILE_APPLY_EXPERIMENT),
        driverWorkingZeroAngleDegrees(0.0f),
        driverWorkingZeroAngleValid(false),
        startMs(0UL),
        lastStatusRequestMs(0UL),
        lastStatusFlags(0U),
        lastStatusResponseMs(0UL),
        lastFastArrivalQueryMs(0UL),
        lastRequestKind(ARM_LINEAR_REQUEST_NONE),
        timeoutMs(ARM_AXIS_MINIMUM_TIMEOUT_MS) {}
};

LinearAxisMotion extensionAxis(ARM_EXTENSION_ADDRESS);
LinearAxisMotion liftAxis(ARM_LIFT_ADDRESS);
bool m6ContactSoftLandingPending = false;
float m6ContactSoftLandingTargetMm = M6_STANDARD_EXTENSION_MM;
float m6ContactSoftLandingMinimumMm = M6_STANDARD_EXTENSION_MM;
uint16_t m6ContactSoftLandingSpeedRpm = M6_SPEED_RPM;
uint8_t m6ContactSoftLandingAcceleration =
    M6_CONTACT_SOFT_LANDING_ACCELERATION;
bool m7SoftLandingPending = false;
bool m7SoftLandingIsContact = false;
float m7SoftLandingTargetMm = M7_STANDARD_HEIGHT_MM;
float m7SoftLandingDistanceMm =
    M7_ZERO_SOFT_LANDING_DISTANCE_MM;
uint16_t m7SoftLandingSpeedRpm = M7_SPEED_RPM;
uint8_t m7SoftLandingAcceleration =
    M7_ZERO_SOFT_LANDING_ACCELERATION;
M7ProfilePolicy m7SoftLandingProfilePolicy =
    M7_PROFILE_APPLY_EXPERIMENT;
bool armLinearReferenceValid = false;
bool armLinearSerialInitialized = false;
bool armLinearBootReady = false;
bool manipulationServosOnline = false;
uint8_t armLinearReceiveWindow[8] = {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
uint8_t armLinearReceiveCount = 0U;
uint8_t armLinearExpectedFrameLength = 0U;
LinearAxisMotion *armLinearPositionQueryAxis = nullptr;
uint8_t armLinearStatusPollCursor = 0U;
uint32_t armLinearNextStatusRequestMs = 0UL;

LinearAxisMotion *linearAxisForAddress(uint8_t address) {
  if (address == extensionAxis.address) {
    return &extensionAxis;
  }
  if (address == liftAxis.address) {
    return &liftAxis;
  }
  return nullptr;
}

void noteArmLinearRequest(
    uint8_t address,
    ArmLinearRequestKind kind) {
  LinearAxisMotion *axis = linearAxisForAddress(address);
  if (axis == nullptr) {
    return;
  }
  axis->lastRequestKind = kind;
}

void writeArmLinearEnable(uint8_t address, bool enable) {
  const uint8_t frame[6] = {
      address, 0xF3U, 0xABU,
      static_cast<uint8_t>(enable ? 1U : 0U),
      0x00U, 0x6BU};
  noteArmLinearRequest(address, ARM_LINEAR_REQUEST_ENABLE);
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearPosition(
    uint8_t address,
    uint8_t direction,
    uint16_t speedRpm,
    uint8_t acceleration,
    uint32_t pulses) {

  const uint8_t frame[13] = {
      address,
      0xFDU,
      direction,
      static_cast<uint8_t>(speedRpm >> 8),
      static_cast<uint8_t>(speedRpm),
      acceleration,
      static_cast<uint8_t>(pulses >> 24),
      static_cast<uint8_t>(pulses >> 16),
      static_cast<uint8_t>(pulses >> 8),
      static_cast<uint8_t>(pulses),
      0x00U,
      0x00U,
      0x6BU};
  noteArmLinearRequest(address, ARM_LINEAR_REQUEST_POSITION);
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearStatusRequest(uint8_t address) {
  const uint8_t frame[3] = {address, 0x3AU, 0x6BU};
  noteArmLinearRequest(address, ARM_LINEAR_REQUEST_STATUS);
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearCurrentPositionRequest(uint8_t address) {
  const uint8_t frame[3] = {address, 0x36U, 0x6BU};
  noteArmLinearRequest(
      address,
      ARM_LINEAR_REQUEST_POSITION_QUERY);
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearResetStallProtection(uint8_t address) {
  const uint8_t frame[4] = {
      address, 0x0EU, 0x52U, 0x6BU};
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearResetCurrentPositionZero(
    uint8_t address) {
  const uint8_t frame[4] = {
      address, 0x0AU, 0x6DU, 0x6BU};
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearStop(uint8_t address) {
  const uint8_t frame[5] = {
      address, 0xFEU, 0x98U, 0x00U, 0x6BU};
  SerialArmLinear.write(frame, sizeof(frame));
}

void clearArmLinearReceiveBuffer() {
  while (SerialArmLinear.available()) {
    SerialArmLinear.read();
  }
  armLinearReceiveCount = 0U;
  armLinearExpectedFrameLength = 0U;
  if (armLinearPositionQueryAxis != nullptr) {
    armLinearPositionQueryAxis
        ->terminalPositionRequestSent = false;
    armLinearPositionQueryAxis = nullptr;
  }
}

void printArmLinearFrame(
    const char *label,
    const uint8_t *frame) {
  SerialDebug.print("[EMM RX] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, ");
  SerialDebug.print(label);
  SerialDebug.print(": ");
  for (uint8_t i = 0U; i < 4U; ++i) {
    if (frame[i] < 0x10U) {
      SerialDebug.print('0');
    }
    SerialDebug.print(
        static_cast<unsigned int>(frame[i]), HEX);
    if (i + 1U < 4U) {
      SerialDebug.print(' ');
    }
  }
  SerialDebug.println();
}

bool waitForArmLinearSimpleResponse(
    uint8_t address,
    uint8_t functionCode,
    uint32_t timeoutMs) {
  uint8_t window[4] = {0U, 0U, 0U, 0U};
  uint8_t count = 0U;
  const uint32_t startMs = millis();

  while (millis() - startMs < timeoutMs) {
    while (SerialArmLinear.available()) {
      const uint8_t incoming =
          static_cast<uint8_t>(SerialArmLinear.read());
      if (count < 4U) {
        window[count++] = incoming;
      } else {
        window[0] = window[1];
        window[1] = window[2];
        window[2] = window[3];
        window[3] = incoming;
      }
      if (count == 4U &&
          window[0] == address &&
          window[1] == functionCode &&
          window[3] == 0x6BU) {
        printArmLinearFrame(
            window[2] == 0x02U
                ? "startup command accepted"
                : "startup command rejected",
            window);
        return window[2] == 0x02U;
      }
    }
    delay(1);
  }
  return false;
}

bool readArmLinearCurrentMotorAngleDegrees(
    uint8_t address,
    float &angleDegrees) {

  if (extensionAxis.active ||
      liftAxis.active ||
      armLinearPositionQueryAxis != nullptr) {
    SerialDebug.println(
        "[EMM POSITION] blocking read refused while axis/query active");
    return false;
  }
  clearArmLinearReceiveBuffer();
  writeArmLinearCurrentPositionRequest(address);
  SerialArmLinear.flush();

  uint8_t window[8] = {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint8_t count = 0U;
  const uint32_t startMs = millis();

  while (millis() - startMs <
         ARM_LINEAR_POSITION_READ_TIMEOUT_MS) {
    while (SerialArmLinear.available()) {
      const uint8_t incoming =
          static_cast<uint8_t>(SerialArmLinear.read());
      if (count < 8U) {
        window[count++] = incoming;
      } else {
        for (uint8_t i = 0U; i < 7U; ++i) {
          window[i] = window[i + 1U];
        }
        window[7] = incoming;
      }

      if (count != 8U ||
          window[0] != address ||
          window[1] != 0x36U ||
          window[7] != 0x6BU ||
          (window[2] != 0x00U &&
           window[2] != 0x01U)) {
        continue;
      }

      const uint32_t rawPosition =
          (static_cast<uint32_t>(window[3]) << 24) |
          (static_cast<uint32_t>(window[4]) << 16) |
          (static_cast<uint32_t>(window[5]) << 8) |
          static_cast<uint32_t>(window[6]);
      angleDegrees =
          static_cast<float>(rawPosition) *
          360.0f / 65536.0f;
      if (window[2] == 0x01U) {
        angleDegrees = -angleDegrees;
      }
      armLinearReceiveCount = 0U;
      armLinearExpectedFrameLength = 0U;
      return true;
    }
    delay(1);
  }

  armLinearReceiveCount = 0U;
  armLinearExpectedFrameLength = 0U;
  return false;
}

void cancelLinearAxisTerminalVerification(
    LinearAxisMotion &axis) {
  if (armLinearPositionQueryAxis == &axis) {
    armLinearPositionQueryAxis = nullptr;
  }
  axis.terminalOnPositionSamples = 0U;
  axis.terminalVerificationPending = false;
  axis.terminalPositionRequestSent = false;
  axis.terminalStatusFlags = 0U;
  axis.terminalVerificationReason = ARM_AXIS_VERIFY_NONE;
  axis.terminalPositionSamples = 0U;
  axis.terminalFirstPositionMm = 0.0f;
  axis.terminalPositionRequestDeadlineMs = 0UL;
  axis.terminalNextPositionRequestMs = 0UL;
}

void resetLinearAxisTerminalVerification(
    LinearAxisMotion &axis) {
  cancelLinearAxisTerminalVerification(axis);
  axis.terminalVerificationFailures = 0U;
}

void markLinearAxisArrived(LinearAxisMotion &axis) {
  if (axis.address == liftAxis.address) {
    SerialDebug.print(
        "[M7 TRACE] start/target/delta/rpm/acc/elapsed-ms=");
    SerialDebug.print(axis.commandStartMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(axis.targetMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(
        axis.targetMm - axis.commandStartMm,
        2);
    SerialDebug.print("/");
    SerialDebug.print(axis.commandSpeedRpm);
    SerialDebug.print("/");
    SerialDebug.print(axis.commandAcceleration);
    SerialDebug.print("/");
    SerialDebug.println(millis() - axis.startMs);
  }
  resetLinearAxisTerminalVerification(axis);
  axis.active = false;
  axis.currentMm = axis.targetMm;
  axis.positionCommandRejected = false;
  axis.stallProtectionSamples = 0U;
  axis.recoveryPending = false;
  axis.recoveryAttemptCount = 0U;
  axis.recoveryReason = ARM_AXIS_RECOVERY_NONE;
  axis.recoveryDeadlineMs = 0UL;
  SerialDebug.print("Arm axis M");
  SerialDebug.print(axis.address);
  SerialDebug.print(" arrived at ");
  SerialDebug.print(axis.currentMm, 2);
  SerialDebug.println(" mm");
}

void faultLinearAxis(
    LinearAxisMotion &axis,
    const char *reason) {
  if (axis.fault) {
    return;
  }
  resetLinearAxisTerminalVerification(axis);
  axis.active = false;
  axis.fault = true;
  axis.recoveryPending = false;
  axis.recoveryReason = ARM_AXIS_RECOVERY_NONE;
  axis.recoveryDeadlineMs = 0UL;
  if (axis.address == extensionAxis.address) {
    m6ContactSoftLandingPending = false;
  } else if (axis.address == liftAxis.address) {
    m7SoftLandingPending = false;
    m7SoftLandingProfilePolicy =
        M7_PROFILE_APPLY_EXPERIMENT;
  }
  SerialDebug.print("Arm axis M");
  SerialDebug.print(axis.address);
  SerialDebug.print(" fault: ");
  SerialDebug.println(reason);
  routeFault(reason);
}

bool startLinearAxisMove(
    LinearAxisMotion &axis,
    float targetMm,
    float minimumMm,
    float maximumMm,
    float pulsesPerMm,
    uint8_t positiveDirection,
    uint8_t negativeDirection,
    uint16_t speedRpm,
    uint8_t acceleration,
    bool recoveryRetry = false,
    M7ProfilePolicy m7ProfilePolicy =
        M7_PROFILE_APPLY_EXPERIMENT,
    bool allowEmptyGripperBoost = true,
    bool allowRawTrackingConcurrentM7 = false);

const char *linearAxisRecoveryReasonName(
    LinearAxisRecoveryReason reason) {
  switch (reason) {
    case ARM_AXIS_RECOVERY_STALL:
      return "stall/protection";
    case ARM_AXIS_RECOVERY_TIMEOUT:
      return "motion timeout";
    case ARM_AXIS_RECOVERY_COMMAND_REJECTED:
      return "position command rejected";
    case ARM_AXIS_RECOVERY_ACK_TIMEOUT:
      return "position ACK timeout";
    case ARM_AXIS_RECOVERY_NONE:
      break;
  }
  return "unknown";
}

bool linearAxisPositionMmFromMotorAngle(
    const LinearAxisMotion &axis,
    float motorAngleDegrees,
    float &positionMm) {
  if (!axis.driverWorkingZeroAngleValid) {
    return false;
  }
  const float relativeAngleDegrees =
      motorAngleDegrees -
      axis.driverWorkingZeroAngleDegrees;
  if (axis.address == extensionAxis.address) {
    positionMm =
        relativeAngleDegrees *
        M6_TRAVEL_PER_REVOLUTION_MM / 360.0f;
    return true;
  }
  if (axis.address == liftAxis.address) {

    positionMm =
        -relativeAngleDegrees *
        M7_TRAVEL_PER_REVOLUTION_MM / 360.0f;
    return true;
  }
  return false;
}

void scheduleLinearAxisRecovery(
    LinearAxisMotion &axis,
    LinearAxisRecoveryReason reason) {
  if (axis.recoveryPending || axis.fault) {
    return;
  }
  if (!armLinearReferenceValid) {
    faultLinearAxis(
        axis,
        "EMM recovery unavailable before working zero");
    return;
  }
  if (!axis.driverWorkingZeroAngleValid) {
    faultLinearAxis(
        axis,
        "EMM recovery has no verified driver-angle reference");
    return;
  }
  if (axis.recoveryAttemptCount >=
      ARM_AXIS_MAXIMUM_RECOVERY_ATTEMPTS) {
    faultLinearAxis(axis, "EMM automatic recovery exhausted");
    return;
  }

  resetLinearAxisTerminalVerification(axis);
  axis.active = false;
  axis.recoveryPending = true;
  ++axis.recoveryAttemptCount;
  axis.recoveryReason = reason;
  axis.recoveryReadyMs =
      millis() + ARM_AXIS_RECOVERY_STOP_SETTLE_MS;
  axis.recoveryDeadlineMs =
      millis() + ARM_AXIS_RECOVERY_TOTAL_TIMEOUT_MS;

  writeArmLinearStop(axis.address);
  SerialArmLinear.flush();

  SerialDebug.print("[EMM RECOVERY] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, M");
  SerialDebug.print(axis.address);
  SerialDebug.print(" reason=");
  SerialDebug.print(linearAxisRecoveryReasonName(reason));
  SerialDebug.print(", attempt=");
  SerialDebug.print(axis.recoveryAttemptCount);
  SerialDebug.print("/");
  SerialDebug.print(ARM_AXIS_MAXIMUM_RECOVERY_ATTEMPTS);
  SerialDebug.print(", commanded=");
  SerialDebug.print(axis.commandStartMm, 2);
  SerialDebug.print(" -> ");
  SerialDebug.println(axis.targetMm, 2);
}

bool beginEndpointM6ReachRecovery(
    LinearAxisMotion &axis,
    bool cameraReachOverflow);

void serviceLinearAxisRecovery(LinearAxisMotion &axis) {
  if (!axis.recoveryPending) {
    return;
  }
  if (deadlineReached(axis.recoveryDeadlineMs)) {
    faultLinearAxis(axis, "EMM recovery service timeout");
    return;
  }
  if (!deadlineReached(axis.recoveryReadyMs)) {
    return;
  }

  if (armMotors.isM5Running()) {
    return;
  }

  const LinearAxisMotion &otherAxis =
      axis.address == extensionAxis.address
          ? liftAxis
          : extensionAxis;
  if (otherAxis.active ||
      otherAxis.recoveryPending ||
      armLinearPositionQueryAxis != nullptr) {
    return;
  }

  float firstAngleDegrees = 0.0f;
  float secondAngleDegrees = 0.0f;
  if (!readArmLinearCurrentMotorAngleDegrees(
          axis.address,
          firstAngleDegrees)) {
    faultLinearAxis(axis, "EMM recovery position read failed");
    return;
  }
  delay(ARM_AXIS_RECOVERY_SAMPLE_SETTLE_MS);
  if (!readArmLinearCurrentMotorAngleDegrees(
          axis.address,
          secondAngleDegrees)) {
    faultLinearAxis(axis, "EMM recovery position recheck failed");
    return;
  }

  float firstPositionMm = 0.0f;
  float secondPositionMm = 0.0f;
  if (!linearAxisPositionMmFromMotorAngle(
          axis,
          firstAngleDegrees,
          firstPositionMm) ||
      !linearAxisPositionMmFromMotorAngle(
          axis,
          secondAngleDegrees,
          secondPositionMm) ||
      !isfinite(firstPositionMm) ||
      !isfinite(secondPositionMm)) {
    faultLinearAxis(axis, "EMM recovery position invalid");
    return;
  }
  if (fabsf(secondPositionMm - firstPositionMm) >
      ARM_AXIS_RECOVERY_POSITION_STABILITY_MM) {
    faultLinearAxis(
        axis,
        "EMM recovery position still moving");
    return;
  }

  const float measuredPositionMm =
      0.5f * (firstPositionMm + secondPositionMm);
  const float pathMinimumMm =
      (axis.commandStartMm < axis.targetMm
           ? axis.commandStartMm
           : axis.targetMm) -
      ARM_AXIS_RECOVERY_PATH_MARGIN_MM;
  const float pathMaximumMm =
      (axis.commandStartMm > axis.targetMm
           ? axis.commandStartMm
           : axis.targetMm) +
      ARM_AXIS_RECOVERY_PATH_MARGIN_MM;
  if (measuredPositionMm <
          axis.commandMinimumMm -
              ARM_AXIS_RECOVERY_PATH_MARGIN_MM ||
      measuredPositionMm >
          axis.commandMaximumMm +
              ARM_AXIS_RECOVERY_PATH_MARGIN_MM ||
      measuredPositionMm < pathMinimumMm ||
      measuredPositionMm > pathMaximumMm) {
    SerialDebug.print(
        "[EMM RECOVERY] rejected measured/path=");
    SerialDebug.print(measuredPositionMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(pathMinimumMm, 2);
    SerialDebug.print("..");
    SerialDebug.println(pathMaximumMm, 2);
    faultLinearAxis(
        axis,
        "EMM recovery position outside commanded path");
    return;
  }

  axis.currentMm = measuredPositionMm;
  SerialDebug.print("[EMM RECOVERY] M");
  SerialDebug.print(axis.address);
  SerialDebug.print(" measured/stability=");
  SerialDebug.print(measuredPositionMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(
      fabsf(secondPositionMm - firstPositionMm),
      3);
  SerialDebug.print(" mm, remaining=");
  SerialDebug.println(
      axis.targetMm - measuredPositionMm,
      2);

  clearArmLinearReceiveBuffer();
  writeArmLinearResetStallProtection(axis.address);
  SerialArmLinear.flush();
  const bool resetAcknowledged =
      waitForArmLinearSimpleResponse(
          axis.address,
          0x0EU,
          ARM_LINEAR_POSITION_READ_TIMEOUT_MS);
  clearArmLinearReceiveBuffer();
  if (!resetAcknowledged) {
    faultLinearAxis(
        axis,
        "EMM recovery stall-reset ACK missing");
    return;
  }

  // The measured position is stable and the driver's protection reset was
  // acknowledged. In endpoint scanning, move the chassis instead of pushing
  // M6 into the same outward stop a second time.
  if (beginEndpointM6ReachRecovery(axis, false)) {
    return;
  }

  const float retryTargetMm = axis.targetMm;
  if (fabsf(retryTargetMm - measuredPositionMm) <=
      ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM) {
    SerialDebug.print("[EMM RECOVERY] M");
    SerialDebug.print(axis.address);
    SerialDebug.println(
        " target verified by real-time position");
    axis.currentMm = retryTargetMm;
    markLinearAxisArrived(axis);
    return;
  }

  SerialDebug.print("[EMM RECOVERY] M");
  SerialDebug.print(axis.address);
  uint16_t retrySpeedRpm = axis.commandSpeedRpm;
  uint8_t retryAcceleration = axis.commandAcceleration;
  // commandSpeedRpm/commandAcceleration already contain the effective values
  // sent by the original motion.  After clamping them to the recovery limits,
  // M7 must bypass the experiment transform or 2160/171 would be expanded a
  // second time to 5616/240.
  const M7ProfilePolicy retryM7ProfilePolicy =
      axis.address == liftAxis.address
          ? M7_PROFILE_USE_REQUESTED
          : axis.commandM7ProfilePolicy;
  if (axis.address == extensionAxis.address) {
    if (retrySpeedRpm > M6_RECOVERY_SPEED_RPM) {
      retrySpeedRpm = M6_RECOVERY_SPEED_RPM;
    }
    if (retryAcceleration > M6_RECOVERY_ACCELERATION) {
      retryAcceleration = M6_RECOVERY_ACCELERATION;
    }
  } else if (axis.address == liftAxis.address) {
    if (retrySpeedRpm > M7_RECOVERY_SPEED_RPM) {
      retrySpeedRpm = M7_RECOVERY_SPEED_RPM;
    }
    if (retryAcceleration > M7_RECOVERY_ACCELERATION) {
      retryAcceleration = M7_RECOVERY_ACCELERATION;
    }
  }

  SerialDebug.print(" retry remaining at reduced rpm/acc=");
  SerialDebug.print(retrySpeedRpm);
  SerialDebug.print("/");
  SerialDebug.println(retryAcceleration);

  if (deadlineReached(axis.recoveryDeadlineMs)) {
    faultLinearAxis(
        axis,
        "EMM recovery total timeout before retry");
    return;
  }
  axis.recoveryPending = false;
  axis.recoveryReason = ARM_AXIS_RECOVERY_NONE;

  if (!startLinearAxisMove(
          axis,
          retryTargetMm,
          axis.commandMinimumMm,
          axis.commandMaximumMm,
          axis.commandPulsesPerMm,
          axis.commandPositiveDirection,
          axis.commandNegativeDirection,
          retrySpeedRpm,
          retryAcceleration,
          true,
          retryM7ProfilePolicy) &&
      programState != PROGRAM_FAULT) {
    faultLinearAxis(axis, "EMM recovery retry rejected");
  }
}

void requestLinearAxisTerminalVerification(
    LinearAxisMotion &axis,
    uint8_t statusFlags,
    LinearAxisTerminalVerificationReason reason) {
  if (!axis.active ||
      axis.recoveryPending ||
      axis.terminalVerificationFailures >=
          ARM_AXIS_TERMINAL_VERIFY_MAX_FAILURES) {
    return;
  }

  if (axis.terminalVerificationPending) {
    return;
  }

  axis.terminalStatusFlags = statusFlags;
  axis.terminalVerificationReason = reason;
  axis.terminalVerificationPending = true;
  axis.terminalPositionRequestSent = false;
  axis.terminalPositionSamples = 0U;
  axis.terminalFirstPositionMm = 0.0f;
  axis.terminalNextPositionRequestMs = millis();

  // Fast M6/M7 polling runs while approaching the target. Avoid
  // flooding the debug UART and perturbing the very timing being measured;
  // only near-target samples and the accepted arrival are printed later.
  if (reason == ARM_AXIS_VERIFY_M7_FAST_ARRIVAL ||
      reason == ARM_AXIS_VERIFY_M6_FAST_ARRIVAL) {
    return;
  }

  SerialDebug.print("[EMM VERIFY] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, M");
  SerialDebug.print(axis.address);
  SerialDebug.print(" reason=");
  switch (reason) {
    case ARM_AXIS_VERIFY_HEALTHY_WITHOUT_COMMAND_EVIDENCE:
      SerialDebug.print("healthy/no-command-evidence");
      break;
    case ARM_AXIS_VERIFY_LOCKED_ON_POSITION:
      SerialDebug.print("locked/on-position");
      break;
    case ARM_AXIS_VERIFY_REJECTED_ON_POSITION:
      SerialDebug.print("rejected/on-position");
      break;
    case ARM_AXIS_VERIFY_EXPECTED_COMPLETION:
      SerialDebug.print("expected-completion/no-terminal-frame");
      break;
    case ARM_AXIS_VERIFY_M7_FAST_ARRIVAL:
      SerialDebug.print("M7 encoder fast-arrival");
      break;
    case ARM_AXIS_VERIFY_M6_FAST_ARRIVAL:
      SerialDebug.print("M6 encoder fast-arrival");
      break;
    case ARM_AXIS_VERIFY_NONE:
      SerialDebug.print("none");
      break;
  }
  SerialDebug.print(", flags=0x");
  SerialDebug.println(axis.terminalStatusFlags, HEX);
}

void noteLinearAxisTerminalVerificationFailure(
    LinearAxisMotion &axis,
    const char *reason) {
  if (armLinearPositionQueryAxis == &axis) {
    armLinearPositionQueryAxis = nullptr;
  }
  axis.terminalPositionRequestSent = false;
  axis.terminalPositionSamples = 0U;
  axis.terminalFirstPositionMm = 0.0f;
  if (axis.terminalVerificationFailures < 255U) {
    ++axis.terminalVerificationFailures;
  }

  SerialDebug.print("[EMM VERIFY] M");
  SerialDebug.print(axis.address);
  SerialDebug.print(" failed ");
  SerialDebug.print(axis.terminalVerificationFailures);
  SerialDebug.print("/");
  SerialDebug.print(
      ARM_AXIS_TERMINAL_VERIFY_MAX_FAILURES);
  SerialDebug.print(": ");
  SerialDebug.println(reason);

  if (axis.terminalVerificationFailures >=
      ARM_AXIS_TERMINAL_VERIFY_MAX_FAILURES) {
    axis.terminalVerificationPending = false;
    axis.terminalVerificationReason = ARM_AXIS_VERIFY_NONE;
    SerialDebug.println(
        "[EMM VERIFY] verification exhausted; retain normal axis timeout");
    return;
  }
  axis.terminalVerificationPending = true;
  axis.terminalNextPositionRequestMs =
      millis() + ARM_AXIS_RECOVERY_SAMPLE_SETTLE_MS;
}

void handleArmLinearPositionFrame(const uint8_t *frame) {
  LinearAxisMotion *axis = armLinearPositionQueryAxis;
  if (axis == nullptr ||
      !axis->active ||
      !axis->terminalVerificationPending ||
      frame[0] != axis->address ||
      frame[1] != 0x36U ||
      frame[7] != 0x6BU ||
      (frame[2] != 0x00U && frame[2] != 0x01U)) {
    return;
  }

  armLinearPositionQueryAxis = nullptr;
  axis->terminalPositionRequestSent = false;
  const uint32_t rawPosition =
      (static_cast<uint32_t>(frame[3]) << 24) |
      (static_cast<uint32_t>(frame[4]) << 16) |
      (static_cast<uint32_t>(frame[5]) << 8) |
      static_cast<uint32_t>(frame[6]);
  float angleDegrees =
      static_cast<float>(rawPosition) *
      360.0f / 65536.0f;
  if (frame[2] == 0x01U) {
    angleDegrees = -angleDegrees;
  }

  float measuredPositionMm = 0.0f;
  if (!linearAxisPositionMmFromMotorAngle(
          *axis,
          angleDegrees,
          measuredPositionMm) ||
      !isfinite(measuredPositionMm)) {
    if (axis->terminalVerificationReason ==
            ARM_AXIS_VERIFY_M7_FAST_ARRIVAL ||
        axis->terminalVerificationReason ==
            ARM_AXIS_VERIFY_M6_FAST_ARRIVAL) {
      cancelLinearAxisTerminalVerification(*axis);
      return;
    }
    noteLinearAxisTerminalVerificationFailure(
        *axis,
        "invalid encoder position");
    return;
  }

  if (axis->terminalPositionSamples == 0U) {
    axis->terminalFirstPositionMm = measuredPositionMm;
    axis->terminalPositionSamples = 1U;
    axis->terminalNextPositionRequestMs =
        millis() +
        (axis->terminalVerificationReason ==
                 ARM_AXIS_VERIFY_M7_FAST_ARRIVAL
             ? M7_FAST_ARRIVAL_SAMPLE_GAP_MS
             : (axis->terminalVerificationReason ==
                        ARM_AXIS_VERIFY_M6_FAST_ARRIVAL
                    ? M6_FAST_ARRIVAL_SAMPLE_GAP_MS
                    : ARM_AXIS_RECOVERY_SAMPLE_SETTLE_MS));
    return;
  }

  const float stabilityMm =
      fabsf(
          measuredPositionMm -
          axis->terminalFirstPositionMm);
  const float averagedPositionMm =
      0.5f *
      (measuredPositionMm +
       axis->terminalFirstPositionMm);
  const float targetErrorMm =
      fabsf(axis->targetMm - averagedPositionMm);
  const LinearAxisTerminalVerificationReason
      verificationReason =
          axis->terminalVerificationReason;
  const bool m6FastArrival =
      verificationReason ==
      ARM_AXIS_VERIFY_M6_FAST_ARRIVAL;
  const bool m7FastArrival =
      verificationReason ==
      ARM_AXIS_VERIFY_M7_FAST_ARRIVAL;
  const bool encoderFastArrival =
      m6FastArrival || m7FastArrival;
  const bool stable =
      stabilityMm <=
      (encoderFastArrival
           ? (m6FastArrival
                  ? M6_FAST_ARRIVAL_STABILITY_MM
                  : M7_FAST_ARRIVAL_STABILITY_MM)
           : ARM_AXIS_RECOVERY_POSITION_STABILITY_MM);
  const bool targetVerified =
      stable &&
      targetErrorMm <=
          (encoderFastArrival
               ? (m6FastArrival
                      ? M6_FAST_ARRIVAL_TARGET_TOLERANCE_MM
                      : M7_FAST_ARRIVAL_TARGET_TOLERANCE_MM)
               : ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM);
  const bool protectionLatched =
      (axis->terminalStatusFlags & 0x08U) != 0U;
  const bool fastStatusFresh =
      axis->lastStatusResponseMs != 0UL &&
      millis() - axis->lastStatusResponseMs <=
          (m6FastArrival
               ? M6_FAST_ARRIVAL_STATUS_FRESH_MS
               : M7_FAST_ARRIVAL_STATUS_FRESH_MS);
  const bool fastStatusHealthy =
      fastStatusFresh &&
      (axis->terminalStatusFlags & 0x01U) != 0U &&
      !protectionLatched &&
      !axis->positionCommandRejected &&
      (axis->commandAcknowledged || axis->motionObserved);

  if (!encoderFastArrival || targetErrorMm <= 2.0f) {
    SerialDebug.print("[EMM VERIFY] M");
    SerialDebug.print(axis->address);
    SerialDebug.print(
        " measured/target/error/stability/flags=");
    SerialDebug.print(averagedPositionMm, 3);
    SerialDebug.print("/");
    SerialDebug.print(axis->targetMm, 3);
    SerialDebug.print("/");
    SerialDebug.print(targetErrorMm, 3);
    SerialDebug.print("/");
    SerialDebug.print(stabilityMm, 3);
    SerialDebug.print("/0x");
    SerialDebug.println(axis->terminalStatusFlags, HEX);
  }

  axis->terminalVerificationPending = false;
  axis->terminalPositionSamples = 0U;
  if (encoderFastArrival) {
    if (protectionLatched) {
      scheduleLinearAxisRecovery(
          *axis,
          ARM_AXIS_RECOVERY_STALL);
      return;
    }
    if (targetVerified && fastStatusHealthy) {
      SerialDebug.print("[M");
      SerialDebug.print(axis->address);
      SerialDebug.print(
          " FAST ARRIVAL] encoder stable; bypass delayed driver "
          "terminal, elapsed/status-age-ms=");
      SerialDebug.print(millis() - axis->startMs);
      SerialDebug.print("/");
      SerialDebug.println(
          millis() - axis->lastStatusResponseMs);
      axis->currentMm = axis->targetMm;
      markLinearAxisArrived(*axis);
    }
    return;
  }
  if (!stable) {
    noteLinearAxisTerminalVerificationFailure(
        *axis,
        "encoder position still moving");
    return;
  }

  if (protectionLatched) {

    if (!armLinearReferenceValid) {
      faultLinearAxis(
          *axis,
          "EMM protection latched during startup move");
      return;
    }
    scheduleLinearAxisRecovery(
        *axis,
        ARM_AXIS_RECOVERY_STALL);
    return;
  }

  if (targetVerified) {
    axis->currentMm = axis->targetMm;
    SerialDebug.print("[EMM VERIFY] M");
    SerialDebug.print(axis->address);
    SerialDebug.println(
        " target accepted from two encoder samples");
    markLinearAxisArrived(*axis);
    return;
  }

  if (verificationReason ==
      ARM_AXIS_VERIFY_REJECTED_ON_POSITION) {
    if (!armLinearReferenceValid) {
      faultLinearAxis(
          *axis,
          "EMM rejected startup move missed encoder target");
      return;
    }
    scheduleLinearAxisRecovery(
        *axis,
        ARM_AXIS_RECOVERY_COMMAND_REJECTED);
    return;
  }

  noteLinearAxisTerminalVerificationFailure(
      *axis,
      verificationReason ==
              ARM_AXIS_VERIFY_LOCKED_ON_POSITION
          ? "locked position is not at target"
          : (verificationReason ==
                     ARM_AXIS_VERIFY_EXPECTED_COMPLETION
                 ? "expected completion is not at target"
                 : "healthy status still reports old position"));
}

void serviceLinearAxisTerminalVerification(
    LinearAxisMotion &axis,
    uint32_t nowMs) {
  if (!axis.active ||
      !axis.terminalVerificationPending) {
    return;
  }

  if (axis.terminalPositionRequestSent) {
    if (deadlineReached(
            axis.terminalPositionRequestDeadlineMs)) {
      if (axis.terminalVerificationReason ==
              ARM_AXIS_VERIFY_M7_FAST_ARRIVAL ||
          axis.terminalVerificationReason ==
              ARM_AXIS_VERIFY_M6_FAST_ARRIVAL) {
        cancelLinearAxisTerminalVerification(axis);
      } else {
        noteLinearAxisTerminalVerificationFailure(
            axis,
            "encoder response timeout");
      }
    }
    return;
  }
  if (!deadlineReached(
          axis.terminalNextPositionRequestMs) ||
      armLinearPositionQueryAxis != nullptr) {
    return;
  }

  const LinearAxisMotion &otherAxis =
      axis.address == extensionAxis.address
          ? liftAxis
          : extensionAxis;
  if (otherAxis.active || otherAxis.recoveryPending) {
    return;
  }
  if (axis.address == extensionAxis.address &&
      armMotors.isM5Running()) {
    return;
  }

  armLinearPositionQueryAxis = &axis;
  axis.terminalPositionRequestSent = true;
  axis.terminalPositionRequestDeadlineMs =
      nowMs +
      (axis.terminalVerificationReason ==
               ARM_AXIS_VERIFY_M7_FAST_ARRIVAL
           ? M7_FAST_ARRIVAL_RESPONSE_TIMEOUT_MS
           : (axis.terminalVerificationReason ==
                      ARM_AXIS_VERIFY_M6_FAST_ARRIVAL
                  ? M6_FAST_ARRIVAL_RESPONSE_TIMEOUT_MS
                  : ARM_LINEAR_POSITION_READ_TIMEOUT_MS));
  writeArmLinearCurrentPositionRequest(axis.address);
  SerialArmLinear.flush();
}

bool consumeArmLinearEnableResponse(
    LinearAxisMotion &axis) {
  uint8_t window[4] = {0U, 0U, 0U, 0U};
  uint8_t count = 0U;
  bool enableResponseSeen = false;
  bool enableResponseAccepted = false;
  const uint32_t waitStartMs = millis();
  const uint32_t additionalWaitMs =
      ARM_AXIS_ENABLE_RESPONSE_WAIT_MS >
              ARM_AXIS_COMMAND_GUARD_MS
          ? ARM_AXIS_ENABLE_RESPONSE_WAIT_MS -
                ARM_AXIS_COMMAND_GUARD_MS
          : 0UL;

  while (true) {
    while (SerialArmLinear.available()) {
    const uint8_t incoming =
        static_cast<uint8_t>(SerialArmLinear.read());
    if (count < 4U) {
      window[count++] = incoming;
    } else {
      window[0] = window[1];
      window[1] = window[2];
      window[2] = window[3];
      window[3] = incoming;
    }

    if (count != 4U || incoming != 0x6BU) {
      continue;
    }

    if (window[0] == axis.address &&
        window[1] == 0xF3U) {
      enableResponseSeen = true;
      if (window[2] == 0x02U) {
        enableResponseAccepted = true;
        printArmLinearFrame("enable accepted", window);
      } else {
        printArmLinearFrame(
            "enable condition not met", window);

      }
    } else if (
        window[0] == axis.address &&
        window[1] == 0x00U &&
        window[2] == 0xEEU) {
      printArmLinearFrame(
          "enable wrong command", window);
      faultLinearAxis(axis, "EMM enable wrong command");
      armLinearReceiveCount = 0U;
      armLinearExpectedFrameLength = 0U;
      return false;
    }
    count = 0U;
  }

    if (enableResponseSeen ||
        millis() - waitStartMs >= additionalWaitMs) {
      break;
    }

    armMotors.serviceM5();
  }

  armLinearReceiveCount = 0U;
  armLinearExpectedFrameLength = 0U;
  if (!enableResponseSeen) {

    SerialDebug.print("[EMM ENABLE] t=");
    SerialDebug.print(millis());
    SerialDebug.print(" ms, M");
    SerialDebug.print(axis.address);
    SerialDebug.println(" no F3 ACK");
  } else if (!enableResponseAccepted) {
    faultLinearAxis(axis, "EMM enable condition not met");
    return false;
  }
  return true;
}

void handleArmLinearFrame(const uint8_t *frame) {
  if (frame[3] != 0x6BU) {
    return;
  }

  LinearAxisMotion *axis =
      linearAxisForAddress(frame[0]);
  if (axis == nullptr) {
    return;
  }
  const bool encoderFastArrivalEnabledForAxis =
      (M6_ENCODER_FAST_ARRIVAL_ENABLED &&
       axis->address == ARM_EXTENSION_ADDRESS) ||
      (M7_ENCODER_FAST_ARRIVAL_ENABLED &&
       axis->address == ARM_LIFT_ADDRESS);
  if (encoderFastArrivalEnabledForAxis &&
      frame[1] == 0xFDU &&
      frame[2] == 0x9FU) {
    SerialDebug.print("[M");
    SerialDebug.print(axis->address);
    SerialDebug.print(" DRIVER TERMINAL] t/active=");
    SerialDebug.print(millis());
    SerialDebug.print("/");
    SerialDebug.print(axis->active ? 1 : 0);
    SerialDebug.println(
        "; ignored to prevent a delayed old frame completing a new move");
    return;
  }
  if (!axis->active) {
    return;
  }

  const uint32_t elapsedMs = millis() - axis->startMs;
  const uint32_t minimumOnPositionMs =
      axis->address == ARM_LIFT_ADDRESS
          ? M7_AXIS_MINIMUM_ON_POSITION_MS
          : ARM_AXIS_MINIMUM_ON_POSITION_MS;

  if (frame[1] == 0x00U && frame[2] == 0xEEU) {
    const ArmLinearRequestKind requestKind =
        axis->lastRequestKind;
    const bool queryRelated =
        requestKind == ARM_LINEAR_REQUEST_STATUS ||
        requestKind == ARM_LINEAR_REQUEST_POSITION_QUERY;

    printArmLinearFrame("wrong command", frame);
    // Request kinds in the diagnostic are 2=position, 3=status and
    // 4=position-query.  Numeric output keeps this safety path flash-small.
    SerialDebug.print("[EMM WRONG] kind=");
    SerialDebug.println(static_cast<unsigned int>(requestKind));

    if (queryRelated) {
      if (requestKind ==
          ARM_LINEAR_REQUEST_POSITION_QUERY) {
        if (armLinearPositionQueryAxis == axis) {
          armLinearPositionQueryAxis = nullptr;
        }
        axis->terminalPositionRequestSent = false;
        if (axis->terminalVerificationPending) {
          if (axis->terminalVerificationReason ==
                  ARM_AXIS_VERIFY_M7_FAST_ARRIVAL ||
              axis->terminalVerificationReason ==
                  ARM_AXIS_VERIFY_M6_FAST_ARRIVAL) {
            cancelLinearAxisTerminalVerification(*axis);
          } else {
            noteLinearAxisTerminalVerificationFailure(
                *axis,
                "encoder query rejected");
          }
        }
      }
      axis->lastRequestKind = ARM_LINEAR_REQUEST_NONE;
      return;
    }

    if (requestKind == ARM_LINEAR_REQUEST_POSITION &&
        !axis->commandAcknowledged &&
        !axis->motionObserved) {
      if (!armLinearReferenceValid) {
        faultLinearAxis(
            *axis,
            "EMM position malformed at zero");
      } else {
        scheduleLinearAxisRecovery(
            *axis,
            ARM_AXIS_RECOVERY_COMMAND_REJECTED);
      }
      return;
    }

    if (axis->commandAcknowledged || axis->motionObserved) {
      axis->lastRequestKind = ARM_LINEAR_REQUEST_NONE;
      return;
    }

    faultLinearAxis(*axis, "EMM wrong command unclassified");
    return;
  }

  if (frame[1] == 0xFDU && frame[2] == 0xE2U) {
    printArmLinearFrame(
        "position condition not met", frame);

    resetLinearAxisTerminalVerification(*axis);
    axis->positionCommandRejected = true;
    axis->commandAcknowledged = false;
    axis->motionObserved = false;
    axis->stallProtectionSamples = 0U;
    axis->lastStatusRequestMs = millis();
    writeArmLinearStatusRequest(axis->address);
    return;
  }

  if (frame[1] == 0xFDU && frame[2] == 0x02U) {
    if (axis->terminalVerificationPending &&
        axis->terminalVerificationReason ==
            ARM_AXIS_VERIFY_REJECTED_ON_POSITION) {
      resetLinearAxisTerminalVerification(*axis);
    }
    axis->positionCommandRejected = false;
    axis->commandAcknowledged = true;
    SerialDebug.print("[EMM ACK] t=");
    SerialDebug.print(millis());
    SerialDebug.print(" ms, M");
    SerialDebug.print(axis->address);
    SerialDebug.println(" position accepted");
    return;
  }

  if (frame[1] == 0x3AU) {
    const uint8_t flags = frame[2];
    axis->lastStatusFlags = flags;
    axis->lastStatusResponseMs = millis();
    const bool enabled = (flags & 0x01U) != 0U;
    const bool onPosition = (flags & 0x02U) != 0U;
    const bool locked = (flags & 0x04U) != 0U;
    const bool protectionLatched =
        (flags & 0x08U) != 0U;
    const bool verifiedOnPosition =
        enabled &&
        onPosition &&
        !locked &&
        !protectionLatched &&
        !axis->positionCommandRejected &&
        elapsedMs >= minimumOnPositionMs &&
        (axis->commandAcknowledged ||
         axis->motionObserved);

    if (verifiedOnPosition) {
      markLinearAxisArrived(*axis);
      return;
    }
    const bool stallCandidate =
        protectionLatched ||
        (locked &&
         (!onPosition ||
          elapsedMs >= minimumOnPositionMs));
    if ((protectionLatched || (locked && !onPosition)) &&
        axis->terminalVerificationPending) {
      cancelLinearAxisTerminalVerification(*axis);
    }

    if (stallCandidate) {
      if (axis->stallProtectionSamples < 255U) {
        ++axis->stallProtectionSamples;
      }
      SerialDebug.print("[EMM STALL CHECK] t=");
      SerialDebug.print(millis());
      SerialDebug.print(" ms, M");
      SerialDebug.print(axis->address);
      SerialDebug.print(" flags=0x");
      SerialDebug.print(flags, HEX);
      SerialDebug.print(", confirm=");
      SerialDebug.print(axis->stallProtectionSamples);
      SerialDebug.print("/");
      SerialDebug.println(
          ARM_AXIS_STALL_CONFIRMATION_SAMPLES);

      const bool lockedOnPositionForVerification =
          locked &&
          onPosition &&
          !protectionLatched &&
          !axis->positionCommandRejected &&
          elapsedMs >= minimumOnPositionMs;
      if (lockedOnPositionForVerification &&
          !axis->terminalVerificationPending &&
          axis->terminalVerificationFailures <
              ARM_AXIS_TERMINAL_VERIFY_MAX_FAILURES) {
        requestLinearAxisTerminalVerification(
            *axis,
            flags,
            ARM_AXIS_VERIFY_LOCKED_ON_POSITION);
      }

      if (lockedOnPositionForVerification &&
          axis->terminalVerificationPending) {
        return;
      }
      if (axis->stallProtectionSamples <
          ARM_AXIS_STALL_CONFIRMATION_SAMPLES) {
        return;
      }

      printArmLinearFrame("stall/protection confirmed", frame);
      if (!armLinearReferenceValid) {
        faultLinearAxis(
            *axis,
            "EMM stall confirmed while establishing working zero");
        return;
      }
      scheduleLinearAxisRecovery(
          *axis,
          ARM_AXIS_RECOVERY_STALL);
      return;
    }

    axis->stallProtectionSamples = 0U;
    if (!enabled) {
      printArmLinearFrame("axis disabled", frame);
      faultLinearAxis(*axis, "EMM axis is not enabled");
      return;
    }

    if (axis->positionCommandRejected) {
      SerialDebug.print("[EMM REJECT CLASSIFY] t=");
      SerialDebug.print(millis());
      SerialDebug.print(" ms, M");
      SerialDebug.print(axis->address);
      SerialDebug.print(", healthy flags=0x");
      SerialDebug.println(flags, HEX);
      if (onPosition &&
          elapsedMs >= minimumOnPositionMs) {
        requestLinearAxisTerminalVerification(
            *axis,
            flags,
            ARM_AXIS_VERIFY_REJECTED_ON_POSITION);
        return;
      }
      if (!armLinearReferenceValid) {
        faultLinearAxis(
            *axis,
            "EMM position command rejected during working zero");
        return;
      }
      scheduleLinearAxisRecovery(
          *axis,
          ARM_AXIS_RECOVERY_COMMAND_REJECTED);
      return;
    }

    if (!onPosition) {

      cancelLinearAxisTerminalVerification(*axis);
      axis->motionObserved = true;
      return;
    }

    if (elapsedMs < minimumOnPositionMs) {
      axis->terminalOnPositionSamples = 0U;
      return;
    }
    if (axis->commandAcknowledged ||
        axis->motionObserved) {
      markLinearAxisArrived(*axis);
      return;
    }

    if (axis->terminalOnPositionSamples < 255U) {
      ++axis->terminalOnPositionSamples;
    }
    if (axis->terminalOnPositionSamples >=
        ARM_AXIS_TERMINAL_CONFIRMATION_SAMPLES) {
      requestLinearAxisTerminalVerification(
          *axis,
          flags,
          ARM_AXIS_VERIFY_HEALTHY_WITHOUT_COMMAND_EVIDENCE);
    }
    return;
  }

  if (frame[1] == 0xFDU &&
      frame[2] == 0x9FU) {
    markLinearAxisArrived(*axis);
  }
}

void serviceArmLinearAxes() {
  while (SerialArmLinear.available()) {
    const uint8_t incoming =
        static_cast<uint8_t>(SerialArmLinear.read());

    if (armLinearReceiveCount == 0U) {
      if (linearAxisForAddress(incoming) == nullptr) {
        continue;
      }
      armLinearReceiveWindow[0] = incoming;
      armLinearReceiveCount = 1U;
      armLinearExpectedFrameLength = 0U;
      continue;
    }

    if (armLinearReceiveCount >=
        sizeof(armLinearReceiveWindow)) {
      armLinearReceiveCount = 0U;
      armLinearExpectedFrameLength = 0U;
      continue;
    }
    armLinearReceiveWindow[armLinearReceiveCount++] =
        incoming;
    if (armLinearReceiveCount == 2U) {
      armLinearExpectedFrameLength =
          armLinearReceiveWindow[1] == 0x36U
              ? 8U
              : 4U;
    }
    if (armLinearExpectedFrameLength == 0U ||
        armLinearReceiveCount <
            armLinearExpectedFrameLength) {
      continue;
    }

    const bool completeFrame =
        armLinearReceiveWindow[
            armLinearExpectedFrameLength - 1U] ==
        0x6BU;
    if (completeFrame &&
        armLinearExpectedFrameLength == 8U) {
      handleArmLinearPositionFrame(
          armLinearReceiveWindow);
    } else if (completeFrame &&
               armLinearExpectedFrameLength == 4U) {
      handleArmLinearFrame(armLinearReceiveWindow);
    }

    armLinearReceiveCount = 0U;
    armLinearExpectedFrameLength = 0U;
  }

  LinearAxisMotion *const axes[2] = {
      &extensionAxis, &liftAxis};
  const uint32_t nowMs = millis();
  bool statusRequestIssued = false;
  for (uint8_t offset = 0U; offset < 2U; ++offset) {
    const uint8_t i =
        static_cast<uint8_t>(
            (armLinearStatusPollCursor + offset) % 2U);
    LinearAxisMotion &axis = *axes[i];
    if (axis.recoveryDeadlineMs != 0UL &&
        deadlineReached(axis.recoveryDeadlineMs)) {
      faultLinearAxis(
          axis,
          "EMM recovery total timeout");
      continue;
    }
    if (axis.recoveryPending) {
      serviceLinearAxisRecovery(axis);
      continue;
    }
    if (!axis.active) {
      continue;
    }

    const uint32_t commandAgeMs = nowMs - axis.startMs;
    if (!axis.commandAcknowledged &&
        !axis.motionObserved &&
        commandAgeMs >=
            ARM_AXIS_POSITION_ACK_TIMEOUT_MS) {
      const bool postCommandStatusSeen =
          axis.lastStatusResponseMs != 0UL;
      const bool statusGraceExpired =
          commandAgeMs >=
              ARM_AXIS_POSITION_ACK_TIMEOUT_MS +
                  ARM_AXIS_POSITION_ACK_STATUS_GRACE_MS;
      if (postCommandStatusSeen || statusGraceExpired) {
        SerialDebug.print("[EMM ACK TIMEOUT] M");
        SerialDebug.println(axis.address);
        if (!armLinearReferenceValid) {
          faultLinearAxis(
              axis,
              "EMM ACK timeout at zero");
        } else {
          scheduleLinearAxisRecovery(
              axis,
              ARM_AXIS_RECOVERY_ACK_TIMEOUT);
        }
        continue;
      }
    }

    if (nowMs - axis.startMs >= axis.timeoutMs) {
      scheduleLinearAxisRecovery(
          axis,
          ARM_AXIS_RECOVERY_TIMEOUT);
      continue;
    }

    const uint32_t expectedCompletionVerifyMs =
        axis.timeoutMs >
                ARM_AXIS_EXPECTED_COMPLETION_VERIFY_MARGIN_MS
            ? axis.timeoutMs -
                  ARM_AXIS_EXPECTED_COMPLETION_VERIFY_MARGIN_MS
            : axis.timeoutMs;
    if (!axis.terminalVerificationPending &&
        axis.terminalVerificationFailures <
            ARM_AXIS_TERMINAL_VERIFY_MAX_FAILURES &&
        nowMs - axis.startMs >=
            expectedCompletionVerifyMs) {
      requestLinearAxisTerminalVerification(
          axis,
          0U,
          ARM_AXIS_VERIFY_EXPECTED_COMPLETION);
    }
    const bool fastArrivalAxis =
        (M6_ENCODER_FAST_ARRIVAL_ENABLED &&
         axis.address == ARM_EXTENSION_ADDRESS) ||
        (M7_ENCODER_FAST_ARRIVAL_ENABLED &&
         axis.address == ARM_LIFT_ADDRESS);
    const uint32_t fastArrivalStatusFreshMs =
        axis.address == ARM_EXTENSION_ADDRESS
            ? M6_FAST_ARRIVAL_STATUS_FRESH_MS
            : M7_FAST_ARRIVAL_STATUS_FRESH_MS;
    const uint32_t fastArrivalMinimumAgeMs =
        axis.address == ARM_EXTENSION_ADDRESS
            ? ARM_AXIS_MINIMUM_ON_POSITION_MS
            : M7_AXIS_MINIMUM_ON_POSITION_MS;
    const uint32_t fastArrivalQueryIntervalMs =
        axis.address == ARM_EXTENSION_ADDRESS
            ? M6_FAST_ARRIVAL_QUERY_INTERVAL_MS
            : M7_FAST_ARRIVAL_QUERY_INTERVAL_MS;
    const bool fastArrivalStatusHealthy =
        fastArrivalAxis &&
        armLinearReferenceValid &&
        axis.driverWorkingZeroAngleValid &&
        axis.lastStatusResponseMs != 0UL &&
        nowMs - axis.lastStatusResponseMs <=
            fastArrivalStatusFreshMs &&
        (axis.lastStatusFlags & 0x01U) != 0U &&
        (axis.lastStatusFlags & 0x08U) == 0U &&
        !axis.positionCommandRejected &&
        (axis.commandAcknowledged || axis.motionObserved);
    if (!axis.terminalVerificationPending &&
        armLinearPositionQueryAxis == nullptr &&
        fastArrivalStatusHealthy &&
        nowMs - axis.startMs >=
            fastArrivalMinimumAgeMs &&
        (axis.lastFastArrivalQueryMs == 0UL ||
         nowMs - axis.lastFastArrivalQueryMs >=
             fastArrivalQueryIntervalMs)) {
      axis.lastFastArrivalQueryMs = nowMs;
      requestLinearAxisTerminalVerification(
          axis,
          axis.lastStatusFlags,
          axis.address == ARM_EXTENSION_ADDRESS
              ? ARM_AXIS_VERIFY_M6_FAST_ARRIVAL
              : ARM_AXIS_VERIFY_M7_FAST_ARRIVAL);
    }
    serviceLinearAxisTerminalVerification(
        axis,
        nowMs);
    if (axis.terminalVerificationPending) {
      continue;
    }

    if (!statusRequestIssued &&
        armLinearPositionQueryAxis == nullptr &&
        deadlineReached(armLinearNextStatusRequestMs) &&
        nowMs - axis.lastStatusRequestMs >=
            (axis.address == ARM_LIFT_ADDRESS
                 ? M7_AXIS_STATUS_INTERVAL_MS
                 : ARM_AXIS_STATUS_INTERVAL_MS)) {
      axis.lastStatusRequestMs = nowMs;
      writeArmLinearStatusRequest(axis.address);
      statusRequestIssued = true;
      armLinearStatusPollCursor =
          static_cast<uint8_t>((i + 1U) % 2U);
      armLinearNextStatusRequestMs = nowMs + 2UL;
    }
  }

  if (m6ContactSoftLandingPending &&
      !extensionAxis.active &&
      !extensionAxis.recoveryPending) {
    if (extensionAxis.fault ||
        programState == PROGRAM_FAULT) {
      m6ContactSoftLandingPending = false;
      return;
    }

    const float finalTargetMm =
        m6ContactSoftLandingTargetMm;
    const float finalMinimumMm =
        m6ContactSoftLandingMinimumMm;
    const uint16_t finalSpeedRpm =
        m6ContactSoftLandingSpeedRpm;
    const uint8_t finalAcceleration =
        m6ContactSoftLandingAcceleration;
    m6ContactSoftLandingPending = false;
    SerialDebug.print(
        "[M6 CONTACT SOFT] final 2.0 mm, target/rpm/acc=");
    SerialDebug.print(finalTargetMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(finalSpeedRpm);
    SerialDebug.print("/");
    SerialDebug.println(finalAcceleration);
    if (!startLinearAxisMove(
            extensionAxis,
            finalTargetMm,
            finalMinimumMm,
            M6_MAXIMUM_EXTENSION_MM,
            M6_PULSES_PER_MM,
            M6_EXTEND_DIRECTION,
            M6_RETRACT_DIRECTION,
            finalSpeedRpm,
            finalAcceleration,
            false,
            M7_PROFILE_APPLY_EXPERIMENT,
            false) &&
        programState != PROGRAM_FAULT) {
      routeFault("M6 contact-soft final segment rejected");
    }
  }

  if (m7SoftLandingPending &&
      !liftAxis.active &&
      !liftAxis.recoveryPending) {
    if (liftAxis.fault || programState == PROGRAM_FAULT) {
      m7SoftLandingPending = false;
      m7SoftLandingProfilePolicy =
          M7_PROFILE_APPLY_EXPERIMENT;
      return;
    }

    const float finalTargetMm =
        m7SoftLandingTargetMm;
    const float finalDistanceMm =
        m7SoftLandingDistanceMm;
    const bool contactSegment =
        m7SoftLandingIsContact;
    const uint16_t finalSpeedRpm =
        m7SoftLandingSpeedRpm;
    const uint8_t finalAcceleration =
        m7SoftLandingAcceleration;
    const M7ProfilePolicy finalM7ProfilePolicy =
        m7SoftLandingProfilePolicy;
    m7SoftLandingPending = false;
    m7SoftLandingProfilePolicy =
        M7_PROFILE_APPLY_EXPERIMENT;
    SerialDebug.print(
        contactSegment
            ? "[M7 CONTACT SOFT] final "
            : "[M7 SOFT ZERO] final ");
    SerialDebug.print(finalDistanceMm, 1);
    SerialDebug.print(" mm, target/rpm/acc=");
    SerialDebug.print(finalTargetMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(finalSpeedRpm);
    SerialDebug.print("/");
    SerialDebug.println(finalAcceleration);
    if (!startLinearAxisMove(
            liftAxis,
            finalTargetMm,
            M7_MINIMUM_HEIGHT_MM,
            M7_STANDARD_HEIGHT_MM,
            M7_PULSES_PER_MM,
            M7_RAISE_DIRECTION,
            M7_LOWER_DIRECTION,
            finalSpeedRpm,
            finalAcceleration,
            false,
            finalM7ProfilePolicy,
            false) &&
        programState != PROGRAM_FAULT) {
      routeFault("M7 soft final segment rejected");
    }
  }
}

bool startLinearAxisMove(
    LinearAxisMotion &axis,
    float targetMm,
    float minimumMm,
    float maximumMm,
    float pulsesPerMm,
    uint8_t positiveDirection,
    uint8_t negativeDirection,
    uint16_t speedRpm,
    uint8_t acceleration,
    bool recoveryRetry,
    M7ProfilePolicy m7ProfilePolicy,
    bool allowEmptyGripperBoost,
    bool allowRawTrackingConcurrentM7) {
  if (axis.fault) {
    routeFault("EMM axis remains faulted");
    return false;
  }
  const LinearAxisMotion &otherAxis =
      axis.address == extensionAxis.address
          ? liftAxis
          : extensionAxis;
  // M6 and M7 normally remain mutually exclusive. The only exception is the
  // new RAW pickup tracker: while M7 is already descending, one addressed M6
  // correction may run on the shared bus. The normal status arbiter already
  // alternates active-axis queries; every other caller retains the old gate.
  const bool rawTrackingConcurrentM7Allowed =
      allowRawTrackingConcurrentM7 &&
      axis.address == extensionAxis.address &&
      otherAxis.address == liftAxis.address &&
      otherAxis.active &&
      !otherAxis.recoveryPending &&
      !otherAxis.fault;
  if (axis.active ||
      axis.recoveryPending ||
      ((!rawTrackingConcurrentM7Allowed) &&
       (otherAxis.active ||
        otherAxis.recoveryPending)) ||
      armLinearPositionQueryAxis != nullptr) {
    routeFault(
        "Concurrent M6/M7 command rejected on shared EMM serial");
    return false;
  }
  resetLinearAxisTerminalVerification(axis);

  axis.stallProtectionSamples = 0U;
  axis.positionCommandRejected = false;
  if (!recoveryRetry) {
    axis.recoveryAttemptCount = 0U;
    axis.recoveryDeadlineMs = 0UL;
  }
  axis.recoveryPending = false;
  axis.recoveryReason = ARM_AXIS_RECOVERY_NONE;
  if (targetMm < minimumMm - ARM_AXIS_POSITION_TOLERANCE_MM ||
      targetMm > maximumMm + ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("Arm target outside calibrated travel");
    return false;
  }

  if (targetMm < minimumMm) {
    targetMm = minimumMm;
  } else if (targetMm > maximumMm) {
    targetMm = maximumMm;
  }

  // Normal M7 work keeps the global experiment.  A caller may explicitly use
  // the requested-profile policy for a dedicated low-speed motion. M6 is
  // never transformed here.
  if (axis.address == liftAxis.address &&
      m7ProfilePolicy == M7_PROFILE_APPLY_EXPERIMENT) {
    speedRpm = m7_experiment::experimentalSpeedRpm(speedRpm);
    acceleration =
        m7_experiment::experimentalAcceleration(acceleration);
  }

  // Recovery retries reuse already-latched command values and must not apply
  // the boost twice. Final contact/zero soft-landing segments pass false so
  // their proven low-speed endpoint behavior is preserved.
  if (allowEmptyGripperBoost &&
      !recoveryRetry &&
      !armPayloadPresent) {
    speedRpm = emptyGripperBoostedSpeedRpm(speedRpm);
    acceleration =
        emptyGripperBoostedEmmAcceleration(acceleration);
  }

  const float deltaMm = targetMm - axis.currentMm;
  axis.targetMm = targetMm;
  axis.commandStartMm = axis.currentMm;
  axis.commandMinimumMm = minimumMm;
  axis.commandMaximumMm = maximumMm;
  axis.commandPulsesPerMm = pulsesPerMm;
  axis.commandPositiveDirection = positiveDirection;
  axis.commandNegativeDirection = negativeDirection;
  axis.commandSpeedRpm = speedRpm;
  axis.commandAcceleration = acceleration;
  axis.commandM7ProfilePolicy = m7ProfilePolicy;
  if (fabsf(deltaMm) <= ARM_AXIS_POSITION_TOLERANCE_MM) {
    axis.active = false;
    axis.currentMm = targetMm;
    return true;
  }

  const uint32_t pulses =
      static_cast<uint32_t>(
          lroundf(fabsf(deltaMm) * pulsesPerMm));
  if (pulses == 0U) {
    axis.active = false;
    axis.currentMm = targetMm;
    return true;
  }

  const float travelPerRevolutionMm =
      axis.address == extensionAxis.address
          ? M6_TRAVEL_PER_REVOLUTION_MM
          : M7_TRAVEL_PER_REVOLUTION_MM;

  const float pulsesPerSecond =
      static_cast<float>(speedRpm) *
      pulsesPerMm *
      travelPerRevolutionMm / 60.0f;
  uint32_t estimatedMs =
      static_cast<uint32_t>(
          static_cast<float>(pulses) /
          pulsesPerSecond * 3000.0f) +
      2000UL;
  if (estimatedMs < ARM_AXIS_MINIMUM_TIMEOUT_MS) {
    estimatedMs = ARM_AXIS_MINIMUM_TIMEOUT_MS;
  } else if (estimatedMs > ARM_AXIS_MAXIMUM_TIMEOUT_MS) {
    estimatedMs = ARM_AXIS_MAXIMUM_TIMEOUT_MS;
  }
  axis.timeoutMs = estimatedMs;

  clearArmLinearReceiveBuffer();
  writeArmLinearEnable(axis.address, true);
  SerialArmLinear.flush();

  const uint32_t commandGuardStartMs = millis();
  while (millis() - commandGuardStartMs <
         ARM_AXIS_COMMAND_GUARD_MS) {
    armMotors.serviceM5();
  }
  if (!consumeArmLinearEnableResponse(axis)) {
    return false;
  }
  clearArmLinearReceiveBuffer();

  axis.active = true;
  axis.commandAcknowledged = false;
  axis.motionObserved = false;
  axis.startMs = millis();
  axis.lastStatusRequestMs = axis.startMs;
  axis.lastStatusFlags = 0U;
  axis.lastStatusResponseMs = 0UL;
  axis.lastFastArrivalQueryMs = 0UL;

  writeArmLinearPosition(
      axis.address,
      deltaMm > 0.0f
          ? positiveDirection
          : negativeDirection,
      speedRpm,
      acceleration,
      pulses);
  SerialArmLinear.flush();

  SerialDebug.print("Arm axis M");
  SerialDebug.print(axis.address);
  SerialDebug.print(": ");
  SerialDebug.print(axis.currentMm, 2);
  SerialDebug.print(" -> ");
  SerialDebug.print(axis.targetMm, 2);
  SerialDebug.print(" mm, pulses=");
  SerialDebug.print(pulses);
  SerialDebug.print(", dir=");
  SerialDebug.print(
      deltaMm > 0.0f
          ? positiveDirection
          : negativeDirection);
  SerialDebug.print(", rpm=");
  SerialDebug.print(speedRpm);
  SerialDebug.print(", acc=");
  SerialDebug.println(acceleration);
  return true;
}

bool startExtensionToMm(float extensionMm) {
  return startLinearAxisMove(
      extensionAxis,
      extensionMm,
      M6_STANDARD_EXTENSION_MM,
      M6_MAXIMUM_EXTENSION_MM,
      M6_PULSES_PER_MM,
      M6_EXTEND_DIRECTION,
      M6_RETRACT_DIRECTION,
      M6_SPEED_RPM,
      M6_ACCELERATION);
}

bool startMappedRingExtensionToMm(float extensionMm) {

  return startLinearAxisMove(
      extensionAxis,
      extensionMm,
      M6_RING2_MINIMUM_EXTENSION_MM,
      M6_MAXIMUM_EXTENSION_MM,
      M6_PULSES_PER_MM,
      M6_EXTEND_DIRECTION,
      M6_RETRACT_DIRECTION,
      M6_SPEED_RPM,
      M6_ACCELERATION);
}

bool startExtensionToMmWithProfile(
    float extensionMm,
    uint16_t speedRpm,
    uint8_t acceleration) {
  return startLinearAxisMove(
      extensionAxis,
      extensionMm,
      M6_STANDARD_EXTENSION_MM,
      M6_MAXIMUM_EXTENSION_MM,
      M6_PULSES_PER_MM,
      M6_EXTEND_DIRECTION,
      M6_RETRACT_DIRECTION,
      speedRpm,
      acceleration);
}

bool startExtensionToContactMmWithProfile(
    float extensionMm,
    float minimumMm,
    uint16_t speedRpm,
    uint8_t acceleration) {
  const float deltaMm =
      extensionMm - extensionAxis.currentMm;
  const uint8_t finalAcceleration =
      acceleration <
              M6_CONTACT_SOFT_LANDING_ACCELERATION
          ? acceleration
          : M6_CONTACT_SOFT_LANDING_ACCELERATION;
  if (fabsf(deltaMm) <=
      M6_CONTACT_SOFT_LANDING_DISTANCE_MM +
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    SerialDebug.print(
        "[M6 CONTACT SOFT] direct short final, target/rpm/acc=");
    SerialDebug.print(extensionMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(speedRpm);
    SerialDebug.print("/");
    SerialDebug.println(finalAcceleration);
    return startLinearAxisMove(
        extensionAxis,
        extensionMm,
        minimumMm,
        M6_MAXIMUM_EXTENSION_MM,
        M6_PULSES_PER_MM,
        M6_EXTEND_DIRECTION,
        M6_RETRACT_DIRECTION,
        speedRpm,
        finalAcceleration,
        false,
        M7_PROFILE_APPLY_EXPERIMENT,
        false);
  }

  const float directionSign =
      deltaMm > 0.0f ? 1.0f : -1.0f;
  const float approachMm =
      extensionMm -
      directionSign *
          M6_CONTACT_SOFT_LANDING_DISTANCE_MM;
  m6ContactSoftLandingPending = true;
  m6ContactSoftLandingTargetMm = extensionMm;
  m6ContactSoftLandingMinimumMm = minimumMm;
  m6ContactSoftLandingSpeedRpm = speedRpm;
  m6ContactSoftLandingAcceleration =
      finalAcceleration;
  SerialDebug.print(
      "[M6 CONTACT SOFT] fast segment -> ");
  SerialDebug.print(approachMm, 2);
  SerialDebug.print(" mm; final ");
  SerialDebug.print(
      M6_CONTACT_SOFT_LANDING_DISTANCE_MM,
      1);
  SerialDebug.println(" mm low acceleration");
  if (startLinearAxisMove(
          extensionAxis,
          approachMm,
          minimumMm,
          M6_MAXIMUM_EXTENSION_MM,
          M6_PULSES_PER_MM,
          M6_EXTEND_DIRECTION,
          M6_RETRACT_DIRECTION,
          speedRpm,
          acceleration)) {
    return true;
  }
  m6ContactSoftLandingPending = false;
  return false;
}

bool startLiftMoveWithZeroSoftLanding(
    float heightMm,
    uint16_t speedRpm,
    uint8_t acceleration,
    M7ProfilePolicy m7ProfilePolicy =
        M7_PROFILE_APPLY_EXPERIMENT) {

  const bool returningToZero =
      fabsf(heightMm - M7_STANDARD_HEIGHT_MM) <=
          ARM_AXIS_POSITION_TOLERANCE_MM &&
      liftAxis.currentMm <
          M7_STANDARD_HEIGHT_MM -
              ARM_AXIS_POSITION_TOLERANCE_MM;
  if (!returningToZero) {
    return startLinearAxisMove(
        liftAxis,
        heightMm,
        M7_MINIMUM_HEIGHT_MM,
        M7_STANDARD_HEIGHT_MM,
        M7_PULSES_PER_MM,
        M7_RAISE_DIRECTION,
        M7_LOWER_DIRECTION,
        speedRpm,
        acceleration,
        false,
        m7ProfilePolicy);
  }

  const uint16_t finalSpeedRpm =
      speedRpm < M7_ZERO_SOFT_LANDING_SPEED_RPM
          ? speedRpm
          : M7_ZERO_SOFT_LANDING_SPEED_RPM;
  const uint8_t finalAcceleration =
      acceleration < M7_ZERO_SOFT_LANDING_ACCELERATION
          ? acceleration
          : M7_ZERO_SOFT_LANDING_ACCELERATION;
  const float softLandingStartMm =
      M7_STANDARD_HEIGHT_MM -
      M7_ZERO_SOFT_LANDING_DISTANCE_MM;
  if (liftAxis.currentMm <
      softLandingStartMm -
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    m7SoftLandingPending = true;
    m7SoftLandingIsContact = false;
    m7SoftLandingTargetMm = M7_STANDARD_HEIGHT_MM;
    m7SoftLandingDistanceMm =
        M7_ZERO_SOFT_LANDING_DISTANCE_MM;
    m7SoftLandingSpeedRpm = finalSpeedRpm;
    m7SoftLandingAcceleration =
        finalAcceleration;
    m7SoftLandingProfilePolicy = m7ProfilePolicy;
    SerialDebug.print(
        "[M7 SOFT ZERO] fast segment -> ");
    SerialDebug.print(softLandingStartMm, 1);
    SerialDebug.println(" mm");
    if (startLinearAxisMove(
            liftAxis,
            softLandingStartMm,
            M7_MINIMUM_HEIGHT_MM,
            M7_STANDARD_HEIGHT_MM,
            M7_PULSES_PER_MM,
            M7_RAISE_DIRECTION,
            M7_LOWER_DIRECTION,
            speedRpm,
            acceleration,
            false,
            m7ProfilePolicy)) {
      return true;
    }
    m7SoftLandingPending = false;
    m7SoftLandingProfilePolicy =
        M7_PROFILE_APPLY_EXPERIMENT;
    return false;
  }

  SerialDebug.print(
      "[M7 SOFT ZERO] short final segment, rpm/acc=");
  SerialDebug.print(finalSpeedRpm);
  SerialDebug.print("/");
  SerialDebug.println(finalAcceleration);
  return startLinearAxisMove(
      liftAxis,
      heightMm,
      M7_MINIMUM_HEIGHT_MM,
      M7_STANDARD_HEIGHT_MM,
      M7_PULSES_PER_MM,
      M7_RAISE_DIRECTION,
      M7_LOWER_DIRECTION,
      finalSpeedRpm,
      finalAcceleration,
      false,
      m7ProfilePolicy,
      false);
}

bool startLiftMoveWithContactSoftLanding(
    float heightMm,
    uint16_t speedRpm,
    uint8_t acceleration) {
  const float deltaMm =
      heightMm - liftAxis.currentMm;
  if (deltaMm >=
      -ARM_AXIS_POSITION_TOLERANCE_MM) {
    return startLiftMoveWithZeroSoftLanding(
        heightMm,
        speedRpm,
        acceleration);
  }

  const uint8_t finalAcceleration =
      acceleration <
              M7_CONTACT_SOFT_LANDING_ACCELERATION
          ? acceleration
          : M7_CONTACT_SOFT_LANDING_ACCELERATION;
  if (fabsf(deltaMm) <=
      M7_CONTACT_SOFT_LANDING_DISTANCE_MM +
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    SerialDebug.print(
        "[M7 CONTACT SOFT] direct short final, target/rpm/acc=");
    SerialDebug.print(heightMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(speedRpm);
    SerialDebug.print("/");
    SerialDebug.println(finalAcceleration);
    return startLinearAxisMove(
        liftAxis,
        heightMm,
        M7_MINIMUM_HEIGHT_MM,
        M7_STANDARD_HEIGHT_MM,
        M7_PULSES_PER_MM,
        M7_RAISE_DIRECTION,
        M7_LOWER_DIRECTION,
        speedRpm,
        finalAcceleration,
        false,
        M7_PROFILE_APPLY_EXPERIMENT,
        false);
  }

  const float approachHeightMm =
      heightMm +
      M7_CONTACT_SOFT_LANDING_DISTANCE_MM;
  m7SoftLandingPending = true;
  m7SoftLandingIsContact = true;
  m7SoftLandingTargetMm = heightMm;
  m7SoftLandingDistanceMm =
      M7_CONTACT_SOFT_LANDING_DISTANCE_MM;
  m7SoftLandingSpeedRpm = speedRpm;
  m7SoftLandingAcceleration =
      finalAcceleration;
  m7SoftLandingProfilePolicy =
      M7_PROFILE_APPLY_EXPERIMENT;
  SerialDebug.print(
      "[M7 CONTACT SOFT] fast segment -> ");
  SerialDebug.print(approachHeightMm, 2);
  SerialDebug.print(" mm; final ");
  SerialDebug.print(
      M7_CONTACT_SOFT_LANDING_DISTANCE_MM,
      1);
  SerialDebug.println(" mm low acceleration");
  if (startLinearAxisMove(
          liftAxis,
          approachHeightMm,
          M7_MINIMUM_HEIGHT_MM,
          M7_STANDARD_HEIGHT_MM,
          M7_PULSES_PER_MM,
          M7_RAISE_DIRECTION,
          M7_LOWER_DIRECTION,
          speedRpm,
          acceleration)) {
    return true;
  }
  m7SoftLandingPending = false;
  m7SoftLandingProfilePolicy =
      M7_PROFILE_APPLY_EXPERIMENT;
  return false;
}

bool startLiftToHeightMm(float heightMm) {

  return startLiftMoveWithZeroSoftLanding(
      heightMm,
      M7_SPEED_RPM,
      M7_ACCELERATION);
}

bool startLiftToHeightMmWithProfile(
    float heightMm,
    uint16_t speedRpm,
    uint8_t acceleration) {

  return startLiftMoveWithZeroSoftLanding(
      heightMm,
      speedRpm,
      acceleration);
}

bool startLiftToHeightMmWithUnboostedProfile(
    float heightMm,
    uint16_t speedRpm,
    uint8_t acceleration) {
  return startLiftMoveWithZeroSoftLanding(
      heightMm,
      speedRpm,
      acceleration,
      M7_PROFILE_USE_REQUESTED);
}

bool extensionMoveFinished() {
  return !extensionAxis.active &&
         !extensionAxis.fault &&
         !extensionAxis.recoveryPending &&
         !m6ContactSoftLandingPending;
}

bool liftMoveFinished() {
  return !liftAxis.active &&
         !liftAxis.fault &&
         !liftAxis.recoveryPending &&
         !m7SoftLandingPending;
}

void emergencyStopArmLinearAxes() {
  if (armLinearSerialInitialized) {
    writeArmLinearStop(extensionAxis.address);
    writeArmLinearStop(liftAxis.address);
  }
  extensionAxis.active = false;
  resetLinearAxisTerminalVerification(extensionAxis);
  extensionAxis.recoveryPending = false;
  extensionAxis.recoveryAttemptCount = 0U;
  extensionAxis.recoveryReason = ARM_AXIS_RECOVERY_NONE;
  extensionAxis.recoveryDeadlineMs = 0UL;
  liftAxis.active = false;
  resetLinearAxisTerminalVerification(liftAxis);
  liftAxis.recoveryPending = false;
  liftAxis.recoveryAttemptCount = 0U;
  liftAxis.recoveryReason = ARM_AXIS_RECOVERY_NONE;
  liftAxis.recoveryDeadlineMs = 0UL;
  m6ContactSoftLandingPending = false;
  m7SoftLandingPending = false;
  m7SoftLandingProfilePolicy =
      M7_PROFILE_APPLY_EXPERIMENT;
  armLinearReceiveCount = 0U;
  armLinearExpectedFrameLength = 0U;
}

void resetArmLinearSoftwareOrigin() {

  extensionAxis.currentMm = M6_STANDARD_EXTENSION_MM;
  extensionAxis.targetMm = M6_STANDARD_EXTENSION_MM;
  extensionAxis.active = false;
  extensionAxis.fault = false;
  extensionAxis.commandAcknowledged = false;
  extensionAxis.motionObserved = false;
  extensionAxis.positionCommandRejected = false;
  resetLinearAxisTerminalVerification(extensionAxis);
  extensionAxis.stallProtectionSamples = 0U;
  extensionAxis.recoveryPending = false;
  extensionAxis.recoveryAttemptCount = 0U;
  extensionAxis.recoveryReason = ARM_AXIS_RECOVERY_NONE;
  extensionAxis.recoveryReadyMs = 0UL;
  extensionAxis.recoveryDeadlineMs = 0UL;
  extensionAxis.driverWorkingZeroAngleDegrees = 0.0f;
  extensionAxis.driverWorkingZeroAngleValid = false;
  liftAxis.currentMm = M7_STANDARD_HEIGHT_MM;
  liftAxis.targetMm = M7_STANDARD_HEIGHT_MM;
  liftAxis.active = false;
  liftAxis.fault = false;
  liftAxis.commandAcknowledged = false;
  liftAxis.motionObserved = false;
  liftAxis.positionCommandRejected = false;
  resetLinearAxisTerminalVerification(liftAxis);
  liftAxis.stallProtectionSamples = 0U;
  liftAxis.recoveryPending = false;
  liftAxis.recoveryAttemptCount = 0U;
  liftAxis.recoveryReason = ARM_AXIS_RECOVERY_NONE;
  liftAxis.recoveryReadyMs = 0UL;
  liftAxis.recoveryDeadlineMs = 0UL;
  liftAxis.driverWorkingZeroAngleDegrees = 0.0f;
  liftAxis.driverWorkingZeroAngleValid = false;
  m6ContactSoftLandingPending = false;
  m6ContactSoftLandingTargetMm = M6_STANDARD_EXTENSION_MM;
  m6ContactSoftLandingMinimumMm = M6_STANDARD_EXTENSION_MM;
  m6ContactSoftLandingSpeedRpm = M6_SPEED_RPM;
  m6ContactSoftLandingAcceleration =
      M6_CONTACT_SOFT_LANDING_ACCELERATION;
  m7SoftLandingPending = false;
  m7SoftLandingIsContact = false;
  m7SoftLandingTargetMm = M7_STANDARD_HEIGHT_MM;
  m7SoftLandingDistanceMm =
      M7_ZERO_SOFT_LANDING_DISTANCE_MM;
  m7SoftLandingSpeedRpm = M7_SPEED_RPM;
  m7SoftLandingAcceleration =
      M7_ZERO_SOFT_LANDING_ACCELERATION;
  m7SoftLandingProfilePolicy =
      M7_PROFILE_APPLY_EXPERIMENT;
  armLinearReferenceValid = false;
  armLinearReceiveCount = 0U;
  armLinearExpectedFrameLength = 0U;
}

bool readArmLinearAngleWithRetry(
    uint8_t address,
    float &angleDegrees) {
  if (readArmLinearCurrentMotorAngleDegrees(
          address,
          angleDegrees)) {
    return true;
  }
  delay(ARM_AXIS_RECOVERY_SAMPLE_SETTLE_MS);
  return readArmLinearCurrentMotorAngleDegrees(
      address,
      angleDegrees);
}

bool waitForLinearAxisStartupMove(
    LinearAxisMotion &axis,
    const char *axisLabel,
    const char *stageLabel) {
  while (axis.active && !axis.fault) {
    serviceArmLinearAxes();
    delay(1);
  }
  if (!axis.fault &&
      programState != PROGRAM_FAULT) {
    return true;
  }

  SerialDebug.print("[");
  SerialDebug.print(axisLabel);
  SerialDebug.print(" ZERO] ");
  SerialDebug.print(stageLabel);
  SerialDebug.println(" move failed");
  return false;
}

bool validateLinearAxisStartupAngleDelta(
    const char *axisLabel,
    const char *stageLabel,
    float observedSignedAngleDegrees,
    float commandedDistanceMm,
    uint8_t commandedDirection,
    float travelPerRevolutionMm,
    float configuredMicrosteps) {
  const float expectedAngleMagnitudeDegrees =
      fabsf(commandedDistanceMm) /
      travelPerRevolutionMm * 360.0f;
  if (expectedAngleMagnitudeDegrees <= 0.0f) {
    return false;
  }
  const float expectedSignedAngleDegrees =
      commandedDirection == 0U
          ? expectedAngleMagnitudeDegrees
          : -expectedAngleMagnitudeDegrees;
  const float observedAngleRatio =
      fabsf(observedSignedAngleDegrees) /
      expectedAngleMagnitudeDegrees;
  const bool angleDirectionValid =
      observedSignedAngleDegrees *
          expectedSignedAngleDegrees >
      0.0f;

  SerialDebug.print("[");
  SerialDebug.print(axisLabel);
  SerialDebug.print(" ZERO] ");
  SerialDebug.print(stageLabel);
  SerialDebug.print(
      " observed/expected signed angle, ratio=");
  SerialDebug.print(observedSignedAngleDegrees, 3);
  SerialDebug.print("/");
  SerialDebug.print(expectedSignedAngleDegrees, 3);
  SerialDebug.print("/");
  SerialDebug.println(observedAngleRatio, 3);
  if (angleDirectionValid &&
      observedAngleRatio >=
          ARM_LINEAR_ZERO_ANGLE_RATIO_MINIMUM &&
      observedAngleRatio <=
          ARM_LINEAR_ZERO_ANGLE_RATIO_MAXIMUM) {
    return true;
  }

  SerialDebug.print("[");
  SerialDebug.print(axisLabel);
  SerialDebug.print(" ZERO] ");
  SerialDebug.print(stageLabel);
  SerialDebug.println(
      " calibration rejected: check physical endpoint, direction, "
      "microsteps and transmission geometry");
  if (observedAngleRatio > 0.05f &&
      isfinite(observedAngleRatio)) {
    const float inferredDriverMicrosteps =
        configuredMicrosteps / observedAngleRatio;
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.print(
        " ZERO] firmware/estimated-driver microsteps=");
    SerialDebug.print(configuredMicrosteps, 0);
    SerialDebug.print("/");
    SerialDebug.print(inferredDriverMicrosteps, 1);
    SerialDebug.println(
        " (change the real EMM setting; a code constant does not "
        "reprogram the driver)");
  }
  return false;
}

bool establishLinearAxisSafeWorkingZero(
    LinearAxisMotion &axis,
    const char *axisLabel,
    float startupTargetMm,
    float startupMinimumMm,
    float startupMaximumMm,
    float pulsesPerMm,
    uint8_t positiveDirection,
    uint8_t negativeDirection,
    float travelPerRevolutionMm) {
  axis.driverWorkingZeroAngleDegrees = 0.0f;
  axis.driverWorkingZeroAngleValid = false;
  SerialDebug.print("[");
  SerialDebug.print(axisLabel);
  SerialDebug.print(
      " ZERO] read driver angle, move from mechanical "
      "endpoint by ");
  SerialDebug.print(fabsf(startupTargetMm), 2);
  SerialDebug.println(" mm, then re-zero");

  float angleBeforeDegrees = 0.0f;
  bool angleBeforeValid =
      readArmLinearAngleWithRetry(
          axis.address,
          angleBeforeDegrees);
  if (angleBeforeValid) {
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.print(" ZERO] driver angle before move=");
    SerialDebug.print(angleBeforeDegrees, 3);
    SerialDebug.println(
        " deg (diagnostic only, not mechanical absolute)");
  } else {
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] driver angle read failed before move; "
        "refuse unverified startup");
    return false;
  }

  axis.driverWorkingZeroAngleDegrees =
      angleBeforeDegrees;
  axis.driverWorkingZeroAngleValid = true;

  clearArmLinearReceiveBuffer();
  writeArmLinearResetStallProtection(
      axis.address);
  SerialArmLinear.flush();
  bool stallResetAcknowledged =
      waitForArmLinearSimpleResponse(
          axis.address,
          0x0EU,
          ARM_LINEAR_POSITION_READ_TIMEOUT_MS);
  if (!stallResetAcknowledged) {
    clearArmLinearReceiveBuffer();
    delay(ARM_AXIS_COMMAND_GUARD_MS);
    writeArmLinearResetStallProtection(axis.address);
    SerialArmLinear.flush();
    stallResetAcknowledged =
        waitForArmLinearSimpleResponse(
            axis.address,
            0x0EU,
            ARM_LINEAR_POSITION_READ_TIMEOUT_MS);
  }
  if (!stallResetAcknowledged) {
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] stall-reset ACK missing twice; refuse to move");
    return false;
  }
  clearArmLinearReceiveBuffer();

  const uint8_t startupDirection =
      startupTargetMm > 0.0f
          ? positiveDirection
          : negativeDirection;
  const float probeDistanceMm =
      fminf(
          fabsf(startupTargetMm),
          ARM_LINEAR_STARTUP_PROBE_MM);
  const float probeTargetMm =
      startupTargetMm > 0.0f
          ? probeDistanceMm
          : -probeDistanceMm;
  SerialDebug.print("[");
  SerialDebug.print(axisLabel);
  SerialDebug.print(
      " ZERO] safe microstep probe target=");
  SerialDebug.print(probeTargetMm, 3);
  SerialDebug.println(" mm");

  if (!startLinearAxisMove(
          axis,
          probeTargetMm,
          startupMinimumMm,
          startupMaximumMm,
          pulsesPerMm,
          positiveDirection,
          negativeDirection,
          ARM_LINEAR_STARTUP_ZERO_SPEED_RPM,
          ARM_LINEAR_STARTUP_ZERO_ACCELERATION,
          false,
          M7_PROFILE_APPLY_EXPERIMENT,
          false) ||
      !waitForLinearAxisStartupMove(
          axis,
          axisLabel,
          "probe")) {
    return false;
  }

  float probeAngleDegrees = 0.0f;
  if (!readArmLinearAngleWithRetry(
          axis.address,
          probeAngleDegrees)) {
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] probe angle read failed; refuse full startup move");
    return false;
  }
  if (!validateLinearAxisStartupAngleDelta(
          axisLabel,
          "probe",
          probeAngleDegrees - angleBeforeDegrees,
          probeDistanceMm,
          startupDirection,
          travelPerRevolutionMm,
          axis.address == extensionAxis.address
              ? M6_MICROSTEPS
              : M7_MICROSTEPS)) {
    return false;
  }

  float angleAfterDegrees = probeAngleDegrees;
  if (fabsf(startupTargetMm - probeTargetMm) >
      ARM_AXIS_POSITION_TOLERANCE_MM) {
    if (!startLinearAxisMove(
            axis,
            startupTargetMm,
            startupMinimumMm,
            startupMaximumMm,
            pulsesPerMm,
            positiveDirection,
            negativeDirection,
            ARM_LINEAR_STARTUP_ZERO_SPEED_RPM,
            ARM_LINEAR_STARTUP_ZERO_ACCELERATION,
            false,
            M7_PROFILE_APPLY_EXPERIMENT,
            false) ||
        !waitForLinearAxisStartupMove(
            axis,
            axisLabel,
            "full-offset")) {
      return false;
    }
    if (!readArmLinearAngleWithRetry(
            axis.address,
            angleAfterDegrees)) {
      SerialDebug.print("[");
      SerialDebug.print(axisLabel);
      SerialDebug.println(
          " ZERO] full-offset angle read failed");
      return false;
    }
    if (!validateLinearAxisStartupAngleDelta(
            axisLabel,
            "full-offset",
            angleAfterDegrees - angleBeforeDegrees,
            fabsf(startupTargetMm),
            startupDirection,
            travelPerRevolutionMm,
            axis.address == extensionAxis.address
                ? M6_MICROSTEPS
                : M7_MICROSTEPS)) {
      return false;
    }
  }

  SerialDebug.print("[");
  SerialDebug.print(axisLabel);
  SerialDebug.print(" ZERO] driver angle after verified move=");
  SerialDebug.print(angleAfterDegrees, 3);
  SerialDebug.println(" deg");

  clearArmLinearReceiveBuffer();
  writeArmLinearResetCurrentPositionZero(
      axis.address);
  SerialArmLinear.flush();
  const bool driverZeroAcknowledged =
      waitForArmLinearSimpleResponse(
          axis.address,
          0x0AU,
          ARM_LINEAR_POSITION_READ_TIMEOUT_MS);
  if (!driverZeroAcknowledged) {

    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] driver-zero ACK missing; "
        "software working zero remains authoritative");
  }
  clearArmLinearReceiveBuffer();

  float workingZeroAngleDegrees = 0.0f;
  bool workingZeroAngleValid =
      readArmLinearCurrentMotorAngleDegrees(
          axis.address,
          workingZeroAngleDegrees);
  if (!workingZeroAngleValid) {
    delay(ARM_AXIS_RECOVERY_SAMPLE_SETTLE_MS);
    workingZeroAngleValid =
        readArmLinearCurrentMotorAngleDegrees(
            axis.address,
            workingZeroAngleDegrees);
  }
  if (!workingZeroAngleValid && driverZeroAcknowledged) {
    workingZeroAngleDegrees = 0.0f;
    workingZeroAngleValid = true;
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] angle read missed; accepted driver-zero ACK as 0 deg");
  }
  if (!workingZeroAngleValid) {
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] cannot establish recovery angle reference");
    return false;
  }
  axis.driverWorkingZeroAngleDegrees =
      workingZeroAngleDegrees;
  axis.driverWorkingZeroAngleValid = true;
  SerialDebug.print("[");
  SerialDebug.print(axisLabel);
  SerialDebug.print(
      " ZERO] recovery angle reference=");
  SerialDebug.print(workingZeroAngleDegrees, 3);
  SerialDebug.println(" deg");
  clearArmLinearReceiveBuffer();

  axis.currentMm = 0.0f;
  axis.targetMm = 0.0f;
  axis.active = false;
  axis.fault = false;
  axis.commandAcknowledged = false;
  axis.motionObserved = false;
  axis.positionCommandRejected = false;
  return true;
}

bool establishArmLinearSafeWorkingZeros() {
  armLinearReferenceValid = false;

  if (!establishLinearAxisSafeWorkingZero(
          extensionAxis,
          "M6",
          M6_STARTUP_WORKING_ZERO_OFFSET_MM,
          M6_STANDARD_EXTENSION_MM,
          M6_MAXIMUM_PHYSICAL_EXTENSION_MM,
          M6_PULSES_PER_MM,
          M6_EXTEND_DIRECTION,
          M6_RETRACT_DIRECTION,
          M6_TRAVEL_PER_REVOLUTION_MM)) {
    return false;
  }

  if (!establishLinearAxisSafeWorkingZero(
          liftAxis,
          "M7",
          -M7_STARTUP_WORKING_ZERO_OFFSET_MM,
          M7_MINIMUM_PHYSICAL_HEIGHT_MM,
          M7_STANDARD_HEIGHT_MM,
          M7_PULSES_PER_MM,
          M7_RAISE_DIRECTION,
          M7_LOWER_DIRECTION,
          M7_TRAVEL_PER_REVOLUTION_MM)) {
    return false;
  }

  armLinearReferenceValid = true;
  SerialDebug.print(
      "[M6/M7 ZERO] READY: offsets/usable M6/"
      "usable M7 descent=");
  SerialDebug.print(
      M6_STARTUP_WORKING_ZERO_OFFSET_MM,
      2);
  SerialDebug.print(",");
  SerialDebug.print(
      M7_STARTUP_WORKING_ZERO_OFFSET_MM,
      2);
  SerialDebug.print("/");
  SerialDebug.print(M6_MAXIMUM_EXTENSION_MM, 2);
  SerialDebug.print("/");
  SerialDebug.println(-M7_MINIMUM_HEIGHT_MM, 2);
  return true;
}

void invalidateArmLinearReference() {
  armLinearReferenceValid = false;
}

void commandGripperOpen() {
  gripperServo.setRawAngle(
      GRIPPER_OPEN_ANGLE_DEGREES,
      GRIPPER_INTERVAL_MS,
      GRIPPER_OPEN_POWER_MW);
}

void commandRawGripperOpen() {
  gripperServo.setRawAngle(
      RAW_GRIPPER_OPEN_ANGLE_DEGREES,
      GRIPPER_INTERVAL_MS,
      GRIPPER_OPEN_POWER_MW);
}

void commandGripperMaxOpen() {
  gripperServo.setRawAngle(
      GRIPPER_MAX_OPEN_ANGLE_DEGREES,
      GRIPPER_INTERVAL_MS,
      GRIPPER_OPEN_POWER_MW);
}

void commandGripperClose() {
  gripperServo.setRawAngle(
      GRIPPER_CLOSE_ANGLE_DEGREES,
      GRIPPER_INTERVAL_MS,
      GRIPPER_CLOSE_POWER_MW);
}

void commandGripperTargetPlaceOpen() {
  gripperServo.setRawAngle(
      GRIPPER_OPEN_ANGLE_DEGREES,
      GRIPPER_TARGET_PLACE_OPEN_INTERVAL_MS,
      GRIPPER_OPEN_POWER_MW);
}

void commandGripperDoubleSpeedOpen() {
  gripperServo.setRawAngle(
      GRIPPER_OPEN_ANGLE_DEGREES,
      GRIPPER_DOUBLE_SPEED_INTERVAL_MS,
      GRIPPER_OPEN_POWER_MW);
}

void commandGripperDoubleSpeedClose() {
  gripperServo.setRawAngle(
      GRIPPER_CLOSE_ANGLE_DEGREES,
      GRIPPER_DOUBLE_SPEED_INTERVAL_MS,
      GRIPPER_CLOSE_POWER_MW);
}

void commandStorageServoPosition(
    uint8_t positionIndex,
    uint16_t intervalMs = STORAGE_SERVO_INTERVAL_MS) {
  if (positionIndex > 3U) {
    routeFault("Invalid storage servo position");
    return;
  }

  storageServo.setRawAngleMTurn(
      STORAGE_SERVO_POSITIONS_DEGREES[positionIndex],
      intervalMs);
  SerialDebug.print("Storage servo multi-turn index ");
  SerialDebug.print(positionIndex);
  SerialDebug.print(", angle=");
  SerialDebug.println(
      STORAGE_SERVO_POSITIONS_DEGREES[positionIndex],
      1);
}

void resetStorageServoArrivalGate() {
  storageServoArrivalGate = StorageServoArrivalGate();
}

void armStorageServoArrivalGateProfile(
    uint8_t positionIndex,
    uint32_t commandMs,
    uint16_t commandIntervalMs,
    uint32_t firstQueryDelayMs) {
  storageServoArrivalGate.active = true;
  storageServoArrivalGate.confirmed = false;
  storageServoArrivalGate.slot = positionIndex;
  storageServoArrivalGate.consecutiveConfirmations = 0U;
  storageServoArrivalGate.retryCount = 0U;
  storageServoArrivalGate.commandStartMs = commandMs;
  storageServoArrivalGate.commandIntervalMs = commandIntervalMs;
  storageServoArrivalGate.firstQueryDelayMs = firstQueryDelayMs;
  storageServoArrivalGate.nextQueryMs =
      commandMs + firstQueryDelayMs;
}

void commandStorageServoPositionWithArrivalGateProfile(
    uint8_t positionIndex,
    uint16_t commandIntervalMs,
    uint32_t firstQueryDelayMs) {
  if (positionIndex > 2U) {
    routeFault("Invalid verified storage servo position");
    return;
  }

  commandStorageServoPosition(
      positionIndex,
      commandIntervalMs);
  if (programState == PROGRAM_FAULT) {
    return;
  }

  const uint32_t commandMs = millis();
  armStorageServoArrivalGateProfile(
      positionIndex,
      commandMs,
      commandIntervalMs,
      firstQueryDelayMs);
}

void commandStorageServoPositionWithArrivalGate(
    uint8_t positionIndex) {
  commandStorageServoPositionWithArrivalGateProfile(
      positionIndex,
      STORAGE_SERVO_INTERVAL_MS,
      STORAGE_SERVO_VERIFY_FIRST_QUERY_MS);
}

void commandRawStorageServoPositionWithArrivalGate(
    uint8_t positionIndex) {
  commandStorageServoPositionWithArrivalGateProfile(
      positionIndex,
      RAW_STORAGE_SERVO_INTERVAL_MS,
      RAW_STORAGE_SERVO_VERIFY_FIRST_QUERY_MS);
}

bool serviceStorageServoArrivalGate() {
  if (!storageServoArrivalGate.active) {
    routeFault("Storage pickup missing real arrival gate");
    return false;
  }
  if (storageServoArrivalGate.confirmed) {
    return true;
  }

  if (!deadlineReached(storageServoArrivalGate.nextQueryMs)) {
    return false;
  }

  // This SDK query can block for up to 100 ms on a broken servo link.  Every
  // caller therefore invokes this service only after M5/M6/M7 and the chassis
  // are stationary, so a failed response cannot starve software step pulses.
  const float actualAngleDegrees =
      storageServo.queryRawAngleMTurn();
  const uint32_t queryCompleteMs = millis();
  const uint8_t responseStatus =
      servoProtocol.responsePack.recv_status;
  const uint8_t responseCommand =
      servoProtocol.responsePack.cmdId;
  const uint8_t responseSize =
      servoProtocol.responsePack.content_size;
  const int16_t responseServoId =
      responseSize > 0U
          ? static_cast<int16_t>(
                servoProtocol.responsePack.content[0])
          : static_cast<int16_t>(-1);
  // The bundled FashionStar SDK deliberately decodes the angle for both
  // SUCCESS and CHECKSUM_ERROR (see recvQueryAngleMTurn).  Match that exact
  // contract here while still requiring the expected command, payload shape,
  // and servo ID so a stale or unrelated packet cannot release the pickup.
  const bool validResponse =
      (responseStatus == FSUS_STATUS_SUCCESS ||
       responseStatus == FSUS_STATUS_CHECKSUM_ERROR) &&
      responseCommand == FSUS_CMD_QUERY_ANGLE_MTURN &&
      responseSize >= 5U &&
      responseServoId == STORAGE_SERVO_ID;
  const float targetAngleDegrees =
      STORAGE_SERVO_POSITIONS_DEGREES[
          storageServoArrivalGate.slot];
  const float errorDegrees =
      fabsf(actualAngleDegrees - targetAngleDegrees);
  const bool inPosition =
      validResponse &&
      errorDegrees <= STORAGE_SERVO_VERIFY_TOLERANCE_DEGREES;

  const uint32_t observationElapsedMs =
      queryCompleteMs - storageServoArrivalGate.commandStartMs;

  if (inPosition) {
    if (storageServoArrivalGate.consecutiveConfirmations < 255U) {
      ++storageServoArrivalGate.consecutiveConfirmations;
    }
    if (storageServoArrivalGate.consecutiveConfirmations >=
        STORAGE_SERVO_VERIFY_CONFIRMATIONS) {
      storageServoArrivalGate.confirmed = true;
      SerialDebug.print(
          "[STORAGE SERVO VERIFY] REAL ARRIVAL confirmed slot=");
      SerialDebug.println(storageServoArrivalGate.slot);
      return true;
    }
    // Always obtain the required second real sample.  In particular, do not
    // let the old 510 ms retry boundary interrupt a servo that has just entered
    // the arrival band.
    storageServoArrivalGate.nextQueryMs =
        queryCompleteMs + STORAGE_SERVO_VERIFY_QUERY_INTERVAL_MS;
    return false;
  } else {
    storageServoArrivalGate.consecutiveConfirmations = 0U;
  }

  if (observationElapsedMs >=
      STORAGE_SERVO_VERIFY_MAX_OBSERVE_MS) {
    if (storageServoArrivalGate.retryCount >=
        STORAGE_SERVO_VERIFY_MAX_RETRIES) {
      routeFault("Storage servo failed real arrival after retry");
      return false;
    }

    ++storageServoArrivalGate.retryCount;
    const uint8_t retrySlot = storageServoArrivalGate.slot;
    SerialDebug.print(
        "[STORAGE SERVO VERIFY] not arrived; re-drive same slot retry=");
    SerialDebug.print(storageServoArrivalGate.retryCount);
    SerialDebug.print("/");
    SerialDebug.print(STORAGE_SERVO_VERIFY_MAX_RETRIES);
    SerialDebug.println(", reason=800-ms-limit");
    commandStorageServoPosition(
        retrySlot,
        storageServoArrivalGate.commandIntervalMs);
    if (programState == PROGRAM_FAULT) {
      return false;
    }
    storageServoArrivalGate.consecutiveConfirmations = 0U;
    storageServoArrivalGate.commandStartMs = millis();
    storageServoArrivalGate.nextQueryMs =
        storageServoArrivalGate.commandStartMs +
        storageServoArrivalGate.firstQueryDelayMs;
    return false;
  }

  storageServoArrivalGate.nextQueryMs =
      queryCompleteMs + STORAGE_SERVO_VERIFY_QUERY_INTERVAL_MS;
  return false;
}

void commandStorageServoParkingPosition(
    uint16_t intervalMs = STORAGE_SERVO_INTERVAL_MS) {
  resetStorageServoArrivalGate();
  storageServo.setRawAngleMTurn(
      STORAGE_SERVO_PARK_ANGLE_DEGREES,
      intervalMs);
  SerialDebug.print("Storage servo parking angle/interval-ms=");
  SerialDebug.print(
      STORAGE_SERVO_PARK_ANGLE_DEGREES,
      1);
  SerialDebug.print("/");
  SerialDebug.println(intervalMs);
}

void commandRing1TrayClearancePosition() {
  resetStorageServoArrivalGate();
  storageServo.setRawAngleMTurn(
      RING1_TRAY_CLEARANCE_ANGLE_DEGREES,
      STORAGE_SERVO_INTERVAL_MS);
  SerialDebug.print("[R1 CLEAR] target=");
  SerialDebug.println(
      RING1_TRAY_CLEARANCE_ANGLE_DEGREES,
      1);
}

bool verifyManipulationServosOnline() {

  const bool gripperOnline = gripperServo.ping();
  const bool storageOnline = storageServo.ping();
  manipulationServosOnline =
      gripperOnline && storageOnline;
  SerialDebug.print("[SERVO PING] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, gripper(ID4)/storage(ID5)=");
  SerialDebug.print(gripperOnline ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.println(storageOnline ? 1 : 0);
  return manipulationServosOnline;
}

gongchuang::MaixCamClient maixCam(
    SerialMaixcam,
    SerialDebug,
    routeFault);

void stopMaixRequest() {
  maixCam.stopRequest();
}

void beginMaixRequest(uint8_t request) {
  maixCam.beginRequest(request);
}

void serviceMaixcam() {
  maixCam.service();
}

bool readNewMaixCoordinate(
    uint32_t &lastSequence,
    uint8_t &targetId,
    int16_t &x,
    int16_t &y) {
  return maixCam.readNewCoordinate(
      lastSequence,
      targetId,
      x,
      y);
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

bool updateHeadingLock(
    uint32_t timeoutMs,
    uint32_t postMotionSettleTimeMs =
        IMU_POST_MOTION_SETTLE_TIME_MS,
    uint32_t stableTimeOverrideMs = 0UL) {
  if (!imuIsFresh()) {
    routeFault("IMU data timeout");
    return false;
  }

  if (LOCAL_WORKSTATION_SIMULATION_MODE) {

    (void)timeoutMs;
    stopAllMotorsImmediately();
    headingStableStartMs = 0UL;
    motorsArrivedStartMs = 0UL;
    return true;
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
      postMotionSettleTimeMs) {
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
      stableTimeOverrideMs != 0UL
          ? stableTimeOverrideMs
          : (preciseMotionEnabled
                 ? FINAL_HEADING_STABLE_TIME_MS
                 : (turnMotionEnabled
                        ? TURN_HEADING_STABLE_TIME_MS
                        : (workstationApproachEnabled
                               ? WORKSTATION_HEADING_STABLE_TIME_MS
                               : TRANSLATION_HEADING_STABLE_TIME_MS)));

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

  if (turnMotionEnabled &&
      activeTurnCorrectionCount >= MAXIMUM_TURN_CORRECTIONS) {
    routeFault("Turn correction limit exceeded");
    return false;
  }

  startHeadingCorrection(error);
  return false;
}

struct PlanarPoint {
  float outwardMm;
  float leftMm;

  PlanarPoint(
      float outward = 0.0f,
      float left = 0.0f)
      : outwardMm(outward),
        leftMm(left) {}
};

ArmPose measuredRingPoses[4];
PlanarPoint measuredRingPoints[4];
bool measuredRingPointValid[4] = {
    false, false, false, false};
bool measuredRingPoseValid[4] = {
    false, false, false, false};
uint16_t measuredRingRadiiPixels[4] = {
    0U, 0U, 0U, 0U};
uint16_t measuredRingConfidence[4] = {
    0U, 0U, 0U, 0U};
float measuredRingHeadingsDegrees[4] = {
    0.0f, 0.0f, 0.0f, 0.0f};
float endpointMapLockedHeadingDegrees = 0.0f;

enum ArmStandardPhase {
  ARM_STANDARD_IDLE,
  ARM_STANDARD_WAIT_PRE_OPEN_GAP,
  ARM_STANDARD_WAIT_OPEN,
  ARM_STANDARD_WAIT_RAW_PARALLEL,
  ARM_STANDARD_WAIT_LIFT,
  ARM_STANDARD_WAIT_RETRACT,
  ARM_STANDARD_WAIT_BASE,
  ARM_STANDARD_WAIT_ENDPOINT_PARALLEL,
  ARM_STANDARD_WAIT_BASE_SETTLE,
  ARM_STANDARD_COMPLETE
};

ArmStandardPhase armStandardPhase = ARM_STANDARD_IDLE;
uint32_t armStandardDeadlineMs = 0UL;
uint32_t armStandardHandoffDeadlineUs = 0UL;
float armStandardBaseTargetDegrees = 0.0f;
float armStandardExtensionTargetMm =
    M6_STANDARD_EXTENSION_MM;
bool armStandardEndpointPreparation = false;
bool armStandardKeepBaseAngle = false;
bool armStandardRawViewPreparation = false;
bool armStandardEndpointExtensionPrepositioned = false;
bool armGripperLiftIsolationEnabled = false;
bool rawProcessPickupM7RaiseAdditionalDelayEnabled = false;

void commandArmStandardizationGripperOpen() {
  if (armStandardRawViewPreparation) {
    commandRawGripperOpen();
    return;
  }
  commandGripperOpen();
}

void beginArmStandardization(
    float baseTargetDegrees = 0.0f,
    bool keepBaseAngle = false,
    float extensionTargetMm =
        M6_STANDARD_EXTENSION_MM) {
  if (extensionTargetMm <
          M6_STANDARD_EXTENSION_MM -
              ARM_AXIS_POSITION_TOLERANCE_MM ||
      extensionTargetMm >
          M6_MAXIMUM_EXTENSION_MM +
              ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("Arm standard extension outside M6 travel");
    return;
  }
  const bool rawViewPreparation =
      fabsf(extensionTargetMm -
            RAW_VIEW_EXTENSION_MM) <=
      ARM_AXIS_POSITION_TOLERANCE_MM;
  if (rawViewPreparation) {
    useArmBaseRawTransferMotionProfile();
  } else {
    useArmBaseTransferMotionProfile();
  }
  armStandardEndpointPreparation = false;
  armStandardKeepBaseAngle = keepBaseAngle;
  armStandardRawViewPreparation = rawViewPreparation;
  armStandardEndpointExtensionPrepositioned = false;
  armStandardBaseTargetDegrees = baseTargetDegrees;
  armStandardExtensionTargetMm = extensionTargetMm;
  armStandardHandoffDeadlineUs = 0UL;
  SerialDebug.print("[ARM STD] t=");
  SerialDebug.print(millis());
  SerialDebug.print(
      " ms, start: open gripper; M6 current=");
  SerialDebug.print(extensionAxis.currentMm, 2);
  SerialDebug.print(" mm, M7 current=");
  SerialDebug.print(liftAxis.currentMm, 2);
  if (armStandardKeepBaseAngle) {
    SerialDebug.println(
        " mm, M5 holds current angle (no zero waypoint)");
  } else {
    SerialDebug.print(" mm, M5 direct target=");
    SerialDebug.println(
        armStandardBaseTargetDegrees,
        2);
  }
  if (armGripperLiftIsolationEnabled) {
    armStandardDeadlineMs = 0UL;
    armStandardPhase = ARM_STANDARD_WAIT_PRE_OPEN_GAP;
    SerialDebug.println(
        "[WORK INTERLOCK] standardization waits for M7 short handoff "
        "before gripper open");
  } else {
    commandArmStandardizationGripperOpen();
    armStandardDeadlineMs =
        millis() + GRIPPER_OPEN_SETTLE_MS;
    armStandardPhase = ARM_STANDARD_WAIT_OPEN;
  }
}

bool rawParallelPreparationSafe() {
  const bool extensionAtSafeStart =
      fabsf(
          extensionAxis.currentMm -
          M6_STANDARD_EXTENSION_MM) <=
          ARM_AXIS_POSITION_TOLERANCE_MM ||
      fabsf(
          extensionAxis.currentMm -
          RAW_VIEW_EXTENSION_MM) <=
          ARM_AXIS_POSITION_TOLERANCE_MM;
  return liftMoveFinished() &&
         extensionMoveFinished() &&
         armLinearPositionQueryAxis == nullptr &&
         !armBaseMotionLockedUntilReset &&
         !armMotors.isM5Running() &&
         !armBaseMotionWatchdogActive &&
         fabsf(
             liftAxis.currentMm -
             M7_STANDARD_HEIGHT_MM) <=
             ARM_AXIS_POSITION_TOLERANCE_MM &&
         extensionAtSafeStart;
}

void beginRawParallelPreparation() {
  useArmBaseRawTransferMotionProfile();
  armStandardEndpointPreparation = false;
  armStandardKeepBaseAngle = false;
  armStandardRawViewPreparation = true;
  armStandardEndpointExtensionPrepositioned = false;
  armStandardBaseTargetDegrees = 0.0f;
  armStandardExtensionTargetMm =
      RAW_VIEW_EXTENSION_MM;
  armStandardHandoffDeadlineUs = 0UL;

  commandArmStandardizationGripperOpen();
  armStandardDeadlineMs =
      millis() + GRIPPER_OPEN_SETTLE_MS;
  if (!startExtensionToMmWithProfile(
          RAW_VIEW_EXTENSION_MM,
          RAW_M6_SPEED_RPM,
          RAW_M6_ACCELERATION)) {
    return;
  }
  // M6 serial command is now on the wire; start M5 in this same scheduler pass.
  startArmBaseStandardFrameDegrees(0.0f);
  armStandardPhase = ARM_STANDARD_WAIT_RAW_PARALLEL;
}

bool startInitialStorageServoHeadStart();

void beginArmEndpointPreparation(
    bool extensionPrepositioned = false) {
  useArmBaseEndpointTravelMotionProfile();
  armStandardEndpointPreparation = true;
  armStandardKeepBaseAngle = false;
  armStandardRawViewPreparation = false;
  armStandardEndpointExtensionPrepositioned =
      extensionPrepositioned;
  armStandardExtensionTargetMm = extensionPrepositioned
                                     ? ENDPOINT_SEARCH_SEED_EXTENSION_MM
                                     : M6_STANDARD_EXTENSION_MM;
  armStandardBaseTargetDegrees =
      ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES;
  armStandardHandoffDeadlineUs = 0UL;
  SerialDebug.print("[ARM ENDPOINT PREP] t=");
  SerialDebug.print(millis());
  SerialDebug.print(
      " ms, open gripper; parallel targets M7/M5=");
  SerialDebug.print(ENDPOINT_VISION_HEIGHT_MM, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      armStandardBaseTargetDegrees,
      2);
  if (armGripperLiftIsolationEnabled) {
    armStandardDeadlineMs = 0UL;
    armStandardPhase = ARM_STANDARD_WAIT_PRE_OPEN_GAP;
    SerialDebug.println(
        "[WORK INTERLOCK] endpoint preparation waits for M7 short "
        "handoff before gripper open");
  } else {
    if (!startInitialStorageServoHeadStart()) {
      return;
    }
    commandGripperOpen();
    armStandardHandoffDeadlineUs =
        micros() +
        static_cast<uint32_t>(GRIPPER_OPEN_SETTLE_MS) * 1000UL +
        ENDPOINT_GRIPPER_M7_HANDOFF_US;
    armStandardPhase = ARM_STANDARD_WAIT_OPEN;
  }
}

bool serviceArmStandardization() {
  switch (armStandardPhase) {
    case ARM_STANDARD_IDLE:
      return false;

    case ARM_STANDARD_WAIT_PRE_OPEN_GAP:
      if (!liftMoveFinished()) {
        break;
      }
      if (armStandardEndpointPreparation) {
        if (armStandardHandoffDeadlineUs == 0UL) {
          armStandardHandoffDeadlineUs =
              micros() + ENDPOINT_GRIPPER_M7_HANDOFF_US;
          break;
        }
        if (static_cast<int32_t>(
                micros() - armStandardHandoffDeadlineUs) < 0) {
          break;
        }
        if (!startInitialStorageServoHeadStart()) {
          break;
        }
        commandArmStandardizationGripperOpen();
        armStandardHandoffDeadlineUs =
            micros() +
            static_cast<uint32_t>(GRIPPER_OPEN_SETTLE_MS) * 1000UL +
            ENDPOINT_GRIPPER_M7_HANDOFF_US;
        armStandardPhase = ARM_STANDARD_WAIT_OPEN;
        SerialDebug.println(
            "[ENDPOINT INTERLOCK] M7 handoff 0.1 ms -> gripper open; "
            "post-open handoff 0.1 ms");
        break;
      }
      if (armStandardDeadlineMs == 0UL) {
        armStandardDeadlineMs =
            millis() + WORK_M7_TO_GRIPPER_GAP_MS;
        break;
      }
      if (deadlineReached(armStandardDeadlineMs)) {
        commandArmStandardizationGripperOpen();
        armStandardDeadlineMs =
            millis() + GRIPPER_OPEN_SETTLE_MS +
            WORK_GRIPPER_TO_M7_GAP_MS;
        armStandardPhase = ARM_STANDARD_WAIT_OPEN;
        SerialDebug.println(
            "[WORK INTERLOCK] M7 handoff -> gripper open; "
            "M7 remains blocked through open completion + short handoff");
      }
      break;

    case ARM_STANDARD_WAIT_OPEN:
      if (armStandardEndpointPreparation
              ? (armStandardHandoffDeadlineUs != 0UL &&
                 static_cast<int32_t>(
                     micros() - armStandardHandoffDeadlineUs) >= 0)
              : deadlineReached(armStandardDeadlineMs)) {
        armStandardHandoffDeadlineUs = 0UL;
        if (armStandardEndpointPreparation) {
          const float requiredExtensionMm =
              armStandardEndpointExtensionPrepositioned
                  ? ENDPOINT_SEARCH_SEED_EXTENSION_MM
                  : M6_STANDARD_EXTENSION_MM;
          if (extensionAxis.active ||
              extensionAxis.recoveryPending ||
              extensionAxis.fault ||
              fabsf(
                  extensionAxis.currentMm -
                  requiredExtensionMm) >
                  ARM_AXIS_POSITION_TOLERANCE_MM) {
            routeFault(
                "Endpoint preparation M6 pose is invalid");
            break;
          }

          if (!startLiftToHeightMm(
                  ENDPOINT_VISION_HEIGHT_MM)) {
            break;
          }
          startArmBaseStandardFrameDegrees(
              armStandardBaseTargetDegrees);
          SerialDebug.print(
              "[ARM ENDPOINT PREP] original-time M7/M5 started at t=");
          SerialDebug.println(millis());
          armStandardPhase =
              ARM_STANDARD_WAIT_ENDPOINT_PARALLEL;
          break;
        }
        SerialDebug.print("[ARM STD] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, gripper settled -> M7 safe zero");
        startLiftToHeightMm(M7_STANDARD_HEIGHT_MM);
        armStandardPhase = ARM_STANDARD_WAIT_LIFT;
      }
      break;

    case ARM_STANDARD_WAIT_RAW_PARALLEL:
      if (deadlineReached(armStandardDeadlineMs) &&
          extensionMoveFinished() &&
          liftMoveFinished() &&
          !armMotors.isM5Running() &&
          !armBaseMotionWatchdogActive &&
          fabsf(
              liftAxis.currentMm -
              M7_STANDARD_HEIGHT_MM) <=
              ARM_AXIS_POSITION_TOLERANCE_MM) {
        armStandardPhase = ARM_STANDARD_COMPLETE;
      }
      break;

    case ARM_STANDARD_WAIT_LIFT:
      if (liftMoveFinished()) {
        SerialDebug.print("[ARM STD] t=");
        SerialDebug.print(millis());
        SerialDebug.print(
            " ms, M7 complete -> M6 target ");
        SerialDebug.print(
            armStandardExtensionTargetMm,
            2);
        SerialDebug.println(" mm");
        const bool rawViewTarget =
            fabsf(
                armStandardExtensionTargetMm -
                RAW_VIEW_EXTENSION_MM) <=
            ARM_AXIS_POSITION_TOLERANCE_MM;
        if (rawViewTarget) {
          startExtensionToMmWithProfile(
              armStandardExtensionTargetMm,
              RAW_M6_SPEED_RPM,
              RAW_M6_ACCELERATION);
        } else {
          startExtensionToMm(
              armStandardExtensionTargetMm);
        }
        armStandardPhase = ARM_STANDARD_WAIT_RETRACT;
      }
      break;

    case ARM_STANDARD_WAIT_RETRACT:
      if (extensionMoveFinished()) {
        if (armStandardKeepBaseAngle) {
          if (armMotors.isM5Running()) {
            break;
          }
          SerialDebug.print("[ARM SAFE] t=");
          SerialDebug.print(millis());
          SerialDebug.println(
              " ms, M6/M7 safe; keep M5 at current angle");
          armStandardPhase = ARM_STANDARD_COMPLETE;
          break;
        }
        SerialDebug.print("[ARM STD] t=");
        SerialDebug.print(millis());
        SerialDebug.print(
            " ms, M6 complete -> M5 direct target ");
        SerialDebug.print(
            armStandardBaseTargetDegrees,
            2);
        SerialDebug.println(" deg");
        startArmBaseStandardFrameDegrees(
            armStandardBaseTargetDegrees);
        armStandardPhase = ARM_STANDARD_WAIT_BASE;
      }
      break;

    case ARM_STANDARD_WAIT_BASE:
      if (!armMotors.isM5Running()) {
        SerialDebug.print("[ARM STD] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, M5 pulse target complete -> settle");
        armStandardDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        armStandardPhase =
            ARM_STANDARD_WAIT_BASE_SETTLE;
      }
      break;

    case ARM_STANDARD_WAIT_ENDPOINT_PARALLEL:
      if (liftMoveFinished() &&
          !armMotors.isM5Running()) {
        SerialDebug.print("[ARM ENDPOINT PREP] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, fixed endpoint height and ring-1 angle ready -> settle");
        armStandardDeadlineMs =
            millis() + FIRST_ENDPOINT_M7_SETTLE_MS;
        armStandardPhase =
            ARM_STANDARD_WAIT_BASE_SETTLE;
      }
      break;

    case ARM_STANDARD_WAIT_BASE_SETTLE:
      if (deadlineReached(armStandardDeadlineMs)) {
        SerialDebug.print("[ARM STD] t=");
        SerialDebug.print(millis());
        SerialDebug.println(" ms, COMPLETE");
        armStandardPhase = ARM_STANDARD_COMPLETE;
      }
      break;

    case ARM_STANDARD_COMPLETE:
      return true;
  }
  return armStandardPhase == ARM_STANDARD_COMPLETE;
}

enum ArmTransferPhase {
  ARM_TRANSFER_IDLE,
  ARM_TRANSFER_WAIT_PRE_SOURCE_OPEN_GAP,
  ARM_TRANSFER_WAIT_SOURCE_OPEN,
  ARM_TRANSFER_WAIT_PREPARE_CLEARANCE,
  ARM_TRANSFER_WAIT_PREPARE_LIFT,
  ARM_TRANSFER_WAIT_SOURCE_ROTATION,
  ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE,
  ARM_TRANSFER_WAIT_SOURCE_LOWER,
  ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP,
  ARM_TRANSFER_WAIT_GRIP_CLOSE,
  ARM_TRANSFER_WAIT_LOADED_CLEARANCE,
  ARM_TRANSFER_WAIT_DESTINATION_ROTATION,
  ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE,
  ARM_TRANSFER_WAIT_DESTINATION_APPROACH,
  ARM_TRANSFER_WAIT_DESTINATION_LOWER,
  ARM_TRANSFER_WAIT_DESTINATION_GRIPPER_GAP,
  ARM_TRANSFER_WAIT_DESTINATION_OPEN,
  ARM_TRANSFER_WAIT_FINAL_CLEARANCE,
  ARM_TRANSFER_WAIT_FINAL_RETRACT,
  ARM_TRANSFER_WAIT_STANDARD_SETTLE,
  ARM_TRANSFER_COMPLETE
};

ArmTransferPhase armTransferPhase = ARM_TRANSFER_IDLE;
ArmPose armTransferSourcePose;
ArmPose armTransferDestinationPose;
uint32_t armTransferDeadlineMs = 0UL;
// Both RAW visits may recognize and pick while the initial tray slot moves.
// The gate is still consumed at the stationary destination boundary before
// M7 descends.
bool rawInitialStorageArrivalOverlapPending = false;
uint32_t armTransferM7HandoffStartMs = 0UL;
uint32_t armTransferRawM7ToGripperDeadlineUs = 0UL;
bool armTransferGentleDestinationRelease = false;
bool armTransferMapSource = false;
bool armTransferMapDestination = false;
bool armTransferConcurrentSourcePreparation = false;
bool armTransferSourceAlreadyPrepared = false;
bool armTransferPrepareNextSource = false;
ArmPose armTransferNextSourcePose;
bool armTransferNextSourceMapped = false;
int8_t armTransferNextStorageSlot = -1;
uint32_t armTransferNextStorageDeadlineMs = 0UL;
uint32_t armTransferStorageCommandDueMs = 0UL;
bool armTransferNextStorageCommanded = false;
bool armTransferNextSourcePreparedAtEnd = false;
bool armTransferStorage2Batch = false;
bool armTransferReturnToRawViewAtEnd = false;
bool armTransferSourceRotationStarted = false;
bool armTransferDestinationRotationStarted = false;
bool armTransferFinalRotationStarted = false;
// Latched per transfer so the first PROCESS ring-return timing remains valid
// even after the surrounding work-item state advances or is reset.
bool armTransferProcessFirstRingReturnTiming = false;
bool armTransferDestinationM5FinalBufferPending = false;
// Ring-1 tray clearance is a command overlay on the existing transfer phases.
// 1=tray-to-ring1 placement, 2=ring1-to-tray pickup, 3=pre-clear the tray while
// moving to the next ring-1 pickup. Never add a wait phase.
uint8_t armTransferRing1TrayClearanceMode = 0U;
int8_t armTransferRing1TrayRestoreSlot = -1;
#if GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT > 100U
uint8_t armTransferSourceRingPosition = 0U;
#endif
arm_transfer::MotionProfile armTransferMotionProfile =
    arm_transfer::PROFILE_STANDARD;

#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
void beginRawPickupTrackingVision();
extern bool rawTrackingActive;
#endif

bool armTransferUsesRawProfile() {
  return armTransferMotionProfile ==
         arm_transfer::PROFILE_RAW;
}

bool armTransferUsesPlaceProfile() {
  return armTransferMotionProfile ==
         arm_transfer::PROFILE_RING_PLACE;
}

bool armTransferUsesReturnProfile() {
  return armTransferMotionProfile ==
         arm_transfer::PROFILE_RING_RETURN;
}

void restoreArmTransferRing1Tray() {
  commandStorageServoPosition(
      static_cast<uint8_t>(armTransferRing1TrayRestoreSlot));
  armTransferRing1TrayClearanceMode = 0U;
  if (armTransferMapDestination && armTransferPrepareNextSource &&
      armTransferNextStorageSlot == armTransferRing1TrayRestoreSlot) {
    armTransferNextStorageCommanded = true;
  }
}

bool armTransferUsesRing1LoadedReturnCushion() {
  // PROFILE_RING_RETURN仅用于粗加工区把已夹物料从圆环送回料盘；暂存区
  // 没有该回收动作。用实际动作类型和来源环位判定，正式流程与调试一致。
  return armTransferUsesReturnProfile() &&
#if GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT > 100U
         armTransferSourceRingPosition == 1U &&
#endif
         RING1_LOADED_RETURN_RAMP_TIME_PERCENT > 100U;
}

bool armTransferUsesFastWorkGripperProfile() {
  return armTransferUsesPlaceProfile() ||
         armTransferUsesReturnProfile();
}

uint32_t armTransferDestinationM7ToGripperGapMs() {
  return armTransferUsesPlaceProfile()
             ? WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_MS
             : WORK_M7_TO_GRIPPER_GAP_MS;
}

void commandArmTransferGripperOpen() {
  // Command-only boundary: phase/deadline changes belong to the transfer
  // state machine. Mixing orchestration into this function can suppress the
  // physical open command and let M7 reverse before the material is released.
  if (armTransferUsesRawProfile()) {
    commandRawGripperOpen();
    return;
  }
  if (armTransferUsesReturnProfile()) {
    SerialDebug.println(
        "[DOUBLE GRIPPER] tray release -> 20 ms open command");
    commandGripperDoubleSpeedOpen();
    return;
  }
  if (armTransferUsesPlaceProfile()) {
    SerialDebug.println(
        "[TARGET RELEASE] keep 40 ms open command");
    commandGripperTargetPlaceOpen();
    return;
  }
  commandGripperOpen();
}

void commandArmTransferGripperClose() {
  // Keep this symmetric with commandArmTransferGripperOpen(): issue only the
  // servo command; the caller owns both short M7/gripper handoffs and the
  // full mechanical close-completion gate.
  if (armTransferUsesFastWorkGripperProfile()) {
    commandGripperDoubleSpeedClose();
    return;
  }
  commandGripperClose();
}

uint32_t armTransferTransitionSettleMs() {
  return armTransferUsesFastWorkGripperProfile()
             ? ARM_TRANSFER_RETURN_SETTLE_MS
             : ARM_TRANSFER_BASE_SETTLE_MS;
}

bool armTransferIsProcessFirstRingReturn() {
  return armTransferProcessFirstRingReturnTiming &&
         armTransferUsesReturnProfile();
}

uint32_t armTransferGripperOpenSettleMs() {
  if (armTransferUsesReturnProfile()) {
    return GRIPPER_TRAY_RELEASE_OPEN_SETTLE_MS;
  }
  if (armTransferUsesPlaceProfile()) {
    return GRIPPER_TARGET_PLACE_OPEN_SETTLE_MS;
  }
  return GRIPPER_OPEN_SETTLE_MS;
}

uint32_t armTransferGripperCloseSettleMs() {
  if (armTransferUsesPlaceProfile()) {
    return GRIPPER_TRAY_PICK_CLOSE_SETTLE_MS;
  }
  if (armTransferUsesReturnProfile()) {
    return GRIPPER_TARGET_PICK_CLOSE_SETTLE_MS;
  }
  return GRIPPER_CLOSE_SETTLE_MS;
}

uint32_t armTransferRingReturnM7RaiseExtraDelayMs() {
  if (!armTransferUsesReturnProfile()) {
    return 0UL;
  }
  return RING_RETURN_M7_RAISE_EXTRA_DELAY_MS +
         (armTransferIsProcessFirstRingReturn()
              ? PROCESS_FIRST_RING_PICK_M7_RAISE_EXTRA_DELAY_MS
              : 0UL);
}

uint32_t armTransferRawProcessPickupM7RaiseAdditionalDelayMs() {
  return rawProcessPickupM7RaiseAdditionalDelayEnabled
             ? RAW_PROCESS_PICKUP_M7_RAISE_ADDITIONAL_DELAY_MS
             : 0UL;
}

float armTransferRotationClearanceHeightMm(
    bool mappedRingSide) {
  return mappedRingSide
             ? arm_config::RING_ROTATION_CLEARANCE_HEIGHT_MM
             : arm_config::TRAY_ROTATION_CLEARANCE_HEIGHT_MM;
}

bool ringMapHeadingStillValid();

float armTransferExtensionMinimumMm(
    float extensionMm,
    bool mappedRingPose = false) {

  if (armTransferUsesRawProfile()) {
    return M6_RAW_PICK_MINIMUM_EXTENSION_MM;
  }

  const bool pathUsesMappedNearSideRange =
      mappedRingPose ||
      extensionMm <
          M6_STANDARD_EXTENSION_MM -
              ARM_AXIS_POSITION_TOLERANCE_MM ||
      extensionAxis.currentMm <
          M6_STANDARD_EXTENSION_MM -
              ARM_AXIS_POSITION_TOLERANCE_MM;
  return
      pathUsesMappedNearSideRange
          ? M6_RING2_MINIMUM_EXTENSION_MM
          : M6_STANDARD_EXTENSION_MM;
}

bool startArmTransferSourceExtensionToMm(
    float extensionMm,
    bool mappedRingPose = false) {
  const float minimumMm =
      armTransferExtensionMinimumMm(
          extensionMm,
          mappedRingPose);
  if (armTransferUsesRawProfile()) {
    return startExtensionToContactMmWithProfile(
        extensionMm,
        minimumMm,
        RAW_M6_SPEED_RPM,
        RAW_M6_ACCELERATION);
  }
  if (armTransferUsesReturnProfile() ||
      mappedRingPose) {
    return startLinearAxisMove(
        extensionAxis,
        extensionMm,
        minimumMm,
        M6_MAXIMUM_EXTENSION_MM,
        M6_PULSES_PER_MM,
        M6_EXTEND_DIRECTION,
        M6_RETRACT_DIRECTION,
        RETURN_M6_SPEED_RPM,
        RETURN_M6_ACCELERATION);
  }
  if (armTransferUsesPlaceProfile()) {
    return startLinearAxisMove(
        extensionAxis,
        extensionMm,
        minimumMm,
        M6_MAXIMUM_EXTENSION_MM,
        M6_PULSES_PER_MM,
        M6_EXTEND_DIRECTION,
        M6_RETRACT_DIRECTION,
        PLACE_M6_SPEED_RPM,
        PLACE_M6_ACCELERATION);
  }
  return startLinearAxisMove(
      extensionAxis,
      extensionMm,
      minimumMm,
      M6_MAXIMUM_EXTENSION_MM,
      M6_PULSES_PER_MM,
      M6_EXTEND_DIRECTION,
      M6_RETRACT_DIRECTION,
      M6_SPEED_RPM,
      M6_ACCELERATION);
}

bool startArmTransferDestinationExtensionToMm(
    float extensionMm,
    bool mappedRingPose = false,
    bool loadedTrayMotion = false) {
  const float minimumMm =
      armTransferExtensionMinimumMm(
          extensionMm,
          mappedRingPose);
  if (armTransferUsesRawProfile()) {
    return startExtensionToContactMmWithProfile(
        extensionMm,
        minimumMm,
        RAW_M6_SPEED_RPM,
        loadedTrayMotion
            ? TRAY_LOAD_RAW_M6_ACCELERATION
            : RAW_M6_ACCELERATION);
  }
  if (armTransferUsesPlaceProfile()) {
    return startLinearAxisMove(
        extensionAxis,
        extensionMm,
        minimumMm,
        M6_MAXIMUM_EXTENSION_MM,
        M6_PULSES_PER_MM,
        M6_EXTEND_DIRECTION,
        M6_RETRACT_DIRECTION,
        PLACE_M6_SPEED_RPM,
        PLACE_M6_ACCELERATION);
  }
  if (armTransferUsesReturnProfile()) {
    const uint8_t returnAcceleration =
        loadedTrayMotion &&
                armTransferUsesRing1LoadedReturnCushion()
            ? RING1_LOADED_RETURN_M6_ACCELERATION
            : (loadedTrayMotion
                   ? TRAY_LOAD_RETURN_M6_ACCELERATION
                   : RETURN_M6_ACCELERATION);
    return startLinearAxisMove(
        extensionAxis,
        extensionMm,
        minimumMm,
        M6_MAXIMUM_EXTENSION_MM,
        M6_PULSES_PER_MM,
        M6_EXTEND_DIRECTION,
        M6_RETRACT_DIRECTION,
        loadedTrayMotion
            ? arm_config::M6_LOADED_RETURN_SPEED_RPM
            : RETURN_M6_SPEED_RPM,
        returnAcceleration);
  }
  return startExtensionToContactMmWithProfile(
      extensionMm,
      minimumMm,
      M6_SPEED_RPM,
      M6_ACCELERATION);
}

float armTransferClearanceTargetMm(bool mappedDepartureSide) {
  const float requiredClearanceMm =
      armTransferRotationClearanceHeightMm(
          mappedDepartureSide);
  return liftAxis.currentMm > requiredClearanceMm
             ? liftAxis.currentMm
             : requiredClearanceMm;
}

bool armTransferPlanarMoveIsSafe(
    bool mappedDepartureSide) {
  const float requiredClearanceMm =
      armTransferRotationClearanceHeightMm(
          mappedDepartureSide);
  if (!liftMoveFinished() ||
      liftAxis.currentMm <
          requiredClearanceMm -
              ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault(
        "M5/M6 planar move requested below M7 clearance");
    return false;
  }
  if (extensionAxis.active ||
      extensionAxis.recoveryPending ||
      extensionAxis.fault ||
      armBaseMotionLockedUntilReset ||
      armMotors.isM5Running()) {
    routeFault(
        "M5/M6 planar move requested while an axis is busy");
    return false;
  }
  return true;
}

bool startArmTransferPlanarMove(
    const ArmPose &pose,
    bool mappedPose,
    bool mappedDepartureSide,
    bool sourceExtensionPolicy,
    bool &rotationStarted,
    const char *label,
    bool loadedTrayMotion = false) {
  if (!armTransferPlanarMoveIsSafe(
          mappedDepartureSide)) {
    return false;
  }
  if (mappedPose && !ringMapHeadingStillValid()) {
    return false;
  }

  const bool clockwiseRotation =
      pose.standardFrameAngleDegrees <
      armTransferSourcePose.standardFrameAngleDegrees -
          FINAL_RETURN_M5_TOLERANCE_DEGREES;
  const bool retractingExtension =
      pose.extensionMm <
      extensionAxis.currentMm -
          ARM_AXIS_POSITION_TOLERANCE_MM;
  const bool ring1LoadedReturnCushion =
      loadedTrayMotion &&
      armTransferUsesRing1LoadedReturnCushion();

  if (ring1LoadedReturnCushion &&
      (!clockwiseRotation || !retractingExtension)) {
    routeFault("Ring-1 loaded return direction mismatch");
    return false;
  }

  if (ring1LoadedReturnCushion) {
    useArmBaseRing1LoadedReturnCushionMotionProfile();
  } else if (loadedTrayMotion && armTransferUsesRawProfile()) {
    useArmBaseRawTrayLoadCushionMotionProfile();
  } else if (loadedTrayMotion && armTransferUsesReturnProfile()) {
    useArmBaseTrayLoadReturnCushionMotionProfile();
  } else if (armTransferUsesReturnProfile()) {
    useArmBaseReturnMotionProfile();
  } else if (armTransferUsesRawProfile()) {
    useArmBaseRawTransferMotionProfile();
  } else if (armTransferUsesPlaceProfile()) {
    useArmBasePlaceMotionProfile();
  } else {
    useArmBaseTransferMotionProfile();
  }
  const bool extensionStarted =
      sourceExtensionPolicy
          ? startArmTransferSourceExtensionToMm(
                pose.extensionMm,
                mappedPose)
          : startArmTransferDestinationExtensionToMm(
                pose.extensionMm,
                mappedPose,
                loadedTrayMotion);
  if (!extensionStarted) {
    if (programState != PROGRAM_FAULT) {
      routeFault("M5/M6 planar parallel start rejected");
    }
    return false;
  }
  // The M6 command is sent and returns without waiting for travel completion.
  // Start M5 immediately afterward; both axes then move during the same phase.
  startArmBaseStandardFrameDegrees(
      pose.standardFrameAngleDegrees);
  rotationStarted = true;

  SerialDebug.print("[TRANSFER PLANAR] ");
  SerialDebug.print(label);
  SerialDebug.println(
      ": M5 rotation + M6 extension/retraction started together");
  if (ring1LoadedReturnCushion) {
    SerialDebug.print(
        "[RING1 LOADED RETURN CUSHION] speed/acceleration/ramp-x=");
    SerialDebug.print(
        arm_config::M5_LOADED_RETURN_MAXIMUM_STEP_RATE,
        0);
    SerialDebug.print("/");
    SerialDebug.print(
        arm_config::M6_LOADED_RETURN_SPEED_RPM);
    SerialDebug.print("/");
    SerialDebug.print(
        RING1_LOADED_RETURN_M5_STEP_ACCELERATION,
        0);
    SerialDebug.print("/");
    SerialDebug.print(
        RING1_LOADED_RETURN_M6_ACCELERATION);
    SerialDebug.print("/");
    SerialDebug.println(
        static_cast<float>(
            RING1_LOADED_RETURN_RAMP_TIME_PERCENT) /
            100.0f,
        1);
  } else if (loadedTrayMotion) {
    SerialDebug.print(
        "[TRAY LOAD CUSHION] profile/ramp-x=");
    SerialDebug.print(
        armTransferUsesRawProfile() ? "RAW/" : "RETURN/");
    SerialDebug.println(1.5f, 1);
  }
  return true;
}

bool startArmTransferLiftToHeightMm(float heightMm) {
  if (armTransferUsesReturnProfile()) {
    return startLinearAxisMove(
        liftAxis,
        heightMm,
        M7_MINIMUM_HEIGHT_MM,
        M7_STANDARD_HEIGHT_MM,
        M7_PULSES_PER_MM,
        M7_RAISE_DIRECTION,
        M7_LOWER_DIRECTION,
        RETURN_M7_SPEED_RPM,
        RETURN_M7_ACCELERATION);
  }
  if (armTransferUsesRawProfile()) {
    return startLiftToHeightMmWithProfile(
        heightMm,
        RAW_M7_SPEED_RPM,
        RAW_M7_ACCELERATION);
  }
  return startLiftToHeightMm(heightMm);
}

bool startArmTransferLiftToSourceHeightMm(
    float heightMm) {
  if (armTransferUsesReturnProfile()) {
    SerialDebug.println(
        "[RING RETURN] direct single-segment source descent");
    return startLinearAxisMove(
        liftAxis,
        heightMm,
        M7_MINIMUM_HEIGHT_MM,
        M7_STANDARD_HEIGHT_MM,
        M7_PULSES_PER_MM,
        M7_RAISE_DIRECTION,
        M7_LOWER_DIRECTION,
        RETURN_M7_SPEED_RPM,
        RETURN_M7_ACCELERATION);
  }
  if (armTransferUsesRawProfile()) {
    SerialDebug.println(
        "[RAW PICK] test-matched direct single-segment M7 descent");
    return startLinearAxisMove(
        liftAxis,
        heightMm,
        M7_MINIMUM_HEIGHT_MM,
        M7_STANDARD_HEIGHT_MM,
        M7_PULSES_PER_MM,
        M7_RAISE_DIRECTION,
        M7_LOWER_DIRECTION,
        M7_SPEED_RPM,
        M7_ACCELERATION);
  }
  if (!armTransferMapSource) {
    SerialDebug.println(
        "[TRAY PICK] direct single-segment descent; no contact cushion");
    return startLinearAxisMove(
        liftAxis,
        heightMm,
        M7_MINIMUM_HEIGHT_MM,
        M7_STANDARD_HEIGHT_MM,
        M7_PULSES_PER_MM,
        M7_RAISE_DIRECTION,
        M7_LOWER_DIRECTION,
        M7_SPEED_RPM,
        M7_ACCELERATION);
  }
  if (armTransferMapSource) {
    return startLiftToHeightMmWithProfile(
        heightMm,
        RAW_M7_SPEED_RPM,
        RAW_M7_ACCELERATION);
  }
  return startLiftToHeightMm(heightMm);
}

bool startArmTransferLiftToContactHeightMm(
    float heightMm) {
  if (armTransferUsesReturnProfile()) {
    SerialDebug.println(
        "[RING RETURN] direct single-segment tray descent");
    return startLinearAxisMove(
        liftAxis,
        heightMm,
        M7_MINIMUM_HEIGHT_MM,
        M7_STANDARD_HEIGHT_MM,
        M7_PULSES_PER_MM,
        M7_RAISE_DIRECTION,
        M7_LOWER_DIRECTION,
        RETURN_M7_SPEED_RPM,
        RETURN_M7_ACCELERATION);
  }
  if (armTransferUsesRawProfile()) {
    SerialDebug.println(
        "[TRAY PLACE] RAW direct single-segment descent; no contact cushion");
    return startLinearAxisMove(
        liftAxis,
        heightMm,
        M7_MINIMUM_HEIGHT_MM,
        M7_STANDARD_HEIGHT_MM,
        M7_PULSES_PER_MM,
        M7_RAISE_DIRECTION,
        M7_LOWER_DIRECTION,
        RAW_M7_SPEED_RPM,
        RAW_M7_ACCELERATION);
  }
  return startLiftMoveWithContactSoftLanding(
      heightMm,
      M7_SPEED_RPM,
      M7_ACCELERATION);
}

bool armPoseIsValid(
    const ArmPose &pose,
    float minimumExtensionMm) {
  return pose.extensionMm >=
             minimumExtensionMm -
                 ARM_AXIS_POSITION_TOLERANCE_MM &&
         pose.extensionMm <=
             M6_MAXIMUM_EXTENSION_MM +
                 ARM_AXIS_POSITION_TOLERANCE_MM &&
         pose.heightMm >=
             M7_MINIMUM_HEIGHT_MM -
                 ARM_AXIS_POSITION_TOLERANCE_MM &&
         pose.heightMm <=
             M7_STANDARD_HEIGHT_MM +
                 ARM_AXIS_POSITION_TOLERANCE_MM;
}

void beginArmTransfer(
    const ArmPose &source,
    const ArmPose &destination,
    bool gentleDestinationRelease,
    bool mapSource = false,
    bool mapDestination = false,
    bool concurrentSourcePreparation = false,
    bool sourceAlreadyPrepared = false,
    const ArmPose *nextSourcePreparation = nullptr,
    int8_t nextStorageSlot = -1,
    bool returnToRawViewAtEnd = false,
    bool rawFastProfile = false,
    bool sourceGripperAlreadyOpen = false,
    bool nextSourceMapped = false,
    bool processFirstRingReturnTiming = false,
    bool storage2Batch = false,
    uint8_t sourceRingPosition = 0U) {
  if (armPayloadPresent) {
    routeFault("Arm payload state fault");
    return;
  }
  armPayloadPresent = false;
  armTransferStorage2Batch = storage2Batch;
  const bool effectiveConcurrentSourcePreparation =
      concurrentSourcePreparation &&
      !armGripperLiftIsolationEnabled;
  // RAW pickup coordinates are shared by both RAW visits and may use the
  // calibrated near-side M6 range down to -9.5 mm. Keep this validation
  // independent from mapSource: a RAW item is not a mapped process ring.
  const float sourceMinimumExtensionMm =
      rawFastProfile
          ? M6_RAW_PICK_MINIMUM_EXTENSION_MM
          : (mapSource
                 ? M6_RING2_MINIMUM_EXTENSION_MM
                 : M6_STANDARD_EXTENSION_MM);
  const float destinationMinimumExtensionMm =
      mapDestination
          ? M6_RING2_MINIMUM_EXTENSION_MM
          : M6_STANDARD_EXTENSION_MM;
  if (!armPoseIsValid(source, sourceMinimumExtensionMm) ||
      !armPoseIsValid(
          destination,
          destinationMinimumExtensionMm)) {
    routeFault("Transfer pose outside arm travel");
    return;
  }
  if (sourceAlreadyPrepared &&
      effectiveConcurrentSourcePreparation) {
    routeFault("Transfer source preparation mode conflict");
    return;
  }
  if (nextSourcePreparation != nullptr &&
      (!armPoseIsValid(
           *nextSourcePreparation,
           rawFastProfile
               ? M6_RAW_PICK_MINIMUM_EXTENSION_MM
               : (nextSourceMapped
                      ? M6_RING2_MINIMUM_EXTENSION_MM
                      : M6_STANDARD_EXTENSION_MM)) ||
       nextStorageSlot < 0 ||
       nextStorageSlot > 2)) {
    routeFault("Next pickup preparation is invalid");
    return;
  }
  if (returnToRawViewAtEnd &&
      nextSourcePreparation != nullptr) {
    routeFault("Transfer final M5 target is ambiguous");
    return;
  }
  if (sourceAlreadyPrepared &&
      !sourceGripperAlreadyOpen) {
    routeFault(
        "Prepared transfer source requires gripper already open");
    return;
  }
#if GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT > 100U
  if (sourceRingPosition > 3U ||
      (sourceRingPosition != 0U && !mapSource)) {
    routeFault("Transfer source ring position is invalid");
    return;
  }
#endif

  armTransferMotionProfile =
      arm_transfer::selectMotionProfile(
          rawFastProfile,
          mapSource,
          mapDestination,
          gentleDestinationRelease);
  if (armTransferUsesRawProfile()) {
    useArmBaseRawTransferMotionProfile();
  } else if (armTransferUsesPlaceProfile()) {
    useArmBasePlaceMotionProfile();
  } else if (armTransferUsesReturnProfile()) {
    useArmBaseReturnMotionProfile();
  } else {
    useArmBaseTransferMotionProfile();
  }
  armTransferSourcePose = source;
  armTransferDestinationPose = destination;
  armTransferGentleDestinationRelease =
      gentleDestinationRelease;
  armTransferMapSource = mapSource;
  armTransferMapDestination = mapDestination;
  armTransferConcurrentSourcePreparation =
      effectiveConcurrentSourcePreparation;
  armTransferSourceAlreadyPrepared =
      sourceAlreadyPrepared;
  armTransferPrepareNextSource =
      nextSourcePreparation != nullptr;
  if (armTransferPrepareNextSource) {
    armTransferNextSourcePose =
        *nextSourcePreparation;
  } else {
    armTransferNextSourcePose = ArmPose();
  }
  armTransferNextSourceMapped =
      armTransferPrepareNextSource &&
      nextSourceMapped;
  armTransferNextStorageSlot = nextStorageSlot;
  armTransferNextStorageDeadlineMs = 0UL;
  armTransferStorageCommandDueMs = 0UL;
  armTransferNextStorageCommanded = false;
  armTransferM7HandoffStartMs = 0UL;
  armTransferRawM7ToGripperDeadlineUs = 0UL;
  armTransferNextSourcePreparedAtEnd = false;
  armTransferReturnToRawViewAtEnd =
      returnToRawViewAtEnd;
  armTransferSourceRotationStarted = false;
  armTransferDestinationRotationStarted = false;
  armTransferFinalRotationStarted = false;
  armTransferProcessFirstRingReturnTiming =
      processFirstRingReturnTiming;
#if GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT > 100U
  armTransferSourceRingPosition = sourceRingPosition;
#endif
  armTransferDestinationM5FinalBufferPending =
      processFirstRingReturnTiming &&
      fabsf(
          destination.standardFrameAngleDegrees -
          source.standardFrameAngleDegrees) >
          PROCESS_FIRST_RING_M5_FINAL_BUFFER_DEGREES +
              FINAL_RETURN_M5_TOLERANCE_DEGREES;

  // Keep production serial bandwidth for faults and phase boundaries. The
  // detailed source/destination pose dump was removed to fit the 128 KB image
  // and to avoid adding runtime cost to the new parallel tray commands.
  if (concurrentSourcePreparation &&
      !effectiveConcurrentSourcePreparation) {
    SerialDebug.println(
        "[WORK INTERLOCK] concurrent gripper/M7 source preparation "
        "disabled");
  }

  if (armTransferSourceAlreadyPrepared) {
    const float requiredClearanceMm =
        armTransferRotationClearanceHeightMm(
            armTransferMapSource);
    if (liftAxis.active ||
        liftAxis.recoveryPending ||
        liftAxis.fault ||
        extensionAxis.active ||
        extensionAxis.recoveryPending ||
        extensionAxis.fault ||
        armMotors.isM5Running() ||
        liftAxis.currentMm <
            requiredClearanceMm -
                ARM_AXIS_POSITION_TOLERANCE_MM ||
        fabsf(
            extensionAxis.currentMm -
            armTransferSourcePose.extensionMm) >
            ARM_AXIS_POSITION_TOLERANCE_MM) {
      routeFault(
          "Prepared container pickup pose was not retained");
      return;
    }
    if (armTransferStorage2Batch) {
      armTransferDeadlineMs = millis();
      armTransferPhase = ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE;
      return;
    }
    SerialDebug.println(
        "[TRANSFER SAFE] retained M5/M6 source pose and open gripper "
        "at/above M7 clearance; lower directly for next pickup");
    if (!startArmTransferLiftToSourceHeightMm(
            armTransferSourcePose.heightMm)) {
      return;
    }
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
    if (armTransferUsesRawProfile()) {
      beginRawPickupTrackingVision();
    }
#endif
    armTransferPhase =
        ARM_TRANSFER_WAIT_SOURCE_LOWER;
    return;
  }

  if (sourceGripperAlreadyOpen) {
    if (!startArmTransferLiftToHeightMm(
            armTransferClearanceTargetMm(
                armTransferMapSource))) {
      return;
    }
    SerialDebug.println(
        "[TRANSFER DIRECT] gripper already open; "
        "skip source-open settle");
    armTransferPhase =
        ARM_TRANSFER_WAIT_PREPARE_LIFT;
    return;
  }

  if (armGripperLiftIsolationEnabled) {
    // New safe path: this pre-source wait is orchestration and must stay here,
    // never inside commandArmTransferGripperOpen(). The old direct-open path
    // remains below for non-workstation/manual callers with isolation off.
    armTransferDeadlineMs = 0UL;
    armTransferPhase =
        ARM_TRANSFER_WAIT_PRE_SOURCE_OPEN_GAP;
    SerialDebug.println(
        "[WORK INTERLOCK] unprepared source waits for M7 short "
        "handoff before gripper open");
    return;
  }

  commandArmTransferGripperOpen();
  if (armTransferConcurrentSourcePreparation) {

    if (!startArmTransferLiftToHeightMm(
            arm_config::RING_ROTATION_CLEARANCE_HEIGHT_MM)) {
      return;
    }
    SerialDebug.println(
        "[TRANSFER PIPELINE] ring 3 accepted: M7 raises to "
        "rotation clearance before first tray pickup");
    armTransferPhase =
        ARM_TRANSFER_WAIT_PREPARE_CLEARANCE;
  } else {
    armTransferDeadlineMs =
        millis() + armTransferGripperOpenSettleMs() +
        (armGripperLiftIsolationEnabled
             ? WORK_GRIPPER_TO_M7_GAP_MS
             : 0UL);
    armTransferPhase =
        ARM_TRANSFER_WAIT_SOURCE_OPEN;
  }
}

void startArmTransferDestinationDescent() {
  if (!armTransferGentleDestinationRelease) {
    if (startArmTransferLiftToContactHeightMm(
            armTransferDestinationPose.heightMm)) {
      armTransferPhase =
          ARM_TRANSFER_WAIT_DESTINATION_LOWER;
    }
    return;
  }

  float approachHeightMm =
      armTransferDestinationPose.heightMm +
      RING_PLACE_FINAL_DESCENT_MM;
  if (approachHeightMm > M7_STANDARD_HEIGHT_MM) {
    approachHeightMm = M7_STANDARD_HEIGHT_MM;
  }
  SerialDebug.print("[RING PLACE] normal descent -> ");
  SerialDebug.print(approachHeightMm, 2);
  SerialDebug.print(" mm; final ");
  SerialDebug.print(RING_PLACE_FINAL_DESCENT_MM, 1);
  SerialDebug.println(" mm will use mild slow profile");
  if (startLiftToHeightMm(approachHeightMm)) {
    armTransferPhase =
        ARM_TRANSFER_WAIT_DESTINATION_APPROACH;
  }
}

void releaseArmTransferDestination() {
  if (armTransferPrepareNextSource &&
      !armTransferNextStorageCommanded &&
      !armTransferUsesReturnProfile() &&
      !armTransferUsesRawProfile() &&
      armTransferRing1TrayClearanceMode != 1U &&
      armTransferRing1TrayClearanceMode != 3U) {
    commandStorageServoPosition(
        static_cast<uint8_t>(
            armTransferNextStorageSlot));
    if (programState == PROGRAM_FAULT) {
      return;
    }
    armTransferNextStorageDeadlineMs =
        millis() + STORAGE_SERVO_SETTLE_MS;
    armTransferNextStorageCommanded = true;
    SerialDebug.print(
        "[TRANSFER PIPELINE] next storage slot moving now: ");
    SerialDebug.println(
        static_cast<int>(
            armTransferNextStorageSlot));
  }
  commandArmTransferGripperOpen();
  const uint32_t gripperOpenCommandMs = millis();
  if (armTransferUsesRawProfile() &&
      armTransferPrepareNextSource) {
    armTransferStorageCommandDueMs =
        gripperOpenCommandMs +
        armTransferGripperOpenSettleMs() +
        RAW_NEXT_STORAGE_COMMAND_DELAY_MS;
    SerialDebug.println(
        "[RAW STORAGE] gripper release completion + 1000 ms "
        "required before next-slot command");
  } else if (armTransferUsesReturnProfile()) {
    armTransferStorageCommandDueMs =
        gripperOpenCommandMs +
        armTransferGripperOpenSettleMs() +
        RING_RETURN_STORAGE_COMMAND_DELAY_MS;
    SerialDebug.println(
        "[RING RETURN STORAGE] gripper release completion + 500 ms "
        "required before turntable command");
  }
  armTransferDeadlineMs =
      gripperOpenCommandMs +
      armTransferGripperOpenSettleMs() +
      (armGripperLiftIsolationEnabled
           ? WORK_GRIPPER_TO_M7_GAP_MS
           : 0UL);
  if ((armTransferRing1TrayClearanceMode == 1U ||
       armTransferRing1TrayClearanceMode == 3U) &&
      armTransferPrepareNextSource) {
    // Retain the original release-based gate. The actual slot command waits
    // for M7 clearance and therefore adds no new deadline.
    armTransferNextStorageDeadlineMs =
        armTransferUsesReturnProfile()
            ? armTransferStorageCommandDueMs + STORAGE_SERVO_SETTLE_MS
            : gripperOpenCommandMs + STORAGE_SERVO_SETTLE_MS;
  }
  armTransferPhase =
      ARM_TRANSFER_WAIT_DESTINATION_OPEN;
}

void serviceArmTransferDeferredStorageCommand() {
  const bool rawNextSlotDeferred =
      armTransferUsesRawProfile() &&
      armTransferPrepareNextSource;
  const bool ringReturnSlotDeferred =
      armTransferUsesReturnProfile() &&
      armTransferPrepareNextSource &&
      armTransferRing1TrayClearanceMode != 3U;
  if ((!rawNextSlotDeferred &&
       !ringReturnSlotDeferred) ||
      !armTransferPrepareNextSource ||
      armTransferNextStorageCommanded ||
      armTransferStorageCommandDueMs == 0UL ||
      !deadlineReached(armTransferStorageCommandDueMs)) {
    return;
  }

  if (rawNextSlotDeferred) {
    commandRawStorageServoPositionWithArrivalGate(
        static_cast<uint8_t>(armTransferNextStorageSlot));
  } else {
    commandStorageServoPosition(
        static_cast<uint8_t>(armTransferNextStorageSlot));
  }
  if (programState == PROGRAM_FAULT) {
    return;
  }
  armTransferNextStorageDeadlineMs =
      millis() +
      (rawNextSlotDeferred
           ? RAW_STORAGE_SERVO_SETTLE_MS
           : STORAGE_SERVO_SETTLE_MS);
  armTransferNextStorageCommanded = true;
  if (rawNextSlotDeferred) {
    SerialDebug.print(
        "[RAW STORAGE] 1000 ms post-release delay complete; "
        "next slot commanded, first verification-ms=");
    SerialDebug.println(RAW_STORAGE_SERVO_SETTLE_MS);
  } else {
    SerialDebug.print(
        "[RING RETURN STORAGE] 500 ms post-release delay complete; "
        "turntable commanded, arrival-gate-ms=");
    SerialDebug.println(STORAGE_SERVO_SETTLE_MS);
  }
}

void serviceArmTransfer() {
  serviceArmTransferDeferredStorageCommand();
  switch (armTransferPhase) {
    case ARM_TRANSFER_IDLE:
    case ARM_TRANSFER_COMPLETE:
      return;

    case ARM_TRANSFER_WAIT_PRE_SOURCE_OPEN_GAP:
      if (!liftMoveFinished()) {
        break;
      }
      if (armTransferDeadlineMs == 0UL) {
        armTransferDeadlineMs =
            millis() + WORK_M7_TO_GRIPPER_GAP_MS;
        break;
      }
      if (deadlineReached(armTransferDeadlineMs)) {
        commandArmTransferGripperOpen();
        armTransferDeadlineMs =
            millis() + armTransferGripperOpenSettleMs() +
            WORK_GRIPPER_TO_M7_GAP_MS;
        armTransferPhase =
            ARM_TRANSFER_WAIT_SOURCE_OPEN;
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_OPEN:
      if (deadlineReached(armTransferDeadlineMs)) {

        if (!startArmTransferLiftToHeightMm(
                armTransferClearanceTargetMm(
                    armTransferMapSource))) {
          break;
        }
        armTransferPhase =
            ARM_TRANSFER_WAIT_PREPARE_LIFT;
      }
      break;

    case ARM_TRANSFER_WAIT_PREPARE_CLEARANCE:
      if (liftMoveFinished()) {
        if (startArmTransferPlanarMove(
                armTransferSourcePose,
                armTransferMapSource,
                armTransferMapSource,
                true,
                armTransferSourceRotationStarted,
                "clearance -> source")) {
          armTransferPhase =
              ARM_TRANSFER_WAIT_SOURCE_ROTATION;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_PREPARE_LIFT:
      if (liftMoveFinished()) {
        if (startArmTransferPlanarMove(
                armTransferSourcePose,
                armTransferMapSource,
                armTransferMapSource,
                true,
                armTransferSourceRotationStarted,
                "M7 safe -> source")) {
          armTransferPhase =
              ARM_TRANSFER_WAIT_SOURCE_ROTATION;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_ROTATION:
      if (!armMotors.isM5Running() &&
          extensionMoveFinished() &&
          liftMoveFinished()) {
        armTransferDeadlineMs =
            millis() + armTransferTransitionSettleMs();
        armTransferPhase =
            ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE;
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE:
      if (deadlineReached(armTransferDeadlineMs)) {
        // Query only while stationary, immediately before entering the tray.
        if (armTransferStorage2Batch &&
            (armMotors.isM5Running() || armBaseMotionWatchdogActive ||
             !extensionMoveFinished() || !liftMoveFinished() ||
             !allMotorsArrived() || !serviceStorageServoArrivalGate())) {
          break;
        }
        if (armTransferMapSource &&
            !ringMapHeadingStillValid()) {
          return;
        }
        if (startArmTransferLiftToSourceHeightMm(
                armTransferSourcePose.heightMm)) {
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
          if (armTransferUsesRawProfile()) {
            beginRawPickupTrackingVision();
          }
#endif
          armTransferPhase =
              ARM_TRANSFER_WAIT_SOURCE_LOWER;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_LOWER:
      if (liftMoveFinished()) {
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
        if (armTransferUsesRawProfile() &&
            (extensionAxis.active ||
             extensionAxis.recoveryPending ||
             armMotors.isM5Running() ||
             armBaseMotionWatchdogActive)) {
          // The last camera frame may have launched an M5/M6 correction just
          // before M7 reached pickup height. Do not close until all three
          // axes have reached the same tracked target.
          break;
        }
        if (armTransferUsesRawProfile() && rawTrackingActive) {
          stopMaixRequest();
          rawTrackingActive = false;
        }
#endif
        if (armTransferUsesRawProfile()) {
          armTransferRawM7ToGripperDeadlineUs =
              micros() + RAW_M7_TO_GRIPPER_HANDOFF_US;
          armTransferPhase =
              ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP;
        } else if (armGripperLiftIsolationEnabled) {
          armTransferM7HandoffStartMs = millis();
          armTransferDeadlineMs =
              millis() + WORK_M7_TO_GRIPPER_GAP_MS;
          armTransferPhase =
              ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP;
          SerialDebug.println(
              "[WORK INTERLOCK] M7 source stop -> short handoff "
              "before gripper close");
        } else {
          commandArmTransferGripperClose();
          armTransferDeadlineMs =
              millis() + armTransferGripperCloseSettleMs() +
              armTransferRingReturnM7RaiseExtraDelayMs() +
              armTransferRawProcessPickupM7RaiseAdditionalDelayMs();
          armTransferPhase =
              ARM_TRANSFER_WAIT_GRIP_CLOSE;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP:
      if (armTransferUsesRawProfile()) {
        if (static_cast<int32_t>(
                micros() -
                armTransferRawM7ToGripperDeadlineUs) < 0) {
          break;
        }
        commandArmTransferGripperClose();
        armTransferRawM7ToGripperDeadlineUs = 0UL;
        armTransferDeadlineMs =
            millis() + armTransferGripperCloseSettleMs() +
            WORK_GRIPPER_TO_M7_GAP_MS +
            armTransferRawProcessPickupM7RaiseAdditionalDelayMs();
        armTransferPhase = ARM_TRANSFER_WAIT_GRIP_CLOSE;
        break;
      }
      if (deadlineReached(armTransferDeadlineMs)) {
        SerialDebug.print(
            "[WORK TIMING] M7 verified source stop -> gripper close "
            "command ms=");
        SerialDebug.print(
            millis() - armTransferM7HandoffStartMs);
        SerialDebug.print(" (limit ");
        SerialDebug.print(WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS);
        SerialDebug.println(" ms)");
        commandArmTransferGripperClose();
        armTransferM7HandoffStartMs = 0UL;
        armTransferDeadlineMs =
            millis() + armTransferGripperCloseSettleMs() +
            WORK_GRIPPER_TO_M7_GAP_MS +
            armTransferRingReturnM7RaiseExtraDelayMs() +
            armTransferRawProcessPickupM7RaiseAdditionalDelayMs();
        armTransferPhase =
            ARM_TRANSFER_WAIT_GRIP_CLOSE;
        SerialDebug.println(
            "[WORK INTERLOCK] source gap complete -> gripper close; "
            "M7 blocked through close completion + short handoff");
      }
      break;

    case ARM_TRANSFER_WAIT_GRIP_CLOSE:
      if (deadlineReached(armTransferDeadlineMs)) {
        // The source close and its mechanical settle are complete. Every move
        // from this point through destination release uses loaded profiles.
        armPayloadPresent = true;
        const float clearanceHeightMm =
            armTransferRotationClearanceHeightMm(
                armTransferMapSource);
        if (startArmTransferLiftToHeightMm(
                clearanceHeightMm)) {
          SerialDebug.print(
              "[TRANSFER PIPELINE] loaded M7 -> rotation "
              "clearance ");
          SerialDebug.println(clearanceHeightMm, 1);
          armTransferPhase =
              ARM_TRANSFER_WAIT_LOADED_CLEARANCE;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_LOADED_CLEARANCE:
      if (liftMoveFinished()) {
        if (armTransferRing1TrayClearanceMode == 1U) {
          // Keep the original tray slot throughout pickup and M7 ascent.
          // Only after M7 reaches clearance may the tray park, overlapping
          // the existing loaded M5/M6 move toward ring 1.
          commandRing1TrayClearancePosition();
        } else if (armTransferRing1TrayClearanceMode == 2U) {
          // Ring-1 pickup is vertically clear. Restore the destination tray
          // slot in the same pass that starts the loaded planar return.
          restoreArmTransferRing1Tray();
        }
        ArmPose initialDestinationPose =
            armTransferDestinationPose;
        if (armTransferDestinationM5FinalBufferPending) {
          const float rotationDeltaDegrees =
              armTransferDestinationPose.standardFrameAngleDegrees -
              armTransferSourcePose.standardFrameAngleDegrees;
          initialDestinationPose.standardFrameAngleDegrees -=
              rotationDeltaDegrees > 0.0f
                  ? PROCESS_FIRST_RING_M5_FINAL_BUFFER_DEGREES
                  : -PROCESS_FIRST_RING_M5_FINAL_BUFFER_DEGREES;
        }
        if (startArmTransferPlanarMove(
                initialDestinationPose,
                armTransferMapDestination,
                armTransferMapSource,
                false,
                armTransferDestinationRotationStarted,
                "loaded clearance -> destination",
                armTransferUsesRawProfile() ||
                    armTransferUsesReturnProfile())) {
          if (armTransferStorage2Batch && armTransferPrepareNextSource &&
              !armTransferNextStorageCommanded) {
            // M7 is at verified clearance; the picked item is clear of the tray.
            // M5/M6 have started towards the ring. Prepare the next color now.
            commandStorageServoPositionWithArrivalGate(
                static_cast<uint8_t>(armTransferNextStorageSlot));
            if (programState == PROGRAM_FAULT) { break; }
            armTransferNextStorageDeadlineMs = millis();
            armTransferNextStorageCommanded = true;
          }
          armTransferPhase =
              ARM_TRANSFER_WAIT_DESTINATION_ROTATION;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION:
      if (!armMotors.isM5Running() &&
          extensionMoveFinished() &&
          liftMoveFinished()) {
        if (armTransferDestinationM5FinalBufferPending) {
          setArmBaseMotionProfileForPayload(
              PROCESS_FIRST_RING_M5_FINAL_BUFFER_MAXIMUM_STEP_RATE,
              PROCESS_FIRST_RING_M5_FINAL_BUFFER_STEP_ACCELERATION);
          startArmBaseStandardFrameDegrees(
              armTransferDestinationPose.standardFrameAngleDegrees);
          armTransferDestinationM5FinalBufferPending = false;
          break;
        }
        if (armTransferUsesRawProfile() &&
            rawInitialStorageArrivalOverlapPending) {
          if (!serviceStorageServoArrivalGate()) {
            break;
          }
          resetStorageServoArrivalGate();
          rawInitialStorageArrivalOverlapPending = false;
          SerialDebug.println(
              "[RAW STORAGE OVERLAP] initial tray slot confirmed "
              "before destination M7 descent");
        }
        armTransferDeadlineMs =
            millis() + armTransferTransitionSettleMs();
        armTransferPhase =
            ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE;
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE:
      if (deadlineReached(armTransferDeadlineMs)) {
        if (armTransferMapDestination &&
            !ringMapHeadingStillValid()) {
          return;
        }
        startArmTransferDestinationDescent();
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_APPROACH:
      if (liftMoveFinished()) {
        if (armTransferMapDestination &&
            !ringMapHeadingStillValid()) {
          return;
        }
        SerialDebug.print(
            "[RING PLACE] slow final descent rpm/acc=");
        SerialDebug.print(M7_RING_PLACE_SPEED_RPM);
        SerialDebug.print("/");
        SerialDebug.println(
            M7_RING_PLACE_ACCELERATION);
        if (startLiftToHeightMmWithProfile(
                armTransferDestinationPose.heightMm,
                M7_RING_PLACE_SPEED_RPM,
                M7_RING_PLACE_ACCELERATION)) {
          armTransferPhase =
              ARM_TRANSFER_WAIT_DESTINATION_LOWER;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_LOWER:
      if (liftMoveFinished()) {
        if (armGripperLiftIsolationEnabled) {
          const uint32_t destinationGapMs =
              armTransferDestinationM7ToGripperGapMs();
          armTransferM7HandoffStartMs = millis();
          armTransferDeadlineMs =
              millis() + destinationGapMs;
          armTransferPhase =
              ARM_TRANSFER_WAIT_DESTINATION_GRIPPER_GAP;
          SerialDebug.println(
              "[WORK INTERLOCK] M7 destination stop -> short handoff "
              "before gripper open");
        } else {
          releaseArmTransferDestination();
        }
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_GRIPPER_GAP:
      if (deadlineReached(armTransferDeadlineMs)) {
        SerialDebug.print(
            "[WORK TIMING] M7 verified destination stop -> gripper open "
            "command ms=");
        SerialDebug.print(
            millis() - armTransferM7HandoffStartMs);
        SerialDebug.print(" (limit ");
        SerialDebug.print(WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS);
        SerialDebug.println(" ms)");
        releaseArmTransferDestination();
        armTransferM7HandoffStartMs = 0UL;
        SerialDebug.println(
            "[WORK INTERLOCK] destination gap complete -> gripper "
            "open; M7 blocked through open completion + short handoff");
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_OPEN:
      if (deadlineReached(armTransferDeadlineMs)) {
        // Keep loaded limits until the open command has physically settled;
        // only the following clearance/return motions are empty-gripper moves.
        armPayloadPresent = false;
        const float clearanceHeightMm =
            armTransferRotationClearanceHeightMm(
                armTransferMapDestination);
        if (startArmTransferLiftToHeightMm(
                clearanceHeightMm)) {
          SerialDebug.print(
              "[TRANSFER PIPELINE] released M7 -> rotation "
              "clearance ");
          SerialDebug.println(clearanceHeightMm, 1);
          armTransferPhase =
              ARM_TRANSFER_WAIT_FINAL_CLEARANCE;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_FINAL_CLEARANCE:
      if (liftMoveFinished()) {
        if (armTransferRing1TrayClearanceMode == 1U) {
          // Ring-1 placement is vertically clear. Restore concurrently with
          // the existing next-pose/retract motion.
          restoreArmTransferRing1Tray();
        } else if (armTransferRing1TrayClearanceMode == 3U &&
                   !armTransferNextStorageCommanded) {
          // The following pickup is ring 1. The current placement is already
          // complete and vertically clear, so park during the existing
          // M5/M6 preposition interval without delaying either transfer.
          commandRing1TrayClearancePosition();
          armTransferNextStorageCommanded = true;
        }
        if (finalStorageDeparturePending) {
          if (fabsf(liftAxis.currentMm - M7_STANDARD_HEIGHT_MM) >
                  ARM_AXIS_POSITION_TOLERANCE_MM ||
              armMotors.isM5Running() || armBaseMotionWatchdogActive) {
            routeFault("Final return clearance invalid");
            break;
          }
          if (!startExtensionToMmWithProfile(
                  M6_STANDARD_EXTENSION_MM,
                  STORAGE_TRAVEL_M6_SPEED_RPM,
                  STORAGE_TRAVEL_M6_ACCELERATION)) {
            break;
          }
          commandStorageServoParkingPosition();
          finalStorageDeparturePending = false;
          storage2ReturnTravelActive = true;
          armTransferPhase = ARM_TRANSFER_COMPLETE;
          break;
        }
        if (armTransferPrepareNextSource &&
            !armTransferUsesReturnProfile() &&
            !armTransferUsesRawProfile() &&
            !armTransferNextStorageCommanded) {
          commandStorageServoPosition(
              static_cast<uint8_t>(
                  armTransferNextStorageSlot));
          armTransferNextStorageDeadlineMs =
              millis() + STORAGE_SERVO_SETTLE_MS;
          armTransferNextStorageCommanded = true;
        }
        const float extensionTargetMm =
            armTransferPrepareNextSource
                ? armTransferNextSourcePose.extensionMm
                : (armTransferReturnToRawViewAtEnd
                       ? RAW_VIEW_EXTENSION_MM
                       : M6_STANDARD_EXTENSION_MM);
        const float angleTargetDegrees =
            armTransferPrepareNextSource
                ? armTransferNextSourcePose
                      .standardFrameAngleDegrees
                : (armTransferReturnToRawViewAtEnd
                       ? 0.0f
                       : armTransferDestinationPose
                             .standardFrameAngleDegrees);
        const bool mappedNextPose =
            armTransferPrepareNextSource &&
            armTransferNextSourceMapped;
        const ArmPose finalPose(
            angleTargetDegrees,
            extensionTargetMm,
            liftAxis.currentMm);
        if (startArmTransferPlanarMove(
            finalPose,
            mappedNextPose,
                armTransferMapDestination,
                true,
                armTransferFinalRotationStarted,
                "release clearance -> next pose")) {
          armTransferPhase =
              ARM_TRANSFER_WAIT_FINAL_RETRACT;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_FINAL_RETRACT:
      if (extensionMoveFinished() &&
          liftMoveFinished() &&
          (!(armTransferPrepareNextSource ||
             armTransferReturnToRawViewAtEnd) ||
           !armMotors.isM5Running())) {
        armTransferDeadlineMs =
            millis() + armTransferTransitionSettleMs();
        armTransferPhase =
            ARM_TRANSFER_WAIT_STANDARD_SETTLE;
      }
      break;

    case ARM_TRANSFER_WAIT_STANDARD_SETTLE:
      if (deadlineReached(armTransferDeadlineMs) &&
          (!armTransferUsesReturnProfile() ||
           (armTransferStorageCommandDueMs != 0UL &&
            deadlineReached(armTransferStorageCommandDueMs))) &&
          (!armTransferPrepareNextSource ||
           armTransferNextStorageCommanded)) {
        if (armTransferPrepareNextSource) {
          armTransferNextSourcePreparedAtEnd = true;
          SerialDebug.println(
              "[XFER] next pickup pose ready");
        }
        armTransferPhase = ARM_TRANSFER_COMPLETE;
      }
      break;
  }
}

bool armTransferFinished() {
  return armTransferPhase == ARM_TRANSFER_COMPLETE;
}

void consumeArmTransferCompletion() {
  armPayloadPresent = false;
  armTransferPhase = ARM_TRANSFER_IDLE;
  rawInitialStorageArrivalOverlapPending = false;
  armTransferM7HandoffStartMs = 0UL;
  armTransferRawM7ToGripperDeadlineUs = 0UL;
  armTransferMapSource = false;
  armTransferMapDestination = false;
  armTransferConcurrentSourcePreparation = false;
  armTransferSourceAlreadyPrepared = false;
  armTransferPrepareNextSource = false;
  armTransferNextSourcePose = ArmPose();
  armTransferNextSourceMapped = false;
  armTransferNextStorageSlot = -1;
  armTransferNextStorageDeadlineMs = 0UL;
  armTransferStorageCommandDueMs = 0UL;
  armTransferNextStorageCommanded = false;
  armTransferNextSourcePreparedAtEnd = false;
  armTransferReturnToRawViewAtEnd = false;
  armTransferSourceRotationStarted = false;
  armTransferDestinationRotationStarted = false;
  armTransferFinalRotationStarted = false;
  armTransferProcessFirstRingReturnTiming = false;
  armTransferDestinationM5FinalBufferPending = false;
  armTransferRing1TrayClearanceMode = 0U;
  armTransferRing1TrayRestoreSlot = -1;
#if GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT > 100U
  armTransferSourceRingPosition = 0U;
#endif
  armTransferMotionProfile =
      arm_transfer::PROFILE_STANDARD;
}

bool nominalRingPose(
    uint8_t ringPosition,
    float lowerMm,
    ArmPose &pose) {
  if (ringPosition != 1U && ringPosition != 3U) {
    routeFault("Endpoint seed requires ring 1 or 3");
    return false;
  }

  pose.standardFrameAngleDegrees =
      ringPosition == 1U
          ? ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES
          : ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES;
  if (pose.standardFrameAngleDegrees <
          RING_SCAN_MINIMUM_ANGLE_DEGREES ||
      pose.standardFrameAngleDegrees >
          RING_SCAN_MAXIMUM_ANGLE_DEGREES) {
    routeFault("Nominal endpoint outside -80..80 scan range");
    return false;
  }
  pose.extensionMm = ENDPOINT_SEARCH_SEED_EXTENSION_MM;
  pose.heightMm = -lowerMm;
  return true;
}

bool planarPointToRingPose(
    uint8_t ringPosition,
    const PlanarPoint &point,
    float lowerMm,
    ArmPose &pose) {
  if (ringPosition < 1U || ringPosition > 3U) {
    routeFault("Invalid ring position for mapped pose");
    return false;
  }
  const float radialDistanceMm =
      hypotf(point.outwardMm, point.leftMm);
  const float angleDegrees =
      atan2f(point.leftMm, point.outwardMm) *
      180.0f / PI_F;
  const float rawExtensionMm =
      radialDistanceMm -
      ARM_PIVOT_TO_GRIPPER_CENTER_MM;
  const float correctedExtensionMm =
      rawExtensionMm -
      arm_config::MAPPED_RING_EXTENSION_REDUCTION_MM;
  const bool ring2 = ringPosition == 2U;
  const float minimumExtensionMm =
      ring2
          ? M6_RING2_MINIMUM_EXTENSION_MM
          : M6_STANDARD_EXTENSION_MM;

  if ((!ring2 &&
       correctedExtensionMm <
           minimumExtensionMm -
               ARM_AXIS_POSITION_TOLERANCE_MM) ||
      correctedExtensionMm >
          M6_MAXIMUM_EXTENSION_MM +
              ARM_AXIS_POSITION_TOLERANCE_MM ||
      angleDegrees <
          RING_TARGET_MINIMUM_ANGLE_DEGREES ||
      angleDegrees >
          RING_TARGET_MAXIMUM_ANGLE_DEGREES) {
    SerialDebug.print(
        "[RING MAP] unreachable xy/r/angle/raw-ext/corrected-ext=");
    SerialDebug.print(point.outwardMm, 2);
    SerialDebug.print(",");
    SerialDebug.print(point.leftMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(radialDistanceMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(angleDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.print(rawExtensionMm, 2);
    SerialDebug.print("/");
    SerialDebug.println(correctedExtensionMm, 2);
    routeFault("Measured ring outside safe arm workspace");
    return false;
  }

  float commandedExtensionMm = correctedExtensionMm;
  if (commandedExtensionMm < minimumExtensionMm) {
    commandedExtensionMm = minimumExtensionMm;
  }
  if (ring2 &&
      correctedExtensionMm < M6_STANDARD_EXTENSION_MM) {
    SerialDebug.print(
        "[RING MAP] ring 2 near-side M6 raw/corrected/command/"
        "physical-margin=");
    SerialDebug.print(rawExtensionMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(correctedExtensionMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(commandedExtensionMm, 2);
    SerialDebug.print("/");
    SerialDebug.println(
        M6_STARTUP_WORKING_ZERO_OFFSET_MM +
            commandedExtensionMm,
        2);
  }

  pose.standardFrameAngleDegrees = angleDegrees;
  pose.extensionMm = commandedExtensionMm;
  pose.heightMm = -lowerMm;
  SerialDebug.print(
      "[RING MAP] M6 visual raw/0mm/command=");
  SerialDebug.print(rawExtensionMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(correctedExtensionMm, 2);
  SerialDebug.print("/");
  SerialDebug.println(commandedExtensionMm, 2);
  return true;
}

bool planarPointToCameraPose(
    const PlanarPoint &point,
    float lowerMm,
    ArmPose &pose,
    bool deferTooFarFault = false) {
  const float radialDistanceMm =
      hypotf(point.outwardMm, point.leftMm);
  const float angleDegrees =
      atan2f(point.leftMm, point.outwardMm) *
      180.0f / PI_F;
  const float extensionMm =
      radialDistanceMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;

  // Preserve the calculated pose when the only problem is insufficient M6
  // reach. The endpoint state machine can then move the chassis forward and
  // rescan instead of either commanding an out-of-range axis target or
  // discarding the measured shortage.
  pose.standardFrameAngleDegrees = angleDegrees;
  pose.extensionMm = extensionMm;
  pose.heightMm = -lowerMm;
  const bool extensionTooFar =
      extensionMm >
      M6_MAXIMUM_EXTENSION_MM + ARM_AXIS_POSITION_TOLERANCE_MM;
  if (extensionMm <
          M6_STANDARD_EXTENSION_MM -
              ARM_AXIS_POSITION_TOLERANCE_MM ||
      extensionTooFar ||
      angleDegrees <
          RING_SCAN_MINIMUM_ANGLE_DEGREES ||
      angleDegrees >
          RING_SCAN_MAXIMUM_ANGLE_DEGREES) {
    SerialDebug.print(
        "[ENDPOINT SERVO] unreachable camera xy/r/angle/ext=");
    SerialDebug.print(point.outwardMm, 2);
    SerialDebug.print(",");
    SerialDebug.print(point.leftMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(radialDistanceMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(angleDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.println(extensionMm, 2);
    // A pure outward-reach shortage is recoverable by approaching with the
    // chassis. All other workspace violations remain immediate faults.
    if (!(deferTooFarFault && extensionTooFar &&
          extensionMm >=
              M6_STANDARD_EXTENSION_MM -
                  ARM_AXIS_POSITION_TOLERANCE_MM &&
          angleDegrees >= RING_SCAN_MINIMUM_ANGLE_DEGREES &&
          angleDegrees <= RING_SCAN_MAXIMUM_ANGLE_DEGREES)) {
      routeFault("Endpoint outside safe camera workspace");
    }
    return false;
  }

  pose.extensionMm =
      extensionMm < M6_STANDARD_EXTENSION_MM
          ? M6_STANDARD_EXTENSION_MM
          : extensionMm;
  return true;
}

bool predictedRing3SearchPose(ArmPose &pose) {
  if (!measuredRingPointValid[1U]) {
    routeFault("Ring 1 is unavailable for direct ring-3 handoff");
    return false;
  }

  const PlanarPoint predictedPoint(
      measuredRingPoints[1U].outwardMm,
      measuredRingPoints[1U].leftMm -
          RING_ENDPOINT_EXPECTED_SPAN_MM);
  if (!planarPointToCameraPose(
          predictedPoint,
          ENDPOINT_FINE_VISION_LOWER_MM,
          pose)) {
    return false;
  }
  const float rawPredictedAngleDegrees =
      pose.standardFrameAngleDegrees;
  const bool predictedPoseInsideRing3Window =
      rawPredictedAngleDegrees >=
          ENDPOINT_RING3_SEARCH_MINIMUM_ANGLE_DEGREES &&
      rawPredictedAngleDegrees <=
          ENDPOINT_RING3_SEARCH_MAXIMUM_ANGLE_DEGREES;
  if (!predictedPoseInsideRing3Window) {
    // A prediction outside the ring-specific safe search window must never be
    // executed. Fall back to the verified nominal seed; vision still obtains
    // and confirms the final ring-3 coordinate independently.
    pose.standardFrameAngleDegrees =
        ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES;
    pose.extensionMm = ENDPOINT_SEARCH_SEED_EXTENSION_MM;
    pose.heightMm = ENDPOINT_VISION_HEIGHT_MM;
  }

  SerialDebug.print(
      "[ENDPOINT HANDOFF] ring1 xy -> predicted ring3 "
      "xy/raw-M5/search-M5/M6/source=");
  SerialDebug.print(measuredRingPoints[1U].outwardMm, 2);
  SerialDebug.print(",");
  SerialDebug.print(measuredRingPoints[1U].leftMm, 2);
  SerialDebug.print(" -> ");
  SerialDebug.print(predictedPoint.outwardMm, 2);
  SerialDebug.print(",");
  SerialDebug.print(predictedPoint.leftMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(rawPredictedAngleDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(pose.standardFrameAngleDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(pose.extensionMm, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      predictedPoseInsideRing3Window ? "predicted" : "nominal-fallback");
  return true;
}

bool ringPose(
    uint8_t ringPosition,
    float lowerMm,
    ArmPose &pose) {
  if (ringPosition < 1U || ringPosition > 3U) {
    routeFault("Invalid ring position");
    return false;
  }
  if (!measuredRingPoseValid[ringPosition]) {
    routeFault("Ring endpoint map is not ready");
    return false;
  }

  pose = measuredRingPoses[ringPosition];
  pose.heightMm = -lowerMm;
  return true;
}

void logTargetPlacementM6Limit(
    uint8_t ringPosition,
    float requestedExtensionMm,
    float correctedExtensionMm) __attribute__((noinline));
void logTargetPlacementM6Limit(
    uint8_t ringPosition,
    float requestedExtensionMm,
    float correctedExtensionMm) {
  SerialDebug.print(
      "[M6 LIMIT] "); // ring/requested/clamped-mm
  SerialDebug.print(ringPosition);
  SerialDebug.write('/');
  SerialDebug.print(requestedExtensionMm, 2);
  SerialDebug.write('/');
  SerialDebug.println(correctedExtensionMm, 2);
}

bool applyTargetPlacementExtraLower(
    ArmPose &pose,
    float extensionReductionMm,
    uint8_t ringPosition,
    bool deferExtensionLimit = false) {
  const float requestedExtensionMm =
      pose.extensionMm - extensionReductionMm;
  float correctedExtensionMm = requestedExtensionMm;
  // 第二次暂存区须先完成所有放置修正，再统一限制最终 M6 目标。
  if (!deferExtensionLimit &&
      requestedExtensionMm < M6_RING2_MINIMUM_EXTENSION_MM) {
    if (ringPosition != 2U) {
      routeFault("Target placement M6 reduction exceeds travel");
      return false;
    }
    correctedExtensionMm = M6_RING2_MINIMUM_EXTENSION_MM;
    logTargetPlacementM6Limit(
        ringPosition, requestedExtensionMm, correctedExtensionMm);
  }
  const float correctedHeightMm =
      pose.heightMm -
      arm_config::TARGET_PLACE_EXTRA_LOWER_MM;
  if (correctedHeightMm <
      M7_MINIMUM_HEIGHT_MM -
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("Target placement extra lower exceeds M7 travel");
    return false;
  }
  SerialDebug.print(
      "[TARGET PLACE M6] mapped/reduction/corrected-mm=");
  SerialDebug.print(pose.extensionMm, 2);
  SerialDebug.print("/-");
  SerialDebug.print(extensionReductionMm, 1);
  SerialDebug.print("/");
  SerialDebug.println(correctedExtensionMm, 2);
  SerialDebug.print(
      "[TARGET PLACE Z] mapped/extra/corrected-mm=");
  SerialDebug.print(pose.heightMm, 2);
  SerialDebug.print("/-");
  SerialDebug.print(
      arm_config::TARGET_PLACE_EXTRA_LOWER_MM,
      1);
  SerialDebug.print("/");
  SerialDebug.println(correctedHeightMm, 2);
  pose.extensionMm = correctedExtensionMm;
  pose.heightMm = correctedHeightMm;
  return true;
}

bool applyRing3ExtensionCorrection(
    ArmPose &pose,
    uint8_t ringPosition) {
  // 正式流程和工位调试共用同一修正。放置和粗加工取回都从不含本修正的
  // 基础姿态调用，因此每个目标只加一次+5 mm，不会累计成+10 mm。
  if (ringPosition != 3U ||
      RING3_EXTENSION_CORRECTION_MM <= 0.0f) {
    return true;
  }

  const float baseExtensionMm = pose.extensionMm;
  const float correctedExtensionMm =
      baseExtensionMm +
      RING3_EXTENSION_CORRECTION_MM;
  if (correctedExtensionMm >
      M6_MAXIMUM_EXTENSION_MM +
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("Ring3 correction travel");
    return false;
  }

  pose.extensionMm = correctedExtensionMm;
  SerialDebug.print(
      "[R3 M6] base/add/target=");
  SerialDebug.print(baseExtensionMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(RING3_EXTENSION_CORRECTION_MM, 1);
  SerialDebug.print("/");
  SerialDebug.println(pose.extensionMm, 2);
  return true;
}

void resetMeasuredRingMap() {
  for (uint8_t ring = 0U; ring < 4U; ++ring) {
    measuredRingPoses[ring] = ArmPose();
    measuredRingPoints[ring] = PlanarPoint();
    measuredRingPointValid[ring] = false;
    measuredRingPoseValid[ring] = false;
    measuredRingRadiiPixels[ring] = 0U;
    measuredRingConfidence[ring] = 0U;
    measuredRingHeadingsDegrees[ring] = 0.0f;
  }
}

bool endpointVisionToPlanarPoint(
    const ArmPose &scanPose,
    int16_t imageX,
    int16_t imageY,
    uint16_t radiusPixels,
    PlanarPoint &point,
    float coordinateMmPerPixel = 0.0f) {
  if (radiusPixels <
          RING_ENDPOINT_MINIMUM_RADIUS_PIXELS ||
      radiusPixels >
          RING_ENDPOINT_MAXIMUM_RADIUS_PIXELS) {
    SerialDebug.print(
        "[ENDPOINT MAP] rejected radius=");
    SerialDebug.println(radiusPixels);
    return false;
  }

  // A positive override changes only the coordinate conversion.  The radius
  // above remains the uncorrected value used by the endpoint validity gate.
  const float mmPerPixel =
      coordinateMmPerPixel > 0.0f
          ? coordinateMmPerPixel
          : RING_PHYSICAL_RADIUS_MM /
                static_cast<float>(radiusPixels);
  const float pixelDeltaX =
      static_cast<float>(imageX - IMAGE_CENTER_X);
  const float pixelDeltaY =
      static_cast<float>(imageY - IMAGE_CENTER_Y);
  const float localRadialMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      scanPose.extensionMm -
      pixelDeltaY * mmPerPixel;
  const float localLeftMm =
      -pixelDeltaX * mmPerPixel;
  const float angleRadians =
      scanPose.standardFrameAngleDegrees *
      PI_F / 180.0f;
  const float cosine = cosf(angleRadians);
  const float sine = sinf(angleRadians);

  point.outwardMm =
      localRadialMm * cosine -
      localLeftMm * sine;
  point.leftMm =
      localRadialMm * sine +
      localLeftMm * cosine;

  SerialDebug.print(
      "[ENDPOINT MAP] image-xy/r/scale, scan-M5/M6/M7, arm-xy=");
  SerialDebug.print(imageX);
  SerialDebug.print(",");
  SerialDebug.print(imageY);
  SerialDebug.print("/");
  SerialDebug.print(radiusPixels);
  SerialDebug.print(", ");
  SerialDebug.print(mmPerPixel, 4);
  SerialDebug.print(", ");
  SerialDebug.print(
      scanPose.standardFrameAngleDegrees,
      2);
  SerialDebug.print("/");
  SerialDebug.print(scanPose.extensionMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(scanPose.heightMm, 2);
  SerialDebug.print(", ");
  SerialDebug.print(point.outwardMm, 2);
  SerialDebug.print(",");
  SerialDebug.println(point.leftMm, 2);
  return true;
}

bool endpointServoTargetPose(
    const ArmPose &currentPose,
    const PlanarPoint &measuredPoint,
    bool fineStage,
    ArmPose &targetPose,
    float &measuredCorrectionMm,
    float &commandedCorrectionMm) {
  const float cameraRadiusMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      currentPose.extensionMm;
  const float cameraAngleRadians =
      currentPose.standardFrameAngleDegrees *
      PI_F / 180.0f;
  const PlanarPoint currentCameraPoint(
      cameraRadiusMm * cosf(cameraAngleRadians),
      cameraRadiusMm * sinf(cameraAngleRadians));

  const float errorOutwardMm =
      measuredPoint.outwardMm -
      currentCameraPoint.outwardMm;
  const float errorLeftMm =
      measuredPoint.leftMm -
      currentCameraPoint.leftMm;
  measuredCorrectionMm =
      hypotf(errorOutwardMm, errorLeftMm);

  const bool farDistance =
      measuredCorrectionMm >
      ENDPOINT_SERVO_FAR_DISTANCE_THRESHOLD_MM;
  const bool mediumDistance =
      !farDistance &&
      measuredCorrectionMm >=
          ENDPOINT_SERVO_MEDIUM_DISTANCE_THRESHOLD_MM;
  const float gain =
      fineStage
          ? (farDistance
                 ? ENDPOINT_FINE_FAR_DISTANCE_GAIN
                 : (mediumDistance
                        ? ENDPOINT_FINE_MEDIUM_DISTANCE_GAIN
                        : ENDPOINT_FINE_SERVO_GAIN))
          : (farDistance
                 ? ENDPOINT_COARSE_FAR_DISTANCE_GAIN
                 : (mediumDistance
                        ? ENDPOINT_COARSE_MEDIUM_DISTANCE_GAIN
                        : ENDPOINT_COARSE_SERVO_GAIN));
  const float maximumCorrectionMm =
      fineStage
          ? ENDPOINT_FINE_MAXIMUM_CORRECTION_MM
          : ENDPOINT_COARSE_MAXIMUM_CORRECTION_MM;
  float commandOutwardMm = errorOutwardMm * gain;
  float commandLeftMm = errorLeftMm * gain;
  commandedCorrectionMm =
      hypotf(commandOutwardMm, commandLeftMm);
  if (commandedCorrectionMm > maximumCorrectionMm) {
    const float limitScale =
        maximumCorrectionMm /
        commandedCorrectionMm;
    commandOutwardMm *= limitScale;
    commandLeftMm *= limitScale;
    commandedCorrectionMm = maximumCorrectionMm;
  }

  const PlanarPoint requestedCameraPoint(
      currentCameraPoint.outwardMm +
          commandOutwardMm,
      currentCameraPoint.leftMm +
          commandLeftMm);
  return planarPointToCameraPose(
      requestedCameraPoint,
      -currentPose.heightMm,
      targetPose,
      true);
}

bool buildMeasuredRingMap() {
  if (!measuredRingPointValid[1U] ||
      !measuredRingPointValid[3U]) {
    routeFault("Endpoint map missing ring 1 or 3");
    return false;
  }

  const float deltaOutwardMm =
      measuredRingPoints[3U].outwardMm -
      measuredRingPoints[1U].outwardMm;
  const float deltaLeftMm =
      measuredRingPoints[3U].leftMm -
      measuredRingPoints[1U].leftMm;
  const float endpointSpanMm =
      hypotf(deltaOutwardMm, deltaLeftMm);
  const float spanErrorMm =
      fabsf(
          endpointSpanMm -
          RING_ENDPOINT_EXPECTED_SPAN_MM);
  const float midpointOutwardMm =
      0.5f *
      (measuredRingPoints[1U].outwardMm +
       measuredRingPoints[3U].outwardMm);
  const float midpointLeftMm =
      0.5f *
      (measuredRingPoints[1U].leftMm +
       measuredRingPoints[3U].leftMm);
  const float endpointAxisAngleDegrees =
      atan2f(deltaLeftMm, deltaOutwardMm) *
      180.0f / PI_F;

  const uint16_t minimumRadius =
      measuredRingRadiiPixels[1U] <
              measuredRingRadiiPixels[3U]
          ? measuredRingRadiiPixels[1U]
          : measuredRingRadiiPixels[3U];
  const uint16_t maximumRadius =
      measuredRingRadiiPixels[1U] >
              measuredRingRadiiPixels[3U]
          ? measuredRingRadiiPixels[1U]
          : measuredRingRadiiPixels[3U];
  const float radiusRatio =
      static_cast<float>(maximumRadius) /
      static_cast<float>(
          minimumRadius == 0U ? 1U : minimumRadius);
  const float scanHeadingDeltaDegrees =
      fabsf(
          wrapDeltaDegrees(
              measuredRingHeadingsDegrees[3U] -
              measuredRingHeadingsDegrees[1U]));
  const float mapHeadingDriftDegrees =
      fabsf(
          wrapDeltaDegrees(
              currentRouteCounterClockwiseHeading() -
              endpointMapLockedHeadingDegrees));

  SerialDebug.print(
      "[RING MAP] span/error/axisAngle/"
      "radiusRatio/scanHeading/mapDrift=");
  SerialDebug.print(endpointSpanMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(spanErrorMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(endpointAxisAngleDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(radiusRatio, 3);
  SerialDebug.print("/");
  SerialDebug.print(scanHeadingDeltaDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.println(mapHeadingDriftDegrees, 2);

  if (spanErrorMm >
          RING_ENDPOINT_MAXIMUM_SPAN_ERROR_MM ||
      radiusRatio >
          RING_ENDPOINT_MAXIMUM_RADIUS_RATIO ||
      scanHeadingDeltaDegrees >
          RING_ENDPOINT_MAXIMUM_SCAN_HEADING_DELTA_DEGREES ||
      mapHeadingDriftDegrees >
          RING_MAP_MAXIMUM_HEADING_DRIFT_DEGREES) {
    routeFault("Endpoint pair geometry rejected");
    return false;
  }
  if (spanErrorMm >
      RING_ENDPOINT_WARNING_SPAN_ERROR_MM) {
    SerialDebug.println(
        "[RING MAP] warning: endpoint span needs "
        "real-board calibration");
  }

  measuredRingPoints[2U].outwardMm =
      midpointOutwardMm;
  measuredRingPoints[2U].leftMm =
      midpointLeftMm;
  SerialDebug.println(
      "[LOCAL RING MAP] field XY ignored: observed ring 1/3 "
      "define arm-local ring 2 midpoint");

  for (uint8_t ring = 1U; ring <= 3U; ++ring) {
    ArmPose pose;
    if (!planarPointToRingPose(
            ring,
            measuredRingPoints[ring],
            0.0f,
            pose)) {
      return false;
    }
    measuredRingPoses[ring] = pose;
    measuredRingPoseValid[ring] = true;

    SerialDebug.print(
        "[RING MAP] ring xy -> M5/M6 ");
    SerialDebug.print(ring);
    SerialDebug.print(": ");
    SerialDebug.print(
        measuredRingPoints[ring].outwardMm,
        2);
    SerialDebug.print(",");
    SerialDebug.print(
        measuredRingPoints[ring].leftMm,
        2);
    SerialDebug.print(" -> ");
    SerialDebug.print(
        measuredRingPoses[ring]
            .standardFrameAngleDegrees,
        2);
    SerialDebug.print("/");
    SerialDebug.println(
        measuredRingPoses[ring].extensionMm,
        2);
  }
  return true;
}

enum RawTargetPoseResult {
  RAW_TARGET_POSE_VALID,
  RAW_TARGET_WAIT_TOO_NEAR,
  RAW_TARGET_WAIT_TOO_FAR,
  RAW_TARGET_WAIT_OUTSIDE_ANGLE
};

float rawMillimetersPerPixel(float pixelRadius) {
  float blend =
      pixelRadius /
      RAW_PIXEL_MAPPING_BLEND_END_RADIUS_PIXELS;
  if (blend < 0.0f) {
    blend = 0.0f;
  } else if (blend > 1.0f) {
    blend = 1.0f;
  }
  return RAW_NEAR_MM_PER_PIXEL +
         (RAW_FAR_MM_PER_PIXEL -
          RAW_NEAR_MM_PER_PIXEL) *
             blend;
}

RawTargetPoseResult rawTargetPoseFromCameraPose(
    float pixelX,
    float pixelY,
    const ArmPose &cameraPose,
    ArmPose &pose) {

  const float pixelDeltaX =
      pixelX - static_cast<float>(IMAGE_CENTER_X);
  const float pixelDeltaY =
      pixelY - static_cast<float>(IMAGE_CENTER_Y);
  const float pixelRadius =
      hypotf(pixelDeltaX, pixelDeltaY);

  const float rawMmPerPixel =
      rawMillimetersPerPixel(pixelRadius);
  const float localLeftMm =
      -pixelDeltaX * rawMmPerPixel;
  const float cameraRadiusMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      cameraPose.extensionMm;
  const float localOutwardMm =
      cameraRadiusMm +
      pixelDeltaY *
          rawMmPerPixel *
          IMAGE_Y_TO_ARM_OUTWARD_SIGN;
  const float cameraAngleRadians =
      cameraPose.standardFrameAngleDegrees *
      PI_F / 180.0f;
  const float targetOutwardMm =
      localOutwardMm * cosf(cameraAngleRadians) -
      localLeftMm * sinf(cameraAngleRadians);
  const float targetLeftMm =
      localOutwardMm * sinf(cameraAngleRadians) +
      localLeftMm * cosf(cameraAngleRadians);
  const float targetRadiusMm =
      hypotf(targetOutwardMm, targetLeftMm);
  const float originalExtensionMm =
      targetRadiusMm -
      ARM_PIVOT_TO_GRIPPER_CENTER_MM;
  const float extensionMm =
      originalExtensionMm -
      RAW_PICK_EXTENSION_REDUCTION_MM;
  const float standardFrameAngleDegrees =
      atan2f(targetLeftMm, targetOutwardMm) *
      180.0f / PI_F;

#if GONGCHUANG_RAW_VERBOSE_VISION_CALIBRATION
  SerialDebug.print(
      "[RAW VISION CAL] xy/dxy/r/scale/left/out/target-r/M5/"
      "M6-original/M6-corrected=");
  SerialDebug.print(pixelX, 1);
  SerialDebug.print(",");
  SerialDebug.print(pixelY, 1);
  SerialDebug.print("/");
  SerialDebug.print(pixelDeltaX, 1);
  SerialDebug.print(",");
  SerialDebug.print(pixelDeltaY, 1);
  SerialDebug.print("/");
  SerialDebug.print(pixelRadius, 2);
  SerialDebug.print("/");
  SerialDebug.print(rawMmPerPixel, 4);
  SerialDebug.print("/");
  SerialDebug.print(localLeftMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(localOutwardMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(targetRadiusMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(standardFrameAngleDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(originalExtensionMm, 2);
  SerialDebug.print("/");
  SerialDebug.println(extensionMm, 2);
#endif

  if (extensionMm <
      M6_RAW_PICK_MINIMUM_EXTENSION_MM -
          ARM_AXIS_POSITION_TOLERANCE_MM) {
#if GONGCHUANG_RAW_VERBOSE_VISION_CALIBRATION
    SerialDebug.print(
        "[RAW WAIT REACH] target too near, radius/ext=");
    SerialDebug.print(targetRadiusMm, 2);
    SerialDebug.print("/");
    SerialDebug.println(extensionMm, 2);
#endif
    return RAW_TARGET_WAIT_TOO_NEAR;
  }
  if (extensionMm >
      M6_MAXIMUM_EXTENSION_MM +
          ARM_AXIS_POSITION_TOLERANCE_MM) {
#if GONGCHUANG_RAW_VERBOSE_VISION_CALIBRATION
    SerialDebug.print(
        "[RAW WAIT REACH] target too far, radius/ext=");
    SerialDebug.print(targetRadiusMm, 2);
    SerialDebug.print("/");
    SerialDebug.println(extensionMm, 2);
#endif
    return RAW_TARGET_WAIT_TOO_FAR;
  }

  pose.standardFrameAngleDegrees =
      standardFrameAngleDegrees;
  pose.extensionMm =
      extensionMm < M6_RAW_PICK_MINIMUM_EXTENSION_MM
          ? M6_RAW_PICK_MINIMUM_EXTENSION_MM
          : extensionMm;
  pose.heightMm = -RAW_PICK_LOWER_MM;
  return RAW_TARGET_POSE_VALID;
}

RawTargetPoseResult rawTargetPose(
    float pixelX,
    float pixelY,
    ArmPose &pose) {
  return rawTargetPoseFromCameraPose(
      pixelX,
      pixelY,
      ArmPose(0.0f, RAW_VIEW_EXTENSION_MM, M7_STANDARD_HEIGHT_MM),
      pose);
}

enum WorkActionKind {
  WORK_ACTION_NONE,
  WORK_ACTION_RAW,
  WORK_ACTION_PROCESS,
  WORK_ACTION_STORAGE
};

enum WorkActionPhase {
  WORK_PHASE_IDLE,
  WORK_PHASE_PREPARE,
  WORK_PHASE_RAW_WAIT_RESULT,
  WORK_PHASE_RAW_WAIT_EXPECTED_SLOT,
  WORK_PHASE_RAW_WAIT_SEARCH_MOVE,
  WORK_PHASE_RAW_WAIT_TRAVEL_LINEAR_ZERO,
  WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO,
  WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO_SETTLE,
  WORK_PHASE_ENDPOINT_WAIT_PRE_SCAN_HEADING,
  WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE,
  WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE_SETTLE,
  WORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION,
  WORK_PHASE_ENDPOINT_WAIT_ARM_LOWER,
  WORK_PHASE_ENDPOINT_WAIT_COORDINATE,
  WORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE,
  WORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE_SETTLE,
  WORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION,
  WORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE,
  WORK_PHASE_ENDPOINT_REACH_DELAY,
  WORK_PHASE_ENDPOINT_REACH_MOVE,
  WORK_PHASE_STORAGE_COLOR_WAIT_EXTENSION,
  WORK_PHASE_STORAGE_COLOR_WAIT_COORDINATE,
  WORK_PHASE_CIRCLE_WAIT_ARM_LOWER,
  WORK_PHASE_CIRCLE_WAIT_COORDINATE,
  WORK_PHASE_CIRCLE_WAIT_CHASSIS,
  WORK_PHASE_CIRCLE_WAIT_ARM_STANDARD,
  WORK_PHASE_CIRCLE_WAIT_POST_VISION_HEADING,
  WORK_PHASE_START_UNLOAD,
  WORK_PHASE_START_RELOAD,
  WORK_PHASE_WAIT_TRANSFER,
  WORK_PHASE_WAIT_STORAGE_SERVO,
  WORK_PHASE_WAIT_STORAGE1_RAW2_HANDOFF,
  WORK_PHASE_WAIT_STORAGE_PARK,
  WORK_PHASE_WAIT_DEPARTURE_LOCK,
  WORK_PHASE_START_RESTORE,
  WORK_PHASE_WAIT_RESTORE
};

enum TransferPurpose {
  TRANSFER_PURPOSE_NONE,
  TRANSFER_PURPOSE_RAW_TO_CONTAINER,
  TRANSFER_PURPOSE_CONTAINER_TO_RING,
  TRANSFER_PURPOSE_RING_TO_CONTAINER
};

WorkActionKind activeWorkAction = WORK_ACTION_NONE;
WorkActionPhase workActionPhase = WORK_PHASE_IDLE;
TransferPurpose activeTransferPurpose = TRANSFER_PURPOSE_NONE;
uint8_t activePlacementRingPosition = 0U;
bool endpointDirectContainerPickupPending = false;
bool endpointInitialStorageCommanded = false;
bool containerPickupPrepositionedPending = false;
bool ringPickupPrepositionedPending = false;
uint8_t workRoundIndex = 0U;
uint8_t workItemIndex = 0U;
uint32_t workActionStartMs = 0UL;
uint32_t workVisionRequestStartMs = 0UL;
uint8_t workVisionRetryCount = 0U;
uint32_t workStorageServoDeadlineMs = 0UL;
uint32_t workLastMaixSequence = 0UL;
MotorPulses visualCorrectionAccumulator;
float visualCorrectionForwardMm = 0.0f;
float visualCorrectionLeftMm = 0.0f;
uint8_t visualCorrectionMoveCount = 0U;
uint8_t activeEndpointScanRing = 0U;
uint8_t endpointReachRecoveryMoves = 0U;
float endpointReachRecoveryForwardMm = 0.0f;
float endpointReachRecoveryLeftMm = 0.0f;
bool endpointReachRecoveryRestoreOnDeparture = false;
ArmPose activeEndpointScanPose;
bool endpointFineVisionActive = false;
uint8_t activeEndpointServoMoveCount = 0U;
uint8_t activeEndpointStageServoMoveCount = 0U;
uint8_t endpointSearchFallbackMoveCount = 0U;
float endpointSearchBaseAngleDegrees = 0.0f;
uint8_t endpointCenteredConfirmationCount = 0U;
uint32_t endpointLocalSettleDeadlineMs = 0UL;
uint32_t endpointMapStartMs = 0UL;
uint32_t endpointMapCompleteMs = 0UL;
uint32_t endpointScanStartMs[4] = {
    0UL, 0UL, 0UL, 0UL};
uint32_t endpointScanElapsedMs[4] = {
    0UL, 0UL, 0UL, 0UL};
uint8_t endpointServoMoveCounts[4] = {
    0U, 0U, 0U, 0U};
float endpointFinalCenterErrorsPixels[4] = {
    0.0f, 0.0f, 0.0f, 0.0f};

float processRingReturnPickExtraLowerMm() {
  return arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM +
         (workRoundIndex == 0U
               ? arm_config::
                     PROCESS_ROUND1_RING_RETURN_PICK_ADDITIONAL_LOWER_MM
               : arm_config::
                     PROCESS_ROUND2_RING_RETURN_PICK_ADDITIONAL_LOWER_MM);
}

void resetEndpointCenteredConfirmationWindow() {
  endpointCenteredConfirmationCount = 0U;
}

// Sequence -> original QR/tray slot; never mutate task colors/positions.
uint8_t storage2ItemOrder[3] = {0xFFU, 0xFFU, 0xFFU};
uint8_t storage2ScanIndex = 0U;
float storage2ScanExtensionMm = 0.0f;
bool storage2ScanExtensionCaptured = false;
uint8_t storageColorExpected = 0U;
uint8_t storageColorTargetRing = 0U;
ArmPose storageColorScanPose;
uint32_t storageColorAccuracyWindowStartMs = 0UL;
int32_t storageColorAccuracySumX = 0;
int32_t storageColorAccuracySumY = 0;
uint8_t storageColorAccuracySampleCount = 0U;
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
// Debug-only counters explain why a 500 ms STORAGE2 observation window was
// rejected. They do not participate in target selection or motion control.
uint16_t storage2WindowFreshFrameCount = 0U;
uint16_t storage2WindowWrongColorCount = 0U;
uint16_t storage2WindowOutsideRoiCount = 0U;
uint32_t storage2WindowInitialLatestSequence = 0UL;
#endif

enum StorageTravelPreparationPhase {
  STORAGE_TRAVEL_PREP_IDLE,
  STORAGE_TRAVEL_PREP_WAIT_M7_ONLY,
  STORAGE_TRAVEL_PREP_WAIT_ROUND1_M7_HANDOFF,
  STORAGE_TRAVEL_PREP_WAIT_ROUND1_PARALLEL,
  STORAGE_TRAVEL_PREP_WAIT_M7_HEAD_START,
  STORAGE_TRAVEL_PREP_WAIT_M5_M7,
  STORAGE_TRAVEL_PREP_WAIT_M6,
  STORAGE_TRAVEL_PREP_COMPLETE
};

StorageTravelPreparationPhase storageTravelPreparationPhase =
    STORAGE_TRAVEL_PREP_IDLE;
uint8_t storageTravelPreparationRound = 0U;
uint32_t storageTravelM5ZeroHandoffDeadlineUs = 0UL;
uint32_t storageTravelM5StartDeadlineMs = 0UL;

enum Storage1Raw2TravelPhase {
  STORAGE1_RAW2_TRAVEL_IDLE,
  STORAGE1_RAW2_TRAVEL_RUNNING,
  STORAGE1_RAW2_TRAVEL_COMPLETE
};

Storage1Raw2TravelPhase storage1Raw2TravelPhase =
    STORAGE1_RAW2_TRAVEL_IDLE;
uint32_t storage1Raw2HandoffDeadlineUs = 0UL;

uint8_t rawFilledSlotMask = 0U;
uint8_t rawCollectedCount = 0U;
uint8_t rawPendingSlotIndex = 0U;
uint8_t rawPendingColor = 0U;
ArmPose rawPendingSourcePose;
// Legacy mode returns M5/M6 directly to this remembered pickup pose. New mode
// keeps it as the latest tracked pickup target and returns to the active A/B
// scan pose so the next item always receives a fresh coordinate solution.
bool rawPickupPrepositionedPending = false;

#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
enum RawSearchZone : uint8_t {
  RAW_SEARCH_ZONE_A = 0U,
  RAW_SEARCH_ZONE_B_CLOCKWISE = 1U,
  RAW_SEARCH_ZONE_B_COUNTERCLOCKWISE = 2U
};

RawSearchZone rawSearchZone = RAW_SEARCH_ZONE_A;
RawSearchZone rawSearchMoveTargetZone = RAW_SEARCH_ZONE_A;
uint32_t rawNoTargetDeadlineMs = 0UL;
uint32_t rawCurrentNoTargetAllowanceMs =
    RAW_INITIAL_A_NO_TARGET_MS;
bool rawChassisAtB = false;
bool rawFinishAfterSearchMove = false;
bool rawTrackingActive = false;
#endif

bool rawTravelM5ZeroPending = false;
uint32_t processM5ZeroSettleDeadlineUs = 0UL;
uint32_t workDepartureLastImuReceiveMs = 0UL;

// PROCESS2 can qualify the normal 320 ms departure-heading window while M7
// and the autonomous tray servo are still moving. No wheel command is issued
// by this observer. A fresh in-tolerance frame after M7 reaches zero is still
// mandatory; otherwise departure falls back to the full correction lock.
bool process2DepartureImuPrelockReady = false;
uint32_t process2DepartureImuLastFrameSequence = 0UL;
uint32_t process2DepartureM7ArrivalFrameSequence = 0UL;

void resetProcess2DepartureImuPrelock() {
  process2DepartureImuPrelockReady = false;
  process2DepartureImuLastFrameSequence = 0UL;
  process2DepartureM7ArrivalFrameSequence = 0UL;
}

uint32_t circleStableStartMs = 0UL;
uint32_t circleLastStableSampleMs = 0UL;
uint8_t circleStableSampleCount = 0U;

void resetCircleStabilityWindow() {
  circleStableStartMs = 0UL;
  circleLastStableSampleMs = 0UL;
  circleStableSampleCount = 0U;
}

int8_t rawStorageSlotForColor(uint8_t color) {
  for (uint8_t slot = 0U; slot < 3U; ++slot) {
    if (taskColors[workRoundIndex][slot] == color) {
      return static_cast<int8_t>(slot);
    }
  }
  return -1;
}

bool processItemIndexForSequence(
    uint8_t sequenceIndex,
    uint8_t &itemIndex) {
  if (sequenceIndex >= 3U) {
    routeFault("Process transfer sequence index overflow");
    return false;
  }
  const uint8_t destinationRing =
      taskPositions[workRoundIndex][sequenceIndex];
  if (destinationRing < 1U || destinationRing > 3U) {
    routeFault("Process QR destination ring is invalid");
    return false;
  }
  itemIndex = sequenceIndex;
  return true;
}

bool startInitialStorageServoHeadStart() {
  if (endpointInitialStorageCommanded) {
    return true;
  }

  uint8_t initialStorageSlot = 0U;
  if (activeWorkAction == WORK_ACTION_PROCESS) {
    if (!processItemIndexForSequence(
            0U,
            initialStorageSlot)) {
      return false;
    }
  } else if (
      activeWorkAction == WORK_ACTION_STORAGE &&
      workRoundIndex == 0U) {
    // STORAGE1 always starts by taking the first item from tray slot 0.
    // M5/M7 use the same endpoint-preparation state as PROCESS. Launch the
    // tray at the start of the 60 ms gripper gate; its 510 ms arrival gate
    // remains deferred until the first real pickup.
    initialStorageSlot = 0U;
  } else {
    // RAW has its own slot preparation and STORAGE2 has its direct color
    // entry.  Neither may inherit this endpoint-map parallel command.
    return true;
  }
  commandStorageServoPosition(initialStorageSlot);
  workStorageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  endpointInitialStorageCommanded = true;
  SerialDebug.print(
      activeWorkAction == WORK_ACTION_PROCESS
          ? "[PROCESS ENTRY HEAD START] initial storage slot "
          : "[STORAGE1 ENTRY HEAD START] initial storage slot ");
  SerialDebug.print(initialStorageSlot);
  SerialDebug.println(
      " starts about 60 ms before unchanged-time M7/M5; "
      "existing arrival gates unchanged");
  return true;
}

uint8_t storageRingForItem(
    uint8_t roundIndex,
    uint8_t itemIndex) {
  if (itemIndex >= 3U) {
    routeFault("Storage item out of range");
    return 0U;
  }
  return roundIndex == 0U
      ? taskPositions[0][itemIndex]
      : static_cast<uint8_t>(itemIndex + 1U);
}

bool secondStorageColorWorkflowActive() {
  return activeWorkAction == WORK_ACTION_STORAGE &&
         workRoundIndex == 1U;
}

#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
const char *storage2ColorName(uint8_t color) {
  switch (color) {
    case MAIXCAM_RED_REQUEST:
      return "RED";
    case MAIXCAM_YELLOW_REQUEST:
      return "YELLOW";
    case MAIXCAM_BLUE_REQUEST:
      return "BLUE";
    case MAIXCAM_GREEN_REQUEST:
      return "GREEN";
    default:
      return "INVALID";
  }
}

void printStorage2VerboseMap(const char *event) {
  SerialDebug.print("[S2DBG MAPSTATE] event=");
  SerialDebug.print(event);
  for (uint8_t ring = 1U; ring <= 3U; ++ring) {
    SerialDebug.print(" r");
    SerialDebug.print(ring);
    SerialDebug.print("=pose:");
    SerialDebug.print(measuredRingPoseValid[ring] ? 1 : 0);
    SerialDebug.print(",slot:");
    SerialDebug.print(
        static_cast<unsigned int>(storage2ItemOrder[ring - 1U]));
  }
  SerialDebug.println();
}

void printStorage2VerboseFaultSnapshot(const char *reason) {
  const gongchuang::MaixCamClient::Coordinate &latest = maixCam.latest();
  SerialDebug.print("[S2DBG FAULT] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" reason=");
  SerialDebug.print(reason);
  SerialDebug.print(" program/action/round/phase=");
  SerialDebug.print(static_cast<unsigned int>(programState));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(activeWorkAction));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(workRoundIndex));
  SerialDebug.print('/');
  SerialDebug.println(static_cast<unsigned int>(workActionPhase));

  SerialDebug.print("[S2DBG STATE] scan/ring/expected/samples/window-ms=");
  SerialDebug.print(static_cast<unsigned int>(storage2ScanIndex));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorExpected));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorAccuracySampleCount));
  SerialDebug.print('/');
  SerialDebug.println(
      storageColorAccuracyWindowStartMs == 0UL
          ? 0UL
          : millis() - storageColorAccuracyWindowStartMs);

  SerialDebug.print("[S2DBG ARM] M5/M6cur/M6target/M6active/M6fault/");
  SerialDebug.print("M7cur/M7target/M7active/M7fault=");
  SerialDebug.print(currentArmBaseStandardFrameDegrees(), 2);
  SerialDebug.print('/');
  SerialDebug.print(extensionAxis.currentMm, 2);
  SerialDebug.print('/');
  SerialDebug.print(extensionAxis.targetMm, 2);
  SerialDebug.print('/');
  SerialDebug.print(extensionAxis.active ? 1 : 0);
  SerialDebug.print('/');
  SerialDebug.print(extensionAxis.fault ? 1 : 0);
  SerialDebug.print('/');
  SerialDebug.print(liftAxis.currentMm, 2);
  SerialDebug.print('/');
  SerialDebug.print(liftAxis.targetMm, 2);
  SerialDebug.print('/');
  SerialDebug.print(liftAxis.active ? 1 : 0);
  SerialDebug.print('/');
  SerialDebug.println(liftAxis.fault ? 1 : 0);

  SerialDebug.print(
      "[S2DBG CAM] requested/latestSeq/requestSeq/mode/color/x/y/");
  SerialDebug.print("metric/confidence/age-ms=");
  SerialDebug.print(static_cast<unsigned int>(maixCam.requestedMode()));
  SerialDebug.print('/');
  SerialDebug.print(latest.sequence);
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(latest.requestSequence));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(latest.mode));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(latest.targetId));
  SerialDebug.print('/');
  SerialDebug.print(latest.x);
  SerialDebug.print('/');
  SerialDebug.print(latest.y);
  SerialDebug.print('/');
  SerialDebug.print(latest.metric);
  SerialDebug.print('/');
  SerialDebug.print(latest.confidence);
  SerialDebug.print('/');
  SerialDebug.println(
      latest.receivedMs == 0UL ? 0UL : millis() - latest.receivedMs);
  printStorage2VerboseMap("fault");
}

void printStorage2VerboseWindowResult(
    const char *result,
    uint32_t elapsedMs) {
  const gongchuang::MaixCamClient::Coordinate &latest = maixCam.latest();
  SerialDebug.print("[S2DBG RESULT] result=");
  SerialDebug.print(result);
  SerialDebug.print(" elapsed/ring/expected/name/requested/frames/");
  SerialDebug.print("accepted/wrong/roi-reject/seq-start/seq-now/last-age-ms=");
  SerialDebug.print(elapsedMs);
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorExpected));
  SerialDebug.print('/');
  SerialDebug.print(storage2ColorName(storageColorExpected));
  SerialDebug.print('/');
  // stopMaixRequest() may already have switched the client back to mode 0;
  // report the color mode that governed this completed window.
  SerialDebug.print(static_cast<unsigned int>(storageColorExpected));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storage2WindowFreshFrameCount));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorAccuracySampleCount));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storage2WindowWrongColorCount));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storage2WindowOutsideRoiCount));
  SerialDebug.print('/');
  SerialDebug.print(storage2WindowInitialLatestSequence);
  SerialDebug.print('/');
  SerialDebug.print(latest.sequence);
  SerialDebug.print('/');
  SerialDebug.println(
      latest.receivedMs == 0UL ? 0UL : millis() - latest.receivedMs);

  if (storage2WindowFreshFrameCount == 0U) {
    SerialDebug.println(
        "[S2DBG HINT] no valid coordinate reached STM32: check MaixCAM "
        "target detection output first, then UART TX/RX/GND if MaixCAM says sent");
  } else if (storage2WindowOutsideRoiCount > 0U &&
             storageColorAccuracySampleCount == 0U) {
    SerialDebug.println(
        "[S2DBG HINT] target color was returned but remained outside "
        "the center-100px ROI; inspect scan pose/arm calibration");
  } else if (storage2WindowWrongColorCount > 0U &&
             storageColorAccuracySampleCount == 0U) {
    SerialDebug.println(
        "[S2DBG HINT] received color did not match the fixed ring-color "
        "map; verify the state-3 physical color arrangement");
  }
}
#endif

bool processOrStorage1RingNetCorrectionApplies(uint8_t ringPosition) {
  return ringPosition >= 1U && ringPosition <= 3U &&
         (activeWorkAction == WORK_ACTION_PROCESS ||
          (activeWorkAction == WORK_ACTION_STORAGE &&
           workRoundIndex == 0U));
}

bool storage1Ring1PlacementExtraCorrectionApplies(
    uint8_t ringPosition) {
  return activeWorkAction == WORK_ACTION_STORAGE &&
         workRoundIndex == 0U &&
         ringPosition == 1U;
}

bool workstationRingNetCorrectionApplies(uint8_t ringPosition) {
  return processOrStorage1RingNetCorrectionApplies(ringPosition) ||
         (ringPosition >= 1U && ringPosition <= 3U &&
          secondStorageColorWorkflowActive());
}

bool applyRingPositionPoseCorrection(
    ArmPose &pose,
    uint8_t ringPosition) {
  if (!workstationRingNetCorrectionApplies(ringPosition)) {
    return applyRing3ExtensionCorrection(pose, ringPosition);
  }
  // STORAGE2 retains its existing ring-3 +5 mm adjustment and then receives
  // the same exact return net correction as PROCESS and STORAGE1.
  if (secondStorageColorWorkflowActive() &&
      !applyRing3ExtensionCorrection(pose, ringPosition)) {
    return false;
  }
  pose.standardFrameAngleDegrees +=
      WORKSTATION_RING_RETURN_M5_NET_CCW_DEGREES[
          ringPosition - 1U];
  pose.extensionMm +=
      WORKSTATION_RING_RETURN_M6_NET_EXTENSION_MM[
          ringPosition - 1U];
  return true;
}

bool applyRingPlacementPoseCorrection(
    ArmPose &pose,
    uint8_t ringPosition) {
  if (!workstationRingNetCorrectionApplies(ringPosition)) {
    return applyRing3ExtensionCorrection(pose, ringPosition);
  }
  const bool storage2Placement = secondStorageColorWorkflowActive();
  if (storage2Placement) {
    // 此处的 +5 mm 还不是最终目标，后面仍有 0.90 倍率和净修正。
    // 第二次暂存区只在全部修正后限幅，其他工位仍使用原有检查函数。
    if (ringPosition == 3U) {
      pose.extensionMm += RING3_EXTENSION_CORRECTION_MM;
    }
    // Preserve the existing STORAGE2 chain. Apply the shared net afterwards,
    // so its millimetre value remains a direct offset and is not reduced again
    // by the outer-position 0.90 multiplier.
    if (ringPosition == 1U || ringPosition == 3U) {
      pose.extensionMm *=
          arm_config::STORAGE2_OUTER_RING_FINAL_PLACE_M6_EXTENSION_SCALE;
      pose.standardFrameAngleDegrees +=
          STORAGE2_OUTER_RING_PLACE_M5_DIFFERENCE_CCW_DEGREES;
    }
  }
  pose.standardFrameAngleDegrees +=
      PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[
          ringPosition - 1U];
  pose.extensionMm +=
      WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[
          ringPosition - 1U];
  if (storage1Ring1PlacementExtraCorrectionApplies(ringPosition)) {
    pose.standardFrameAngleDegrees -=
        STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES;
    pose.extensionMm +=
        STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM;
  }
  if (storage2Placement &&
      (ringPosition == 1U || ringPosition == 3U)) {
    pose.extensionMm +=
        arm_config::
            STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM;
  }
  // 仅第二次暂存区放置：全部修正完成后限幅一次，避免修正后越界停机。
  // 这里使用工作零点坐标 [-9.5, 160] mm，不能误用物理总行程 170 mm。
  if (storage2Placement) {
    const float requestedExtensionMm = pose.extensionMm;
    if (pose.extensionMm < M6_RING2_MINIMUM_EXTENSION_MM) {
      pose.extensionMm = M6_RING2_MINIMUM_EXTENSION_MM;
    } else if (pose.extensionMm > M6_MAXIMUM_EXTENSION_MM) {
      pose.extensionMm = M6_MAXIMUM_EXTENSION_MM;
    } else {
      return true;
    }
    logTargetPlacementM6Limit(
        ringPosition, requestedExtensionMm, pose.extensionMm);
  }
  return true;
}

void removeRingPositionPoseCorrectionForSavedBase(
    ArmPose &pose,
    uint8_t ringPosition) {
  // Remove the shared placement net once before saving a base pose, preventing
  // a later target from adding it twice.
  if (workstationRingNetCorrectionApplies(ringPosition)) {
    pose.standardFrameAngleDegrees -=
        PROCESS_STORAGE1_RING_PLACE_M5_NET_CCW_DEGREES[
            ringPosition - 1U];
    pose.extensionMm -=
        WORKSTATION_RING_PLACE_M6_NET_EXTENSION_MM[
            ringPosition - 1U];
  }
  // 保存视觉基准位姿前，必须反向扣除第一次暂存区1号位的专用放置量，
  // 否则后续再次生成目标时会把0.2度/1 mm重复叠加。
  if (storage1Ring1PlacementExtraCorrectionApplies(ringPosition)) {
    pose.standardFrameAngleDegrees +=
        STORAGE1_RING1_PLACE_EXTRA_M5_CLOCKWISE_DEGREES;
    pose.extensionMm -=
        STORAGE1_RING1_PLACE_EXTRA_M6_EXTENSION_MM;
  }
  if (secondStorageColorWorkflowActive() &&
      (ringPosition == 1U || ringPosition == 3U)) {
    pose.standardFrameAngleDegrees -=
        STORAGE2_OUTER_RING_PLACE_M5_DIFFERENCE_CCW_DEGREES;
    pose.extensionMm -=
        arm_config::
            STORAGE2_OUTER_RING_FINAL_PLACE_ADDITIONAL_EXTENSION_MM;
  }
  if (processOrStorage1RingNetCorrectionApplies(ringPosition)) {
    return;
  }
  // STORAGE2 also removes its existing ring-3 correction here. The outer-ring
  // final scale remains placement-only, matching the established behavior.
  if (ringPosition == 3U &&
      RING3_EXTENSION_CORRECTION_MM > 0.0f) {
    pose.extensionMm -= RING3_EXTENSION_CORRECTION_MM;
  }
}

float storageColorSearchAngleForRing(uint8_t ringPosition) {
  // User frame: near ring1 parallel=0, near ring3 parallel=180.
  // Standard frame = 90 - user frame; old motor frame = -user frame.
  if (ringPosition < 1U || ringPosition > 3U) {
    routeFault("S2 scan ring invalid");
    return 0.0f;
  }
  float angleDegrees =
      90.0f -
      arm_config::STORAGE2_SCAN_USER_ANGLES_DEGREES[ringPosition - 1U];
  // Keep the two signed debug increments independent.  Hardware testing found
  // that ring 1 needs +2 degrees and ring 3 needs -2 degrees; do not infer the
  // signs again from the user-frame labels. Ring 2 stays unchanged.
  if (ringPosition == 1U) {
    angleDegrees += STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES;
  } else if (ringPosition == 3U) {
    angleDegrees += STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES;
  }
  return angleDegrees;
}

float storageColorSearchExtensionForRing(uint8_t ringPosition) {
  if (ringPosition < 1U || ringPosition > 3U) {
    routeFault("S2 scan ring invalid");
    return storage2ScanExtensionMm;
  }
  return storage2ScanExtensionMm +
      ((ringPosition == 1U || ringPosition == 3U)
           ? STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM
           : 0.0f);
}

bool storage2ExpectedColorAndSlotForRing(
    uint8_t ringPosition,
    uint8_t &expectedColor,
    uint8_t &traySlot) {
  if (ringPosition < 1U || ringPosition > 3U) {
    return false;
  }

  uint8_t firstRoundItem = 0U;
  while (firstRoundItem < 3U &&
         taskPositions[0][firstRoundItem] != ringPosition) {
    ++firstRoundItem;
  }
  if (firstRoundItem >= 3U) {
    return false;
  }

  expectedColor = taskColors[0][firstRoundItem];
  if (expectedColor < MAIXCAM_RED_REQUEST ||
      expectedColor > MAIXCAM_GREEN_REQUEST) {
    return false;
  }

  traySlot = 0U;
  while (traySlot < 3U &&
         taskColors[1][traySlot] != expectedColor) {
    ++traySlot;
  }
  return traySlot < 3U;
}

bool storage2CoordinateWithinTargetRoi(int16_t imageX, int16_t imageY) {
  const int32_t dx = static_cast<int32_t>(imageX - IMAGE_CENTER_X);
  const int32_t dy = static_cast<int32_t>(imageY - IMAGE_CENTER_Y);
  const int32_t maximumRadius = static_cast<int32_t>(
      arm_config::STORAGE2_TARGET_COLOR_ROI_RADIUS_PIXELS);
  return dx * dx + dy * dy <= maximumRadius * maximumRadius;
}

bool storageColorTargetPose(
    const ArmPose &scanPose,
    uint8_t ringPosition,
    int16_t imageX,
    int16_t imageY,
    ArmPose &targetPose) {
  const float pixelDeltaX =
      static_cast<float>(imageX - IMAGE_CENTER_X);
  const float pixelDeltaY =
      static_cast<float>(imageY - IMAGE_CENTER_Y);
  const float pixelRadius =
      hypotf(pixelDeltaX, pixelDeltaY);
  // Keep the synthetic radius used by the endpoint validity gate on the
  // original scale. Apply the STORAGE2 correction only when converting the
  // accepted pixel offset into the mechanical-arm target coordinate.
  const float baseMmPerPixel =
      rawMillimetersPerPixel(pixelRadius);
  const float correctedMmPerPixel =
      baseMmPerPixel *
      arm_config::STORAGE2_PIXEL_TO_MM_CORRECTION_SCALE;
  const uint16_t equivalentRadiusPixels =
      static_cast<uint16_t>(
          lroundf(
              RING_PHYSICAL_RADIUS_MM /
              baseMmPerPixel));
  PlanarPoint measuredPoint;
  if (!endpointVisionToPlanarPoint(
          scanPose,
          imageX,
          imageY,
          equivalentRadiusPixels,
          measuredPoint,
          correctedMmPerPixel)) {
    return false;
  }
  return planarPointToRingPose(
      ringPosition,
      measuredPoint,
      0.0f,
      targetPose);
}

void beginStorageColorVision() {
  if (!secondStorageColorWorkflowActive() ||
      workItemIndex >= 3U ||
      storageColorExpected < MAIXCAM_RED_REQUEST ||
      storageColorExpected > MAIXCAM_GREEN_REQUEST) {
    routeFault("Storage targeted color request state is invalid");
    return;
  }

  workLastMaixSequence = maixCam.latest().sequence;
  const uint8_t requestMode = storageColorExpected;
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
  SerialDebug.print("[S2DBG REQUEST] t/ring/mode/expected/samples/");
  SerialDebug.print("last-seq/window-ms=");
  SerialDebug.print(millis());
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(requestMode));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorExpected));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorAccuracySampleCount));
  SerialDebug.print('/');
  SerialDebug.print(workLastMaixSequence);
  SerialDebug.print('/');
  SerialDebug.println(
      storageColorAccuracyWindowStartMs == 0UL
          ? 0UL
          : millis() - storageColorAccuracyWindowStartMs);
#endif
  beginMaixRequest(requestMode);
  workVisionRequestStartMs = millis();
  workActionPhase =
      WORK_PHASE_STORAGE_COLOR_WAIT_COORDINATE;
}

void beginStorageColorSearch(bool travelPoseAlreadyPrepared) {
  (void)travelPoseAlreadyPrepared;
  if (!secondStorageColorWorkflowActive() || storage2ScanIndex >= 3U) {
    routeFault("S2 sweep state invalid");
    return;
  }
  if (!extensionMoveFinished() || !liftMoveFinished() ||
      armMotors.isM5Running() || armBaseMotionWatchdogActive ||
      armBaseMotionLockedUntilReset ||
      fabsf(liftAxis.currentMm - M7_STANDARD_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("S2 sweep clearance invalid");
    return;
  }
  if (!storage2ScanExtensionCaptured) {
    storage2ScanExtensionCaptured = true;
    // Capture the nominal +10 mm base once.  A debug-only outer-ring offset is
    // always added from this immutable base, so revisit scans cannot accumulate
    // another 20 mm.
    storage2ScanExtensionMm = extensionAxis.currentMm +
        arm_config::STORAGE2_SCAN_EXTENSION_ADVANCE_MM;
  }
  storageColorTargetRing = static_cast<uint8_t>(3U - storage2ScanIndex);
  uint8_t expectedColor = 0U;
  uint8_t expectedTraySlot = 0xFFU;
  if (!storage2ExpectedColorAndSlotForRing(
          storageColorTargetRing,
          expectedColor,
          expectedTraySlot)) {
    routeFault("S2 QR ring/color map invalid");
    return;
  }
  const uint8_t existingSlot =
      storage2ItemOrder[storageColorTargetRing - 1U];
  if (existingSlot < 3U && existingSlot != expectedTraySlot) {
    routeFault("S2 ring/tray map changed during rescan");
    return;
  }
  for (uint8_t ring = 1U; ring <= 3U; ++ring) {
    if (ring != storageColorTargetRing &&
        storage2ItemOrder[ring - 1U] == expectedTraySlot) {
      routeFault("S2 QR map contains duplicate tray slot");
      return;
    }
  }
  storage2ItemOrder[storageColorTargetRing - 1U] = expectedTraySlot;
  storageColorExpected = expectedColor;
  const float correctedScanExtensionMm =
      storageColorSearchExtensionForRing(storageColorTargetRing);
  if (correctedScanExtensionMm < M6_STANDARD_EXTENSION_MM ||
      correctedScanExtensionMm > M6_MAXIMUM_EXTENSION_MM) {
    routeFault("S2 sweep extension outside travel");
    return;
  }
  // The first-round QR plan already fixes the bottom-layer color on this ring.
  // Request only that color; never let another visible ring claim this slot.
  storageColorAccuracySumX = storageColorAccuracySumY = 0;
  storageColorAccuracySampleCount = 0U;
  storageColorAccuracyWindowStartMs = 0UL;
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
  storage2WindowFreshFrameCount = 0U;
  storage2WindowWrongColorCount = 0U;
  storage2WindowOutsideRoiCount = 0U;
  storage2WindowInitialLatestSequence = maixCam.latest().sequence;
#endif
  stopMaixRequest();
  storageColorScanPose = ArmPose(
      storageColorSearchAngleForRing(storageColorTargetRing),
      correctedScanExtensionMm, M7_STANDARD_HEIGHT_MM);
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
  SerialDebug.print("[S2DBG VISIT] t/scan/ring/tray-slot/expected/");
  SerialDebug.print("M5target/M6base/M6target/M7target=");
  SerialDebug.print(millis());
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storage2ScanIndex));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(expectedTraySlot));
  SerialDebug.print('/');
  SerialDebug.print(static_cast<unsigned int>(storageColorExpected));
  SerialDebug.print('/');
  SerialDebug.print(storageColorScanPose.standardFrameAngleDegrees, 2);
  SerialDebug.print('/');
  SerialDebug.print(storage2ScanExtensionMm, 2);
  SerialDebug.print('/');
  SerialDebug.print(storageColorScanPose.extensionMm, 2);
  SerialDebug.print('/');
  SerialDebug.println(storageColorScanPose.heightMm, 2);
  SerialDebug.print("[S2DBG EXPECT] ring=");
  SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
  SerialDebug.print(" target=");
  SerialDebug.print(static_cast<unsigned int>(storageColorExpected));
  SerialDebug.print('(');
  SerialDebug.print(storage2ColorName(storageColorExpected));
  SerialDebug.print(") tray-slot=");
  SerialDebug.print(static_cast<unsigned int>(expectedTraySlot));
  SerialDebug.println("; only this color inside center radius 100 px is accepted");
  printStorage2VerboseMap("visit");
#endif
  useArmBaseTransferMotionProfile();
  if (!startExtensionToMmWithProfile(storageColorScanPose.extensionMm,
          STORAGE_TRAVEL_M6_SPEED_RPM, STORAGE_TRAVEL_M6_ACCELERATION)) {
    if (programState != PROGRAM_FAULT) { routeFault("S2 parallel scan rejected"); }
    return;
  }
  // Keep the survey parallel: issue M6 first, then M5 without waiting for M6.
  startArmBaseStandardFrameDegrees(
      storageColorScanPose.standardFrameAngleDegrees);
  workActionPhase = WORK_PHASE_STORAGE_COLOR_WAIT_EXTENSION;
}

void advanceStorage2Scan() {
  stopMaixRequest();
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
  printStorage2VerboseMap("advance");
#endif
  // Visit nominal user angles 150/90/30 in order.  State-3 debug may move the
  // two outer poses toward ring 2. A missing observation never stalls the sweep.
  // On following laps revisit only missing positions, keeping earlier results.
  for (uint8_t offset = 1U; offset <= 3U; ++offset) {
    const uint8_t next = static_cast<uint8_t>((storage2ScanIndex + offset) % 3U);
    const uint8_t ring = static_cast<uint8_t>(3U - next);
    if (!measuredRingPoseValid[ring] || storage2ItemOrder[ring - 1U] >= 3U) {
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
      SerialDebug.print("[S2DBG NEXT] next-scan/ring/need-pose/need-slot=");
      SerialDebug.print(static_cast<unsigned int>(next));
      SerialDebug.print('/');
      SerialDebug.print(static_cast<unsigned int>(ring));
      SerialDebug.print('/');
      SerialDebug.print(measuredRingPoseValid[ring] ? 0 : 1);
      SerialDebug.print('/');
      SerialDebug.println(storage2ItemOrder[ring - 1U] >= 3U ? 1 : 0);
#endif
      storage2ScanIndex = next;
      beginStorageColorSearch(false);
      return;
    }
  }
  storage2ScanIndex = 3U;
  workItemIndex = 0U;
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
  SerialDebug.print("[S2DBG COMPLETE] map-complete first-tray-slot=");
  SerialDebug.println(static_cast<unsigned int>(storage2ItemOrder[0]));
#endif
  commandStorageServoPositionWithArrivalGate(storage2ItemOrder[0]);
  if (programState == PROGRAM_FAULT) { return; }
  endpointInitialStorageCommanded = true;
  workStorageServoDeadlineMs = millis() + STORAGE_SERVO_SETTLE_MS;
  workActionPhase = WORK_PHASE_START_UNLOAD;
}

bool storageTravelPreparationReady(uint8_t roundNumber) {
  if (storageTravelPreparationPhase !=
          STORAGE_TRAVEL_PREP_COMPLETE ||
      storageTravelPreparationRound != roundNumber ||
      !liftMoveFinished() ||
      fabsf(
          liftAxis.currentMm -
          M7_STANDARD_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    return false;
  }
  if (roundNumber == 1U &&
      (!extensionMoveFinished() ||
       armMotors.isM5Running() ||
       armBaseMotionWatchdogActive ||
       fabsf(
           extensionAxis.currentMm -
           ENDPOINT_SEARCH_SEED_EXTENSION_MM) >
           ARM_AXIS_POSITION_TOLERANCE_MM)) {
    return false;
  }
  if (roundNumber == 2U &&
      (!extensionMoveFinished() ||
       armMotors.isM5Running() ||
       armBaseMotionWatchdogActive ||
       fabsf(
           extensionAxis.currentMm -
           STORAGE_TRAVEL_M6_EXTENSION_MM) >
           ARM_AXIS_POSITION_TOLERANCE_MM)) {
    return false;
  }
  return true;
}

void beginStorageTravelPreparation(uint8_t roundNumber) {
  if (roundNumber < 1U || roundNumber > 2U) {
    routeFault("Storage travel preparation round is invalid");
    return;
  }
  if (!extensionMoveFinished() ||
      !liftMoveFinished() ||
      armMotors.isM5Running() ||
      armBaseMotionWatchdogActive ||
      fabsf(
          extensionAxis.currentMm -
          M6_STANDARD_EXTENSION_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("Process departure arm is not ready for travel preparation");
    return;
  }

  storageTravelPreparationRound = roundNumber;
  storageTravelM5ZeroHandoffDeadlineUs = 0UL;
  storageTravelM5StartDeadlineMs = 0UL;
  workStorageServoDeadlineMs = 0UL;
  // M5 holds its last PROCESS pose throughout chassis travel.
  // Ring selection and all M5 scan commands are deferred until STORAGE arrival.

  // Round 1 and round 2 must use the same M7 departure profile.  Their target
  // poses and later state-machine phases remain round-specific.
  const bool storageTravelM7Started =
      startLiftToHeightMmWithProfile(
          M7_STANDARD_HEIGHT_MM,
          STORAGE_TRAVEL_M7_SPEED_RPM,
          STORAGE_TRAVEL_M7_ACCELERATION);
  if (!storageTravelM7Started) {
    return;
  }
  storageTravelPreparationPhase =
      roundNumber == 1U
          ? STORAGE_TRAVEL_PREP_WAIT_M7_ONLY
          : STORAGE_TRAVEL_PREP_WAIT_M7_HEAD_START;
  if (roundNumber == 2U) {
    storageTravelM5StartDeadlineMs =
        millis() + STORAGE2_M7_HEAD_START_MS;
  }
  markMissionProgress();
}

void serviceStorageTravelPreparation() {
  switch (storageTravelPreparationPhase) {
    case STORAGE_TRAVEL_PREP_IDLE:
    case STORAGE_TRAVEL_PREP_COMPLETE:
      return;

    case STORAGE_TRAVEL_PREP_WAIT_M7_ONLY:
      if (!liftMoveFinished()) {
        return;
      }
      if (fabsf(
              liftAxis.currentMm -
              M7_STANDARD_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
        routeFault("Storage travel M7 did not reach zero");
        return;
      }
      storageTravelM5ZeroHandoffDeadlineUs =
          micros() + PROCESS1_M5_ZERO_HANDOFF_US;
      storageTravelPreparationPhase =
          STORAGE_TRAVEL_PREP_WAIT_ROUND1_M7_HANDOFF;
      return;

    case STORAGE_TRAVEL_PREP_WAIT_ROUND1_M7_HANDOFF:
      if (!liftMoveFinished() ||
          static_cast<int32_t>(
              micros() -
              storageTravelM5ZeroHandoffDeadlineUs) < 0) {
        return;
      }
      if (!startExtensionToMmWithProfile(
              ENDPOINT_SEARCH_SEED_EXTENSION_MM,
              PROCESS_STORAGE_TRAVEL_M6_SPEED_RPM,
              PROCESS_STORAGE_TRAVEL_M6_ACCELERATION)) {
        return;
      }
      // Keep M5 stopped; STORAGE1 entry starts the ring-1 pose after arrival.
      commandStorageServoPosition(
          0U,
          PROCESS1_STORAGE_TRAVEL_SERVO_INTERVAL_MS);
      workStorageServoDeadlineMs =
          millis() + PROCESS1_STORAGE_TRAVEL_SERVO_SETTLE_MS;
      storageTravelM5ZeroHandoffDeadlineUs = 0UL;
      storageTravelPreparationPhase =
          STORAGE_TRAVEL_PREP_WAIT_ROUND1_PARALLEL;
      markMissionProgress();
      return;

    case STORAGE_TRAVEL_PREP_WAIT_ROUND1_PARALLEL:
      if (armMotors.isM5Running() ||
          armBaseMotionWatchdogActive ||
          !extensionMoveFinished() ||
          !liftMoveFinished() ||
          !deadlineReached(workStorageServoDeadlineMs)) {
        return;
      }
      if (fabsf(
              extensionAxis.currentMm -
              ENDPOINT_SEARCH_SEED_EXTENSION_MM) >
              ARM_AXIS_POSITION_TOLERANCE_MM ||
          fabsf(
              liftAxis.currentMm -
              M7_STANDARD_HEIGHT_MM) >
              ARM_AXIS_POSITION_TOLERANCE_MM) {
        routeFault("Storage1 travel preparation pose is incomplete");
        return;
      }
      storageTravelPreparationPhase =
          STORAGE_TRAVEL_PREP_COMPLETE;
      markMissionProgress();
      return;

    case STORAGE_TRAVEL_PREP_WAIT_M7_HEAD_START:
      if (!deadlineReached(storageTravelM5StartDeadlineMs)) {
        return;
      }
      storageTravelM5StartDeadlineMs = 0UL;
      // Keep M5 stopped; only M7/M6 preparation overlaps chassis travel.
      storageTravelPreparationPhase =
          STORAGE_TRAVEL_PREP_WAIT_M5_M7;
      SerialDebug.println(
          "[S2 PREP] M5 hold");
      markMissionProgress();
      return;

    case STORAGE_TRAVEL_PREP_WAIT_M5_M7:
      if (armMotors.isM5Running() ||
          armBaseMotionWatchdogActive ||
          !liftMoveFinished()) {
        return;
      }
      if (fabsf(
              liftAxis.currentMm -
              M7_STANDARD_HEIGHT_MM) >
              ARM_AXIS_POSITION_TOLERANCE_MM) {
        routeFault("Storage travel M7 did not reach zero before M6");
        return;
      }
      if (!startExtensionToMmWithProfile(
              STORAGE_TRAVEL_M6_EXTENSION_MM,
              PROCESS_STORAGE_TRAVEL_M6_SPEED_RPM,
              PROCESS_STORAGE_TRAVEL_M6_ACCELERATION)) {
        return;
      }
      storageTravelPreparationPhase =
          STORAGE_TRAVEL_PREP_WAIT_M6;
      markMissionProgress();
      return;

    case STORAGE_TRAVEL_PREP_WAIT_M6:
      if (extensionMoveFinished()) {
        if (fabsf(
                liftAxis.currentMm -
                M7_STANDARD_HEIGHT_MM) >
                ARM_AXIS_POSITION_TOLERANCE_MM ||
            fabsf(
                extensionAxis.currentMm -
                STORAGE_TRAVEL_M6_EXTENSION_MM) >
                ARM_AXIS_POSITION_TOLERANCE_MM) {
          routeFault("Storage travel preparation pose is incomplete");
          return;
        }
        storageTravelPreparationPhase =
            STORAGE_TRAVEL_PREP_COMPLETE;
        SerialDebug.print(
            "[STORAGE TRAVEL PREP] round 2 complete M6/M7=");
        SerialDebug.print(extensionAxis.currentMm, 2);
        SerialDebug.print("/");
        SerialDebug.println(liftAxis.currentMm, 2);
        markMissionProgress();
      }
      return;
  }
}

void serviceStorage1Raw2TravelPreparation() {
  if (storage1Raw2TravelPhase !=
      STORAGE1_RAW2_TRAVEL_RUNNING) {
    return;
  }
  if (!extensionMoveFinished()) {
    return;
  }
  if (!liftMoveFinished() ||
      fabsf(
          liftAxis.currentMm -
          M7_STANDARD_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("STORAGE1 RAW2 preparation fault");
    return;
  }
  if (fabsf(
          extensionAxis.currentMm -
          RAW_VIEW_EXTENSION_MM) >
      ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("STORAGE1 RAW2 preparation fault");
    return;
  }

  storage1Raw2TravelPhase =
      STORAGE1_RAW2_TRAVEL_COMPLETE;
  markMissionProgress();
}

bool storage1Raw2TravelPreparationReady() {
  serviceStorage1Raw2TravelPreparation();
  return storage1Raw2TravelPhase ==
         STORAGE1_RAW2_TRAVEL_COMPLETE;
}

void printMeasuredRingMapSummary() {
  if (!measuredRingPoseValid[1U] ||
      !measuredRingPoseValid[2U] ||
      !measuredRingPoseValid[3U]) {
    SerialDebug.println(
        "[RING MAP SUMMARY] unavailable");
    return;
  }

  // Geometry and per-scan timing are already logged when measured. Keep the
  // final poses here without duplicating those reports in the 128 KB image.
  for (uint8_t ring = 1U; ring <= 3U; ++ring) {
    SerialDebug.print(
        "[RING MAP SUMMARY] P");
    SerialDebug.print(ring);
    SerialDebug.print(" xy/M5/M6=");
    SerialDebug.print(
        measuredRingPoints[ring].outwardMm,
        2);
    SerialDebug.print(",");
    SerialDebug.print(
        measuredRingPoints[ring].leftMm,
        2);
    SerialDebug.print("/");
    SerialDebug.print(
        measuredRingPoses[ring]
            .standardFrameAngleDegrees,
        2);
    SerialDebug.print("/");
    SerialDebug.println(
        measuredRingPoses[ring].extensionMm,
        2);
  }
}

bool completeEndpointMapAndStartTransfers() {
  if (!measuredRingPointValid[1U] || !measuredRingPointValid[3U]) {
    routeFault("Endpoint scan sequence corrupted");
    return false;
  }
  if (!buildMeasuredRingMap()) {
    return false;
  }

  endpointMapCompleteMs = millis();
  workItemIndex = 0U;
  printMeasuredRingMapSummary();
  SerialDebug.println(
      "[ENDPOINT MAP] complete; direct tray pickup");
  endpointDirectContainerPickupPending = true;
  workActionPhase = WORK_PHASE_START_UNLOAD;
  return true;
}

void finishActiveWorkAction() {
  const WorkActionKind completedAction = activeWorkAction;
  stopMaixRequest();
  switch (activeWorkAction) {
    case WORK_ACTION_RAW:
      rawActionFinished = true;
      break;
    case WORK_ACTION_PROCESS:
      processActionFinished = true;
      break;
    case WORK_ACTION_STORAGE:
      storageActionFinished = true;
      break;
    case WORK_ACTION_NONE:
      break;
  }

  activeWorkAction = WORK_ACTION_NONE;
  armPayloadPresent = false;
  armGripperLiftIsolationEnabled = false;
  rawProcessPickupM7RaiseAdditionalDelayEnabled = false;
  workActionPhase = WORK_PHASE_IDLE;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  activePlacementRingPosition = 0U;
  armStandardPhase = ARM_STANDARD_IDLE;
  armStandardHandoffDeadlineUs = 0UL;
  armStandardEndpointPreparation = false;
  armStandardKeepBaseAngle = false;
  armStandardRawViewPreparation = false;
  armStandardEndpointExtensionPrepositioned = false;
  armStandardExtensionTargetMm =
      M6_STANDARD_EXTENSION_MM;
  armTransferPhase = ARM_TRANSFER_IDLE;
  armTransferM7HandoffStartMs = 0UL;
  armTransferRawM7ToGripperDeadlineUs = 0UL;
  armTransferMapSource = false;
  armTransferMapDestination = false;
  armTransferConcurrentSourcePreparation = false;
  armTransferSourceAlreadyPrepared = false;
  armTransferPrepareNextSource = false;
  armTransferNextSourcePose = ArmPose();
  armTransferNextSourceMapped = false;
  armTransferNextStorageSlot = -1;
  armTransferNextStorageDeadlineMs = 0UL;
  armTransferStorageCommandDueMs = 0UL;
  armTransferNextStorageCommanded = false;
  armTransferNextSourcePreparedAtEnd = false;
  armTransferReturnToRawViewAtEnd = false;
  armTransferSourceRotationStarted = false;
  armTransferDestinationRotationStarted = false;
  armTransferFinalRotationStarted = false;
  armTransferProcessFirstRingReturnTiming = false;
  armTransferDestinationM5FinalBufferPending = false;
#if GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT > 100U
  armTransferSourceRingPosition = 0U;
#endif
  armTransferMotionProfile =
      arm_transfer::PROFILE_STANDARD;
  endpointDirectContainerPickupPending = false;
  endpointInitialStorageCommanded = false;
  containerPickupPrepositionedPending = false;
  ringPickupPrepositionedPending = false;
  workActionStartMs = 0UL;
  workVisionRequestStartMs = 0UL;
  workVisionRetryCount = 0U;
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
  rawTrackingActive = false;
  rawNoTargetDeadlineMs = 0UL;
#endif
  storageColorExpected = 0U;
  storageColorTargetRing = 0U;
  storageColorScanPose = ArmPose();
  workStorageServoDeadlineMs = 0UL;
  resetProcess2DepartureImuPrelock();
  resetStorageServoArrivalGate();
  if (completedAction == WORK_ACTION_STORAGE) {
    storageTravelPreparationPhase =
        STORAGE_TRAVEL_PREP_IDLE;
    storageTravelPreparationRound = 0U;
    storageTravelM5ZeroHandoffDeadlineUs = 0UL;
    storageTravelM5StartDeadlineMs = 0UL;
  }
  markMissionProgress();
  SerialDebug.println("Workstation action complete");

  if (ROUGH_PROCESSING_CALIBRATION_MODE &&
      completedAction == WORK_ACTION_PROCESS) {

    stopAllMotorsImmediately();
    enableDriveMotors();
    disableArmBaseMotor();
    commandGripperClose();
    programState = PROGRAM_FINISHED;
        startRequested = false;
    hmiSetRunStatus("CALDONE");
    hmiSetText("t1", "RING123");
    hmiSetTaskCounts();
    printMeasuredRingMapSummary();
    SerialDebug.println(
        "Rough-processing calibration complete: "
        "three placements and three pickups finished; "
        "power-cycle before another run.");
  }
}

bool process2FinalStorageMayOverlapDeparture() {
  return activeWorkAction == WORK_ACTION_PROCESS &&
         workRoundIndex == 1U &&
         workItemIndex >= 3U &&
         activeTransferPurpose ==
             TRANSFER_PURPOSE_RING_TO_CONTAINER;
}

void beginProcess2DepartureImuPrelock() {
  resetProcess2DepartureImuPrelock();
  headingStableStartMs = 0UL;
  process2DepartureImuLastFrameSequence =
      route_chassis::imuValidFrameSequence;
  workDepartureLastImuReceiveMs =
      route_chassis::lastImuReceiveMs;
}

void serviceProcess2DepartureImuPrelock() {
  const uint32_t nowMs = millis();
  const uint32_t frameSequence =
      route_chassis::imuValidFrameSequence;
  const uint32_t receiveMs =
      route_chassis::lastImuReceiveMs;
  const bool driveStationary = allMotorsArrived();
  const bool streamContinuous =
      receiveMs - workDepartureLastImuReceiveMs <=
      WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS;
  if (!motorPulsesAreZero(
          visualCorrectionAccumulator) ||
      !driveStationary ||
      !imuIsFresh() ||
      nowMs - receiveMs >
          WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS) {
    headingStableStartMs = 0UL;
    process2DepartureImuPrelockReady = false;
    return;
  }
  if (frameSequence ==
      process2DepartureImuLastFrameSequence) {
    return;
  }

  process2DepartureImuLastFrameSequence = frameSequence;
  workDepartureLastImuReceiveMs = receiveMs;
  if (!streamContinuous ||
      fabsf(headingErrorDegrees()) >
          WORKSTATION_HEADING_TOLERANCE_DEGREES) {
    headingStableStartMs = 0UL;
    process2DepartureImuPrelockReady = false;
    return;
  }

  if (headingStableStartMs == 0UL) {
    headingStableStartMs = receiveMs;
    return;
  }
  process2DepartureImuPrelockReady =
      receiveMs - headingStableStartMs >=
      WORK_DEPARTURE_POST_ARM_SETTLE_MS +
          WORK_DEPARTURE_IMU_STABLE_MS;
}

bool workstationMechanismsStoppedForDeparture() {
  const float requiredLiftHeightMm =
      activeWorkAction == WORK_ACTION_PROCESS
          ? arm_config::RING_ROTATION_CLEARANCE_HEIGHT_MM
          : M7_STANDARD_HEIGHT_MM;
  const bool storageServoReadyForDeparture =
      process2FinalStorageMayOverlapDeparture() ||
      deadlineReached(workStorageServoDeadlineMs);
  return !armMotors.isM5Running() &&
         !armBaseMotionWatchdogActive &&
         extensionMoveFinished() &&
         liftMoveFinished() &&
         fabsf(
             extensionAxis.currentMm -
             M6_STANDARD_EXTENSION_MM) <=
             ARM_AXIS_POSITION_TOLERANCE_MM &&
         fabsf(
             liftAxis.currentMm -
             requiredLiftHeightMm) <=
             ARM_AXIS_POSITION_TOLERANCE_MM &&
         storageServoReadyForDeparture;
}

void beginWorkstationDepartureLock() {
  commandStartMs = 0UL;
  workDepartureLastImuReceiveMs =
      route_chassis::lastImuReceiveMs;
  preciseMotionEnabled = false;
  turnMotionEnabled = false;
  workstationApproachEnabled = true;
  workActionPhase = WORK_PHASE_WAIT_DEPARTURE_LOCK;
}

bool serviceWorkstationDepartureLock() {
  const uint32_t nowMs = millis();
  if (!workstationMechanismsStoppedForDeparture()) {
    commandStartMs = 0UL;
    workDepartureLastImuReceiveMs =
        route_chassis::lastImuReceiveMs;
    headingStableStartMs = 0UL;
    motorsArrivedStartMs = 0UL;
    return false;
  }

  if (LOCAL_WORKSTATION_SIMULATION_MODE) {
    // Local tests must never turn or translate the chassis. Arm/tray
    // completion checks remain mandatory; only route IMU correction is skipped.
    return true;
  }

  if (commandStartMs == 0UL) {
    commandStartMs = nowMs;
    workDepartureLastImuReceiveMs =
        route_chassis::lastImuReceiveMs;
    headingStableStartMs = 0UL;
    motorsArrivedStartMs = 0UL;
    return false;
  }

  if (nowMs - commandStartMs >=
      WORK_DEPARTURE_MAX_WAIT_MS) {
    // Never release a new route command while a heading-correction pulse train
    // is still active. The 1000 ms cap applies to the IMU stability observation;
    // a correction already in flight is allowed to stop cleanly first.
    return allMotorsArrived();
  }

  const uint32_t imuReceiveMs =
      route_chassis::lastImuReceiveMs;
  if (imuReceiveMs == workDepartureLastImuReceiveMs) {
    return false;
  }
  if (imuReceiveMs - workDepartureLastImuReceiveMs >
      WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS) {
    headingStableStartMs = 0UL;
  }
  workDepartureLastImuReceiveMs = imuReceiveMs;

  return updateHeadingLock(
      WORK_DEPARTURE_MAX_WAIT_MS,
      WORK_DEPARTURE_POST_ARM_SETTLE_MS,
      WORK_DEPARTURE_IMU_STABLE_MS);
}

void beginStorageParkingBeforeWorkFinish() {
  commandStorageServoParkingPosition();

  beginArmStandardization(0.0f, true);
  workStorageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  workActionPhase = WORK_PHASE_WAIT_STORAGE_PARK;
}

void startRawTravelParkingAtSafeLinearZero() {
  const bool linearAxesSafe =
      extensionMoveFinished() &&
      liftMoveFinished() &&
      fabsf(
          extensionAxis.currentMm -
          M6_STANDARD_EXTENSION_MM) <=
          ARM_AXIS_POSITION_TOLERANCE_MM &&
      fabsf(
          liftAxis.currentMm -
          M7_STANDARD_HEIGHT_MM) <=
          ARM_AXIS_POSITION_TOLERANCE_MM;
  if (!linearAxesSafe) {
    routeFault(
        "RAW travel start requires M6/M7 safe zero");
    return;
  }

  commandStorageServoParkingPosition(
      DEPARTURE_STORAGE_SERVO_INTERVAL_MS);
  workStorageServoDeadlineMs =
      millis() + DEPARTURE_STORAGE_SERVO_SETTLE_MS;
  if (RAW_WORKSTATION_SIMULATION_MODE) {
    // finishActiveWorkAction() intentionally clears the generic deadline for
    // the formal route handoff. Preserve it only for the standalone RAW build,
    // which has no following route segment to overlap the tray parking with.
    rawSimulationParkingDeadlineMs =
        workStorageServoDeadlineMs;
  }
  setArmBaseMotionProfileForPayload(
      RAW_EXIT_M5_MAXIMUM_STEP_RATE,
      RAW_EXIT_M5_STEP_ACCELERATION);
  startArmBaseStandardFrameDegrees(0.0f);
  if (programState != PROGRAM_RUNNING) {
    return;
  }
  rawTravelM5ZeroPending = true;
  SerialDebug.println(
      "[RAW EXIT PARALLEL] M7 zero; 30%-speed M5 zero and tray park "
      "started, release chassis now");
  // M5 is software-stepped globally and the tray servo is autonomous, so both
  // continue after the work pause is released.  PROCESS entry retains the
  // existing rawTravelM5ZeroPending gate before endpoint work can start.
  finishActiveWorkAction();
}

void beginRawTravelParkingAfterFinalStore() {
  const bool transferClearanceReady =
      extensionMoveFinished() &&
      liftMoveFinished() &&
      fabsf(
          extensionAxis.currentMm -
          M6_STANDARD_EXTENSION_MM) <=
          ARM_AXIS_POSITION_TOLERANCE_MM;
  if (!transferClearanceReady) {
    routeFault(
        "RAW final store did not finish at M6 zero/M7 clearance");
    return;
  }

  // Rotation clearance is now the true 0 mm M7 working zero. Keep this
  // idempotent command so the final RAW transfer still passes through the
  // same verified safe-zero gate before the chassis route is released.
  if (!startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)) {
    return;
  }
  workActionPhase =
      WORK_PHASE_RAW_WAIT_TRAVEL_LINEAR_ZERO;
  SerialDebug.print("[RAW TRAVEL] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, final tray release complete; M7 held at 0 mm clearance");
}

void beginRawExpectedSlotPreparation();

#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
ArmPose rawSearchPose(RawSearchZone zone) {
  if (zone == RAW_SEARCH_ZONE_B_CLOCKWISE) {
    return ArmPose(
        RAW_SEARCH_B_CLOCKWISE_ANGLE_DEGREES,
        RAW_SEARCH_B_EXTENSION_MM,
        M7_STANDARD_HEIGHT_MM);
  }
  if (zone == RAW_SEARCH_ZONE_B_COUNTERCLOCKWISE) {
    return ArmPose(
        RAW_SEARCH_B_COUNTERCLOCKWISE_ANGLE_DEGREES,
        RAW_SEARCH_B_EXTENSION_MM,
        M7_STANDARD_HEIGHT_MM);
  }
  return ArmPose(
      0.0f,
      RAW_VIEW_EXTENSION_MM,
      M7_STANDARD_HEIGHT_MM);
}
#endif

void beginWorkAction(
    WorkActionKind kind,
    uint8_t roundNumber) {
  SerialDebug.print("[WORK ENTER] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, kind=");
  SerialDebug.print(static_cast<unsigned int>(kind));
  SerialDebug.print(", round=");
  SerialDebug.print(static_cast<unsigned int>(roundNumber));
  SerialDebug.print(", scanFlag=");
  SerialDebug.print(scanFlag ? 1 : 0);
  SerialDebug.print(", taskDecoded=");
  SerialDebug.print(taskCodeDecoded ? 1 : 0);
  SerialDebug.print(", first-round colors=");
  SerialDebug.print(taskColors[0][0]);
  SerialDebug.print("/");
  SerialDebug.print(taskColors[0][1]);
  SerialDebug.print("/");
  SerialDebug.println(taskColors[0][2]);

  if (!taskCodeDecoded ||
      roundNumber < 1U ||
      roundNumber > 2U) {
    routeFault("Work action missing valid task code");
    return;
  }
  if (!verifyManipulationServosOnline()) {
    // A single workstation-entry ping is diagnostic only; transient bus
    // errors must not abort the route. Motion/arrival checks remain active.
    SerialDebug.println("[WARN] Servo ping failed; continuing workstation");
  }

  const bool useStorage1TravelPreparation =
      kind == WORK_ACTION_STORAGE &&
      roundNumber == 1U &&
      storageTravelPreparationReady(1U);
  const uint32_t preparedStorage1ServoDeadlineMs =
      useStorage1TravelPreparation
          ? workStorageServoDeadlineMs
          : 0UL;

  activeWorkAction = kind;
  resetProcess2DepartureImuPrelock();
  // Every workstation transfer can command the same gripper and M7 pair.
  // Keep a short nonzero two-way handoff active for RAW, PROCESS and STORAGE;
  // servo motion completion is covered by the independent settle gates.
  armGripperLiftIsolationEnabled =
      kind != WORK_ACTION_NONE;
  rawProcessPickupM7RaiseAdditionalDelayEnabled =
      kind == WORK_ACTION_RAW ||
      kind == WORK_ACTION_PROCESS;
  workActionStartMs = millis();
  workVisionRequestStartMs = 0UL;
  workVisionRetryCount = 0U;
  workRoundIndex =
      static_cast<uint8_t>(roundNumber - 1U);
  workItemIndex = 0U;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  activePlacementRingPosition = 0U;
  armTransferPhase = ARM_TRANSFER_IDLE;
  armTransferM7HandoffStartMs = 0UL;
  armTransferRawM7ToGripperDeadlineUs = 0UL;
  armTransferMapSource = false;
  armTransferMapDestination = false;
  armTransferConcurrentSourcePreparation = false;
  armTransferSourceAlreadyPrepared = false;
  armTransferPrepareNextSource = false;
  armTransferNextSourcePose = ArmPose();
  armTransferNextSourceMapped = false;
  armTransferNextStorageSlot = -1;
  armTransferNextStorageDeadlineMs = 0UL;
  armTransferStorageCommandDueMs = 0UL;
  armTransferNextStorageCommanded = false;
  armTransferNextSourcePreparedAtEnd = false;
  armTransferReturnToRawViewAtEnd = false;
  armTransferSourceRotationStarted = false;
  armTransferDestinationRotationStarted = false;
  armTransferFinalRotationStarted = false;
  armTransferProcessFirstRingReturnTiming = false;
  armTransferDestinationM5FinalBufferPending = false;
  endpointDirectContainerPickupPending = false;
  endpointInitialStorageCommanded =
      useStorage1TravelPreparation;
  containerPickupPrepositionedPending = false;
  ringPickupPrepositionedPending = false;
  workStorageServoDeadlineMs =
      preparedStorage1ServoDeadlineMs;
  resetStorageServoArrivalGate();
  visualCorrectionAccumulator = MotorPulses();
  visualCorrectionForwardMm = 0.0f;
  visualCorrectionLeftMm = 0.0f;
  visualCorrectionMoveCount = 0U;
  activeEndpointScanRing = 0U;
  endpointReachRecoveryMoves = 0U;
  endpointReachRecoveryForwardMm = 0.0f;
  endpointReachRecoveryLeftMm = 0.0f;
  endpointReachRecoveryRestoreOnDeparture = false;
  activeEndpointScanPose = ArmPose();
  endpointFineVisionActive = false;
  activeEndpointServoMoveCount = 0U;
  activeEndpointStageServoMoveCount = 0U;
  endpointSearchFallbackMoveCount = 0U;
  endpointSearchBaseAngleDegrees = 0.0f;
  resetEndpointCenteredConfirmationWindow();
  endpointLocalSettleDeadlineMs = 0UL;
  endpointMapStartMs = 0UL;
  endpointMapCompleteMs = 0UL;
  memset(
      endpointScanStartMs,
      0,
      sizeof(endpointScanStartMs));
  memset(
      endpointScanElapsedMs,
      0,
      sizeof(endpointScanElapsedMs));
  memset(
      endpointServoMoveCounts,
      0,
      sizeof(endpointServoMoveCounts));
  memset(
      endpointFinalCenterErrorsPixels,
      0,
      sizeof(endpointFinalCenterErrorsPixels));
  endpointMapLockedHeadingDegrees = 0.0f;
  resetMeasuredRingMap();
  storageColorExpected = 0U;
  storageColorTargetRing = 0U;
  storageColorScanPose = ArmPose();
  rawFilledSlotMask = 0U;
  rawCollectedCount = 0U;
  rawPendingSlotIndex = 0U;
  rawPendingColor = 0U;
  rawPendingSourcePose = ArmPose();
  rawPickupPrepositionedPending = false;
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
  rawSearchZone = RAW_SEARCH_ZONE_A;
  rawSearchMoveTargetZone = RAW_SEARCH_ZONE_A;
  rawNoTargetDeadlineMs = 0UL;
  rawCurrentNoTargetAllowanceMs =
      RAW_INITIAL_A_NO_TARGET_MS;
  rawChassisAtB = false;
  rawFinishAfterSearchMove = false;
  rawTrackingActive = false;
#endif
  workLastMaixSequence =
      maixCam.latest().sequence;
  resetCircleStabilityWindow();

  if (kind == WORK_ACTION_RAW) {
    const bool storage1Raw2Prepositioned =
        workRoundIndex == 1U &&
        storage1Raw2TravelPhase ==
            STORAGE1_RAW2_TRAVEL_COMPLETE;
    const bool parallelPreparation =
        rawParallelPreparationSafe();
    beginRawExpectedSlotPreparation();
    if (programState != PROGRAM_RUNNING) {
      return;
    }
    if (parallelPreparation) {
      beginRawParallelPreparation();
    } else {
      beginArmStandardization(
          0.0f,
          false,
          RAW_VIEW_EXTENSION_MM);
    }
    // RAW1 and RAW2 use the same initial overlap: arm readiness releases
    // vision/pickup, while the independently moving tray remains protected
    // by its real-angle gate before placement descent.
    rawInitialStorageArrivalOverlapPending =
        rawCollectedCount == 0U;
    if (storage1Raw2Prepositioned) {
      storage1Raw2TravelPhase =
          STORAGE1_RAW2_TRAVEL_IDLE;
      storage1Raw2HandoffDeadlineUs = 0UL;
    }
    workActionPhase = WORK_PHASE_PREPARE;
    return;
  }

  if (kind == WORK_ACTION_PROCESS) {

    if (!rawTravelM5ZeroPending) {
      useArmBaseRawTransferMotionProfile();
      startArmBaseStandardFrameDegrees(0.0f);
      rawTravelM5ZeroPending = true;
      SerialDebug.println(
          "[PROCESS ENTRY] standard M5 0 deg commanded "
          "(calibration/direct entry)");
    }
    processM5ZeroSettleDeadlineUs = 0UL;
    workStorageServoDeadlineMs = 0UL;
    workActionPhase =
        WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO;
    SerialDebug.print("[PROCESS ENTRY] t=");
    SerialDebug.print(millis());
    SerialDebug.println(
        " ms, waiting for standard M5 0 deg before ring 1");
    return;
  }

  if (kind == WORK_ACTION_STORAGE &&
      workRoundIndex == 1U) {
    endpointMapLockedHeadingDegrees =
        currentRouteCounterClockwiseHeading();
    for (uint8_t item = 0U; item < 3U; ++item) {
      storage2ItemOrder[item] = 0xFFU;
    }
    // The chassis is stopped. Scan 3/2/1 once, then place 1/2/3.
    storage2ScanIndex = 0U;
    storage2ScanExtensionCaptured = false;
    endpointDirectContainerPickupPending = true;
    beginStorageColorSearch(false);
    return;
  }

  beginArmEndpointPreparation(
      useStorage1TravelPreparation);
  workActionPhase = WORK_PHASE_PREPARE;
  SerialDebug.print("[WORK PREPARE] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, arm moves to ring-1 pose first; "
      "storage remains parked");
}

void beginRawExpectedSlotPreparation() {
  if (rawCollectedCount >= 3U) {
    routeFault("Raw slot preparation index overflow");
    return;
  }
  const uint8_t expectedColor =
      taskColors[workRoundIndex][rawCollectedCount];
  const int8_t mappedSlot =
      rawStorageSlotForColor(expectedColor);
  if (mappedSlot < 0 ||
      static_cast<uint8_t>(mappedSlot) !=
          rawCollectedCount) {
    routeFault("Raw expected color-to-slot mapping invalid");
    return;
  }

  commandRawStorageServoPositionWithArrivalGate(
      static_cast<uint8_t>(mappedSlot));
  workStorageServoDeadlineMs = 0UL;
  workActionPhase =
      WORK_PHASE_RAW_WAIT_EXPECTED_SLOT;
}

void beginRawItemVision() {
  if (rawCollectedCount >= 3U) {
    routeFault("Raw vision requested after all items stored");
    return;
  }
  const uint8_t expectedColor =
      taskColors[workRoundIndex][rawCollectedCount];
  if (expectedColor < MAIXCAM_RED_REQUEST ||
      expectedColor > MAIXCAM_GREEN_REQUEST) {
    routeFault("Raw expected color is invalid");
    return;
  }
  workLastMaixSequence =
      maixCam.latest().sequence;
  SerialDebug.print("[RAW VISION] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, collected=");
  SerialDebug.print(
      static_cast<unsigned int>(rawCollectedCount));
  SerialDebug.print(", expected color/request mode=");
  SerialDebug.print(expectedColor);
  // The camera policy is intentionally independent of the A/B mechanical
  // search policy. The current default restores the historical mode-8 request;
  // the main controller still rejects a returned color that is not next in QR
  // order. Policy 1 remains available for targeted single-color experiments.
  const uint8_t requestMode =
      GONGCHUANG_RAW_COLOR_REQUEST_POLICY == 1
          ? expectedColor
          : MAIXCAM_ALL_COLORS_REQUEST;
  SerialDebug.print("/");
  SerialDebug.println(requestMode);

  beginMaixRequest(requestMode);
  workVisionRequestStartMs = millis();
  workActionPhase = WORK_PHASE_RAW_WAIT_RESULT;
}

#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
void beginRawSearchWindow(uint32_t noTargetAllowanceMs) {
  rawCurrentNoTargetAllowanceMs = noTargetAllowanceMs;
  rawNoTargetDeadlineMs = millis() + noTargetAllowanceMs;
  beginRawItemVision();
}

bool startRawSearchZoneMove(
    RawSearchZone targetZone,
    bool finishAfterMove = false) {
  if (!liftMoveFinished() ||
      liftAxis.currentMm <
          M7_STANDARD_HEIGHT_MM - ARM_AXIS_POSITION_TOLERANCE_MM ||
      extensionAxis.active || extensionAxis.recoveryPending ||
      armMotors.isM5Running() || armBaseMotionWatchdogActive ||
      !allMotorsArrived()) {
    routeFault("RAW search move requested while mechanism busy");
    return false;
  }

  const ArmPose targetPose =
      finishAfterMove
          ? ArmPose(
                0.0f,
                M6_STANDARD_EXTENSION_MM,
                M7_STANDARD_HEIGHT_MM)
          : rawSearchPose(targetZone);
  float chassisLeftMm = 0.0f;
  if (targetZone == RAW_SEARCH_ZONE_A && rawChassisAtB) {
    chassisLeftMm = -RAW_SEARCH_CHASSIS_LEFT_MM;
  } else if (targetZone != RAW_SEARCH_ZONE_A && !rawChassisAtB) {
    chassisLeftMm = RAW_SEARCH_CHASSIS_LEFT_MM;
  }

#if GONGCHUANG_DEBUG_RAW
  SerialDebug.print(
      "[RAW SEARCH MOVE] from/to/M5/M6/chassis-left-mm=");
  SerialDebug.print(static_cast<uint8_t>(rawSearchZone));
  SerialDebug.print("/");
  SerialDebug.print(static_cast<uint8_t>(targetZone));
  SerialDebug.print("/");
  SerialDebug.print(targetPose.standardFrameAngleDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(targetPose.extensionMm, 2);
  SerialDebug.print("/");
  SerialDebug.println(chassisLeftMm, 2);
#endif

  stopMaixRequest();
  rawTrackingActive = false;
  useArmBaseRawTransferMotionProfile();
  if (!startExtensionToMmWithProfile(
          targetPose.extensionMm,
          RAW_M6_SPEED_RPM,
          RAW_M6_ACCELERATION)) {
    return false;
  }
  startArmBaseStandardFrameDegrees(
      targetPose.standardFrameAngleDegrees);

  if (fabsf(chassisLeftMm) > 0.01f) {
    const MotorPulses chassisPulses =
        bodyDisplacementToMotorPulses(
            0.0f,
            chassisLeftMm / 1000.0f,
            0.0f);
    visualCorrectionAccumulator =
        addMotorPulses(
            visualCorrectionAccumulator,
            chassisPulses);
    visualCorrectionLeftMm += chassisLeftMm;
    setDriveMotionProfile(
        WORKSTATION_MAXIMUM_STEP_RATE,
        WORKSTATION_STEP_ACCELERATION);
    turnMotionEnabled = false;
    workstationApproachEnabled = true;
    translationPreciseArrivalEnabled = true;
    startRelativeMotorMove(chassisPulses);
  }

  rawSearchMoveTargetZone = targetZone;
  rawFinishAfterSearchMove = finishAfterMove;
  rawNoTargetDeadlineMs = 0UL;
  workActionStartMs = millis();
  workActionPhase = WORK_PHASE_RAW_WAIT_SEARCH_MOVE;
  return true;
}

void advanceRawSearchZone() {
  if (rawSearchZone == RAW_SEARCH_ZONE_A) {
    (void)startRawSearchZoneMove(
        RAW_SEARCH_ZONE_B_CLOCKWISE);
  } else if (rawSearchZone ==
             RAW_SEARCH_ZONE_B_CLOCKWISE) {
    (void)startRawSearchZoneMove(
        RAW_SEARCH_ZONE_B_COUNTERCLOCKWISE);
  } else {
    (void)startRawSearchZoneMove(RAW_SEARCH_ZONE_A);
  }
}

void beginRawPickupTrackingVision() {
  if (activeWorkAction != WORK_ACTION_RAW ||
      rawCollectedCount >= 3U) {
    return;
  }
  const uint8_t expectedColor =
      taskColors[workRoundIndex][rawCollectedCount];
  workLastMaixSequence = maixCam.latest().sequence;
  beginMaixRequest(expectedColor);
  rawTrackingActive = true;
}

float rawTrackingRemainingSeconds() {
  const float commandedMmPerSecond =
      static_cast<float>(liftAxis.commandSpeedRpm) *
      M7_TRAVEL_PER_REVOLUTION_MM / 60.0f *
      RAW_TRACKING_M7_EFFECTIVE_SPEED_SCALE;
  if (commandedMmPerSecond <= 0.0f) {
    return RAW_TRACKING_MINIMUM_REMAINING_SECONDS;
  }
  const float totalDistanceMm =
      fabsf(liftAxis.targetMm - liftAxis.commandStartMm);
  const float elapsedDistanceMm =
      commandedMmPerSecond *
      static_cast<float>(millis() - liftAxis.startMs) /
      1000.0f;
  float remainingSeconds =
      (totalDistanceMm - elapsedDistanceMm) /
      commandedMmPerSecond;
  if (remainingSeconds <
      RAW_TRACKING_MINIMUM_REMAINING_SECONDS) {
    remainingSeconds =
        RAW_TRACKING_MINIMUM_REMAINING_SECONDS;
  }
  return remainingSeconds;
}

void serviceRawPickupTracking() {
  if (!rawTrackingActive ||
      activeWorkAction != WORK_ACTION_RAW ||
      armTransferPhase != ARM_TRANSFER_WAIT_SOURCE_LOWER ||
      !liftAxis.active) {
    return;
  }

  uint8_t detectedColor = 0U;
  int16_t x = 0;
  int16_t y = 0;
  if (!readNewMaixCoordinate(
          workLastMaixSequence,
          detectedColor,
          x,
          y)) {
    return;
  }

  const uint8_t expectedColor =
      taskColors[workRoundIndex][rawCollectedCount];
  // Each Maix color request deliberately returns one stable result. Queue a
  // new sequence immediately so tracking remains live throughout descent.
  workLastMaixSequence = maixCam.latest().sequence;
  beginMaixRequest(expectedColor);
  if (detectedColor != expectedColor) {
    return;
  }

  const float mappingExtensionMm =
      extensionAxis.active
          ? extensionAxis.targetMm
          : extensionAxis.currentMm;
  const ArmPose cameraPose(
      currentArmBaseStandardFrameDegrees(),
      mappingExtensionMm,
      liftAxis.currentMm);
  ArmPose trackedPose;
  if (rawTargetPoseFromCameraPose(
          static_cast<float>(x),
          static_cast<float>(y),
          cameraPose,
          trackedPose) != RAW_TARGET_POSE_VALID) {
    return;
  }

  const float remainingSeconds =
      rawTrackingRemainingSeconds();
  const float m5ErrorDegrees =
      trackedPose.standardFrameAngleDegrees -
      currentArmBaseStandardFrameDegrees();
  const float m5StepsPerDegree =
      fabsf(static_cast<float>(
          armMotors.m5PulsesForDegrees(1.0f) -
          armMotors.m5PulsesForDegrees(0.0f)));
  if (fabsf(m5ErrorDegrees) >
      FINAL_RETURN_M5_TOLERANCE_DEGREES) {
    float m5StepRate =
        fabsf(m5ErrorDegrees) * m5StepsPerDegree /
        remainingSeconds *
        RAW_PICK_TRACKING_VELOCITY_GAIN;
    const float maximumM5StepRate =
        arm_config::M5_RAW_MAXIMUM_STEP_RATE *
        EMPTY_GRIPPER_SPEED_SCALE;
    if (m5StepRate < RAW_TRACKING_MINIMUM_M5_STEP_RATE) {
      m5StepRate = RAW_TRACKING_MINIMUM_M5_STEP_RATE;
    } else if (m5StepRate > maximumM5StepRate) {
      m5StepRate = maximumM5StepRate;
    }
    armMotors.setM5MotionProfile(
        m5StepRate,
        arm_config::M5_RAW_STEP_ACCELERATION *
            EMPTY_GRIPPER_ACCELERATION_SCALE *
            M5_GLOBAL_ACCELERATION_SCALE);
    startArmBaseStandardFrameDegrees(
        trackedPose.standardFrameAngleDegrees);
  }

  const float m6ErrorMm =
      trackedPose.extensionMm - extensionAxis.currentMm;
  if (!extensionAxis.active &&
      !extensionAxis.recoveryPending &&
      armLinearPositionQueryAxis == nullptr &&
      fabsf(m6ErrorMm) > ARM_AXIS_POSITION_TOLERANCE_MM) {
    float m6RpmFloat =
        fabsf(m6ErrorMm) / remainingSeconds *
        RAW_PICK_TRACKING_VELOCITY_GAIN * 60.0f /
        M6_TRAVEL_PER_REVOLUTION_MM;
    if (m6RpmFloat <
        static_cast<float>(RAW_TRACKING_MINIMUM_M6_SPEED_RPM)) {
      m6RpmFloat =
          static_cast<float>(RAW_TRACKING_MINIMUM_M6_SPEED_RPM);
    } else if (m6RpmFloat >
               static_cast<float>(RAW_M6_SPEED_RPM)) {
      m6RpmFloat = static_cast<float>(RAW_M6_SPEED_RPM);
    }
    (void)startLinearAxisMove(
        extensionAxis,
        trackedPose.extensionMm,
        M6_RAW_PICK_MINIMUM_EXTENSION_MM,
        M6_MAXIMUM_EXTENSION_MM,
        M6_PULSES_PER_MM,
        M6_EXTEND_DIRECTION,
        M6_RETRACT_DIRECTION,
        static_cast<uint16_t>(lroundf(m6RpmFloat)),
        RAW_M6_ACCELERATION,
        false,
        M7_PROFILE_APPLY_EXPERIMENT,
        false,
        true);
  }

  rawPendingSourcePose = trackedPose;
  armTransferSourcePose = trackedPose;
}
#endif

inline __attribute__((always_inline)) void beginRawPickupTransfer(
    uint8_t detectedColor,
    uint8_t slotIndex,
    const ArmPose &source) {
  stopMaixRequest();
  workVisionRetryCount = 0U;
  // The vision wait can be indefinite. Start a fresh mechanical-action
  // timeout only after an expected, reachable target is accepted.
  workActionStartMs = millis();
  rawPendingColor = detectedColor;
  rawPendingSlotIndex = slotIndex;
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
  const bool sourceAlreadyPrepared = false;
  rawPickupPrepositionedPending = false;
  rawPendingSourcePose = source;
#else
  const bool sourceAlreadyPrepared =
      rawPickupPrepositionedPending;
  rawPickupPrepositionedPending = false;
  if (!sourceAlreadyPrepared) {
    rawPendingSourcePose = source;
  } else {
    SerialDebug.println(
        "[RAW REUSE] expected material confirmed; retain remembered "
        "M5/M6 and lower M7 directly");
  }
#endif
  const bool prepareNextRawPickup =
      rawCollectedCount < 2U;
#if GONGCHUANG_RAW_VERBOSE_VISION_CALIBRATION
  SerialDebug.print(
      "[RAW DIRECT] latest two-frame color/slot/angle=");
  SerialDebug.print(rawPendingColor);
  SerialDebug.print("/");
  SerialDebug.print(rawPendingSlotIndex);
  SerialDebug.print("/");
  SerialDebug.println(
      STORAGE_SERVO_POSITIONS_DEGREES[
          rawPendingSlotIndex],
      1);
#endif
  armTransferRing1TrayClearanceMode = 0U;
  armTransferRing1TrayRestoreSlot = -1;
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
  const ArmPose nextSearchPose = rawSearchPose(rawSearchZone);
#endif
  beginArmTransfer(
      rawPendingSourcePose,
      arm_transfer::containerPlacePose(),
      false,
      false,
      false,
      false,
      sourceAlreadyPrepared,
      prepareNextRawPickup
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
          ? &nextSearchPose
#else
          ? &rawPendingSourcePose
#endif
          : nullptr,
      prepareNextRawPickup
          ? static_cast<int8_t>(rawCollectedCount + 1U)
          : static_cast<int8_t>(-1),
      false,
      true,
      true);
  activeTransferPurpose =
      TRANSFER_PURPOSE_RAW_TO_CONTAINER;
  workActionPhase = WORK_PHASE_WAIT_TRANSFER;
}

void beginCircleVision() {

  resetCircleStabilityWindow();
  workLastMaixSequence =
      maixCam.latest().sequence;
  beginMaixRequest(MAIXCAM_HOUGH_CIRCLE_REQUEST);
  workVisionRequestStartMs = millis();
  workActionPhase =
      WORK_PHASE_CIRCLE_WAIT_COORDINATE;
}

void beginEndpointVision() {
  workLastMaixSequence =
      maixCam.latest().sequence;
  SerialDebug.print("[ENDPOINT VISION] t/ring=");
  SerialDebug.print(millis());
  SerialDebug.print("/");
  SerialDebug.println(activeEndpointScanRing);
  beginMaixRequest(MAIXCAM_ENDPOINT_CIRCLE_REQUEST);
  workVisionRequestStartMs = millis();
  workActionPhase =
      WORK_PHASE_ENDPOINT_WAIT_COORDINATE;
}

void initializeEndpointScanState(
    uint8_t ringPosition,
    const ArmPose &searchPose) {
  useArmBaseEndpointTravelMotionProfile();
  activeEndpointScanRing = ringPosition;
  activeEndpointScanPose = searchPose;
  endpointFineVisionActive = false;
  activeEndpointServoMoveCount = 0U;
  activeEndpointStageServoMoveCount = 0U;
  endpointSearchFallbackMoveCount = 0U;
  endpointSearchBaseAngleDegrees =
      searchPose.standardFrameAngleDegrees;
  resetEndpointCenteredConfirmationWindow();
  endpointLocalSettleDeadlineMs = 0UL;
  endpointScanStartMs[ringPosition] = millis();
  endpointScanElapsedMs[ringPosition] = 0UL;
  workVisionRetryCount = 0U;
  workVisionRequestStartMs = 0UL;
}

bool startEndpointSearchPlanarMove() {
  if (extensionAxis.active ||
      extensionAxis.recoveryPending ||
      extensionAxis.fault) {
    routeFault("Endpoint M5-first move requires idle M6");
    return false;
  }

  startArmBaseStandardFrameDegrees(
      activeEndpointScanPose.standardFrameAngleDegrees);
  workActionPhase =
      WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE;
  return true;
}

void beginEndpointScan(uint8_t ringPosition) {
  if (ringPosition != 1U && ringPosition != 3U) {
    routeFault("Endpoint scan requires ring 1 or 3");
    return;
  }
  // PROCESS arrives with M6 at the safe 0 mm working zero. STORAGE1 may reuse
  // the verified 90 mm ring-1 search seed that was completed during chassis
  // travel. Both poses feed the same bounded M5-first search; accepting the
  // prepared seed avoids faulting before the mode-10 camera request is sent.
  const bool extensionAtSafeZero =
      fabsf(
          extensionAxis.currentMm -
          M6_STANDARD_EXTENSION_MM) <=
      ARM_AXIS_POSITION_TOLERANCE_MM;
  const bool extensionAtSearchSeed =
      fabsf(
          extensionAxis.currentMm -
          ENDPOINT_SEARCH_SEED_EXTENSION_MM) <=
      ARM_AXIS_POSITION_TOLERANCE_MM;
  if (extensionAxis.active ||
      extensionAxis.recoveryPending ||
      extensionAxis.fault ||
      liftAxis.active ||
      (!extensionAtSafeZero &&
       !extensionAtSearchSeed) ||
      fabsf(
          liftAxis.currentMm -
          ENDPOINT_VISION_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault(
      "Initial endpoint scan requires M6 safe zero/search seed and fixed M7 height");
    return;
  }

  ArmPose searchPose;
  if (!nominalRingPose(
          ringPosition,
          ENDPOINT_FINE_VISION_LOWER_MM,
          searchPose)) {
    return;
  }
  initializeEndpointScanState(ringPosition, searchPose);
  if (!startEndpointSearchPlanarMove()) {
    return;
  }

  SerialDebug.print(
      "[ENDPOINT SCAN] prepared seed ring/M5/M6/M7=");
  SerialDebug.print(ringPosition);
  SerialDebug.print("/");
  SerialDebug.print(
      searchPose.standardFrameAngleDegrees,
      2);
  SerialDebug.print("/");
  SerialDebug.print(searchPose.extensionMm, 2);
  SerialDebug.print("/");
  SerialDebug.println(searchPose.heightMm, 2);
}

bool beginDirectRing3EndpointScan() {
  if (!liftMoveFinished() ||
      fabsf(
          liftAxis.currentMm -
          ENDPOINT_VISION_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("Ring-3 direct handoff requires fixed M7 height");
    return false;
  }

  ArmPose searchPose;
  if (!predictedRing3SearchPose(searchPose)) {
    return false;
  }
  initializeEndpointScanState(3U, searchPose);
  return startEndpointSearchPlanarMove();
}

bool startEndpointSearchFallbackMove() {
  if (endpointFineVisionActive ||
      activeEndpointStageServoMoveCount != 0U ||
      endpointSearchFallbackMoveCount >=
          ENDPOINT_SEARCH_MAXIMUM_FALLBACK_MOVES) {
    return false;
  }

  ++endpointSearchFallbackMoveCount;
  const bool scanningRing1 =
      activeEndpointScanRing == 1U;
  const float seedAngleDegrees =
      scanningRing1
          ? ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES
          : ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES;
  const float fallbackStepDegrees =
      scanningRing1
          ? ENDPOINT_RING1_SEARCH_FALLBACK_STEP_DEGREES
          : ENDPOINT_RING3_SEARCH_FALLBACK_STEP_DEGREES;
  const float fallbackOffsetDirection =
      endpointSearchFallbackMoveCount == 1U
          ? (scanningRing1 ? 1.0f : -1.0f)
          : (scanningRing1 ? -1.0f : 1.0f);

  const float fallbackAngleDegrees =
      seedAngleDegrees +
      fallbackOffsetDirection *
          fallbackStepDegrees;
  const float minimumAngleDegrees =
      scanningRing1
          ? ENDPOINT_RING1_SEARCH_MINIMUM_ANGLE_DEGREES
          : ENDPOINT_RING3_SEARCH_MINIMUM_ANGLE_DEGREES;
  const float maximumAngleDegrees =
      scanningRing1
          ? ENDPOINT_RING1_SEARCH_MAXIMUM_ANGLE_DEGREES
          : ENDPOINT_RING3_SEARCH_MAXIMUM_ANGLE_DEGREES;
  if (fallbackAngleDegrees < minimumAngleDegrees ||
      fallbackAngleDegrees > maximumAngleDegrees ||
      fallbackAngleDegrees <
          RING_SCAN_MINIMUM_ANGLE_DEGREES ||
      fallbackAngleDegrees >
          RING_SCAN_MAXIMUM_ANGLE_DEGREES) {
    routeFault(
        "Endpoint fallback outside ring-specific search window");
    return false;
  }

  stopMaixRequest();
  workVisionRequestStartMs = 0UL;
  workVisionRetryCount = 0U;
  activeEndpointScanPose.standardFrameAngleDegrees =
      fallbackAngleDegrees;
  useArmBaseEndpointTravelMotionProfile();
  startArmBaseStandardFrameDegrees(fallbackAngleDegrees);
  workActionPhase = WORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE;

  SerialDebug.print(
      "[ENDPOINT SEARCH] bounded fallback ring/move/M5=");
  SerialDebug.print(activeEndpointScanRing);
  SerialDebug.print("/");
  SerialDebug.print(endpointSearchFallbackMoveCount);
  SerialDebug.print("/");
  SerialDebug.println(fallbackAngleDegrees, 2);
  return true;
}

void startPreEndpointHeadingCorrection() {
  setDriveMotionProfile(
      WORKSTATION_MAXIMUM_STEP_RATE,
      WORKSTATION_STEP_ACCELERATION);
  turnMotionEnabled = false;
  workstationApproachEnabled = true;
  translationPreciseArrivalEnabled = true;
  stopAllMotorsImmediately();
  commandStartMs = millis();
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
  endpointMapStartMs = millis();
  workActionPhase =
      WORK_PHASE_ENDPOINT_WAIT_PRE_SCAN_HEADING;
  SerialDebug.print("[ENDPOINT IMU] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, heading lock before endpoint map");
}

bool ringMapHeadingStillValid() {
  if (!imuIsFresh()) {
    routeFault("IMU stale while using endpoint map");
    return false;
  }

  const float driftDegrees =
      fabsf(
          wrapDeltaDegrees(
              currentRouteCounterClockwiseHeading() -
              endpointMapLockedHeadingDegrees));
  if (driftDegrees <=
      RING_MAP_MAXIMUM_HEADING_DRIFT_DEGREES) {
    return true;
  }

  SerialDebug.print(
      "[RING MAP] heading drift warning; continue=");
  SerialDebug.println(driftDegrees, 2);
  return true;
}

bool startAccumulatedWorkstationMove(
    float forwardMm,
    float leftMm,
    const char *reason,
    bool restoreOnDeparture = true) {
  const float nextForwardMm =
      visualCorrectionForwardMm + forwardMm;
  const float nextLeftMm =
      visualCorrectionLeftMm + leftMm;
  if (hypotf(nextForwardMm, nextLeftMm) >
          MAXIMUM_ACCUMULATED_VISUAL_CORRECTION_MM ||
      visualCorrectionMoveCount >=
          MAXIMUM_VISUAL_CORRECTION_MOVES) {
    routeFault("Visual correction safety envelope exceeded");
    return false;
  }

  const MotorPulses correctionPulses =
      bodyDisplacementToMotorPulses(
          forwardMm / 1000.0f,
          leftMm / 1000.0f,
          0.0f);
  if (motorPulsesAreZero(correctionPulses)) {
    return false;
  }

  // Reach recovery corrects the chassis position permanently. Only ordinary
  // temporary visual moves belong in the departure return accumulator.
  if (restoreOnDeparture) {
    visualCorrectionAccumulator =
        addMotorPulses(
            visualCorrectionAccumulator,
            correctionPulses);
  }
  visualCorrectionForwardMm = nextForwardMm;
  visualCorrectionLeftMm = nextLeftMm;
  ++visualCorrectionMoveCount;
  stopMaixRequest();
  setDriveMotionProfile(
      WORKSTATION_MAXIMUM_STEP_RATE,
      WORKSTATION_STEP_ACCELERATION);
  turnMotionEnabled = false;
  workstationApproachEnabled = true;
  translationPreciseArrivalEnabled = true;
  startRelativeMotorMove(correctionPulses);
  commandStartMs = millis();
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
  workActionPhase = WORK_PHASE_CIRCLE_WAIT_CHASSIS;

  SerialDebug.print(reason);
  SerialDebug.print(" mm fwd/left: ");
  SerialDebug.print(forwardMm, 2);
  SerialDebug.print(", ");
  SerialDebug.println(leftMm, 2);
  return true;
}

bool beginEndpointM6ReachRecovery(
    LinearAxisMotion &axis,
    bool cameraReachOverflow) {
  const bool endpointMove =
      workActionPhase == WORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION ||
      workActionPhase == WORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION;
  if (LOCAL_WORKSTATION_SIMULATION_MODE ||
      programState != PROGRAM_RUNNING ||
      axis.address != extensionAxis.address ||
      (activeWorkAction != WORK_ACTION_PROCESS &&
       !(activeWorkAction == WORK_ACTION_STORAGE && workRoundIndex == 0U)) ||
      (activeEndpointScanRing != 1U && activeEndpointScanRing != 3U) ||
      (cameraReachOverflow
           ? workActionPhase != WORK_PHASE_ENDPOINT_WAIT_COORDINATE
           : (axis.recoveryReason != ARM_AXIS_RECOVERY_STALL ||
              !endpointMove || axis.targetMm <= axis.commandStartMm ||
              axis.targetMm - axis.currentMm <=
                  ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM))) {
    return false;
  }
  if (endpointReachRecoveryMoves >=
          gongchuang::config::arm_hardware::M6_SCAN_RECOVERY_MAX_MOVES ||
      (cameraReachOverflow && !extensionMoveFinished()) ||
      !allMotorsArrived() || armMotors.isM5Running() ||
      armBaseMotionWatchdogActive || !liftMoveFinished() ||
      !imuIsFresh()) {
    if (cameraReachOverflow) {
      routeFault("M6 scan recovery unavailable");
    } else {
      faultLinearAxis(axis, "M6 scan recovery unavailable");
    }
    return true;
  }

  if (!cameraReachOverflow) {
    // Cancel the stalled target without reporting the old target as reached.
    axis.targetMm = axis.currentMm;
    markLinearAxisArrived(axis);
  }
  activeEndpointScanPose = ArmPose(
      currentArmBaseStandardFrameDegrees(), axis.currentMm, liftAxis.currentMm);
  endpointReachRecoveryForwardMm =
      cameraReachOverflow
          ? gongchuang::config::arm_hardware::
                M6_CAMERA_REACH_RECOVERY_FORWARD_MM
          : 0.0f;
  endpointReachRecoveryLeftMm =
      cameraReachOverflow
          ? 0.0f
          : gongchuang::config::arm_hardware::M6_SCAN_RECOVERY_LEFT_MM;
  // Camera reach approaches are local and are restored before route departure.
  // A confirmed physical stall keeps the established lateral correction.
  endpointReachRecoveryRestoreOnDeparture = cameraReachOverflow;
  stopMaixRequest();
  workVisionRequestStartMs = 0UL;
  resetEndpointCenteredConfirmationWindow();
  endpointLocalSettleDeadlineMs = millis() +
      gongchuang::config::arm_hardware::M6_SCAN_RECOVERY_DELAY_MS;
  workActionPhase = WORK_PHASE_ENDPOINT_REACH_DELAY;
  return true;
}

float circleMmPerPixelForRadius(float pixelRadius) {

  (void)pixelRadius;
  return CIRCLE_MM_PER_PIXEL;
}

void startVisualCorrection(
    int16_t imageX,
    int16_t imageY) {
  const float pixelDeltaX =
      static_cast<float>(imageX - IMAGE_CENTER_X);
  const float pixelDeltaY =
      static_cast<float>(imageY - IMAGE_CENTER_Y);
  const float pixelRadius =
      hypotf(pixelDeltaX, pixelDeltaY);
  const float circleMmPerPixel =
      circleMmPerPixelForRadius(pixelRadius);
  float forwardMm =
      pixelDeltaX *
      circleMmPerPixel *
      IMAGE_X_TO_BODY_FORWARD_SIGN;
  float leftMm =
      pixelDeltaY *
      circleMmPerPixel *
      IMAGE_Y_TO_ARM_OUTWARD_SIGN *
      IMAGE_Y_TO_BODY_LEFT_SIGN;

  const float correctionMagnitudeMm =
      hypotf(forwardMm, leftMm);
  if (correctionMagnitudeMm >
      MAXIMUM_VISUAL_CORRECTION_MM) {
    const float scale =
        MAXIMUM_VISUAL_CORRECTION_MM /
        correctionMagnitudeMm;
    forwardMm *= scale;
    leftMm *= scale;
  }

  if (!startAccumulatedWorkstationMove(
          forwardMm,
          leftMm,
          "Visual chassis correction") &&
      programState == PROGRAM_RUNNING) {
    beginCircleVision();
  }
}

void startPostVisionHeadingCorrection() {

  setDriveMotionProfile(
      WORKSTATION_MAXIMUM_STEP_RATE,
      WORKSTATION_STEP_ACCELERATION);
  turnMotionEnabled = false;
  workstationApproachEnabled = true;
  translationPreciseArrivalEnabled = true;
  stopAllMotorsImmediately();
  commandStartMs = millis();
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
  workActionPhase =
      WORK_PHASE_CIRCLE_WAIT_POST_VISION_HEADING;
  SerialDebug.print("[VISION IMU] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, start post-vision heading lock");
}

// Keep this large, infrequent workstation transition out of the main service
// loop's inlined body. This preserves scarce STM32H750 internal Flash without
// changing any motion timing or state-machine boundary.
void beginUnloadingTransfer() __attribute__((noinline));
void beginUnloadingTransfer() {
  if (!ringMapHeadingStillValid()) {
    return;
  }
  if (endpointDirectContainerPickupPending) {

    if (!endpointInitialStorageCommanded ||
        workStorageServoDeadlineMs == 0UL) {
      routeFault(
          "Initial storage slot was not commanded after ring-1 pose");
      return;
    }
  }

  if (secondStorageColorWorkflowActive()) {
    if (storage2ScanIndex != 3U || workItemIndex >= 3U ||
        !storageServoArrivalGate.active ||
        storageServoArrivalGate.slot != storage2ItemOrder[workItemIndex]) {
      routeFault("S2 map/tray gate mismatch");
      return;
    }
  } else if (endpointDirectContainerPickupPending) {
    if (!deadlineReached(workStorageServoDeadlineMs)) {
      return;
    }
    workStorageServoDeadlineMs = 0UL;
    SerialDebug.println(
        "[TRANSFER SAFE] initial storage slot settled; pickup may start");
  }

  ArmPose destination;
  uint8_t ringPosition = 0U;
  uint8_t transferItemIndex = workItemIndex;
  float lowerMm = PROCESS_PLACE_LOWER_MM;

  if (activeWorkAction == WORK_ACTION_PROCESS) {
    if (!processItemIndexForSequence(
            workItemIndex,
            transferItemIndex)) {
      return;
    }
    ringPosition =
        taskPositions[workRoundIndex][transferItemIndex];
    lowerMm = PROCESS_PLACE_LOWER_MM;
    SerialDebug.print(
        "[PROCESS PLACE QR] item/color/ring/slot=");
    SerialDebug.print(workItemIndex);
    SerialDebug.print("/");
    SerialDebug.print(
        taskColors[workRoundIndex][transferItemIndex]);
    SerialDebug.print("/");
    SerialDebug.print(ringPosition);
    SerialDebug.print("/");
    SerialDebug.println(transferItemIndex);
  } else if (activeWorkAction == WORK_ACTION_STORAGE) {
    ringPosition =
        storageRingForItem(
            workRoundIndex, workItemIndex);
    lowerMm =
        workRoundIndex == 0U
            ? STORAGE_ROUND1_PLACE_LOWER_MM
            : STORAGE_ROUND2_PLACE_LOWER_MM;
  } else {
    routeFault("Unload requested outside ring station");
    return;
  }

  if (!ringPose(ringPosition, lowerMm, destination)) {
    return;
  }
  // PROCESS and STORAGE1 M6 already use one signed final net value below, so
  // their legacy 6/6/9 mm placement reduction must be zero here. STORAGE2
  // retains its independent ground-placement reductions. M7 extra-lower
  // remains inside applyTargetPlacementExtraLower() for every workflow.
  const float mappedRingPlacementExtensionReductionMm =
      processOrStorage1RingNetCorrectionApplies(ringPosition)
          ? 0.0f
          : (secondStorageColorWorkflowActive()
                        ? (ringPosition == 1U
                               ? arm_config::
                                     STORAGE2_RING1_TARGET_PLACE_EXTENSION_REDUCTION_MM
                               : (ringPosition == 2U
                                      ? arm_config::
                                            STORAGE2_MIDDLE_TARGET_PLACE_EXTENSION_REDUCTION_MM
                                      : arm_config::
                                            STORAGE2_RING3_TARGET_PLACE_EXTENSION_REDUCTION_MM))
                        : 0.0f);
  if (!applyTargetPlacementExtraLower(
          destination,
          mappedRingPlacementExtensionReductionMm,
          ringPosition,
          secondStorageColorWorkflowActive())) {
    return;
  }
  if (!applyRingPlacementPoseCorrection(
          destination,
          ringPosition)) {
    return;
  }
  activePlacementRingPosition = ringPosition;
  const bool concurrentSourcePreparation =
      endpointDirectContainerPickupPending &&
      !containerPickupPrepositionedPending;
  endpointDirectContainerPickupPending = false;
  const bool sourceAlreadyPrepared =
      containerPickupPrepositionedPending;
  containerPickupPrepositionedPending = false;
  const bool prepareNextContainerPickup = workItemIndex < 2U;
  const bool prepareFirstPlacedRingPickup =
      activeWorkAction == WORK_ACTION_PROCESS &&
      workItemIndex == 2U;
  bool prepareRing1PickupClearance = false;
  const bool prepareNextSource =
      prepareNextContainerPickup ||
      prepareFirstPlacedRingPickup;
  const ArmPose currentContainerPickupPose =
      arm_transfer::containerPickPose(workItemIndex);
  ArmPose nextSourcePose;
  uint8_t nextStorageSlot =
      static_cast<uint8_t>(workItemIndex + 1U);
  if (prepareNextContainerPickup) {
    if (secondStorageColorWorkflowActive()) {
      nextStorageSlot = storage2ItemOrder[workItemIndex + 1U];
    }
    nextSourcePose =
        arm_transfer::containerPickPose(
            static_cast<uint8_t>(
                workItemIndex + 1U));
    if (activeWorkAction == WORK_ACTION_PROCESS &&
        !processItemIndexForSequence(
            static_cast<uint8_t>(
                workItemIndex + 1U),
             nextStorageSlot)) {
      return;
    }
  } else if (prepareFirstPlacedRingPickup) {
    uint8_t firstTransferItemIndex = 0U;
    if (!processItemIndexForSequence(
            0U,
            firstTransferItemIndex)) {
      return;
    }
    const uint8_t firstRingPosition =
        taskPositions[workRoundIndex][firstTransferItemIndex];
    prepareRing1PickupClearance = firstRingPosition == 1U;
    if (!ringPose(
            firstRingPosition,
            PROCESS_PLACE_LOWER_MM +
                processRingReturnPickExtraLowerMm(),
            nextSourcePose)) {
      return;
    }
    if (!applyRingPositionPoseCorrection(
            nextSourcePose,
            firstRingPosition)) {
      return;
    }
    nextStorageSlot = firstTransferItemIndex;
    SerialDebug.print("[P2P] first ring/slot=");
    SerialDebug.print(firstRingPosition);
    SerialDebug.print("/");
    SerialDebug.println(nextStorageSlot);
  }
  if (workItemIndex == 1U) {
    SerialDebug.print(
        "[TRAY PICK 2] extra M7 lower/target-mm=");
    SerialDebug.print(
        arm_config::SECOND_PICK_EXTRA_LOWER_MM,
        1);
    SerialDebug.print("/");
    SerialDebug.println(
        currentContainerPickupPose.heightMm,
        1);
  }
  const bool ring1TrayClearanceWorkflow =
      activeWorkAction == WORK_ACTION_PROCESS ||
      (activeWorkAction == WORK_ACTION_STORAGE && workRoundIndex == 0U);
  armTransferRing1TrayClearanceMode =
      ring1TrayClearanceWorkflow && ringPosition == 1U
          ? 1U
          : (prepareRing1PickupClearance ? 3U : 0U);
  armTransferRing1TrayRestoreSlot =
      armTransferRing1TrayClearanceMode == 1U
          ? static_cast<int8_t>(
                prepareNextSource ? nextStorageSlot : transferItemIndex)
          : static_cast<int8_t>(-1);
  beginArmTransfer(
      currentContainerPickupPose,
      destination,
      true,
      false,
      true,
      concurrentSourcePreparation,
      sourceAlreadyPrepared,
      prepareNextSource
          ? &nextSourcePose
          : nullptr,
      prepareNextSource
          ? static_cast<int8_t>(
                nextStorageSlot)
          : static_cast<int8_t>(-1),
      false,
      false,
      sourceAlreadyPrepared ||
          concurrentSourcePreparation,
      prepareFirstPlacedRingPickup,
      false,
      secondStorageColorWorkflowActive());
  if (programState == PROGRAM_FAULT) {
    return;
  }
  finalStorageDeparturePending =
      secondStorageColorWorkflowActive() && workItemIndex == 2U &&
      !LOCAL_WORKSTATION_SIMULATION_MODE;
  activeTransferPurpose =
      TRANSFER_PURPOSE_CONTAINER_TO_RING;
  workActionPhase = WORK_PHASE_WAIT_TRANSFER;
}

void beginReloadingTransfer() {
  if (!ringMapHeadingStillValid()) {
    return;
  }

  uint8_t transferItemIndex = 0U;
  if (!processItemIndexForSequence(
          workItemIndex,
          transferItemIndex)) {
    return;
  }
  ArmPose source;
  const uint8_t ringPosition =
      taskPositions[workRoundIndex][transferItemIndex];
  SerialDebug.print(
      "[PROCESS PICK QR] item/color/ring/slot=");
  SerialDebug.print(workItemIndex);
  SerialDebug.print("/");
  SerialDebug.print(
      taskColors[workRoundIndex][transferItemIndex]);
  SerialDebug.print("/");
  SerialDebug.print(ringPosition);
  SerialDebug.print("/");
  SerialDebug.println(transferItemIndex);
  if (!ringPose(
          ringPosition,
          PROCESS_PLACE_LOWER_MM +
              processRingReturnPickExtraLowerMm(),
          source)) {
    return;
  }
  if (!applyRingPositionPoseCorrection(
          source,
          ringPosition)) {
    return;
  }
  SerialDebug.print("[RR PICK] dz/z=");
  SerialDebug.print(
      processRingReturnPickExtraLowerMm(),
      1);
  SerialDebug.print("/");
  SerialDebug.println(source.heightMm, 1);

  const bool sourceAlreadyPrepared =
      ringPickupPrepositionedPending;
  ringPickupPrepositionedPending = false;
  const bool prepareNextRingPickup =
      workItemIndex < 2U;
  bool prepareRing1PickupClearance = false;
  ArmPose nextRingPickupPose;
  uint8_t nextStorageSlot = 0U;
  if (prepareNextRingPickup) {
    uint8_t nextTransferItemIndex = 0U;
    if (!processItemIndexForSequence(
            static_cast<uint8_t>(workItemIndex + 1U),
            nextTransferItemIndex)) {
      return;
    }
    const uint8_t nextRingPosition =
        taskPositions[workRoundIndex][nextTransferItemIndex];
    prepareRing1PickupClearance = nextRingPosition == 1U;
    if (!ringPose(
            nextRingPosition,
            PROCESS_PLACE_LOWER_MM +
                processRingReturnPickExtraLowerMm(),
            nextRingPickupPose)) {
      return;
    }
    if (!applyRingPositionPoseCorrection(
            nextRingPickupPose,
            nextRingPosition)) {
      return;
    }
    nextStorageSlot = nextTransferItemIndex;
    SerialDebug.print(
        "[RETURN PIPELINE] next ring/slot preposition=");
    SerialDebug.print(nextRingPosition);
    SerialDebug.print("/");
    SerialDebug.println(nextStorageSlot);
  }

  armTransferRing1TrayClearanceMode =
      ringPosition == 1U
          ? 2U
          : (prepareRing1PickupClearance ? 3U : 0U);
  armTransferRing1TrayRestoreSlot =
      armTransferRing1TrayClearanceMode == 2U
          ? static_cast<int8_t>(transferItemIndex)
          : static_cast<int8_t>(-1);
  if (armTransferRing1TrayClearanceMode == 2U) {
    // Hold the tray at the fixed departure-angle-minus-180 (-15 degree)
    // clearance before any ring-1 M7 descent.
    commandRing1TrayClearancePosition();
  }

  beginArmTransfer(
      source,
      arm_transfer::containerReturnPlacePose(),
      false,
      true,
      false,
      false,
      sourceAlreadyPrepared,
      prepareNextRingPickup
          ? &nextRingPickupPose
          : nullptr,
      prepareNextRingPickup
          ? static_cast<int8_t>(nextStorageSlot)
          : static_cast<int8_t>(-1),
      false,
      false,
      true,
      true,
      workItemIndex == 0U,
      false,
      ringPosition);
  activeTransferPurpose =
      TRANSFER_PURPOSE_RING_TO_CONTAINER;
  workActionPhase = WORK_PHASE_WAIT_TRANSFER;
}

void startVisualCorrectionRestore();

void completeTransferAndRotateStorage() {
  markMissionProgress();
  switch (activeTransferPurpose) {
    case TRANSFER_PURPOSE_RAW_TO_CONTAINER: {
      ++correctGrabCount;
      hmiSetTaskCounts();
      const bool preparedNextRawPickup =
          armTransferNextSourcePreparedAtEnd &&
          armTransferNextStorageCommanded;
      consumeArmTransferCompletion();

      rawFilledSlotMask = static_cast<uint8_t>(
          rawFilledSlotMask |
          (1U << rawPendingSlotIndex));
      ++rawCollectedCount;
      SerialDebug.print(
          "[RAW STORED] color/slot/count=");
      SerialDebug.print(rawPendingColor);
      SerialDebug.print("/");
      SerialDebug.print(rawPendingSlotIndex);
      SerialDebug.print("/");
      SerialDebug.println(rawCollectedCount);

      if (rawCollectedCount >= 3U) {
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
        // If the final pickup was performed from B, finish the matching
        // 40 mm chassis retreat while M5/M6 take the normal travel-safe zero
        // before releasing the route. This makes the A/B offset reversible.
        if (rawChassisAtB) {
          (void)startRawSearchZoneMove(
              RAW_SEARCH_ZONE_A,
              true);
          return;
        }
#endif
        beginRawTravelParkingAfterFinalStore();
      } else {
        if (!preparedNextRawPickup) {
          routeFault(
              "RAW next pickup M5/M6 pose was not retained");
          return;
        }
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
        // M5/M6 returned to the scan zone, not to the last pickup point. The
        // next material therefore gets a fresh pixel-to-arm solution.
        rawPickupPrepositionedPending = false;
        rawCurrentNoTargetAllowanceMs =
            RAW_POST_PICK_NO_TARGET_MS;
#else
        rawPickupPrepositionedPending = true;
#endif
        workVisionRetryCount = 0U;
        workActionPhase =
            WORK_PHASE_RAW_WAIT_EXPECTED_SLOT;
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 0
        SerialDebug.println(
            "[RAW REUSE] remembered M5/M6 ready; waiting only for "
            "the already-commanded next tray slot");
#endif
      }
      return;
    }

    case TRANSFER_PURPOSE_CONTAINER_TO_RING:
      if (activePlacementRingPosition < 1U ||
          activePlacementRingPosition > 3U) {
        routeFault("Completed placement has no valid ring position");
        return;
      }
      // Save the vision-derived base pose, then let every real placement/pick
      // add its applicable ring correction once through the shared helper.
      measuredRingPoses[activePlacementRingPosition] =
          armTransferDestinationPose;
      removeRingPositionPoseCorrectionForSavedBase(
          measuredRingPoses[activePlacementRingPosition],
          activePlacementRingPosition);
      measuredRingPoseValid[activePlacementRingPosition] = true;
      SerialDebug.print(
          "[RING PLACE POSE SAVED] ring/M5/M6=");
      SerialDebug.print(activePlacementRingPosition);
      SerialDebug.print("/");
      SerialDebug.print(
          armTransferDestinationPose.standardFrameAngleDegrees,
          2);
      SerialDebug.print("/");
      SerialDebug.println(
          armTransferDestinationPose.extensionMm,
          2);
      activePlacementRingPosition = 0U;
      ++correctPlacementCount;
      break;

    case TRANSFER_PURPOSE_RING_TO_CONTAINER:
      ++correctGrabCount;
      break;

    case TRANSFER_PURPOSE_NONE:
      routeFault("Completed transfer has no purpose");
      return;
  }
  hmiSetTaskCounts();

  const bool preparedNextSource =
      (activeTransferPurpose ==
           TRANSFER_PURPOSE_CONTAINER_TO_RING ||
       activeTransferPurpose ==
           TRANSFER_PURPOSE_RING_TO_CONTAINER) &&
      armTransferNextSourcePreparedAtEnd &&
      armTransferNextStorageCommanded;
  const bool preparedFirstPlacedRingPickup =
      preparedNextSource &&
      activeWorkAction == WORK_ACTION_PROCESS &&
      activeTransferPurpose ==
          TRANSFER_PURPOSE_CONTAINER_TO_RING &&
      workItemIndex == 2U;
  const uint32_t preparedStorageDeadlineMs =
      armTransferNextStorageDeadlineMs;
  consumeArmTransferCompletion();
  ++workItemIndex;

  if (storage2ReturnTravelActive && secondStorageColorWorkflowActive() &&
      workItemIndex == 3U) {
    // M7 has verified clearance. M6 and tray retract while chassis returns.
    startVisualCorrectionRestore();
    return;
  }

  if (preparedNextSource) {
    if (preparedFirstPlacedRingPickup) {
      workItemIndex = 0U;
      containerPickupPrepositionedPending = false;
      ringPickupPrepositionedPending = true;
      activeTransferPurpose =
          TRANSFER_PURPOSE_RING_TO_CONTAINER;
      SerialDebug.println(
          "[P2 PIPE] pickup");
    } else if (workItemIndex >= 3U) {
      routeFault(
          "Unexpected next-pick preparation after final item");
      return;
    } else if (activeTransferPurpose ==
               TRANSFER_PURPOSE_CONTAINER_TO_RING) {
      containerPickupPrepositionedPending = true;
    } else {
      ringPickupPrepositionedPending = true;
    }
    workStorageServoDeadlineMs =
        preparedStorageDeadlineMs;
    workActionPhase = secondStorageColorWorkflowActive()
        ? WORK_PHASE_START_UNLOAD : WORK_PHASE_WAIT_STORAGE_SERVO;
    SerialDebug.println(
        "[XFER PIPE] next");
    return;
  }

  const bool finishedFinalContainerSequence =
      workItemIndex >= 3U &&
      (activeTransferPurpose ==
           TRANSFER_PURPOSE_RING_TO_CONTAINER ||
       (activeWorkAction == WORK_ACTION_STORAGE &&
        activeTransferPurpose ==
            TRANSFER_PURPOSE_CONTAINER_TO_RING));
  const bool processRound1TravelOwnsTrayCommand =
      finishedFinalContainerSequence &&
      activeWorkAction == WORK_ACTION_PROCESS &&
      workRoundIndex == 0U;
  const bool storageRound1RawTravelOwnsTrayCommand =
      finishedFinalContainerSequence &&
      activeWorkAction == WORK_ACTION_STORAGE &&
      workRoundIndex == 0U;
  if (finishedFinalContainerSequence) {
    // PROCESS1 immediately hands the tray to its concurrent STORAGE1 travel
    // preparation, which issues one 30%-speed slot-0 command.  Avoid starting
    // a parking move that would be overwritten a few loops later. PROCESS2
    // has no following tray target, so it starts one 30%-speed park command.
    // STORAGE1 also defers its park command: M7 must reach zero first, then
    // the 100 us exit handoff starts M6, tray parking and chassis departure.
    if (!processRound1TravelOwnsTrayCommand &&
        !storageRound1RawTravelOwnsTrayCommand) {
      commandStorageServoParkingPosition(
          activeWorkAction == WORK_ACTION_PROCESS
              ? DEPARTURE_STORAGE_SERVO_INTERVAL_MS
              : STORAGE_SERVO_INTERVAL_MS);
    }
    if (!startLiftToHeightMm(
            M7_STANDARD_HEIGHT_MM)) {
      return;
    }
    if (process2FinalStorageMayOverlapDeparture()) {
      beginProcess2DepartureImuPrelock();
    }
  } else {
    if (secondStorageColorWorkflowActive()) {
      routeFault("S2 next tray pose missing");
      return;
    }
    uint8_t nextStorageSlot = workItemIndex;
    if (activeWorkAction == WORK_ACTION_PROCESS) {
      const uint8_t nextSequenceIndex =
          workItemIndex >= 3U
              ? 0U
              : workItemIndex;
      if (!processItemIndexForSequence(
              nextSequenceIndex,
              nextStorageSlot)) {
        return;
      }
    }
    commandStorageServoPosition(nextStorageSlot);
  }
  workStorageServoDeadlineMs =
      (processRound1TravelOwnsTrayCommand ||
       storageRound1RawTravelOwnsTrayCommand)
          ? 0UL
          : millis() +
                ((finishedFinalContainerSequence &&
                  activeWorkAction == WORK_ACTION_PROCESS)
                     ? DEPARTURE_STORAGE_SERVO_SETTLE_MS
                     : STORAGE_SERVO_SETTLE_MS);
  workActionPhase = WORK_PHASE_WAIT_STORAGE_SERVO;
}

void startVisualCorrectionRestore() {
  stopMaixRequest();
  if (STORAGE_ROUND2_SIMULATION_MODE) {
    // Mode 2 is an arm/vision-only bench run.  Even if a future storage
    // change accidentally accumulates a visual chassis correction, discard
    // it here instead of ever issuing wheel pulses.
    route_chassis::stopAllMotorsImmediately();
    stopAllMotorsImmediately();
    visualCorrectionAccumulator = MotorPulses();
    visualCorrectionForwardMm = 0.0f;
    visualCorrectionLeftMm = 0.0f;
    visualCorrectionMoveCount = 0U;
    commandStartMs = millis();
    headingStableStartMs = 0UL;
    motorsArrivedStartMs = 0UL;
    workActionPhase = WORK_PHASE_WAIT_RESTORE;
    return;
  }

  if (ROUGH_PROCESSING_CALIBRATION_MODE ||
      STORAGE_ROUND1_SIMULATION_MODE) {
    stopAllMotorsImmediately();
    visualCorrectionAccumulator = MotorPulses();
    visualCorrectionForwardMm = 0.0f;
    visualCorrectionLeftMm = 0.0f;
    visualCorrectionMoveCount = 0U;
    beginStorageParkingBeforeWorkFinish();
    return;
  }

  if (motorPulsesAreZero(
          visualCorrectionAccumulator)) {

    setDriveMotionProfile(
        WORKSTATION_MAXIMUM_STEP_RATE,
        WORKSTATION_STEP_ACCELERATION);
    turnMotionEnabled = false;
    workstationApproachEnabled = true;
    translationPreciseArrivalEnabled = true;
    stopAllMotorsImmediately();
    commandStartMs = millis();
    headingStableStartMs = 0UL;
    motorsArrivedStartMs = 0UL;
    workActionPhase = WORK_PHASE_WAIT_RESTORE;
    return;
  }

  setDriveMotionProfile(
      WORKSTATION_MAXIMUM_STEP_RATE,
      WORKSTATION_STEP_ACCELERATION);
  turnMotionEnabled = false;
  workstationApproachEnabled = true;
  translationPreciseArrivalEnabled = true;
  startRelativeMotorMove(
      negateMotorPulses(
          visualCorrectionAccumulator));
  commandStartMs = millis();
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
  workActionPhase = WORK_PHASE_WAIT_RESTORE;
}

uint32_t activeWorkActionTimeoutMs() {

  if (LOCAL_WORKSTATION_SIMULATION_MODE) {
    return 0UL;
  }

  switch (activeWorkAction) {
    case WORK_ACTION_RAW:
      // A missing RAW target is a valid stationary wait. Mechanical RAW
      // phases retain the normal timeout after a target is accepted.
      return workActionPhase == WORK_PHASE_RAW_WAIT_RESULT
                 ? 0UL
                 : RAW_ACTION_TIMEOUT_MS;
    case WORK_ACTION_PROCESS:
      return PROCESS_ACTION_TIMEOUT_MS;
    case WORK_ACTION_STORAGE:
      return STORAGE_ACTION_TIMEOUT_MS;
    case WORK_ACTION_NONE:
      return 0UL;
  }
  return 0UL;
}

void serviceCompetitionAction() {
  if (programState != PROGRAM_RUNNING ||
      activeWorkAction == WORK_ACTION_NONE) {
    return;
  }

  const uint32_t nowMs = millis();
  const uint32_t actionTimeoutMs =
      activeWorkActionTimeoutMs();
  if (actionTimeoutMs > 0UL &&
      nowMs - workActionStartMs >= actionTimeoutMs) {
    routeFault("Workstation action timeout");
    return;
  }

  if (workActionPhase == WORK_PHASE_RAW_WAIT_RESULT &&
      workVisionRequestStartMs != 0UL &&
      nowMs - workVisionRequestStartMs >=
          RAW_TARGET_REQUEST_REFRESH_MS) {

    if (workVisionRetryCount < UINT8_MAX) {
      ++workVisionRetryCount;
    }
    SerialDebug.print(
        "[RAW WAIT] refresh same expected color, count=");
    SerialDebug.println(workVisionRetryCount);
    beginRawItemVision();
    return;
  }

  if ((workActionPhase == WORK_PHASE_CIRCLE_WAIT_COORDINATE ||
       workActionPhase ==
            WORK_PHASE_ENDPOINT_WAIT_COORDINATE) &&
      workVisionRequestStartMs != 0UL &&
      nowMs - workVisionRequestStartMs >=
          (workActionPhase ==
                   WORK_PHASE_ENDPOINT_WAIT_COORDINATE
               ? ENDPOINT_VISION_RESULT_TIMEOUT_MS
               : VISION_RESULT_TIMEOUT_MS)) {
    if (workActionPhase ==
            WORK_PHASE_ENDPOINT_WAIT_COORDINATE &&
        !endpointFineVisionActive &&
        activeEndpointStageServoMoveCount == 0U) {
      if (startEndpointSearchFallbackMove()) {
        return;
      }
      routeFault(
          "Endpoint not found in bounded local search window");
      return;
    }
    const bool endpointVisionPhase =
        workActionPhase ==
        WORK_PHASE_ENDPOINT_WAIT_COORDINATE;
    const uint8_t visionRetryLimit =
        endpointVisionPhase
            ? ENDPOINT_VISION_MAXIMUM_RETRIES
            : VISION_MAXIMUM_RETRIES;
    if (workVisionRetryCount >= visionRetryLimit) {
      routeFault("Vision result timeout");
      return;
    }

    ++workVisionRetryCount;
    SerialDebug.print("[VISION RETRY] ");
    SerialDebug.print(workVisionRetryCount);
    SerialDebug.print("/");
    SerialDebug.println(visionRetryLimit);
    if (workActionPhase ==
        WORK_PHASE_ENDPOINT_WAIT_COORDINATE) {
      beginEndpointVision();
    } else {
      beginCircleVision();
    }
    return;
  }

#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
  serviceRawPickupTracking();
#endif
  serviceArmTransfer();

  switch (workActionPhase) {
    case WORK_PHASE_IDLE:
      break;

    case WORK_PHASE_ENDPOINT_REACH_DELAY:
      if (!deadlineReached(endpointLocalSettleDeadlineMs)) {
        break;
      }
      if (!extensionMoveFinished() || !liftMoveFinished() ||
          armMotors.isM5Running() || !allMotorsArrived() || !imuIsFresh()) {
        routeFault("M6 scan recovery unavailable");
        break;
      }
      if (startAccumulatedWorkstationMove(
              endpointReachRecoveryForwardMm,
              endpointReachRecoveryLeftMm,
              "[M6 REACH]",
              endpointReachRecoveryRestoreOnDeparture)) {
        ++endpointReachRecoveryMoves;
        workActionPhase = WORK_PHASE_ENDPOINT_REACH_MOVE;
      }
      break;

    case WORK_PHASE_ENDPOINT_REACH_MOVE:
      if (millis() - commandStartMs >=
          gongchuang::config::arm_hardware::M6_SCAN_RECOVERY_MOVE_TIMEOUT_MS) {
        routeFault("M6 scan chassis timeout");
        break;
      }
      if (!allMotorsArrived()) {
        break;
      }
      // No turn is commanded: M5 holds its angle throughout. Do not reuse an
      // endpoint observed before translation, or silently accept yaw drift.
      if (!imuIsFresh() ||
          fabsf(wrapDeltaDegrees(currentRouteCounterClockwiseHeading() -
                                 endpointMapLockedHeadingDegrees)) >
              RING_ENDPOINT_MAXIMUM_SCAN_HEADING_DELTA_DEGREES) {
        routeFault("M6 scan recovery heading changed");
        break;
      }
      resetMeasuredRingMap();
      endpointFineVisionActive = false;
      activeEndpointServoMoveCount = 0U;
      activeEndpointStageServoMoveCount = 0U;
      endpointSearchFallbackMoveCount = 0U;
      endpointSearchBaseAngleDegrees =
          activeEndpointScanPose.standardFrameAngleDegrees;
      workVisionRetryCount = 0U;
      markMissionProgress();
      SerialDebug.println("[M6 REACH] arrived; rescan");
      beginEndpointVision();
      break;

    case WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO:
      if (armMotors.isM5Running() ||
          armBaseMotionWatchdogActive) {
        break;
      }
      processM5ZeroSettleDeadlineUs =
          micros() + PROCESS_ENTRY_M5_SETTLE_US;
      workActionPhase =
          WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO_SETTLE;
      SerialDebug.print("[PROCESS ENTRY] t=");
      SerialDebug.print(millis());
      SerialDebug.println(
          " ms, standard M5 0 deg pulse target reached; "
          "confirming 0.1 ms PROCESS-entry handoff");
      break;

    case WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO_SETTLE:
      if (static_cast<int32_t>(
              micros() - processM5ZeroSettleDeadlineUs) < 0 ||
          armMotors.isM5Running() ||
          armBaseMotionWatchdogActive) {
        break;
      }
      rawTravelM5ZeroPending = false;
      processM5ZeroSettleDeadlineUs = 0UL;
      beginArmEndpointPreparation();
      workActionPhase = WORK_PHASE_PREPARE;
      SerialDebug.print("[PROCESS ENTRY] t=");
      SerialDebug.print(millis());
      SerialDebug.println(
          " ms, standard M5 0 deg confirmed -> ring 1");
      break;

    case WORK_PHASE_PREPARE:
      if (activeWorkAction == WORK_ACTION_RAW) {
        if (!serviceArmStandardization()) {
          break;
        }
        armStandardPhase = ARM_STANDARD_IDLE;
        if (rawInitialStorageArrivalOverlapPending) {
          SerialDebug.println(
              "[RAW STORAGE OVERLAP] arm ready; start vision/pick "
              "while initial tray slot moves independently");
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
          beginRawSearchWindow(
              RAW_INITIAL_A_NO_TARGET_MS);
#else
          beginRawItemVision();
#endif
          break;
        }
        // The 200 ms RAW tray command is never trusted as a blind delay.
        // Query only after the arm is stationary and require two real-angle
        // confirmations before vision can lead into a pickup.
        if (!serviceStorageServoArrivalGate()) {
          break;
        }
        resetStorageServoArrivalGate();
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
        beginRawSearchWindow(
            RAW_INITIAL_A_NO_TARGET_MS);
#else
        beginRawItemVision();
#endif
        break;
      }

      if (serviceArmStandardization()) {
        armStandardPhase = ARM_STANDARD_IDLE;

        SerialDebug.print("[WORK PREPARE] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, M5/M7 ring-1 pre-pose ready; "
            "storage arrival remains independently gated");

        startPreEndpointHeadingCorrection();
      }
      break;

    case WORK_PHASE_RAW_WAIT_RESULT: {
      uint8_t detectedColor = 0U;
      int16_t x = 0;
      int16_t y = 0;
      if (!readNewMaixCoordinate(
              workLastMaixSequence,
              detectedColor,
              x,
              y)) {
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
        if (rawNoTargetDeadlineMs != 0UL &&
            deadlineReached(rawNoTargetDeadlineMs)) {
          advanceRawSearchZone();
          break;
        }
#endif
        // Both RAW visits use the camera wait to verify the independently
        // moving tray.
        // A failed UART query is allowed only while every arm axis is stopped;
        // otherwise its SDK timeout could starve M5 software step pulses.
        if (rawInitialStorageArrivalOverlapPending &&
            !storageServoArrivalGate.confirmed &&
            !armMotors.isM5Running() &&
            !armBaseMotionWatchdogActive &&
            extensionMoveFinished() &&
            liftMoveFinished()) {
          (void)serviceStorageServoArrivalGate();
        }
        break;
      }
      workVisionRequestStartMs = millis();
      markMissionProgress();

      if (rawCollectedCount >= 3U) {
        routeFault("Raw item index overflow");
        break;
      }

      const uint8_t expectedColor =
          taskColors[workRoundIndex][rawCollectedCount];
      if (detectedColor != expectedColor) {
        SerialDebug.print(
            "[RAW WAIT] expected/detected color=");
        SerialDebug.print(expectedColor);
        SerialDebug.print("/");
        SerialDebug.println(detectedColor);
        beginRawItemVision();
        break;
      }
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
      // A real expected-color signal breaks the continuous-absence interval,
      // even when this particular coordinate is outside arm travel.
      rawNoTargetDeadlineMs =
          nowMs + rawCurrentNoTargetAllowanceMs;
#endif

      const int8_t slot =
          rawStorageSlotForColor(detectedColor);
      if (slot < 0) {
        SerialDebug.print(
            "[RAW IGNORE] color not in this QR batch: ");
        SerialDebug.println(detectedColor);
        beginRawItemVision();
        break;
      }

      const uint8_t slotIndex =
          static_cast<uint8_t>(slot);
      if (slotIndex != rawCollectedCount) {
        routeFault("Raw color-to-slot order mismatch");
        break;
      }
      const uint8_t slotBit =
          static_cast<uint8_t>(1U << slotIndex);
      if ((rawFilledSlotMask & slotBit) != 0U) {
        SerialDebug.print(
            "[RAW IGNORE] slot already filled, color/slot=");
        SerialDebug.print(detectedColor);
        SerialDebug.print("/");
        SerialDebug.println(slotIndex);
        beginRawItemVision();
        break;
      }

      ArmPose source = rawPendingSourcePose;
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
      const ArmPose actualCameraPose(
          currentArmBaseStandardFrameDegrees(),
          extensionAxis.currentMm,
          liftAxis.currentMm);
      const RawTargetPoseResult poseResult =
          rawTargetPoseFromCameraPose(
              static_cast<float>(x),
              static_cast<float>(y),
              actualCameraPose,
              source);
      if (poseResult != RAW_TARGET_POSE_VALID) {
        beginRawItemVision();
        break;
      }
#else
      if (!rawPickupPrepositionedPending) {
        const RawTargetPoseResult poseResult =
            rawTargetPose(
                static_cast<float>(x),
                static_cast<float>(y),
                source);
        if (poseResult != RAW_TARGET_POSE_VALID) {

          beginRawItemVision();
          break;
        }
      } else {
        SerialDebug.println(
            "[RAW REUSE] next material arrived at the remembered "
            "pickup position");
      }
#endif

      beginRawPickupTransfer(
          detectedColor,
          slotIndex,
          source);
      break;
    }

    case WORK_PHASE_RAW_WAIT_EXPECTED_SLOT:
      if (!serviceStorageServoArrivalGate()) {
        break;
      }
      resetStorageServoArrivalGate();
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
      beginRawSearchWindow(
          rawCurrentNoTargetAllowanceMs);
#else
      beginRawItemVision();
#endif
      break;

    case WORK_PHASE_RAW_WAIT_SEARCH_MOVE:
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
      if (!allMotorsArrived() ||
          armMotors.isM5Running() ||
          armBaseMotionWatchdogActive ||
          !extensionMoveFinished() ||
          !liftMoveFinished()) {
        break;
      }
      if (rawSearchMoveTargetZone == RAW_SEARCH_ZONE_A &&
          (!motorPulsesAreZero(visualCorrectionAccumulator) ||
           fabsf(visualCorrectionLeftMm) > 0.01f)) {
        routeFault("RAW 40 mm chassis return did not cancel");
        break;
      }
      rawSearchZone = rawSearchMoveTargetZone;
      rawChassisAtB =
          rawSearchZone != RAW_SEARCH_ZONE_A;
      markMissionProgress();
      if (rawFinishAfterSearchMove) {
        rawFinishAfterSearchMove = false;
        beginRawTravelParkingAfterFinalStore();
      } else {
        beginRawSearchWindow(
            RAW_NORMAL_NO_TARGET_MS);
      }
#else
      routeFault("RAW search state unavailable in legacy mode");
#endif
      break;

    case WORK_PHASE_RAW_WAIT_TRAVEL_LINEAR_ZERO:
      if (!liftMoveFinished()) {
        break;
      }
      startRawTravelParkingAtSafeLinearZero();
      break;

    case WORK_PHASE_ENDPOINT_WAIT_PRE_SCAN_HEADING:
      if (updateHeadingLock(
              MOTION_TIMEOUT_MS,
              ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS,
              ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS)) {
        endpointMapLockedHeadingDegrees =
            currentRouteCounterClockwiseHeading();
        SerialDebug.print(
            "[ENDPOINT IMU] locked heading/error=");
        SerialDebug.print(
            endpointMapLockedHeadingDegrees,
            2);
        SerialDebug.print("/");
        SerialDebug.println(headingErrorDegrees(), 2);
        beginEndpointScan(1U);
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE:
      if (!armMotors.isM5Running()) {
        armStandardDeadlineMs =
            millis() + ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS;
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE_SETTLE;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE_SETTLE:
      if (deadlineReached(armStandardDeadlineMs) &&
          startExtensionToMmWithProfile(
              activeEndpointScanPose.extensionMm,
              ENDPOINT_COARSE_M6_SPEED_RPM,
              ENDPOINT_COARSE_M6_ACCELERATION)) {
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION:
      if (extensionMoveFinished()) {
        activeEndpointScanPose.extensionMm =
            extensionAxis.currentMm;
        // Endpoint preparation already verified the single 105 mm camera
        // height. Do not issue a redundant M7 command after either planar
        // search move.
        activeEndpointScanPose.heightMm =
            ENDPOINT_VISION_HEIGHT_MM;
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_ARM_LOWER;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_ARM_LOWER:
      if (liftMoveFinished()) {
        SerialDebug.print(
            "[ENDPOINT SCAN] arm ready ring/M5/M6/M7=");
        SerialDebug.print(activeEndpointScanRing);
        SerialDebug.print("/");
        SerialDebug.print(
            activeEndpointScanPose
                .standardFrameAngleDegrees,
            2);
        SerialDebug.print("/");
        SerialDebug.print(
            activeEndpointScanPose.extensionMm,
            2);
        SerialDebug.print("/");
        SerialDebug.println(
            activeEndpointScanPose.heightMm,
            2);
        endpointLocalSettleDeadlineMs =
            millis() + ENDPOINT_LOCAL_MOVE_SETTLE_MS;
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE;
        SerialDebug.print(
            "[ENDPOINT SCAN] full coarse pose reached; "
            "re-recognition settle-ms=");
        SerialDebug.println(ENDPOINT_LOCAL_MOVE_SETTLE_MS);
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_COORDINATE: {
      uint8_t targetId = 0U;
      int16_t x = 0;
      int16_t y = 0;
      if (!readNewMaixCoordinate(
              workLastMaixSequence,
              targetId,
              x,
              y)) {
        break;
      }
      (void)targetId;

      if (activeEndpointScanRing != 1U &&
          activeEndpointScanRing != 3U) {
        routeFault("Endpoint response has no active ring");
        break;
      }

      const uint16_t radiusPixels =
          maixCam.latest().metric;
      const uint16_t confidence =
          maixCam.latest().confidence;
      if (confidence <
          RING_ENDPOINT_MINIMUM_CONFIDENCE) {
        SerialDebug.print(
            "[ENDPOINT MAP] rejected confidence=");
        SerialDebug.println(confidence);
        break;
      }

      PlanarPoint measuredPoint;
      if (!endpointVisionToPlanarPoint(
              activeEndpointScanPose,
              x,
              y,
              radiusPixels,
              measuredPoint)) {
        break;
      }

      const float headingNowDegrees =
          currentRouteCounterClockwiseHeading();
      const float headingDriftDegrees =
          fabsf(
              wrapDeltaDegrees(
                  headingNowDegrees -
                  endpointMapLockedHeadingDegrees));
      if (headingDriftDegrees >
          RING_MAP_MAXIMUM_HEADING_DRIFT_DEGREES) {
        routeFault(
            "Chassis drifted during endpoint scan");
        break;
      }

      workVisionRequestStartMs = millis();
      markMissionProgress();
      const uint8_t ring = activeEndpointScanRing;
      const float pixelDeltaX =
          static_cast<float>(x - IMAGE_CENTER_X);
      const float pixelDeltaY =
          static_cast<float>(y - IMAGE_CENTER_Y);
      const float centerErrorPixels =
          hypotf(pixelDeltaX, pixelDeltaY);
      const float centerTolerancePixels =
          endpointFineVisionActive
              ? ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS
              : ENDPOINT_COARSE_CENTER_TOLERANCE_PIXELS;

      if (centerErrorPixels > centerTolerancePixels) {
        resetEndpointCenteredConfirmationWindow();
        if (activeEndpointStageServoMoveCount >=
                ENDPOINT_MAXIMUM_SERVO_MOVES_PER_STAGE ||
            activeEndpointServoMoveCount >=
                ENDPOINT_MAXIMUM_TOTAL_SERVO_MOVES) {
          SerialDebug.print(
              "[ENDPOINT SERVO] fail ring/stage/steps/total/px=");
          SerialDebug.print(ring);
          SerialDebug.print("/");
          SerialDebug.print(
              endpointFineVisionActive ? "fine" : "coarse");
          SerialDebug.print("/");
          SerialDebug.print(activeEndpointStageServoMoveCount);
          SerialDebug.print("/");
          SerialDebug.print(activeEndpointServoMoveCount);
          SerialDebug.print("/");
          SerialDebug.println(centerErrorPixels, 2);
          routeFault(
              "Endpoint servo did not converge");
          break;
        }

        ArmPose targetPose;
        float measuredCorrectionMm = 0.0f;
        float commandedCorrectionMm = 0.0f;
        if (!endpointServoTargetPose(
                activeEndpointScanPose,
                measuredPoint,
                endpointFineVisionActive,
                targetPose,
                measuredCorrectionMm,
                commandedCorrectionMm)) {
          if (programState == PROGRAM_RUNNING &&
              targetPose.extensionMm >
                  M6_MAXIMUM_EXTENSION_MM +
                      ARM_AXIS_POSITION_TOLERANCE_MM &&
              !beginEndpointM6ReachRecovery(extensionAxis, true)) {
            routeFault("Endpoint outside safe camera workspace");
          }
          break;
        }

        stopMaixRequest();
        workVisionRequestStartMs = 0UL;
        workVisionRetryCount = 0U;

        if (endpointFineVisionActive) {
          useArmBaseEndpointMotionProfile();
        } else {
          useArmBaseEndpointCoarseMotionProfile();
        }
        startArmBaseStandardFrameDegrees(
            targetPose.standardFrameAngleDegrees);
        activeEndpointScanPose = targetPose;
        ++activeEndpointStageServoMoveCount;
        ++activeEndpointServoMoveCount;

        SerialDebug.print(
            "[ENDPOINT SERVO] move ring/stage/step/total/"
            "px/mm/cmd/M5/M6=");
        SerialDebug.print(ring);
        SerialDebug.print("/");
        SerialDebug.print(
            endpointFineVisionActive ? "fine" : "coarse");
        SerialDebug.print("/");
        SerialDebug.print(activeEndpointStageServoMoveCount);
        SerialDebug.print("/");
        SerialDebug.print(activeEndpointServoMoveCount);
        SerialDebug.print("/");
        SerialDebug.print(centerErrorPixels, 2);
        SerialDebug.print("/");
        SerialDebug.print(measuredCorrectionMm, 2);
        SerialDebug.print("/");
        SerialDebug.print(commandedCorrectionMm, 2);
        SerialDebug.print("/");
        SerialDebug.print(
            targetPose.standardFrameAngleDegrees,
            2);
        SerialDebug.print("/");
        SerialDebug.println(targetPose.extensionMm, 2);

        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE;
        break;
      }

      const bool stableCameraResultAlreadyFinal =
          centerErrorPixels <=
              ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS &&
          confidence >=
              ENDPOINT_STABLE_DIRECT_ACCEPT_MINIMUM_CONFIDENCE;

      if (!endpointFineVisionActive &&
          !stableCameraResultAlreadyFinal) {
        stopMaixRequest();
        workVisionRequestStartMs = 0UL;
        workVisionRetryCount = 0U;
        endpointFineVisionActive = true;
        activeEndpointStageServoMoveCount = 0U;
        resetEndpointCenteredConfirmationWindow();
        SerialDebug.print(
            "[ENDPOINT SERVO] coarse ok ring/px; start fine=");
        SerialDebug.print(ring);
        SerialDebug.print("/");
        SerialDebug.print(centerErrorPixels, 2);
        SerialDebug.print(" -> ");
        SerialDebug.println(
            activeEndpointScanPose.heightMm,
            2);
        beginEndpointVision();
        break;
      }

      if (stableCameraResultAlreadyFinal) {
        endpointCenteredConfirmationCount =
            ENDPOINT_FINAL_CENTER_CONFIRMATIONS;
        SerialDebug.print(
            "[ENDPOINT SERVO] stable camera result accepted "
            "ring/error-px/conf=");
        SerialDebug.print(ring);
        SerialDebug.print("/");
        SerialDebug.print(centerErrorPixels, 2);
        SerialDebug.print("/");
        SerialDebug.println(confidence);
      } else if (endpointCenteredConfirmationCount < 255U) {
        ++endpointCenteredConfirmationCount;
      }
      if (endpointCenteredConfirmationCount <
          ENDPOINT_FINAL_CENTER_CONFIRMATIONS) {
        SerialDebug.print(
            "[ENDPOINT SERVO] final center confirmation "
            "ring/count/error px=");
        SerialDebug.print(ring);
        SerialDebug.print("/");
        SerialDebug.print(
            endpointCenteredConfirmationCount);
        SerialDebug.print("/");
        SerialDebug.println(centerErrorPixels, 2);
        break;
      }

      measuredRingPoints[ring] = measuredPoint;
      measuredRingPointValid[ring] = true;
      measuredRingRadiiPixels[ring] = radiusPixels;
      measuredRingConfidence[ring] = confidence;
      measuredRingHeadingsDegrees[ring] =
          headingNowDegrees;
      endpointScanElapsedMs[ring] =
          millis() - endpointScanStartMs[ring];
      endpointServoMoveCounts[ring] =
          activeEndpointServoMoveCount;
      endpointFinalCenterErrorsPixels[ring] =
          centerErrorPixels;
      workVisionRequestStartMs = 0UL;
      workVisionRetryCount = 0U;
      stopMaixRequest();

      SerialDebug.print(
          "[ENDPOINT SCAN] overhead center accepted "
          "ring/xy/error-px/r/conf/heading/elapsed=");
      SerialDebug.print(ring);
      SerialDebug.print("/");
      SerialDebug.print(x);
      SerialDebug.print(",");
      SerialDebug.print(y);
      SerialDebug.print("/");
      SerialDebug.print(centerErrorPixels, 2);
      SerialDebug.print("/");
      SerialDebug.print(radiusPixels);
      SerialDebug.print("/");
      SerialDebug.print(confidence);
      SerialDebug.print("/");
      SerialDebug.print(headingNowDegrees, 2);
      SerialDebug.print("/");
      SerialDebug.println(endpointScanElapsedMs[ring]);

      if (ring == 1U) {
        if (!measuredRingPointValid[3U]) {
          SerialDebug.println(
              "[ENDPOINT HANDOFF] ring 1 accepted; hold fixed M7 and "
              "move M5/M6 to predicted ring-3 pose");
          (void)beginDirectRing3EndpointScan();
          break;
        }
      } else if (!measuredRingPointValid[1U]) {
        // Recovery during ring 3 invalidates the old ring 1. Finish scanning
        // the interrupted ring first, then remeasure ring 1 in the new frame.
        ArmPose searchPose;
        if (nominalRingPose(1U, ENDPOINT_FINE_VISION_LOWER_MM, searchPose)) {
          initializeEndpointScanState(1U, searchPose);
          (void)startEndpointSearchPlanarMove();
        }
        break;
      }

      (void)completeEndpointMapAndStartTransfers();
      break;
    }

    case WORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE:
      if (!armMotors.isM5Running()) {
        armStandardDeadlineMs =
            millis() + ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS;
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE_SETTLE;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_LOCAL_BASE_SETTLE:
      if (deadlineReached(armStandardDeadlineMs) &&
          startExtensionToMmWithProfile(
              activeEndpointScanPose.extensionMm,
              endpointFineVisionActive
                  ? ENDPOINT_FINE_M6_SPEED_RPM
                  : ENDPOINT_COARSE_M6_SPEED_RPM,
              endpointFineVisionActive
                  ? ENDPOINT_FINE_M6_ACCELERATION
                  : ENDPOINT_COARSE_M6_ACCELERATION)) {
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_LOCAL_EXTENSION:
      if (extensionMoveFinished()) {
        activeEndpointScanPose.extensionMm =
            extensionAxis.currentMm;
        endpointLocalSettleDeadlineMs =
            millis() + ENDPOINT_LOCAL_MOVE_SETTLE_MS;
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE;
        SerialDebug.print(
            "[ENDPOINT SERVO] correction reached; next vision in ms=");
        SerialDebug.println(ENDPOINT_LOCAL_MOVE_SETTLE_MS);
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE:
      if (deadlineReached(
          endpointLocalSettleDeadlineMs)) {
        if (activeEndpointScanRing == 1U &&
            !endpointInitialStorageCommanded) {

          uint8_t initialStorageSlot = 0U;
          if (activeWorkAction == WORK_ACTION_PROCESS &&
              !processItemIndexForSequence(
                  0U,
                  initialStorageSlot)) {
            break;
          }
          commandStorageServoPosition(
              initialStorageSlot);
          workStorageServoDeadlineMs =
              millis() + STORAGE_SERVO_SETTLE_MS;
          endpointInitialStorageCommanded = true;
          SerialDebug.print("[WORK SAFE] t=");
          SerialDebug.print(millis());
          SerialDebug.print(
              " ms, ring-1 full pose settled -> "
              "initial storage slot ");
          SerialDebug.print(initialStorageSlot);
          SerialDebug.println(
              " starts with endpoint vision");
        }
        beginEndpointVision();
      }
      break;

    case WORK_PHASE_STORAGE_COLOR_WAIT_EXTENSION:
      if (!armMotors.isM5Running() && !armBaseMotionWatchdogActive &&
          extensionMoveFinished() && liftMoveFinished()) {
        if (fabsf(extensionAxis.currentMm -
                      storageColorScanPose.extensionMm) >
                ARM_AXIS_POSITION_TOLERANCE_MM ||
            fabsf(liftAxis.currentMm - M7_STANDARD_HEIGHT_MM) >
                ARM_AXIS_POSITION_TOLERANCE_MM) {
          routeFault("S2 scan pose not reached");
          break;
        }
        storageColorScanPose = ArmPose(currentArmBaseStandardFrameDegrees(),
            extensionAxis.currentMm, liftAxis.currentMm);
        storageColorAccuracyWindowStartMs = nowMs;
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
        SerialDebug.print("[S2DBG POSE] arrived t/ring/M5/M6/M7=");
        SerialDebug.print(nowMs);
        SerialDebug.print('/');
        SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
        SerialDebug.print('/');
        SerialDebug.print(storageColorScanPose.standardFrameAngleDegrees, 2);
        SerialDebug.print('/');
        SerialDebug.print(storageColorScanPose.extensionMm, 2);
        SerialDebug.print('/');
        SerialDebug.println(storageColorScanPose.heightMm, 2);
#endif
        beginStorageColorVision();
      }
      break;

    case WORK_PHASE_STORAGE_COLOR_WAIT_COORDINATE: {
      uint8_t detectedColor = 0U;
      int16_t x = 0, y = 0;
      const bool frame = readNewMaixCoordinate(workLastMaixSequence,
          detectedColor, x, y);
      if (frame) {
        markMissionProgress();
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
        ++storage2WindowFreshFrameCount;
        const uint8_t expectedBeforeFrame = storageColorExpected;
        const char *frameDecision = "ignored-wrong-color";
#endif
        if (detectedColor == storageColorExpected &&
            storage2CoordinateWithinTargetRoi(x, y)) {
          storageColorAccuracySumX += x;
          storageColorAccuracySumY += y;
          ++storageColorAccuracySampleCount;
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
          frameDecision = "target-coordinate-sampled";
#endif
        } else if (detectedColor == storageColorExpected) {
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
          ++storage2WindowOutsideRoiCount;
          frameDecision = "target-rejected-outside-roi";
          SerialDebug.print("[S2DBG ROI] reject ring/color/x/y/center-dx/dy=");
          SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
          SerialDebug.print('/');
          SerialDebug.print(static_cast<unsigned int>(detectedColor));
          SerialDebug.print('/');
          SerialDebug.print(x);
          SerialDebug.print('/');
          SerialDebug.print(y);
          SerialDebug.print('/');
          SerialDebug.print(x - IMAGE_CENTER_X);
          SerialDebug.print('/');
          SerialDebug.println(y - IMAGE_CENTER_Y);
#endif
        } else {
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
          ++storage2WindowWrongColorCount;
#endif
        }
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
        const gongchuang::MaixCamClient::Coordinate &latest =
            maixCam.latest();
        SerialDebug.print("[S2DBG FRAME] t/window/ring/req/expected-before/");
        SerialDebug.print("detected/x/y/metric/confidence/samples/decision=");
        SerialDebug.print(nowMs);
        SerialDebug.print('/');
        SerialDebug.print(nowMs - storageColorAccuracyWindowStartMs);
        SerialDebug.print('/');
        SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
        SerialDebug.print('/');
        SerialDebug.print(static_cast<unsigned int>(maixCam.requestedMode()));
        SerialDebug.print('/');
        SerialDebug.print(static_cast<unsigned int>(expectedBeforeFrame));
        SerialDebug.print('/');
        SerialDebug.print(static_cast<unsigned int>(detectedColor));
        SerialDebug.print('/');
        SerialDebug.print(x);
        SerialDebug.print('/');
        SerialDebug.print(y);
        SerialDebug.print('/');
        SerialDebug.print(latest.metric);
        SerialDebug.print('/');
        SerialDebug.print(latest.confidence);
        SerialDebug.print('/');
        SerialDebug.print(
            static_cast<unsigned int>(storageColorAccuracySampleCount));
        SerialDebug.print('/');
        SerialDebug.println(frameDecision);
#endif
      }
      if (nowMs - storageColorAccuracyWindowStartMs <
          arm_config::STORAGE2_SCAN_DWELL_MS) {
        // Targeted modes 1..4 return one coordinate per completed request.
        // Reissue only after a response; transport retries preserve the live
        // request sequence while no response has arrived.
        if (frame) { beginStorageColorVision(); }
        break;
      }
      stopMaixRequest();
      if (storageColorAccuracySampleCount == 0U) {
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
        const char *missReason =
            storage2WindowFreshFrameCount == 0U
                ? "MISS_NO_VALID_RESPONSE"
                : (storage2WindowOutsideRoiCount > 0U
                       ? "MISS_TARGET_OUTSIDE_ROI"
                       : (storage2WindowWrongColorCount > 0U
                              ? "MISS_WRONG_COLOR"
                              : "MISS_NO_ACCEPTED_SAMPLE"));
        printStorage2VerboseWindowResult(
            missReason,
            nowMs - storageColorAccuracyWindowStartMs);
        SerialDebug.print("[S2DBG WINDOW] result=miss-rescan elapsed/ring/");
        SerialDebug.print("expected/mapped-slot/samples=");
        SerialDebug.print(nowMs - storageColorAccuracyWindowStartMs);
        SerialDebug.print('/');
        SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
        SerialDebug.print('/');
        SerialDebug.print(static_cast<unsigned int>(storageColorExpected));
        SerialDebug.print('/');
        SerialDebug.print(static_cast<unsigned int>(
            storage2ItemOrder[storageColorTargetRing - 1U]));
        SerialDebug.print('/');
        SerialDebug.println(static_cast<unsigned int>(
            storageColorAccuracySampleCount));
#endif
        SerialDebug.print("[S2 SCAN MISS] ring=");
        SerialDebug.println(storageColorTargetRing);
        advanceStorage2Scan();
        break;
      }
      x = static_cast<int16_t>((storageColorAccuracySumX +
          storageColorAccuracySampleCount / 2U) / storageColorAccuracySampleCount);
      y = static_cast<int16_t>((storageColorAccuracySumY +
          storageColorAccuracySampleCount / 2U) / storageColorAccuracySampleCount);
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
      printStorage2VerboseWindowResult(
          "ACCEPTED_TARGET_COORDINATE",
          nowMs - storageColorAccuracyWindowStartMs);
      SerialDebug.print("[S2DBG WINDOW] result=coordinate elapsed/ring/");
      SerialDebug.print("expected/samples/avg-x/avg-y=");
      SerialDebug.print(nowMs - storageColorAccuracyWindowStartMs);
      SerialDebug.print('/');
      SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
      SerialDebug.print('/');
      SerialDebug.print(static_cast<unsigned int>(storageColorExpected));
      SerialDebug.print('/');
      SerialDebug.print(static_cast<unsigned int>(
          storageColorAccuracySampleCount));
      SerialDebug.print('/');
      SerialDebug.print(x);
      SerialDebug.print('/');
      SerialDebug.println(y);
#endif
      ArmPose targetPose;
      if (!storageColorTargetPose(storageColorScanPose,
              storageColorTargetRing, x, y, targetPose)) {
        routeFault("S2 sweep target invalid");
        break;
      }
      measuredRingPoses[storageColorTargetRing] = targetPose;
      measuredRingPoseValid[storageColorTargetRing] = true;
      measuredRingConfidence[storageColorTargetRing] = maixCam.latest().confidence;
      measuredRingHeadingsDegrees[storageColorTargetRing] =
          currentRouteCounterClockwiseHeading();
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
      SerialDebug.print("[S2DBG TARGET] ring/M5/M6/M7/confidence=");
      SerialDebug.print(static_cast<unsigned int>(storageColorTargetRing));
      SerialDebug.print('/');
      SerialDebug.print(targetPose.standardFrameAngleDegrees, 2);
      SerialDebug.print('/');
      SerialDebug.print(targetPose.extensionMm, 2);
      SerialDebug.print('/');
      SerialDebug.print(targetPose.heightMm, 2);
      SerialDebug.print('/');
      SerialDebug.println(maixCam.latest().confidence);
#endif
      SerialDebug.print("[S2 MAP] ring/slot=");
      SerialDebug.print(storageColorTargetRing);
      SerialDebug.print('/');
      SerialDebug.println(storage2ItemOrder[storageColorTargetRing - 1U]);
      advanceStorage2Scan();
      break;
    }

    case WORK_PHASE_CIRCLE_WAIT_ARM_LOWER:
      if (!liftMoveFinished()) {
        break;
      }
      SerialDebug.print("[HOUGH ARM] t=");
      SerialDebug.print(millis());
      SerialDebug.println(
          " ms, M7 logical -80 mm (physical -90 mm); "
          "start circle positioning");
      beginCircleVision();
      break;

    case WORK_PHASE_CIRCLE_WAIT_COORDINATE: {
      uint8_t targetId = 0U;
      int16_t x = 0;
      int16_t y = 0;
      if (!readNewMaixCoordinate(
              workLastMaixSequence,
              targetId,
              x,
              y)) {
        break;
      }
      workVisionRequestStartMs = millis();
      markMissionProgress();
      (void)targetId;

      const int32_t dx =
          static_cast<int32_t>(x - IMAGE_CENTER_X);
      const int32_t dy =
          static_cast<int32_t>(y - IMAGE_CENTER_Y);
      const float pixelRadius =
          hypotf(
              static_cast<float>(dx),
              static_cast<float>(dy));
      const bool centered =
          pixelRadius <=
          CIRCLE_CENTER_TOLERANCE_PIXELS;

      if (!centered) {
        resetCircleStabilityWindow();
        startVisualCorrection(x, y);
        break;
      }

      if (circleStableSampleCount == 0U) {
        circleStableStartMs = millis();
        circleLastStableSampleMs =
            circleStableStartMs;
      } else if (
          millis() - circleLastStableSampleMs >
          VISION_STABILITY_MAXIMUM_SAMPLE_GAP_MS) {
        circleStableStartMs = millis();
        circleStableSampleCount = 0U;
      }
      circleLastStableSampleMs = millis();
      if (circleStableSampleCount < 255U) {
        ++circleStableSampleCount;
      }
      if (circleStableSampleCount >=
              CIRCLE_STABILITY_MINIMUM_SAMPLES &&
          millis() - circleStableStartMs >=
              CIRCLE_CENTER_STABILITY_MS) {
        stopMaixRequest();
        workItemIndex = 0U;
        SerialDebug.print("[HOUGH ARM] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, positioning complete; retract M6/M7 and keep M5");
        beginArmStandardization(0.0f, true);
        workActionPhase =
            WORK_PHASE_CIRCLE_WAIT_ARM_STANDARD;
      }
      break;
    }

    case WORK_PHASE_CIRCLE_WAIT_CHASSIS:
      if (updateHeadingLock(
              MOTION_TIMEOUT_MS,
              VISION_POST_MOTION_SETTLE_TIME_MS,
              VISION_HEADING_STABLE_TIME_MS)) {
        beginCircleVision();
      }
      break;

    case WORK_PHASE_CIRCLE_WAIT_ARM_STANDARD:
      if (serviceArmStandardization()) {
        armStandardPhase = ARM_STANDARD_IDLE;
        SerialDebug.print("[HOUGH ARM] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, M6/M7 safe and M5 retained; correct heading");
        startPostVisionHeadingCorrection();
      }
      break;

    case WORK_PHASE_CIRCLE_WAIT_POST_VISION_HEADING:
      if (updateHeadingLock(
              MOTION_TIMEOUT_MS,
              VISION_POST_MOTION_SETTLE_TIME_MS,
              VISION_HEADING_STABLE_TIME_MS)) {
        SerialDebug.print("[VISION IMU] t=");
        SerialDebug.print(millis());
        SerialDebug.print(" ms, complete; target/actual/error=");
        SerialDebug.print(
            targetCounterClockwiseHeadingDegrees, 2);
        SerialDebug.print("/");
        SerialDebug.print(
            currentRouteCounterClockwiseHeading(), 2);
        SerialDebug.print("/");
        SerialDebug.println(headingErrorDegrees(), 2);
        workActionPhase = WORK_PHASE_START_UNLOAD;
      }
      break;

    case WORK_PHASE_START_UNLOAD:
      beginUnloadingTransfer();
      break;

    case WORK_PHASE_START_RELOAD:
      beginReloadingTransfer();
      break;

    case WORK_PHASE_WAIT_TRANSFER:
      if (armTransferFinished()) {
        completeTransferAndRotateStorage();
      }
      break;

    case WORK_PHASE_WAIT_STORAGE_SERVO: {
      const bool process2DepartureOverlap =
          process2FinalStorageMayOverlapDeparture();
      if (process2DepartureOverlap) {
        serviceProcess2DepartureImuPrelock();
        if (!liftMoveFinished()) {
          break;
        }
        process2DepartureM7ArrivalFrameSequence =
            route_chassis::imuValidFrameSequence;
        // The tray servo is autonomous after its final parking command. Start
        // chassis restoration in this same service pass; neither this gate nor
        // the later departure lock waits for the 1463 ms tray deadline.
        startVisualCorrectionRestore();
        break;
      }

      if (!deadlineReached(workStorageServoDeadlineMs) ||
          !liftMoveFinished()) {
        break;
      }

      if (workItemIndex < 3U) {
        if (activeTransferPurpose ==
            TRANSFER_PURPOSE_CONTAINER_TO_RING) {
          workActionPhase = WORK_PHASE_START_UNLOAD;
        } else if (
            activeTransferPurpose ==
            TRANSFER_PURPOSE_RING_TO_CONTAINER) {
          workActionPhase = WORK_PHASE_START_RELOAD;
        }
        break;
      }

      const bool storage1Raw2TravelHandoff =
          activeWorkAction == WORK_ACTION_STORAGE &&
          workRoundIndex == 0U &&
          activeTransferPurpose ==
              TRANSFER_PURPOSE_CONTAINER_TO_RING;
      if (storage1Raw2TravelHandoff &&
          STORAGE_ROUND1_SIMULATION_MODE) {
        // Bench run stops after the third placement. The formal route's
        // next RAW2 travel preposition is intentionally absent here.
        beginStorageParkingBeforeWorkFinish();
        break;
      }
      if (storage1Raw2TravelHandoff) {
        // The entry gate above has verified the final STORAGE1 M7 rise.  The
        // requested 100 us boundary is now the sole delay before M6, tray and
        // chassis departure are released together.
        storage1Raw2HandoffDeadlineUs =
            micros() + STORAGE1_RAW2_M7_HANDOFF_US;
        workActionPhase =
            WORK_PHASE_WAIT_STORAGE1_RAW2_HANDOFF;
        SerialDebug.println(
            "[S1 EXIT] M7=0; handoff-us=100");
        break;
      }

      if (activeWorkAction == WORK_ACTION_PROCESS &&
          activeTransferPurpose ==
              TRANSFER_PURPOSE_CONTAINER_TO_RING) {
        workItemIndex = 0U;
        workActionPhase = WORK_PHASE_START_RELOAD;
      } else {
        workActionPhase = WORK_PHASE_START_RESTORE;
      }
      break;
    }

    case WORK_PHASE_WAIT_STORAGE1_RAW2_HANDOFF:
      if (!liftMoveFinished()) {
        break;
      }
      if (fabsf(
              liftAxis.currentMm -
              M7_STANDARD_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
        routeFault("STORAGE1 RAW2 preparation fault");
        break;
      }
      if (static_cast<int32_t>(
              micros() -
              storage1Raw2HandoffDeadlineUs) < 0) {
        break;
      }
      if (storage1Raw2TravelPhase !=
          STORAGE1_RAW2_TRAVEL_IDLE) {
        routeFault("STORAGE1 RAW2 preparation fault");
        break;
      }
      if (!startExtensionToMmWithProfile(
              RAW_VIEW_EXTENSION_MM,
              STORAGE1_RAW2_M6_SPEED_RPM,
              STORAGE1_RAW2_M6_ACCELERATION)) {
        break;
      }
      commandStorageServoParkingPosition(
          STORAGE1_RAW2_SERVO_INTERVAL_MS);
      storage1Raw2HandoffDeadlineUs = 0UL;
      storage1Raw2TravelPhase =
          STORAGE1_RAW2_TRAVEL_RUNNING;
      SerialDebug.println(
          "[S1 EXIT] M6/tray/chassis parallel");
      startVisualCorrectionRestore();
      break;

    case WORK_PHASE_WAIT_STORAGE_PARK:
      if (serviceArmStandardization() &&
          deadlineReached(workStorageServoDeadlineMs)) {
        armStandardPhase = ARM_STANDARD_IDLE;
        beginWorkstationDepartureLock();
      }
      break;

    case WORK_PHASE_WAIT_DEPARTURE_LOCK:
      if (serviceWorkstationDepartureLock()) {
        finishActiveWorkAction();
      }
      break;

    case WORK_PHASE_START_RESTORE:
      startVisualCorrectionRestore();
      break;

    case WORK_PHASE_WAIT_RESTORE: {
      if (!allMotorsArrived()) {
        break;
      }

      const bool process2NoVisualRestoreMotion =
          motorPulsesAreZero(
              visualCorrectionAccumulator);
      visualCorrectionAccumulator = MotorPulses();
      visualCorrectionForwardMm = 0.0f;
      visualCorrectionLeftMm = 0.0f;
      visualCorrectionMoveCount = 0U;

      const bool process2PrelockedDeparture =
          process2FinalStorageMayOverlapDeparture() &&
          process2DepartureImuPrelockReady &&
          process2NoVisualRestoreMotion;
      if (process2PrelockedDeparture) {
        const uint32_t postM7FrameSequence =
            route_chassis::imuValidFrameSequence;
        if (postM7FrameSequence ==
                process2DepartureM7ArrivalFrameSequence &&
            millis() - commandStartMs <
                WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS) {
          break;
        }

        const bool freshPostM7HeadingSample =
            postM7FrameSequence !=
                process2DepartureM7ArrivalFrameSequence &&
            imuIsFresh() &&
            millis() - route_chassis::lastImuReceiveMs <=
                WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS &&
            route_chassis::lastImuReceiveMs -
                    workDepartureLastImuReceiveMs <=
                WORK_DEPARTURE_IMU_MAX_SAMPLE_GAP_MS &&
            fabsf(headingErrorDegrees()) <=
                WORKSTATION_HEADING_TOLERANCE_DEGREES;
        if (freshPostM7HeadingSample &&
            workstationMechanismsStoppedForDeparture()) {
          finishActiveWorkAction();
          break;
        }
      }

      const bool storage1ParallelDeparture =
          activeWorkAction == WORK_ACTION_STORAGE &&
          workRoundIndex == 0U &&
          (storage1Raw2TravelPhase ==
               STORAGE1_RAW2_TRAVEL_RUNNING ||
           storage1Raw2TravelPhase ==
               STORAGE1_RAW2_TRAVEL_COMPLETE);
      if (storage2ReturnTravelActive && secondStorageColorWorkflowActive()) {
        finishActiveWorkAction();
      } else if (storage1ParallelDeparture) {
        // The user-selected departure boundary is M7 zero + 100 us.  Do not
        // add the generic post-workstation IMU hold here; any real visual
        // correction has already completed in this WAIT_RESTORE phase.
        finishActiveWorkAction();
      } else {
        beginWorkstationDepartureLock();
      }
      break;
    }
  }
}

void cancelCompetitionAction() {
  finalStorageDeparturePending = false;
  storage2ReturnTravelActive = false;
  activeWorkAction = WORK_ACTION_NONE;
  armPayloadPresent = false;
  armGripperLiftIsolationEnabled = false;
  rawProcessPickupM7RaiseAdditionalDelayEnabled = false;
  workActionPhase = WORK_PHASE_IDLE;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  activePlacementRingPosition = 0U;
  armStandardPhase = ARM_STANDARD_IDLE;
  armStandardHandoffDeadlineUs = 0UL;
  armStandardEndpointPreparation = false;
  armStandardKeepBaseAngle = false;
  armStandardRawViewPreparation = false;
  armStandardEndpointExtensionPrepositioned = false;
  armStandardExtensionTargetMm =
      M6_STANDARD_EXTENSION_MM;
  armTransferPhase = ARM_TRANSFER_IDLE;
  armTransferM7HandoffStartMs = 0UL;
  armTransferRawM7ToGripperDeadlineUs = 0UL;
  armTransferMapSource = false;
  armTransferMapDestination = false;
  armTransferConcurrentSourcePreparation = false;
  armTransferSourceAlreadyPrepared = false;
  armTransferPrepareNextSource = false;
  armTransferNextSourcePose = ArmPose();
  armTransferNextSourceMapped = false;
  armTransferNextStorageSlot = -1;
  armTransferNextStorageDeadlineMs = 0UL;
  armTransferStorageCommandDueMs = 0UL;
  armTransferNextStorageCommanded = false;
  armTransferNextSourcePreparedAtEnd = false;
  armTransferReturnToRawViewAtEnd = false;
  armTransferSourceRotationStarted = false;
  armTransferDestinationRotationStarted = false;
  armTransferFinalRotationStarted = false;
  armTransferProcessFirstRingReturnTiming = false;
  armTransferDestinationM5FinalBufferPending = false;
#if GONGCHUANG_RING1_LOADED_RETURN_RAMP_TIME_PERCENT > 100U
  armTransferSourceRingPosition = 0U;
#endif
  armTransferMotionProfile =
      arm_transfer::PROFILE_STANDARD;
  endpointDirectContainerPickupPending = false;
  endpointInitialStorageCommanded = false;
  containerPickupPrepositionedPending = false;
  ringPickupPrepositionedPending = false;
  workActionStartMs = 0UL;
  workVisionRequestStartMs = 0UL;
  workVisionRetryCount = 0U;
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
  rawTrackingActive = false;
  rawNoTargetDeadlineMs = 0UL;
#endif
  workStorageServoDeadlineMs = 0UL;
  resetProcess2DepartureImuPrelock();
  resetStorageServoArrivalGate();
  visualCorrectionAccumulator = MotorPulses();
  visualCorrectionForwardMm = 0.0f;
  visualCorrectionLeftMm = 0.0f;
  visualCorrectionMoveCount = 0U;
  activeEndpointScanRing = 0U;
  activeEndpointScanPose = ArmPose();
  endpointFineVisionActive = false;
  activeEndpointServoMoveCount = 0U;
  activeEndpointStageServoMoveCount = 0U;
  endpointSearchFallbackMoveCount = 0U;
  endpointSearchBaseAngleDegrees = 0.0f;
  resetEndpointCenteredConfirmationWindow();
  endpointLocalSettleDeadlineMs = 0UL;
  endpointMapStartMs = 0UL;
  endpointMapCompleteMs = 0UL;
  memset(
      endpointScanStartMs,
      0,
      sizeof(endpointScanStartMs));
  memset(
      endpointScanElapsedMs,
      0,
      sizeof(endpointScanElapsedMs));
  memset(
      endpointServoMoveCounts,
      0,
      sizeof(endpointServoMoveCounts));
  memset(
      endpointFinalCenterErrorsPixels,
      0,
      sizeof(endpointFinalCenterErrorsPixels));
  endpointMapLockedHeadingDegrees = 0.0f;
  resetMeasuredRingMap();
  rawTravelM5ZeroPending = false;
  processM5ZeroSettleDeadlineUs = 0UL;
  storageColorExpected = 0U;
  storageColorTargetRing = 0U;
  storageColorScanPose = ArmPose();
  storageTravelPreparationPhase =
      STORAGE_TRAVEL_PREP_IDLE;
  storageTravelPreparationRound = 0U;
  storageTravelM5ZeroHandoffDeadlineUs = 0UL;
  storageTravelM5StartDeadlineMs = 0UL;
  storage1Raw2TravelPhase = STORAGE1_RAW2_TRAVEL_IDLE;
  storage1Raw2HandoffDeadlineUs = 0UL;
  integratedWorkActionStartPending = false;
  finalReturnM5Commanded = false;
  finalStorageDeparturePending = false;
  storage2ReturnTravelActive = false;
  stopMaixRequest();
}

bool plannedFinalFootprintIsInsideSelectedZone() {
  const float centerX =
      static_cast<float>(FINAL_ZONE_CENTER_X_MM);
  const float centerY =
      static_cast<float>(
          selectedStartZone == START_ZONE_1
              ? START_ZONE_1_CENTER_Y_MM
              : START_ZONE_2_CENTER_Y_MM);

  const competition::StartZone zone =
      selectedStartZone == START_ZONE_1
          ? competition::START_ZONE_1
          : competition::START_ZONE_2;
  const competition::Rectangle zoneBounds =
      competition::startZoneBounds(
          zone,
          static_cast<float>(FIELD_SIZE_MM),
          static_cast<float>(START_ZONE_SIZE_MM));
  const competition::Rectangle footprint =
      competition::axisAlignedFootprint(
          centerX,
          centerY,
          static_cast<float>(CHASSIS_FOOTPRINT_X_MM),
          static_cast<float>(CHASSIS_FOOTPRINT_Y_MM),
          180);
  return competition::rectangleContains(
      zoneBounds, footprint);
}

void finishProgram() {
  if (!finalReturnM5Commanded) {
    // Route has reached its endpoint. Complete linear retraction before M5
    // returns to the exact software angle established at startup (old-frame 0).
    if (!extensionMoveFinished() || !liftMoveFinished()) {
      return;
    }
    if (fabsf(extensionAxis.currentMm - M6_STANDARD_EXTENSION_MM) >
            ARM_AXIS_POSITION_TOLERANCE_MM ||
        fabsf(liftAxis.currentMm - M7_STANDARD_HEIGHT_MM) >
            ARM_AXIS_POSITION_TOLERANCE_MM) {
      routeFault("Final linear zero incomplete");
      return;
    }
    useArmBaseReturnMotionProfile();
    startArmBaseRotationToDegrees(FINAL_RETURN_M5_TARGET_DEGREES);
    if (programState == PROGRAM_FAULT) {
      return;
    }
    finalReturnM5Commanded = true;
    return;
  }
  if (armMotors.isM5Running() ||
      armBaseMotionWatchdogActive) {
    hmiSetRunStatus("M5RETURN");
    return;
  }
  if (fabsf(
          armMotors.m5CurrentAngleDegrees() -
          FINAL_RETURN_M5_TARGET_DEGREES) >
      FINAL_RETURN_M5_TOLERANCE_DEGREES) {
    routeFault("Final M5 home incomplete");
    return;
  }

  const bool trayAlreadyParking = storage2ReturnTravelActive;
  cancelCompetitionAction();
  emergencyStopArmLinearAxes();
  disableDriveMotors();
  disableArmBaseMotor();
  commandGripperClose();
  if (!trayAlreadyParking) {
    commandStorageServoParkingPosition();
  }
  programState = PROGRAM_FINISHED;
  hmiSetRunStatus("FINISH");

  if (SHOW_RESULT_PAGE_ON_FINISH) {
    hmiCommand("page page0");
    hmiSetTaskCounts();
  }

  SerialDebug.println("Route finished");
}

void onStartButtonClick() {
  if (programState != PROGRAM_RUNNING) {
    abortRequested = false;
    route_chassis::startRequestAcceptedMs = millis();
    startRequested = true;
    imuWaitStatusDisplayed = false;

    if (!STORAGE_ROUND2_SIMULATION_MODE) {
      startArmBaseRotationToDegrees(
          static_cast<float>(
              ARM_BASE_TRAVEL_OLD_FRAME_DEGREES));
    }
  }
}

void onStartButtonDoubleClick() {
  if (programState != PROGRAM_WAITING || startRequested) {
    return;
  }

  if (LOCAL_WORKSTATION_SIMULATION_MODE) {
    hmiSetRunStatus(
        RAW_WORKSTATION_SIMULATION_MODE
            ? "RAWREADY"
            : (ROUGH_PROCESSING_CALIBRATION_MODE
            ? "CALREADY"
            : (STORAGE_ROUND1_SIMULATION_MODE
                   ? "S1READY"
                   : "S2READY")));
    SerialDebug.println(
        "Local workstation simulation ignores start-zone selection");
    return;
  }

  selectedStartZone =
      selectedStartZone == START_ZONE_1
          ? START_ZONE_2
          : START_ZONE_1;
  hmiSetRunStatus(
      selectedStartZone == START_ZONE_1
          ? "ZONE1"
          : "ZONE2");
  SerialDebug.print("Selected start zone: ");
  SerialDebug.println(
      static_cast<unsigned int>(selectedStartZone));
}

void onStartButtonLongPress() {
  abortRequested = true;
}

bool configureRoughProcessingCalibrationTask() {

  constexpr char CALIBRATION_TASK_CODE[] =
      "123+132+123+132";
  competition::TaskPlan calibrationPlan;
  if (competition::parseTaskCode(
          CALIBRATION_TASK_CODE,
          calibrationPlan) !=
      competition::TASK_CODE_OK) {
    routeFault("Rough-processing calibration task is invalid");
    return false;
  }

  taskPlan = calibrationPlan;
  memcpy(
      qrData,
      CALIBRATION_TASK_CODE,
      sizeof(CALIBRATION_TASK_CODE));
  scanFlag = true;
  taskCodeDecoded = true;
  hmiSetText("t1", "CAL132");
  hmiShowTaskCode(CALIBRATION_TASK_CODE);
  return true;
}

bool configureRawWorkstationSimulationTask() {
  // Keep the same three physical colors for either selectable RAW round so a
  // bench operator can switch rounds without rearranging the material set.
  // A dedicated reproduction wrapper may override the task code while still
  // calling this production parser and production RAW state machine.
  constexpr char RAW_SIMULATION_TASK_CODE[] =
      GONGCHUANG_DEBUG_RAW_TASK_CODE;
  static_assert(
      sizeof(RAW_SIMULATION_TASK_CODE) == QR_CODE_LENGTH + 1U,
      "RAW debug task code must contain exactly 15 characters");
  competition::TaskPlan plan;
  if (competition::parseTaskCode(
          RAW_SIMULATION_TASK_CODE,
          plan) != competition::TASK_CODE_OK) {
    routeFault("RAW workstation simulation task is invalid");
    return false;
  }

  taskPlan = plan;
  memcpy(
      qrData,
      RAW_SIMULATION_TASK_CODE,
      sizeof(RAW_SIMULATION_TASK_CODE));
  scanFlag = true;
  taskCodeDecoded = true;
  hmiSetText("t1", "RAW123");
  hmiShowTaskCode(RAW_SIMULATION_TASK_CODE);
  SerialDebug.print("[RAW SIM] fixed task/round=");
  SerialDebug.print(RAW_SIMULATION_TASK_CODE);
  SerialDebug.print("/");
  SerialDebug.println(GONGCHUANG_DEBUG_RAW_ROUND);
  return true;
}

bool configureStorageRound1SimulationTask() {
  // Bench preset: tray slots 0/1/2 contain colors 1/2/3. The first-batch
  // destination order is ring 1/3/2. No QR scanner is required in this mode.
  constexpr char TASK_CODE[] = "123+132+123+132";
  competition::TaskPlan plan;
  if (competition::parseTaskCode(TASK_CODE, plan) !=
      competition::TASK_CODE_OK) {
    routeFault("Storage round-1 simulation task is invalid");
    return false;
  }
  taskPlan = plan;
  memcpy(qrData, TASK_CODE, sizeof(TASK_CODE));
  scanFlag = true;
  taskCodeDecoded = true;
  hmiSetText("t1", "STACK1");
  hmiShowTaskCode(TASK_CODE);
  SerialDebug.print("[STORAGE1 SIM] fixed task=");
  SerialDebug.println(TASK_CODE);
  return true;
}

bool configureStorageRound2SimulationTask() {
  // Physical setup for RobotConfig run mode 2 / debug workstation state 3:
  // bottom layer from round 1 is 1/3/4 on rings 1/2/3, while tray slots
  // 0/1/2 contain round-2 colors 3/1/4. The final 2/3/1 position group is
  // retained exactly as the supplied QR code even though stacking correctly
  // targets the first-round ring holding the same color.
  constexpr char STORAGE2_SIMULATION_TASK_CODE[] =
      "134+123+314+231";
  competition::TaskPlan simulationPlan;
  if (competition::parseTaskCode(
          STORAGE2_SIMULATION_TASK_CODE,
          simulationPlan) !=
      competition::TASK_CODE_OK) {
    routeFault("Storage round-2 simulation task is invalid");
    return false;
  }

  taskPlan = simulationPlan;
  memcpy(
      qrData,
      STORAGE2_SIMULATION_TASK_CODE,
      sizeof(STORAGE2_SIMULATION_TASK_CODE));
  scanFlag = true;
  taskCodeDecoded = true;
  hmiSetText("t1", "STACK2");
  hmiShowTaskCode(STORAGE2_SIMULATION_TASK_CODE);
  SerialDebug.print("[STORAGE2 SIM] fixed task=");
  SerialDebug.println(STORAGE2_SIMULATION_TASK_CODE);
  SerialDebug.println(
      "[STORAGE2 SIM] setup: bottom colors 1/3/4 at rings 1/2/3; "
      "tray slots 0/1/2 contain colors 3/1/4");
  return true;
}

void suspendRouteChassisProfileForWorkstation() {
  /*
   * route_chassis::runAllMotors()必须持续产生M1～M4脉冲，但工位暂停后
   * 位置目标和运动参数属于GC视觉修正。把route上一段遗留的S形/制动
   * 包络置为中性，避免它在暂停期间覆盖GC设置的maxSpeed/acceleration。
   * 下一路线段的route_chassis::startCurrentCommand()会通过底盘原有
   * setDriveMotionProfile()完整重建这些状态。
   */
  route_chassis::activeDriveProfileShape =
      route_chassis::DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID;
  route_chassis::activeDriveDeceleration = 0.0f;
  route_chassis::driveDecelerationActive = false;
  route_chassis::driveAccelerationDistanceSteps = 0.0f;
  route_chassis::driveDecelerationStartStepRate = 0.0f;
  route_chassis::driveDecelerationStartRemainingSteps = 0.0f;
}

void beginRoute() {

  if (programState != PROGRAM_WAITING ||
      (!LOCAL_WORKSTATION_SIMULATION_MODE &&
       route_chassis::programState !=
           route_chassis::PROGRAM_WAITING)) {
    startRequested = false;
    return;
  }

  if (!imuIsFresh()) {
    if (!imuWaitStatusDisplayed) {
      hmiSetRunStatus("IMUWAIT");
      SerialDebug.println("Waiting for a valid JY901 angle frame");
      imuWaitStatusDisplayed = true;
    }
    return;
  }

  if (!armLinearReferenceValid) {
    startRequested = false;
    hmiSetRunStatus("ARMZERO");
    SerialDebug.println(
        "Start refused: M6/M7 safe working zero is invalid; "
        "power-cycle with M6 at retract stop and M7 at "
        "physical highest point");
    return;
  }

  if (!verifyManipulationServosOnline()) {
    startRequested = false;
    hmiSetRunStatus("SERVOERR");
    SerialDebug.println(
        "Start refused: gripper or storage servo offline");
    return;
  }

  integratedWorkPause = INTEGRATED_WORK_NONE;
  integratedWorkPauseStartMs = 0UL;
  integratedWorkActionStartPending = false;
  chassisStartupHoldActive = false;
  finalReturnM5Commanded = false;
  finalStorageDeparturePending = false;
  storage2ReturnTravelActive = false;
  storageTravelPreparationPhase =
      STORAGE_TRAVEL_PREP_IDLE;
  storageTravelPreparationRound = 0U;
  storageTravelM5ZeroHandoffDeadlineUs = 0UL;
  storageTravelM5StartDeadlineMs = 0UL;
  storage1Raw2TravelPhase = STORAGE1_RAW2_TRAVEL_IDLE;
  storage1Raw2HandoffDeadlineUs = 0UL;
  storageRound2SimulationPhase =
      STORAGE2_SIMULATION_IDLE;
  storageRound2SimulationGripperDeadlineMs = 0UL;
  storageRound1SimulationPhase =
      STORAGE1_SIMULATION_IDLE;
  rawSimulationParkingDeadlineMs = 0UL;
  activeCompetitionRound = 0;
  const uint32_t runStartMs = millis();
  commandStartMs = runStartMs;
  lastMissionProgressMs = runStartMs;
  routeHeadingLockStartMs = 0UL;
  headingStableStartMs = 0;
  motorsArrivedStartMs = 0;
  translationRemainingMm = 0;
  translationCentralChannelEnabled = false;
  preciseMotionEnabled = false;
  turnMotionEnabled = false;
  translationPreciseArrivalEnabled = false;
  workstationApproachEnabled = false;
  turnCoarseTelemetryPending = false;
  activeTurnStartHeadingDegrees = 0.0f;
  activeTurnCommandDegrees = 0.0f;
  activeTurnCorrectionCount = 0;
  cancelCompetitionAction();

  routeImuReferenceDegrees =
      route_chassis::imuCounterClockwiseDegrees;
  targetCounterClockwiseHeadingDegrees = 0.0f;

  correctGrabCount = 0;
  correctPlacementCount = 0;
  rawActionFinished = false;
  processActionFinished = false;
  storageActionFinished = false;
  finalAlignmentFinished = false;
  resetQrReceiver();
  hmiSetTaskCounts();
  hmiSetText("t3", "000+000+");
  hmiSetText("t8", "000+000");
  if (RAW_WORKSTATION_SIMULATION_MODE) {
    if (!configureRawWorkstationSimulationTask()) {
      startRequested = false;
      return;
    }
    hmiSetRunStatus(
        GONGCHUANG_DEBUG_RAW_ROUND == 1
            ? "RAW1PREP"
            : "RAW2PREP");
  } else if (ROUGH_PROCESSING_CALIBRATION_MODE) {
    if (!configureRoughProcessingCalibrationTask()) {
      startRequested = false;
      return;
    }
    hmiSetRunStatus("CALRUN");
  } else if (STORAGE_ROUND1_SIMULATION_MODE) {
    if (!configureStorageRound1SimulationTask()) {
      startRequested = false;
      return;
    }
    hmiSetRunStatus("S1PREP");
  } else if (STORAGE_ROUND2_SIMULATION_MODE) {
    if (!configureStorageRound2SimulationTask()) {
      startRequested = false;
      return;
    }
    hmiSetRunStatus("S2PREP");
  } else {
    hmiSetText("t1", "QRWAIT");
    hmiSetRunStatus("RUN");
  }
  if (STORAGE_ROUND2_SIMULATION_MODE) {
    commandGripperOpen();
  } else {
    commandGripperClose();
  }
  commandStorageServoParkingPosition();

  if (LOCAL_WORKSTATION_SIMULATION_MODE) {
    for (uint8_t i = 0; i < 4; ++i) {
      motors[i]->setCurrentPosition(0);
    }
    setDriveMotionProfile(
        ROUTE_FAST_MAXIMUM_STEP_RATE,
        ROUTE_FAST_STEP_ACCELERATION);
    enableDriveMotors();
    programState = PROGRAM_RUNNING;
    startRequested = false;
    route_chassis::startRequested = false;
    imuWaitStatusDisplayed = false;
    route_chassis::stopAllMotorsImmediately();
    suspendRouteChassisProfileForWorkstation();
    stopAllMotorsImmediately();
    if (RAW_WORKSTATION_SIMULATION_MODE) {
      // Unlike the ring-only workstation fixtures, RAW debug deliberately
      // leaves M1-M4 enabled: the production search state machine owns only
      // its reversible 40 mm left/right correction and never runs the route.
      activeCompetitionRound =
          static_cast<uint8_t>(GONGCHUANG_DEBUG_RAW_ROUND);
      rawActionFinished = false;
      hmiSetRunStatus(
          GONGCHUANG_DEBUG_RAW_ROUND == 1
              ? "RAW1"
              : "RAW2");
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
      SerialDebug.println(
          "[RAW SIM] local RAW action started; route disabled; "
          "only production A/B search may move the chassis");
#else
      SerialDebug.println(
          "[RAW SIM] local legacy fixed-A pickup started; "
          "route disabled; chassis motion blocked");
#endif
      beginWorkAction(
          WORK_ACTION_RAW,
          static_cast<uint8_t>(GONGCHUANG_DEBUG_RAW_ROUND));
      return;
    }
    if (STORAGE_ROUND2_SIMULATION_MODE) {
      storageRound2SimulationGripperDeadlineMs =
          millis() + GRIPPER_OPEN_SETTLE_MS +
          WORK_GRIPPER_TO_M7_GAP_MS;
      storageRound2SimulationPhase =
          STORAGE2_SIMULATION_WAIT_GRIPPER_OPEN;
      SerialDebug.println(
          "Storage round-2 local simulation started: wheels locked; "
          "open gripper, prepare QR-selected first color pose, then run "
          "three targeted-color stack placements");
      return;
    }
    if (STORAGE_ROUND1_SIMULATION_MODE) {
      // Reuse the real PROCESS1-to-STORAGE1 M7/M6/tray preparation. The
      // chassis stays still; only after the exact arrival pose is verified
      // may the same first-storage action used by the formal route start.
      beginStorageTravelPreparation(1U);
      if (programState == PROGRAM_FAULT) {
        return;
      }
      storageRound1SimulationPhase =
          STORAGE1_SIMULATION_WAIT_PREPARATION;
      SerialDebug.println(
          "[STORAGE1 SIM] wheels locked; preparation started");
      return;
    }
    SerialDebug.println(
        "Rough-processing calibration started: "
        "wheels locked; scan rings 1/3, derive ring 2, "
        "QR place/pick ring order 1/3/2");
    beginWorkAction(WORK_ACTION_PROCESS, 1U);
    return;
  }

  /*
   * 正式比赛的四轮清零、使能、速度档、IMU参考及唯一19条运动命令，
   * 只由route_chassis建立；首段在检查通过且距启动请求至少500 ms后下发。
   */
  route_chassis::beginRoute();
  if (route_chassis::programState !=
      route_chassis::PROGRAM_RUNNING) {
    startRequested = false;
    return;
  }
  routeImuReferenceDegrees =
      route_chassis::routeImuReferenceDegrees;
  targetCounterClockwiseHeadingDegrees =
      route_chassis::targetCounterClockwiseHeadingDegrees;
  selectedStartZone =
      route_chassis::selectedStartZoneNumber == 1U
          ? START_ZONE_1
          : START_ZONE_2;
  programState = PROGRAM_RUNNING;
  startRequested = false;
  imuWaitStatusDisplayed = false;

  // 必须用按键请求时间，不能从检查结束重新计500 ms。
  chassisStartupHoldStartMs = route_chassis::startRequestAcceptedMs;
  chassisStartupHoldActive =
      static_cast<uint32_t>(millis() - chassisStartupHoldStartMs) <
      CHASSIS_STARTUP_HOLD_MS;
  if (chassisStartupHoldActive) {
    hmiSetRunStatus("STARTWAIT");
    SerialDebug.println("[START HOLD] complete minimum 500ms; QR reception active");
  }

  SerialDebug.println(
      "GongChuang_route 21-step chassis + GC arm/vision started");
  SerialDebug.print("Locked start zone: ");
  SerialDebug.println(
      static_cast<unsigned int>(selectedStartZone));
}

void serviceStorageRound2Simulation() {
  if (!STORAGE_ROUND2_SIMULATION_MODE ||
      programState != PROGRAM_RUNNING) {
    return;
  }

  switch (storageRound2SimulationPhase) {
    case STORAGE2_SIMULATION_IDLE:
    case STORAGE2_SIMULATION_COMPLETE:
      return;

    case STORAGE2_SIMULATION_WAIT_GRIPPER_OPEN:
      if (!deadlineReached(
              storageRound2SimulationGripperDeadlineMs)) {
        return;
      }
      storageRound2SimulationGripperDeadlineMs = 0UL;
      if (!extensionMoveFinished() ||
          !liftMoveFinished() ||
          armMotors.isM5Running() ||
          armBaseMotionWatchdogActive ||
          fabsf(
              extensionAxis.currentMm -
              M6_STANDARD_EXTENSION_MM) >
              ARM_AXIS_POSITION_TOLERANCE_MM ||
          fabsf(
              liftAxis.currentMm -
              M7_STANDARD_HEIGHT_MM) >
              ARM_AXIS_POSITION_TOLERANCE_MM) {
        routeFault(
            "Storage round-2 simulation requires M5 stopped and M6/M7 zero");
        return;
      }
      beginStorageTravelPreparation(2U);
      if (programState == PROGRAM_FAULT) {
        return;
      }
      storageRound2SimulationPhase =
          STORAGE2_SIMULATION_WAIT_TRAVEL_PREPARATION;
      SerialDebug.println(
          "[STORAGE2 SIM] M5 hold; M7/M6 preparation started");
      return;

    case STORAGE2_SIMULATION_WAIT_TRAVEL_PREPARATION:
      serviceStorageTravelPreparation();
      if (!storageTravelPreparationReady(2U)) {
        return;
      }
      activeCompetitionRound = 2U;
      storageActionFinished = false;
      hmiSetRunStatus("STACK2");
      storageRound2SimulationPhase =
          STORAGE2_SIMULATION_RUN_ACTION;
      SerialDebug.println(
          "[STORAGE2 SIM] exact arrival pose ready; start local stack action");
      beginWorkAction(WORK_ACTION_STORAGE, 2U);
      return;

    case STORAGE2_SIMULATION_RUN_ACTION:
      if (!storageActionFinished) {
        return;
      }
      if (correctPlacementCount != 3U ||
          correctGrabCount != 0U ||
          workItemIndex != 3U ||
          activeWorkAction != WORK_ACTION_NONE ||
          workActionPhase != WORK_PHASE_IDLE) {
        routeFault(
            "Storage round-2 simulation completion state is invalid");
        return;
      }
      stopMaixRequest();
      route_chassis::stopAllMotorsImmediately();
      stopAllMotorsImmediately();
      disableDriveMotors();
      disableArmBaseMotor();
      storageRound2SimulationPhase =
          STORAGE2_SIMULATION_COMPLETE;
      programState = PROGRAM_FINISHED;
          startRequested = false;
      hmiSetRunStatus("S2DONE");
      hmiSetTaskCounts();
      SerialDebug.println(
          "Storage round-2 local simulation complete: three targeted-color stack placements finished; chassis never released");
      return;
  }
}

void serviceStorageRound1Simulation() {
  if (!STORAGE_ROUND1_SIMULATION_MODE ||
      programState != PROGRAM_RUNNING) {
    return;
  }
  if (storageRound1SimulationPhase ==
      STORAGE1_SIMULATION_WAIT_PREPARATION) {
    serviceStorageTravelPreparation();
    if (!storageTravelPreparationReady(1U)) {
      return;
    }
    activeCompetitionRound = 1U;
    storageActionFinished = false;
    storageRound1SimulationPhase = STORAGE1_SIMULATION_RUN_ACTION;
    hmiSetRunStatus("STACK1");
    SerialDebug.println(
        "[STORAGE1 SIM] arrival pose ready; start three placements");
    beginWorkAction(WORK_ACTION_STORAGE, 1U);
    return;
  }
  if (storageRound1SimulationPhase !=
          STORAGE1_SIMULATION_RUN_ACTION ||
      !storageActionFinished) {
    return;
  }
  if (correctPlacementCount != 3U ||
      correctGrabCount != 0U ||
      workItemIndex != 3U ||
      activeWorkAction != WORK_ACTION_NONE ||
      workActionPhase != WORK_PHASE_IDLE) {
    routeFault("Storage round-1 simulation completion state is invalid");
    return;
  }
  stopMaixRequest();
  route_chassis::stopAllMotorsImmediately();
  stopAllMotorsImmediately();
  disableDriveMotors();
  disableArmBaseMotor();
  storageRound1SimulationPhase = STORAGE1_SIMULATION_COMPLETE;
  programState = PROGRAM_FINISHED;
  startRequested = false;
  hmiSetRunStatus("S1DONE");
  hmiSetTaskCounts();
  SerialDebug.println(
      "[STORAGE1 SIM] complete: three placements; chassis never released");
}

void serviceRawWorkstationSimulation() {
  if (!RAW_WORKSTATION_SIMULATION_MODE ||
      programState != PROGRAM_RUNNING ||
      !rawActionFinished) {
    return;
  }

  // The formal RAW handoff marks its action complete as soon as M6/M7 are
  // safe, while M5 and the tray park in parallel with the next route segment.
  // This route-free program must wait for that same parallel parking to end
  // before disabling anything or reporting completion.
  if (armMotors.isM5Running() ||
      armBaseMotionWatchdogActive ||
      !extensionMoveFinished() ||
      !liftMoveFinished() ||
      !allMotorsArrived() ||
      !deadlineReached(rawSimulationParkingDeadlineMs)) {
    return;
  }

  bool searchOffsetReturned = true;
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
  searchOffsetReturned =
      !rawChassisAtB &&
      motorPulsesAreZero(visualCorrectionAccumulator) &&
      fabsf(visualCorrectionLeftMm) <= 0.01f;
#endif
  if (correctGrabCount != 3U ||
      correctPlacementCount != 0U ||
      rawCollectedCount != 3U ||
      activeWorkAction != WORK_ACTION_NONE ||
      workActionPhase != WORK_PHASE_IDLE ||
      !searchOffsetReturned) {
    routeFault("RAW workstation simulation completion state is invalid");
    return;
  }

  stopMaixRequest();
  route_chassis::stopAllMotorsImmediately();
  stopAllMotorsImmediately();
  disableDriveMotors();
  disableArmBaseMotor();
  rawTravelM5ZeroPending = false;
  rawSimulationParkingDeadlineMs = 0UL;
  programState = PROGRAM_FINISHED;
  startRequested = false;
  hmiSetRunStatus("RAWDONE");
  hmiSetTaskCounts();
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
  SerialDebug.println(
      "[RAW SIM] complete: three materials stored; A/B 40 mm "
      "search offset returned; route never started");
#else
  SerialDebug.println(
      "[RAW SIM] complete: three materials stored by legacy fixed-A "
      "pickup; chassis never moved; route never started");
#endif
}

void abortRoute() {
  chassisStartupHoldActive = false;
  if (route_chassis::programState ==
      route_chassis::PROGRAM_RUNNING) {
    route_chassis::abortRoute();
  } else {
    route_chassis::disableDriveMotors();
  }
  route_chassis::startRequested = false;
  route_chassis::abortRequested = false;
  invalidateArmLinearReference();
  cancelCompetitionAction();
  emergencyStopArmLinearAxes();
  disableDriveMotors();
  disableArmBaseMotor();
  programState = PROGRAM_FAULT;
  storageRound2SimulationPhase =
      STORAGE2_SIMULATION_IDLE;
  storageRound2SimulationGripperDeadlineMs = 0UL;
  storageRound1SimulationPhase = STORAGE1_SIMULATION_IDLE;
  rawSimulationParkingDeadlineMs = 0UL;
  abortRequested = false;
  startRequested = false;
  hmiSetRunStatus("STOP");
  SerialDebug.println("Route aborted by long press");
}

void synchronizeTaskCodeToRouteChassis() {
  /*
   * PE0/PE1由GC任务码解析器读取，因为机械臂需要四组颜色/位置数据；
   * 完整解码后只把锁定结果交给底盘二维码状态。启停区1新路线在到达
   * 中心后开启独立左移扫码窗口并原路归位；旧路线和第4段单段调试才在
   * 第4规范段终点前190 mm条件等待。两条路径均只检查这份锁码结果。
   */
  if (!scanFlag || route_chassis::scanFlag) {
    return;
  }

  strncpy(
      route_chassis::qrData,
      qrData,
      sizeof(route_chassis::qrData) - 1U);
  route_chassis::qrData[
      sizeof(route_chassis::qrData) - 1U] = '\0';
  route_chassis::scanFlag = true;
  SerialDebug.println(
      "[CHASSIS BRIDGE] decoded task code shared with chassis QR state");
}

IntegratedWorkPause workPauseAfterRouteStep(
    uint8_t completedStep) {
  // 维护警告：这里的6/8/11/14/16/19是规范段号，不是route[]下标；
  // 底盘实际为19条运动命令，段2/3已删除。修改底盘表后必须核对接力点。
  switch (completedStep) {
    case 6U:
      return INTEGRATED_WORK_RAW_1;
    case 8U:
      return INTEGRATED_WORK_PROCESS_1;
    case 11U:
      return INTEGRATED_WORK_STORAGE_1;
    case 14U:
      return INTEGRATED_WORK_RAW_2;
    case 16U:
      return INTEGRATED_WORK_PROCESS_2;
    case 19U:
      return INTEGRATED_WORK_STORAGE_2;
    default:
      return INTEGRATED_WORK_NONE;
  }
}

void beginIntegratedWorkPause(IntegratedWorkPause pause) {
  integratedWorkPause = pause;
  integratedWorkPauseStartMs = millis();
  commandStartMs = integratedWorkPauseStartMs;
  // 到站后底盘暂停，M1～M4仍由route_chassis::runAllMotors()输出脉冲；
  // GC工位视觉修正共享这些电机对象，停稳并清除上一段运动包络后才能接管。
  route_chassis::stopAllMotorsImmediately();
  suspendRouteChassisProfileForWorkstation();
  routeImuReferenceDegrees =
      route_chassis::routeImuReferenceDegrees;
  targetCounterClockwiseHeadingDegrees =
      route_chassis::targetCounterClockwiseHeadingDegrees;

  WorkActionKind kind = WORK_ACTION_NONE;
  uint8_t roundNumber = 0U;
  switch (pause) {
    case INTEGRATED_WORK_RAW_1:
      kind = WORK_ACTION_RAW;
      roundNumber = 1U;
      rawActionFinished = false;
      hmiSetRunStatus("RAW1");
      break;
    case INTEGRATED_WORK_PROCESS_1:
      kind = WORK_ACTION_PROCESS;
      roundNumber = 1U;
      processActionFinished = false;
      hmiSetRunStatus("PROCESS1");
      break;
    case INTEGRATED_WORK_STORAGE_1:
      kind = WORK_ACTION_STORAGE;
      roundNumber = 1U;
      storageActionFinished = false;
      hmiSetRunStatus("STORAGE1");
      break;
    case INTEGRATED_WORK_RAW_2:
      kind = WORK_ACTION_RAW;
      roundNumber = 2U;
      rawActionFinished = false;
      hmiSetRunStatus("RAW2");
      break;
    case INTEGRATED_WORK_PROCESS_2:
      kind = WORK_ACTION_PROCESS;
      roundNumber = 2U;
      processActionFinished = false;
      hmiSetRunStatus("PROCESS2");
      break;
    case INTEGRATED_WORK_STORAGE_2:
      kind = WORK_ACTION_STORAGE;
      roundNumber = 2U;
      storageActionFinished = false;
      hmiSetRunStatus("STORAGE2");
      break;
    case INTEGRATED_WORK_NONE:
      return;
  }

  activeCompetitionRound = roundNumber;
  markMissionProgress();
  SerialDebug.print(
      "[CHASSIS BRIDGE] pause route for arm/vision kind/round=");
  SerialDebug.print(static_cast<unsigned int>(kind));
  SerialDebug.print("/");
  SerialDebug.println(roundNumber);
  if (kind == WORK_ACTION_STORAGE &&
      !storageTravelPreparationReady(roundNumber)) {
    integratedWorkActionStartPending = true;
    SerialDebug.print(
        "[BRIDGE] wait prep=");
    SerialDebug.print(static_cast<unsigned int>(kind));
    SerialDebug.print("/");
    SerialDebug.println(roundNumber);
    return;
  }
  if (kind == WORK_ACTION_RAW &&
      roundNumber == 2U &&
      storage1Raw2TravelPhase ==
          STORAGE1_RAW2_TRAVEL_RUNNING) {
    integratedWorkActionStartPending = true;
    SerialDebug.println(
        "[RAW2] wait travelling M6");
    return;
  }
  integratedWorkActionStartPending = false;
  beginWorkAction(kind, roundNumber);
}

bool integratedWorkPauseFinished() {
  switch (integratedWorkPause) {
    case INTEGRATED_WORK_RAW_1:
    case INTEGRATED_WORK_RAW_2:
      return rawActionFinished;
    case INTEGRATED_WORK_PROCESS_1:
    case INTEGRATED_WORK_PROCESS_2:
      return processActionFinished;
    case INTEGRATED_WORK_STORAGE_1:
    case INTEGRATED_WORK_STORAGE_2:
      return storageActionFinished;
    case INTEGRATED_WORK_NONE:
      return false;
  }
  return false;
}

void finishIntegratedWorkPause() {
  const uint8_t storageTravelRound =
      integratedWorkPause == INTEGRATED_WORK_PROCESS_1
          ? 1U
          : (integratedWorkPause ==
                     INTEGRATED_WORK_PROCESS_2
                 ? 2U
                 : 0U);
  SerialDebug.print(
      "[CHASSIS BRIDGE] arm/vision complete, resume route after ms=");
  SerialDebug.println(millis() - integratedWorkPauseStartMs);
  integratedWorkPause = INTEGRATED_WORK_NONE;
  integratedWorkPauseStartMs = 0UL;
  integratedWorkActionStartPending = false;
  routeImuReferenceDegrees =
      route_chassis::routeImuReferenceDegrees;
  targetCounterClockwiseHeadingDegrees =
      route_chassis::targetCounterClockwiseHeadingDegrees;
  hmiSetRunStatus("RUN");
  markMissionProgress();
  if (storageTravelRound != 0U) {
    beginStorageTravelPreparation(storageTravelRound);
  }
}

bool serviceChassisStartupHold() {
  if (!chassisStartupHoldActive) {
    return false;
  }
  if (static_cast<uint32_t>(millis() - chassisStartupHoldStartMs) <
      CHASSIS_STARTUP_HOLD_MS) {
    return true;
  }
  chassisStartupHoldActive = false;
  hmiSetRunStatus("RUN");
  markMissionProgress();
  SerialDebug.println("[START RELEASE] minimum 500ms reached; start first route command");
  return false;
}

void serviceIntegratedRoute() {
  // 正式路线唯一调度入口；工位运行时不得调用底盘updateRoute()抢走电机。
  if (LOCAL_WORKSTATION_SIMULATION_MODE ||
      programState != PROGRAM_RUNNING) {
    return;
  }

  if (route_chassis::programState ==
      route_chassis::PROGRAM_FAULT) {
    routeFault("GongChuang_route chassis entered FAULT");
    return;
  }

  // route_chassis::beginRoute()只清零/使能，不下发目标；此处挡住首段调度。
  if (serviceChassisStartupHold()) {
    return;
  }

  serviceStorageTravelPreparation();
  serviceStorage1Raw2TravelPreparation();

  if (integratedWorkPause != INTEGRATED_WORK_NONE) {
    if (integratedWorkActionStartPending) {
      if (integratedWorkPause == INTEGRATED_WORK_RAW_2) {
        if (!storage1Raw2TravelPreparationReady()) {
          return;
        }
        integratedWorkActionStartPending = false;
        SerialDebug.println(
            "[RAW2] travelling M6 ready");
        beginWorkAction(WORK_ACTION_RAW, 2U);
        return;
      }
      if (storageTravelPreparationReady(
              activeCompetitionRound)) {
        integratedWorkActionStartPending = false;
        SerialDebug.println(
            "[BRIDGE] prep ready");
        beginWorkAction(
            WORK_ACTION_STORAGE,
            activeCompetitionRound);
      }
      return;
    }
    if (integratedWorkPauseFinished()) {
      finishIntegratedWorkPause();
    }
    return;
  }

  const size_t previousIndex = route_chassis::routeIndex;
  route_chassis::updateRoute();

  if (route_chassis::programState ==
      route_chassis::PROGRAM_FAULT) {
    routeFault("GongChuang_route chassis entered FAULT");
    return;
  }

  if (route_chassis::programState ==
      route_chassis::PROGRAM_FINISHED) {
    finishProgram();
    return;
  }

  if (route_chassis::routeIndex != previousIndex &&
      previousIndex < route_chassis::ROUTE_COMMAND_COUNT) {
    const uint8_t completedStep =
        route_chassis::route[previousIndex].specificationStep;
    markMissionProgress();
    const IntegratedWorkPause pause =
        workPauseAfterRouteStep(completedStep);
    if (pause != INTEGRATED_WORK_NONE) {
      beginIntegratedWorkPause(pause);
    }
  }
}

void serviceCompetitionWatchdogs() {
  if (LOCAL_WORKSTATION_SIMULATION_MODE ||
      programState != PROGRAM_RUNNING) {
    return;
  }

  const uint32_t nowMs = millis();
  const bool routeWaitingForQrCode =
      route_chassis::programState ==
          route_chassis::PROGRAM_RUNNING &&
      route_chassis::routeIndex <
          route_chassis::ROUTE_COMMAND_COUNT &&
      route_chassis::routePhase ==
          route_chassis::ROUTE_PHASE_WAIT_SCAN_CODE;

  if (routeWaitingForQrCode ||
      (activeWorkAction == WORK_ACTION_RAW &&
       workActionPhase == WORK_PHASE_RAW_WAIT_RESULT)) {
    return;
  }

  if (nowMs - lastMissionProgressMs >=
      MISSION_PROGRESS_TIMEOUT_MS) {
    routeFault("Mission progress watchdog timeout");
  }
}

void updateHmiYaw() {
  if (!DISPLAY_YAW_ON_X0 ||
      !imuTracker.initialized()) {
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
  /* M1～M4及公共使能已经由route_chassis::setup()完整初始化。 */
  armMotors.beginM5();
  useArmBaseEndpointMotionProfile();

  armMotors.setM5CurrentAngle(0.0f);
  armMotors.disableM5();

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
      RAW_WORKSTATION_SIMULATION_MODE
          ? "RAW123"
          : (ROUGH_PROCESSING_CALIBRATION_MODE
          ? "CAL132"
          : (STORAGE_ROUND1_SIMULATION_MODE
                 ? "STACK1"
                 : (STORAGE_ROUND2_SIMULATION_MODE
                 ? "STACK2"
                  : "QRWAIT"))));
  hmiSetText("t3", "000+000+");
  hmiSetText("t8", "000+000");
  hmiSetRunStatus(
      !armLinearReferenceValid
          ? "M6INIT"
           : (RAW_WORKSTATION_SIMULATION_MODE
                  ? "RAWREADY"
                  : (ROUGH_PROCESSING_CALIBRATION_MODE
                  ? "CALREADY"
                  : (STORAGE_ROUND1_SIMULATION_MODE
                         ? "S1READY"
                         : (STORAGE_ROUND2_SIMULATION_MODE
                         ? "S2READY"
                         : (selectedStartZone == START_ZONE_1
                                ? "READY1"
                                 : "READY2"))))));
  hmiSetTaskCounts();

  if (DISPLAY_YAW_ON_X0) {
    hmiSetValue("x0", 0);
  }
  if (DISPLAY_BATTERY_ON_X1) {
    hmiSetValue("x1", 0);
  }
}

void initializeManipulationHardware() {
  maixCam.begin(MAIXCAM_BAUDRATE);

  const uint32_t manipulationStartupMs = millis();
  if (manipulationStartupMs <
      ARM_LINEAR_POWER_ON_SETTLE_MS) {
    delay(
        ARM_LINEAR_POWER_ON_SETTLE_MS -
        manipulationStartupMs);
  }
  SerialArmLinear.begin(ARM_LINEAR_BAUDRATE);
  armLinearSerialInitialized = true;
  clearArmLinearReceiveBuffer();
  resetArmLinearSoftwareOrigin();
  // M7 can retain a protection latch across a controller reset. Stop any
  // residual command, clear that latch and require both replies before the
  // existing 0.5/10 mm startup moves are allowed to prove real holding force.
  writeArmLinearStop(liftAxis.address);
  SerialArmLinear.flush();
  delay(ARM_AXIS_COMMAND_GUARD_MS);
  clearArmLinearReceiveBuffer();
  writeArmLinearResetStallProtection(liftAxis.address);
  SerialArmLinear.flush();
  armLinearBootReady = waitForArmLinearSimpleResponse(
      liftAxis.address,
      0x0EU,
      ARM_LINEAR_POSITION_READ_TIMEOUT_MS);

  clearArmLinearReceiveBuffer();
  writeArmLinearEnable(extensionAxis.address, true);
  SerialArmLinear.flush();
  delay(ARM_AXIS_COMMAND_GUARD_MS);
  clearArmLinearReceiveBuffer();
  writeArmLinearEnable(liftAxis.address, true);
  SerialArmLinear.flush();
  armLinearBootReady =
      waitForArmLinearSimpleResponse(
          liftAxis.address,
          0xF3U,
          ARM_LINEAR_POSITION_READ_TIMEOUT_MS) &&
      armLinearBootReady;
  clearArmLinearReceiveBuffer();

  servoProtocol.init(&SerialServo, SERVO_BAUDRATE);
  delay(100);
  gripperServo.init();
  storageServo.init();
  manipulationServosOnline =
      gripperServo.isOnline && storageServo.isOnline;
  SerialDebug.print("Servo online gripper/storage: ");
  SerialDebug.print(gripperServo.isOnline ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.println(storageServo.isOnline ? 1 : 0);
  commandGripperClose();
  commandStorageServoParkingPosition();

  SerialDebug.print("M6 microsteps/pulses-per-mm=");
  SerialDebug.print(M6_MICROSTEPS, 0);
  SerialDebug.print("/");
  SerialDebug.println(M6_PULSES_PER_MM, 4);
  SerialDebug.print("M7 microsteps/pulses-per-mm=");
  SerialDebug.print(M7_MICROSTEPS, 0);
  SerialDebug.print("/");
  SerialDebug.println(M7_PULSES_PER_MM, 4);
}

}

void setup() {
  /*
   * 串口、PB9、M1～M4、IMU、二维码、HMI和电池先由route正式main初始化；
   * GC随后只追加机械臂、舵机、MaixCAM和工位动作。
   */
  route_chassis::setup();
  if (LOCAL_WORKSTATION_SIMULATION_MODE) {
    // The route controller remains WAITING in bench modes; its own long-press
    // callback only aborts a RUNNING route. Bind the arm abort directly.
    route_chassis::startButton.attachLongPressStart(
        onStartButtonLongPress);
    SerialDebug.print("[DEBUG WORKSTATION] state=");
    SerialDebug.println(
        RAW_WORKSTATION_SIMULATION_MODE
            ? 4
            : (ROUGH_PROCESSING_CALIBRATION_MODE
            ? 1
            : (STORAGE_ROUND1_SIMULATION_MODE ? 2 : 3)));
  }
#if GONGCHUANG_STORAGE2_VERBOSE_SCAN_DEBUG
  SerialDebug.println(
      "[S2DBG ENABLED] detailed STORAGE2 scan telemetry; baud=115200");
  SerialDebug.println(
      "[S2DBG PROTOCOL] STM32 PE8(TX)->Maix A17(RX), "
      "Maix A16(TX)->STM32 PE7(RX), common GND, UART 115200 8N1, V2 CRC8");
  SerialDebug.println(
      "[S2DBG FIXTURE] scan order r3=4(GREEN), r2=3(BLUE), r1=1(RED); "
      "bottom rings must be r1/r2/r3=RED/BLUE/GREEN for fixed task "
      "134+123+314+231");
  SerialDebug.print(
      "[S2DBG CONFIG] dwell-ms/M5-r1/M5-r3/M6-outer-mm/roi-px/px-mm-scale=");
  SerialDebug.print(arm_config::STORAGE2_SCAN_DWELL_MS);
  SerialDebug.print('/');
  SerialDebug.print(STORAGE2_SCAN_RING1_M5_CORRECTION_DEGREES, 2);
  SerialDebug.print('/');
  SerialDebug.print(STORAGE2_SCAN_RING3_M5_CORRECTION_DEGREES, 2);
  SerialDebug.print('/');
  SerialDebug.print(STORAGE2_SCAN_OUTER_RING_M6_EXTENSION_CORRECTION_MM, 2);
  SerialDebug.print('/');
  SerialDebug.print(arm_config::STORAGE2_TARGET_COLOR_ROI_RADIUS_PIXELS);
  SerialDebug.print('/');
  SerialDebug.println(
      arm_config::STORAGE2_PIXEL_TO_MM_CORRECTION_SCALE,
      2);
#endif
  SerialDebug.println("[ARM REV] GC6-20261001 linked-M6-first");
  SerialDebug.println("[ARM REV] GC6-1003 R1+1 E+30 R1R1.5");
  selectedStartZone =
      route_chassis::selectedStartZoneNumber == 1U
          ? START_ZONE_1
          : START_ZONE_2;
  initializeManipulationHardware();
  initializeMotorOutputs();

  SerialDebug.println(
      "[ZERO SAFETY] Before this power-up M6 must be fully "
      "retracted and M7 at its physical highest point. A previous "
      "FAULT does not retract either axis.");
  initializeHmi();
  const bool armWorkingZerosReady =
      armLinearBootReady &&
      establishArmLinearSafeWorkingZeros();
  hmiSetRunStatus(
      armWorkingZerosReady
          ? (RAW_WORKSTATION_SIMULATION_MODE
                 ? "RAWREADY"
                 : (ROUGH_PROCESSING_CALIBRATION_MODE
                 ? "CALREADY"
                 : (STORAGE_ROUND1_SIMULATION_MODE
                        ? "S1READY"
                        : (STORAGE_ROUND2_SIMULATION_MODE
                        ? "S2READY"
                        : (selectedStartZone == START_ZONE_1
                               ? "READY1"
                               : "READY2")))))
          : "ARMZEROERR");
  resetQrReceiver();

  if (!armWorkingZerosReady) {
    SerialDebug.println(
        "Controller NOT READY: M6/M7 failed to establish "
        "their 10 mm safe working zeros");
    SerialDebug.println(
        "SAFETY: power off and manually restore both mechanical "
        "start endpoints before another startup");
  } else if (RAW_WORKSTATION_SIMULATION_MODE) {
#if GONGCHUANG_RAW_PICKUP_LOGIC_MODE == 1
    SerialDebug.println(
        "[RAW SIM] ready: place the chassis at the RAW station in the "
        "normal A pose with 40 mm clear travel on both sides");
#else
    SerialDebug.println(
        "[RAW SIM] ready: legacy fixed-A pickup is selected; "
        "place the chassis at the original RAW station pose");
#endif
    SerialDebug.print("[RAW SIM] tray empty; task/selected round=");
    SerialDebug.print(GONGCHUANG_DEBUG_RAW_TASK_CODE);
    SerialDebug.print('/');
    SerialDebug.println(GONGCHUANG_DEBUG_RAW_ROUND);
    SerialDebug.println(
        "[RAW SIM] PB9 starts only the production RAW action; the route "
        "stays disabled; long-press stops");
  } else if (ROUGH_PROCESSING_CALIBRATION_MODE) {
    SerialDebug.println(
        "Rough-processing calibration ready: place the chassis "
        "at D, face the three rings, and load tray slots 0/1/2");
    SerialDebug.println(
        "Press PB9 once to detect rings 1/3, derive ring 2, "
        "place three items and pick all three back; "
        "long-press to stop");
  } else if (STORAGE_ROUND1_SIMULATION_MODE) {
    SerialDebug.println(
        "[STORAGE1 SIM] place chassis at first storage station; "
        "tray slots 0/1/2 must contain colors 1/2/3");
    SerialDebug.println(
        "[STORAGE1 SIM] PB9 starts ring 1/3/2 placements; "
        "wheels stay locked; long-press to stop");
  } else if (STORAGE_ROUND2_SIMULATION_MODE) {
    SerialDebug.println(
        "Storage round-2 local simulation ready: place chassis at the "
        "second storage station with wheels aligned and stationary");
    SerialDebug.println(
        "Physical setup for 134+123+314+231: bottom colors 1/3/4 on "
        "rings 1/2/3; tray slots 0/1/2 hold colors 3/1/4; press PB9 once");
  } else {
    SerialDebug.println(
        "GongChuang_route chassis + GC arm/vision ready");
    SerialDebug.println(
        "HMI 1/2 selects Zone1/Zone2. PB9 click starts the current zone; "
        "during running, hold 2 s to stop");
  }
}

void loop() {
  route_chassis::runAllMotors();
  armMotors.serviceM5();
  serviceArmBaseMotionWatchdog();
  route_chassis::startButton.tick();
  if (!LOCAL_WORKSTATION_SIMULATION_MODE) {
    route_chassis::receiveHmiStartCommand();
  }
  route_chassis::receiveImuData();
  serviceArmLinearAxes();
  serviceMaixcam();
  if (!LOCAL_WORKSTATION_SIMULATION_MODE &&
      programState == PROGRAM_RUNNING) {
    /*
     * 启停区1新路线只允许到达中心后收到的新任务码生效。底盘状态机先清空
     * 共享UART和自身锁码；这里同步清空正式解析器、TaskPlan和候选帧，再
     * 才读取中心点之后的新字节。旧路线和启停区2不会发出此请求。
     */
    if (route_chassis::consumeZone1FreshQrResetRequest()) {
      // 底盘在中心到达瞬间已经排空旧字节；此处只清解析状态，保留此后到达
      // 的新帧，避免状态机交接的一个loop间隙误丢中心点有效二维码。
      resetQrReceiver(false);
    }
    receiveQrData();
    synchronizeTaskCodeToRouteChassis();
  }
  route_chassis::updateHmiYaw();
  route_chassis::serviceBatteryVoltage();

  if (route_chassis::abortRequested || abortRequested) {

    if (programState == PROGRAM_RUNNING ||
        route_chassis::programState ==
            route_chassis::PROGRAM_RUNNING ||
        (programState == PROGRAM_WAITING &&
         (startRequested ||
          route_chassis::startRequested))) {
      abortRoute();
    } else {
      abortRequested = false;
      route_chassis::abortRequested = false;
    }
  }

  if (route_chassis::startRequested &&
      route_chassis::programState ==
          route_chassis::PROGRAM_WAITING &&
      programState == PROGRAM_WAITING) {
    startRequested = true;
  }

  if (startRequested && programState != PROGRAM_RUNNING) {
    beginRoute();
    if (!startRequested &&
        programState != PROGRAM_RUNNING) {
      route_chassis::startRequested = false;
    }
  }

  serviceCompetitionWatchdogs();
  serviceCompetitionAction();
  serviceRawWorkstationSimulation();
  serviceStorageRound2Simulation();
  serviceStorageRound1Simulation();
  serviceIntegratedRoute();
}
