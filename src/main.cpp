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

using namespace mecanum;

/*
 * 底盘唯一实现来源：GongChuang_route/src/main.cpp 的完整副本。
 *
 * route_chassis_main.inc 保持21步路线、M1～M4、PB9、IMU、二维码、HMI、
 * 电池与全部底盘速度/位移控制原样不变。GC当前文件只在六个工位暂停窗口
 * 运行机械臂和视觉，禁止维护第二套正在运行的底盘状态机。
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

const float ARM_BASE_MAXIMUM_STEP_RATE = 81000.0f;
const float ARM_BASE_STEP_ACCELERATION = 27000.0f;
const float ARM_BASE_ENDPOINT_COARSE_MAXIMUM_STEP_RATE = 121500.0f;
const float ARM_BASE_ENDPOINT_COARSE_STEP_ACCELERATION = 40500.0f;

const float ARM_BASE_ENDPOINT_TRAVEL_MAXIMUM_STEP_RATE = 182250.0f;
const float ARM_BASE_ENDPOINT_TRAVEL_STEP_ACCELERATION = 60750.0f;

// Vision-facing M5 stabilization is capped at 20 ms. Endpoint preparation
// already uses the shorter dedicated 10 ms gate below.
constexpr uint32_t ARM_BASE_SETTLE_MS = 20UL;
constexpr uint32_t FIRST_ENDPOINT_M7_SETTLE_MS = 10UL;
static_assert(
    FIRST_ENDPOINT_M7_SETTLE_MS * 2UL == ARM_BASE_SETTLE_MS,
    "First endpoint M7 settle must remain below the vision delay cap");
// Endpoint vision moves have already been position-verified. The remaining
// M5-to-M6 handoff is reduced by exactly 70% without changing the normal arm
// base settle used by travel, recovery, or non-vision standardization.
constexpr uint32_t ENDPOINT_BASE_TO_EXTENSION_SETTLE_PREVIOUS_MS = 50UL;
constexpr uint32_t ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS = 15UL;
static_assert(
    ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS * 10UL ==
        ENDPOINT_BASE_TO_EXTENSION_SETTLE_PREVIOUS_MS * 3UL,
    "Endpoint M5-to-M6 handoff must remain reduced by 70 percent");
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
constexpr uint32_t ARM_AXIS_STATUS_INTERVAL_MS = 30UL;
constexpr uint32_t ARM_AXIS_MINIMUM_ON_POSITION_MS = 120UL;
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
    M7_FAST_ARRIVAL_SAMPLE_GAP_MS <
            M7_FAST_ARRIVAL_QUERY_INTERVAL_MS &&
        M7_FAST_ARRIVAL_RESPONSE_TIMEOUT_MS <=
            ARM_LINEAR_POSITION_READ_TIMEOUT_MS &&
        M7_FAST_ARRIVAL_TARGET_TOLERANCE_MM <
            ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM &&
        M7_FAST_ARRIVAL_STABILITY_MM <
            ARM_AXIS_RECOVERY_POSITION_STABILITY_MM,
    "M7 encoder arrival gate must remain faster and stricter than recovery");

constexpr float ARM_LINEAR_STARTUP_PROBE_MM = 0.5f;
constexpr float ARM_LINEAR_ZERO_ANGLE_RATIO_MINIMUM = 0.80f;
constexpr float ARM_LINEAR_ZERO_ANGLE_RATIO_MAXIMUM = 1.20f;

constexpr float M6_STANDARD_EXTENSION_MM = 0.0f;

constexpr float M6_RING2_MINIMUM_EXTENSION_MM = -8.0f;
static_assert(
    M6_STARTUP_WORKING_ZERO_OFFSET_MM +
            M6_RING2_MINIMUM_EXTENSION_MM >=
        2.0f,
    "Ring-2 M6 retraction must retain 2 mm hard-stop margin");
constexpr float M7_STANDARD_HEIGHT_MM = 0.0f;

constexpr float M6_MAXIMUM_PHYSICAL_EXTENSION_MM = 150.0f;
constexpr float M6_MAXIMUM_EXTENSION_MM =
    M6_MAXIMUM_PHYSICAL_EXTENSION_MM -
    M6_STARTUP_WORKING_ZERO_OFFSET_MM;

constexpr float M7_MINIMUM_PHYSICAL_HEIGHT_MM = -160.0f;
constexpr float M7_MINIMUM_HEIGHT_MM =
    M7_MINIMUM_PHYSICAL_HEIGHT_MM +
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float ARM_AXIS_POSITION_TOLERANCE_MM = 0.05f;

constexpr float RAW_PICK_PHYSICAL_LOWER_MM = 73.0f;
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
            arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM <=
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
constexpr uint32_t GRIPPER_TARGET_PICK_CLOSE_SETTLE_MS = 120UL;
// A 20 ms command asks the servo to move quickly, but the loaded jaw can take
// longer to reach clamping force. Tray pickup therefore gets a dedicated
// completion gate before M7 is allowed to reverse.
constexpr uint32_t GRIPPER_TRAY_PICK_CLOSE_SETTLE_MS = 160UL;
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
// Target placement is the one requested 50% exception. Pickup from either
// side and release into the tray retain the common 10 ms handoff.
constexpr uint32_t WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_PREVIOUS_MS = 10UL;
constexpr uint32_t WORK_TARGET_PLACE_M7_TO_GRIPPER_GAP_MS = 5UL;
constexpr uint32_t WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS = 100UL;
constexpr uint32_t RING_RETURN_STORAGE_COMMAND_DELAY_MS = 500UL;
static_assert(
    WORK_M7_TO_GRIPPER_GAP_MS > 0UL &&
        WORK_GRIPPER_TO_M7_GAP_MS > 0UL &&
        WORK_M7_TO_GRIPPER_GAP_MS <=
            WORK_M7_TO_GRIPPER_RESPONSE_LIMIT_MS,
    "Workstation M7/gripper command handoffs must remain nonzero");
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
static_assert(
    STORAGE_SERVO_SETTLE_MS >=
        static_cast<uint32_t>(STORAGE_SERVO_INTERVAL_MS) + 100UL,
    "Storage servo settle gate must retain at least 100 ms margin");

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
constexpr float PIXEL_MAPPING_SWITCH_RADIUS_PIXELS = 50.0f;
constexpr float RAW_NEAR_MM_PER_PIXEL = 40.0f / 72.0f;
constexpr float RAW_FAR_MM_PER_PIXEL = 70.0f / 72.0f;

constexpr float RAW_VIEW_EXTENSION_MM = 50.0f;
static_assert(
    RAW_VIEW_EXTENSION_MM >= M6_STANDARD_EXTENSION_MM &&
        RAW_VIEW_EXTENSION_MM <= M6_MAXIMUM_EXTENSION_MM,
    "RAW view extension must stay inside M6 travel");

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
// A mode-10 result already represents at least two camera samples collected
// across the 120 ms endpoint stability window. If that result is both inside
// the stricter 2 px target and completely stable (confidence 1000), accepting
// it immediately removes one redundant 120 ms window. Borderline results
// still require the normal two independent STM32 confirmations below.
constexpr float ENDPOINT_FAST_ACCEPT_CENTER_TOLERANCE_PIXELS = 2.0f;
constexpr uint16_t ENDPOINT_FAST_ACCEPT_MINIMUM_CONFIDENCE = 1000U;
static_assert(
    ENDPOINT_FAST_ACCEPT_CENTER_TOLERANCE_PIXELS <
            ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS &&
        ENDPOINT_FAST_ACCEPT_MINIMUM_CONFIDENCE >=
            RING_ENDPOINT_MINIMUM_CONFIDENCE,
    "Endpoint fast acceptance must be stricter than the normal final gate");
constexpr uint8_t ENDPOINT_MAXIMUM_SERVO_MOVES_PER_STAGE = 5U;

constexpr uint8_t ENDPOINT_MAXIMUM_TOTAL_SERVO_MOVES = 7U;
constexpr float ENDPOINT_COARSE_SERVO_GAIN = 0.85f;
constexpr float ENDPOINT_FINE_SERVO_GAIN = 0.55f;
constexpr float ENDPOINT_COARSE_MAXIMUM_CORRECTION_MM = 25.0f;
constexpr float ENDPOINT_FINE_MAXIMUM_CORRECTION_MM = 6.0f;
// Closed-loop endpoint vision cadence: after each M5/M6 correction reaches
// its verified target, wait at most 20 ms before requesting the next frame.
constexpr uint32_t ENDPOINT_LOCAL_MOVE_SETTLE_PREVIOUS_MS = 80UL;
constexpr uint32_t ENDPOINT_LOCAL_MOVE_SETTLE_MS = 20UL;
static_assert(
    ENDPOINT_LOCAL_MOVE_SETTLE_MS * 4UL ==
        ENDPOINT_LOCAL_MOVE_SETTLE_PREVIOUS_MS,
    "Endpoint re-recognition settle must remain reduced by 75 percent");

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
constexpr float RING_MAP_MAXIMUM_HEADING_DRIFT_DEGREES = 1.0f;
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

constexpr float STEP_01_MOTION_SCALE = 1.02f;
constexpr float STEP_02_MOTION_SCALE = 1.00f;
constexpr float STEP_03_MOTION_SCALE = 1.00f;
constexpr float STEP_04_MOTION_SCALE = 1.005f;
constexpr float STEP_05_MOTION_SCALE = 0.965f;
constexpr float STEP_06_MOTION_SCALE = 1.00f;
constexpr float STEP_07_MOTION_SCALE = 0.998f;
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
constexpr float ROUTE_DECELERATION_ACCELERATION_SCALE = 2.0f / 3.0f;
constexpr float ROUTE_DECELERATION_SWITCH_MARGIN_STEPS = 8.0f;
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
// Vision action gates are deliberately capped at 20 ms. Failure timeouts and
// camera retransmission periods are not action delays and stay independent.
constexpr uint32_t VISION_ACTION_DELAY_CAP_MS = 20UL;
constexpr uint32_t VISION_POST_MOTION_SETTLE_TIME_MS = 20UL;
constexpr uint32_t VISION_HEADING_STABLE_TIME_MS = 20UL;
constexpr uint32_t ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS = 15UL;
constexpr uint32_t ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS = 20UL;
static_assert(
    FIRST_ENDPOINT_M7_SETTLE_MS +
            ENDPOINT_PRE_SCAN_POST_MOTION_SETTLE_TIME_MS +
            ENDPOINT_PRE_SCAN_HEADING_STABLE_TIME_MS +
            ENDPOINT_BASE_TO_EXTENSION_SETTLE_MS ==
        60UL,
    "First endpoint software handoff must stay below 100 ms");
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
    "Every deliberate vision action delay must be at most 20 ms");

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

constexpr uint32_t RAW_TARGET_REQUEST_REFRESH_MS = 15000UL;
constexpr uint32_t RAW_ACTION_TIMEOUT_MS = 65000UL;
constexpr uint32_t PROCESS_ACTION_TIMEOUT_MS = 65000UL;
constexpr uint32_t STORAGE_ACTION_TIMEOUT_MS = 45000UL;
constexpr uint32_t MISSION_PROGRESS_TIMEOUT_MS = 70000UL;

constexpr bool ROUGH_PROCESSING_CALIBRATION_MODE =
    GONGCHUANG_RUN_MODE == 1;

const bool DISPLAY_YAW_ON_X0 = true;
const bool DISPLAY_BATTERY_ON_X1 = true;
const bool SHOW_RESULT_PAGE_ON_FINISH = true;

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

bool armBaseMotionLockedUntilReset = false;
bool armBaseMotionWatchdogActive = false;
uint32_t armBaseMotionDeadlineMs = 0UL;

AccelStepper *const motors[4] = {
    &motor1, &motor2, &motor3, &motor4};

float activeDriveAcceleration = 1.0f;
float activeDriveDeceleration = 1.0f;
bool driveDecelerationActive = false;
bool routeDriveProfileEnabled = false;

bool integratedTurnControlActive = false;
bool integratedTurnBrakeCommandIssued = false;
int8_t integratedTurnDirectionSign = 0;
float activeTurnPulsesPerDegree = 1.0f;

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

  hmiSetValue("n5", correctGrabCount);
  hmiSetValue("n6", correctPlacementCount);
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

void resetQrReceiver() {
  qrDataIndex = 0;
  qrOverflow = false;
  scanFlag = false;
  taskCodeDecoded = false;
  qrData[0] = '\0';
  qrCandidate[0] = '\0';
  qrMatchingFrameCount = 0U;
  taskPlan.clear();

  while (SerialQr.available()) {
    SerialQr.read();
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

void startRelativeMotorMove(const MotorPulses &pulses) {

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

void serviceDriveDecelerationProfile() {

  if (!routeDriveProfileEnabled ||
      integratedTurnControlActive ||
      driveDecelerationActive ||
      activeDriveDeceleration <= 0.0f) {
    return;
  }

  bool shouldStartDeceleration = false;
  for (uint8_t i = 0U; i < 4U; ++i) {
    long remainingSteps = motors[i]->distanceToGo();
    if (remainingSteps < 0L) {
      remainingSteps = -remainingSteps;
    }

    const float currentSpeed = fabsf(motors[i]->speed());
    if (remainingSteps == 0L || currentSpeed <= 0.0f) {
      continue;
    }

    const float stoppingSteps =
        currentSpeed * currentSpeed /
        (2.0f * activeDriveDeceleration);
    if (static_cast<float>(remainingSteps) <=
        stoppingSteps +
            ROUTE_DECELERATION_SWITCH_MARGIN_STEPS) {
      shouldStartDeceleration = true;
      break;
    }
  }

  if (!shouldStartDeceleration) {
    return;
  }

  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->setAcceleration(activeDriveDeceleration);
  }
  driveDecelerationActive = true;
}

void runAllMotors() {

  serviceDriveDecelerationProfile();
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
  driveDecelerationActive = false;
  routeDriveProfileEnabled = false;
  integratedTurnControlActive = false;
  integratedTurnBrakeCommandIssued = false;
  integratedTurnDirectionSign = 0;
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

void useArmBaseEndpointMotionProfile() {
  armMotors.setM5MotionProfile(
      ARM_BASE_MAXIMUM_STEP_RATE,
      ARM_BASE_STEP_ACCELERATION);
}

void useArmBaseEndpointCoarseMotionProfile() {
  armMotors.setM5MotionProfile(
      ARM_BASE_ENDPOINT_COARSE_MAXIMUM_STEP_RATE,
      ARM_BASE_ENDPOINT_COARSE_STEP_ACCELERATION);
}

void useArmBaseEndpointTravelMotionProfile() {
  armMotors.setM5MotionProfile(
      ARM_BASE_ENDPOINT_TRAVEL_MAXIMUM_STEP_RATE,
      ARM_BASE_ENDPOINT_TRAVEL_STEP_ACCELERATION);
}

void useArmBaseTransferMotionProfile() {
  armMotors.setM5MotionProfile(
      arm_config::M5_STANDARD_MAXIMUM_STEP_RATE,
      arm_config::M5_STANDARD_STEP_ACCELERATION);
}

void useArmBasePlaceMotionProfile() {
  armMotors.setM5MotionProfile(
      arm_config::M5_PLACE_MAXIMUM_STEP_RATE,
      arm_config::M5_PLACE_STEP_ACCELERATION);
}

void useArmBaseReturnMotionProfile() {
  armMotors.setM5MotionProfile(
      arm_config::M5_RETURN_MAXIMUM_STEP_RATE,
      arm_config::M5_RETURN_STEP_ACCELERATION);
}

void useArmBaseLoadedReturnMotionProfile() {
  armMotors.setM5MotionProfile(
      arm_config::M5_LOADED_RETURN_MAXIMUM_STEP_RATE,
      arm_config::M5_LOADED_RETURN_STEP_ACCELERATION);
}

void useArmBaseRawTransferMotionProfile() {
  armMotors.setM5MotionProfile(
      arm_config::M5_RAW_MAXIMUM_STEP_RATE,
      arm_config::M5_RAW_STEP_ACCELERATION);
}

void setDriveMotionProfile(float maximumStepRate, float acceleration) {

  routeDriveProfileEnabled = false;
  driveDecelerationActive = false;
  for (uint8_t i = 0; i < 4; ++i) {
    motors[i]->setMaxSpeed(maximumStepRate);
    motors[i]->setAcceleration(acceleration);
  }
}

void setRouteDriveMotionProfile(
    float maximumStepRate,
    float acceleration) {
  activeDriveAcceleration = acceleration;
  activeDriveDeceleration =
      acceleration *
      ROUTE_DECELERATION_ACCELERATION_SCALE;
  driveDecelerationActive = false;
  routeDriveProfileEnabled = true;

  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->setMaxSpeed(maximumStepRate);
    motors[i]->setAcceleration(activeDriveAcceleration);
  }
}

enum CommandType {
  COMMAND_ZONE_LONGITUDINAL_FAST,
  COMMAND_SCAN_SLOW,
  COMMAND_ADJUST_TO_POINT_A,
  COMMAND_FORWARD_FAST,
  COMMAND_BACKWARD_FAST,
  COMMAND_RIGHT_FAST,
  COMMAND_TURN_COUNTERCLOCKWISE,
  COMMAND_TURN_CLOCKWISE,
  COMMAND_RAW_ACTION,
  COMMAND_PROCESS_ACTION,
  COMMAND_STORAGE_ACTION,
  COMMAND_FINISH
};

struct RouteCommand {
  uint8_t specificationStep;
  CommandType type;
  int32_t value;
  float motionScale;
  const char *name;

  RouteCommand(
      uint8_t step,
      CommandType commandType,
      int32_t commandValue,
      float scale,
      const char *commandName)
      : specificationStep(step),
        type(commandType),
        value(commandValue),
        motionScale(scale),
        name(commandName) {}
};

const RouteCommand route[] = {
    {1U, COMMAND_ZONE_LONGITUDINAL_FAST, DISTANCE_A_MM,
     STEP_01_MOTION_SCALE, "Start zone -> slow QR scan start (a)"},
    {2U, COMMAND_SCAN_SLOW, MAXIMUM_SCAN_DISTANCE_B_MM,
     STEP_02_MOTION_SCALE, "Slow scan until QR success, b <= 200"},
    {3U, COMMAND_ADJUST_TO_POINT_A, SCAN_START_TO_POINT_A_MM,
     STEP_03_MOTION_SCALE, "Adjust by 92-b to QR midpoint A"},
    {4U, COMMAND_RIGHT_FAST, DISTANCE_C_MM,
     STEP_04_MOTION_SCALE, "A -> center B, right c"},
    {5U, COMMAND_BACKWARD_FAST, DISTANCE_D_MM,
     STEP_05_MOTION_SCALE, "B -> raw area C, backward d"},
    {6U, COMMAND_TURN_COUNTERCLOCKWISE, 90,
     STEP_06_MOTION_SCALE, "At C turn counter-clockwise 90"},
    {0U, COMMAND_RAW_ACTION, 1, 1.0f, "Raw action round 1 at C"},
    {7U, COMMAND_RIGHT_FAST, DISTANCE_E_MM,
     STEP_07_MOTION_SCALE, "C -> rough process D, right e"},
    {8U, COMMAND_TURN_CLOCKWISE, 180,
     STEP_08_MOTION_SCALE, "At D turn clockwise 180"},
    {0U, COMMAND_PROCESS_ACTION, 1, 1.0f,
     "Verified three-ring process action round 1 at D"},
    {9U, COMMAND_FORWARD_FAST, DISTANCE_F_MM,
     STEP_09_MOTION_SCALE, "D -> lower-left E, forward f"},
    {10U, COMMAND_TURN_CLOCKWISE, 90,
     STEP_10_MOTION_SCALE, "At E turn clockwise 90"},
    {11U, COMMAND_FORWARD_FAST, DISTANCE_G_MM,
     STEP_11_MOTION_SCALE, "E -> temporary storage F, forward g"},
    {0U, COMMAND_STORAGE_ACTION, 1, 1.0f,
     "Storage action round 1 at F"},
    {12U, COMMAND_FORWARD_FAST, DISTANCE_D_MM,
     STEP_12_MOTION_SCALE, "F -> upper-left G, forward d"},
    {13U, COMMAND_TURN_CLOCKWISE, 90,
     STEP_13_MOTION_SCALE, "At G turn clockwise 90"},
    {14U, COMMAND_FORWARD_FAST, DISTANCE_F_MM,
     STEP_14_MOTION_SCALE, "G -> raw area C, forward f"},
    {0U, COMMAND_RAW_ACTION, 2, 1.0f, "Raw action round 2 at C"},
    {15U, COMMAND_RIGHT_FAST, DISTANCE_E_MM,
     STEP_15_MOTION_SCALE, "Repeat step 7: C -> D, right e"},
    {16U, COMMAND_TURN_CLOCKWISE, 180,
     STEP_16_MOTION_SCALE, "Repeat step 8: at D clockwise 180"},
    {0U, COMMAND_PROCESS_ACTION, 2, 1.0f,
     "Verified three-ring process action round 2 at D"},
    {17U, COMMAND_FORWARD_FAST, DISTANCE_F_MM,
     STEP_17_MOTION_SCALE, "Repeat step 9: D -> E, forward f"},
    {18U, COMMAND_TURN_CLOCKWISE, 90,
     STEP_18_MOTION_SCALE, "Repeat step 10: at E clockwise 90"},
    {19U, COMMAND_FORWARD_FAST, DISTANCE_G_MM,
     STEP_19_MOTION_SCALE, "Repeat step 11: E -> F, forward g"},
    {0U, COMMAND_STORAGE_ACTION, 2, 1.0f,
     "Storage action round 2 at F"},
    {20U, COMMAND_RIGHT_FAST, STORAGE_F_TO_SCAN_A_MM,
     STEP_20_MOTION_SCALE, "F -> QR midpoint A, right f+c"},
    {21U, COMMAND_ZONE_LONGITUDINAL_FAST, POINT_A_TO_START_ZONE_MM,
     STEP_21_MOTION_SCALE, "A -> selected start zone"},
    {0U, COMMAND_FINISH, 0, 1.0f, "Finished"}};

constexpr size_t ROUTE_COMMAND_COUNT =
    sizeof(route) / sizeof(route[0]);
static_assert(
    ROUTE_COMMAND_COUNT == 28U,
    "Route must contain 21 moves, six work actions and FINISH");

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

enum QrScanPhase {
  QR_SCAN_IDLE,
  QR_SCAN_FORWARD,
  QR_SCAN_LOCK_AFTER_MOTION,
  QR_SCAN_WAIT_AT_LIMIT,
  QR_SCAN_LOCK_AFTER_CODE,
  QR_SCAN_COMPLETE
};

enum RouteMotionPhase {
  ROUTE_MOTION_IDLE,
  ROUTE_MOTION_COARSE,
  ROUTE_MOTION_HEADING_LOCK
};

ProgramState programState = PROGRAM_WAITING;

enum IntegratedWorkPause {
  INTEGRATED_WORK_NONE,
  INTEGRATED_WORK_RAW_1,
  INTEGRATED_WORK_PROCESS_1,
  INTEGRATED_WORK_STORAGE_1,
  INTEGRATED_WORK_RAW_2,
  INTEGRATED_WORK_PROCESS_2,
  INTEGRATED_WORK_STORAGE_2
};

IntegratedWorkPause integratedWorkPause = INTEGRATED_WORK_NONE;
uint32_t integratedWorkPauseStartMs = 0UL;
StartZone selectedStartZone = START_ZONE_1;
QrScanPhase qrScanPhase = QR_SCAN_IDLE;
RouteMotionPhase routeMotionPhase = ROUTE_MOTION_IDLE;
MotorPulses qrScanOriginMotorPositions;
uint32_t qrScanActionStartMs = 0UL;
float scanDistanceBmm = 0.0f;
float scanCommandedMaximumDistanceMm = 0.0f;
bool scanOriginValid = false;
size_t routeIndex = 0;
RouteCommand activeRouteCommand(
    0U, COMMAND_FINISH, 0, 1.0f,
    "Unresolved route command");
uint8_t activeCompetitionRound = 0;
bool commandStarted = false;
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
bool activeRouteTurnCommand = false;
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

void resetQrScanActionState() {
  qrScanPhase = QR_SCAN_IDLE;
  qrScanOriginMotorPositions = MotorPulses();
  qrScanActionStartMs = 0UL;
  scanDistanceBmm = 0.0f;
  scanCommandedMaximumDistanceMm = 0.0f;
  scanOriginValid = false;
}

void markMissionProgress() {
  lastMissionProgressMs = millis();
}

float selectedStartZoneDirection() {
  return selectedStartZone == START_ZONE_1 ? 1.0f : -1.0f;
}

bool commandIsTurn(CommandType type) {
  return type == COMMAND_TURN_COUNTERCLOCKWISE ||
         type == COMMAND_TURN_CLOCKWISE;
}

float currentRouteCounterClockwiseHeading() {
  return route_chassis::imuCounterClockwiseDegrees -
         routeImuReferenceDegrees;
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

void printCurrentCommand(const RouteCommand &command) {
  SerialDebug.print("Route ");
  SerialDebug.print(static_cast<unsigned int>(routeIndex + 1));
  SerialDebug.print("/");
  SerialDebug.print(static_cast<unsigned int>(ROUTE_COMMAND_COUNT));
  SerialDebug.print(", step=");
  SerialDebug.print(command.specificationStep);
  SerialDebug.print(", scale=");
  SerialDebug.print(command.motionScale, 3);
  SerialDebug.print(": ");
  SerialDebug.println(command.name);
}

void emergencyStopArmLinearAxes();
void stopMaixRequest();
void invalidateArmLinearReference();
void finishProgram();

void routeFault(const char *reason) {
  resetQrScanActionState();
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
  routeMotionPhase = ROUTE_MOTION_IDLE;
  activeRouteTurnCommand = false;
  commandStarted = false;
  hmiSetRunStatus("FAULT");

  SerialDebug.print("FAULT: ");
  SerialDebug.println(reason);
  SerialDebug.println(
      "SAFETY: M6/M7 have no absolute limit reference. Power off, "
      "manually return M6 to the fully retracted stop and M7 to the "
      "physical top before restarting.");
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
  ARM_AXIS_RECOVERY_COMMAND_REJECTED = 3U
};

enum LinearAxisTerminalVerificationReason : uint8_t {
  ARM_AXIS_VERIFY_NONE = 0U,
  ARM_AXIS_VERIFY_HEALTHY_WITHOUT_COMMAND_EVIDENCE = 1U,
  ARM_AXIS_VERIFY_LOCKED_ON_POSITION = 2U,
  ARM_AXIS_VERIFY_REJECTED_ON_POSITION = 3U,
  ARM_AXIS_VERIFY_EXPECTED_COMPLETION = 4U,
  ARM_AXIS_VERIFY_M7_FAST_ARRIVAL = 5U
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
  float driverWorkingZeroAngleDegrees;
  bool driverWorkingZeroAngleValid;
  uint32_t startMs;
  uint32_t lastStatusRequestMs;
  uint8_t lastStatusFlags;
  uint32_t lastStatusResponseMs;
  uint32_t lastFastArrivalQueryMs;
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
        driverWorkingZeroAngleDegrees(0.0f),
        driverWorkingZeroAngleValid(false),
        startMs(0UL),
        lastStatusRequestMs(0UL),
        lastStatusFlags(0U),
        lastStatusResponseMs(0UL),
        lastFastArrivalQueryMs(0UL),
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
bool armLinearReferenceValid = false;
bool armLinearSerialInitialized = false;
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

void writeArmLinearEnable(uint8_t address, bool enable) {
  const uint8_t frame[6] = {
      address, 0xF3U, 0xABU,
      static_cast<uint8_t>(enable ? 1U : 0U),
      0x00U, 0x6BU};
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
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearStatusRequest(uint8_t address) {
  const uint8_t frame[3] = {address, 0x3AU, 0x6BU};
  SerialArmLinear.write(frame, sizeof(frame));
}

void writeArmLinearCurrentPositionRequest(uint8_t address) {
  const uint8_t frame[3] = {address, 0x36U, 0x6BU};
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
    bool recoveryRetry = false);

const char *linearAxisRecoveryReasonName(
    LinearAxisRecoveryReason reason) {
  switch (reason) {
    case ARM_AXIS_RECOVERY_STALL:
      return "stall/protection";
    case ARM_AXIS_RECOVERY_TIMEOUT:
      return "motion timeout";
    case ARM_AXIS_RECOVERY_COMMAND_REJECTED:
      return "position command rejected";
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
          true) &&
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

  // Fast M7 polling runs every 20 ms while approaching the target. Avoid
  // flooding the debug UART and perturbing the very timing being measured;
  // only near-target samples and the accepted arrival are printed later.
  if (reason == ARM_AXIS_VERIFY_M7_FAST_ARRIVAL) {
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
        ARM_AXIS_VERIFY_M7_FAST_ARRIVAL) {
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
             : ARM_AXIS_RECOVERY_SAMPLE_SETTLE_MS);
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
  const bool m7FastArrival =
      verificationReason ==
      ARM_AXIS_VERIFY_M7_FAST_ARRIVAL;
  const bool stable =
      stabilityMm <=
      (m7FastArrival
           ? M7_FAST_ARRIVAL_STABILITY_MM
           : ARM_AXIS_RECOVERY_POSITION_STABILITY_MM);
  const bool targetVerified =
      stable &&
      targetErrorMm <=
          (m7FastArrival
               ? M7_FAST_ARRIVAL_TARGET_TOLERANCE_MM
               : ARM_AXIS_TERMINAL_VERIFY_TOLERANCE_MM);
  const bool protectionLatched =
      (axis->terminalStatusFlags & 0x08U) != 0U;
  const bool fastStatusFresh =
      axis->lastStatusResponseMs != 0UL &&
      millis() - axis->lastStatusResponseMs <=
          M7_FAST_ARRIVAL_STATUS_FRESH_MS;
  const bool fastStatusHealthy =
      fastStatusFresh &&
      (axis->terminalStatusFlags & 0x01U) != 0U &&
      !protectionLatched &&
      !axis->positionCommandRejected &&
      (axis->commandAcknowledged || axis->motionObserved);

  if (!m7FastArrival || targetErrorMm <= 2.0f) {
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
  if (m7FastArrival) {
    if (protectionLatched) {
      scheduleLinearAxisRecovery(
          *axis,
          ARM_AXIS_RECOVERY_STALL);
      return;
    }
    if (targetVerified && fastStatusHealthy) {
      SerialDebug.print(
          "[M7 FAST ARRIVAL] encoder stable; bypass delayed driver "
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
          ARM_AXIS_VERIFY_M7_FAST_ARRIVAL) {
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
           : ARM_LINEAR_POSITION_READ_TIMEOUT_MS);
  writeArmLinearCurrentPositionRequest(axis.address);
  SerialArmLinear.flush();
}

bool consumeArmLinearEnableResponse(
    LinearAxisMotion &axis) {
  uint8_t window[4] = {0U, 0U, 0U, 0U};
  uint8_t count = 0U;
  bool enableResponseSeen = false;
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
  if (M7_ENCODER_FAST_ARRIVAL_ENABLED &&
      axis->address == ARM_LIFT_ADDRESS &&
      frame[1] == 0xFDU &&
      frame[2] == 0x9FU) {
    SerialDebug.print(
        "[M7 DRIVER TERMINAL] t/active=");
    SerialDebug.print(millis());
    SerialDebug.print("/");
    SerialDebug.print(axis->active ? 1 : 0);
    SerialDebug.println(
        "; ignored to prevent a delayed old frame completing a new M7 move");
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
    printArmLinearFrame("wrong command", frame);
    faultLinearAxis(*axis, "EMM wrong command");
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
    const bool m7FastArrivalStatusHealthy =
        M7_ENCODER_FAST_ARRIVAL_ENABLED &&
        axis.address == ARM_LIFT_ADDRESS &&
        armLinearReferenceValid &&
        axis.driverWorkingZeroAngleValid &&
        axis.lastStatusResponseMs != 0UL &&
        nowMs - axis.lastStatusResponseMs <=
            M7_FAST_ARRIVAL_STATUS_FRESH_MS &&
        (axis.lastStatusFlags & 0x01U) != 0U &&
        (axis.lastStatusFlags & 0x08U) == 0U &&
        !axis.positionCommandRejected &&
        (axis.commandAcknowledged || axis.motionObserved);
    if (!axis.terminalVerificationPending &&
        armLinearPositionQueryAxis == nullptr &&
        m7FastArrivalStatusHealthy &&
        nowMs - axis.startMs >=
            M7_AXIS_MINIMUM_ON_POSITION_MS &&
        (axis.lastFastArrivalQueryMs == 0UL ||
         nowMs - axis.lastFastArrivalQueryMs >=
             M7_FAST_ARRIVAL_QUERY_INTERVAL_MS)) {
      axis.lastFastArrivalQueryMs = nowMs;
      requestLinearAxisTerminalVerification(
          axis,
          axis.lastStatusFlags,
          ARM_AXIS_VERIFY_M7_FAST_ARRIVAL);
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
            finalAcceleration) &&
        programState != PROGRAM_FAULT) {
      routeFault("M6 contact-soft final segment rejected");
    }
  }

  if (m7SoftLandingPending &&
      !liftAxis.active &&
      !liftAxis.recoveryPending) {
    if (liftAxis.fault || programState == PROGRAM_FAULT) {
      m7SoftLandingPending = false;
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
    m7SoftLandingPending = false;
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
            finalAcceleration) &&
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
    bool recoveryRetry) {
  if (axis.fault) {
    routeFault("EMM axis remains faulted");
    return false;
  }
  const LinearAxisMotion &otherAxis =
      axis.address == extensionAxis.address
          ? liftAxis
          : extensionAxis;
  if (axis.active ||
      axis.recoveryPending ||
      otherAxis.active ||
      otherAxis.recoveryPending ||
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

  // Apply the experiment at the one shared M7 command boundary so normal,
  // RAW, endpoint, zeroing, recovery, pickup, placement, and return moves all
  // use the same policy. M6 is deliberately unchanged.
  if (axis.address == liftAxis.address) {
    speedRpm = m7_experiment::doubledSpeedRpm(speedRpm);
    acceleration =
        m7_experiment::experimentalAcceleration(acceleration);
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
        finalAcceleration);
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
    uint8_t acceleration) {

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
        acceleration);
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
            acceleration)) {
      return true;
    }
    m7SoftLandingPending = false;
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
      finalAcceleration);
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
        finalAcceleration);
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
          ARM_LINEAR_STARTUP_ZERO_ACCELERATION) ||
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
            ARM_LINEAR_STARTUP_ZERO_ACCELERATION) ||
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

void commandStorageServoPosition(uint8_t positionIndex) {
  if (positionIndex > 3U) {
    routeFault("Invalid storage servo position");
    return;
  }

  storageServo.setRawAngleMTurn(
      STORAGE_SERVO_POSITIONS_DEGREES[positionIndex],
      STORAGE_SERVO_INTERVAL_MS);
  SerialDebug.print("Storage servo multi-turn index ");
  SerialDebug.print(positionIndex);
  SerialDebug.print(", angle=");
  SerialDebug.println(
      STORAGE_SERVO_POSITIONS_DEGREES[positionIndex],
      1);
}

void commandStorageServoParkingPosition() {
  storageServo.setRawAngleMTurn(
      STORAGE_SERVO_PARK_ANGLE_DEGREES,
      STORAGE_SERVO_INTERVAL_MS);
  SerialDebug.print("Storage servo parking angle=");
  SerialDebug.println(
      STORAGE_SERVO_PARK_ANGLE_DEGREES,
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

void advanceRoute() {
  const uint8_t completedStep =
      activeRouteCommand.specificationStep;
  stopAllMotorsImmediately();
  routeMotionPhase = ROUTE_MOTION_IDLE;
  activeRouteTurnCommand = false;

  if (completedStep != 0U) {
    SerialDebug.print("[ROUTE DONE] step=");
    SerialDebug.print(completedStep);
    SerialDebug.print("/21, heading target/actual/error=");
    SerialDebug.print(targetCounterClockwiseHeadingDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.print(currentRouteCounterClockwiseHeading(), 2);
    SerialDebug.print("/");
    SerialDebug.println(headingErrorDegrees(), 2);
  }

  ++routeIndex;
  commandStarted = false;
  commandStartMs = millis();
  routeHeadingLockStartMs = 0UL;
  headingStableStartMs = 0;
  motorsArrivedStartMs = 0;
  markMissionProgress();
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

  if (ROUGH_PROCESSING_CALIBRATION_MODE) {

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

void startRouteVehicleDisplacement(
    float userForwardMm,
    float userRightMm,
    float counterClockwiseDegrees) {

  startBodyDisplacement(
      -userForwardMm / 1000.0f,
      userRightMm / 1000.0f,
      counterClockwiseDegrees * PI_F / 180.0f);
}

void beginRouteCoarseMotion(bool turnCommand) {
  routeMotionPhase = ROUTE_MOTION_COARSE;
  activeRouteTurnCommand = turnCommand;
  turnMotionEnabled = turnCommand;
  preciseMotionEnabled = false;
  workstationApproachEnabled = false;
  routeHeadingLockStartMs = 0UL;
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
  commandStartMs = millis();
}

void startRouteFastLongitudinalTranslation(
    float nominalForwardMm,
    float motionScale) {
  const float commandedForwardMm =
      nominalForwardMm * motionScale;
  const float commandedMaximumStepRate =
      ROUTE_FAST_MAXIMUM_STEP_RATE *
      motionScale *
      ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;
  const float commandedAcceleration =
      ROUTE_FAST_STEP_ACCELERATION *
      motionScale *
      ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;

  SerialDebug.print(
      "[SEGMENT LONG] nominal/commanded/scale mm=");
  SerialDebug.print(nominalForwardMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(commandedForwardMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(motionScale, 3);
  SerialDebug.print(", vmax/acc=");
  SerialDebug.print(commandedMaximumStepRate, 1);
  SerialDebug.print("/");
  SerialDebug.println(commandedAcceleration, 1);

  setRouteDriveMotionProfile(
      commandedMaximumStepRate,
      commandedAcceleration);
  startRouteVehicleDisplacement(
      commandedForwardMm, 0.0f, 0.0f);
  beginRouteCoarseMotion(false);
}

void startRouteFastLateralTranslation(
    float nominalRightMm,
    float motionScale,
    float maximumSpeedProfileScale,
    float accelerationProfileScale) {
  const float commandedRightMm =
      nominalRightMm * motionScale;
  const float commandedMaximumStepRate =
      ROUTE_FAST_MAXIMUM_STEP_RATE *
      motionScale *
      maximumSpeedProfileScale;
  const float lateralAccelerationProfileLimit =
      ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
      ROUTE_LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL;
  const float limitedAccelerationProfileScale =
      fminf(
          accelerationProfileScale,
          lateralAccelerationProfileLimit);
  const float commandedAcceleration =
      ROUTE_FAST_STEP_ACCELERATION *
      motionScale *
      limitedAccelerationProfileScale;

  SerialDebug.print(
      "[SEGMENT LATERAL] nominal/commanded/scale mm=");
  SerialDebug.print(nominalRightMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(commandedRightMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(motionScale, 3);
  SerialDebug.print(
      ", profile speed/requested-acc/limited-acc scale=");
  SerialDebug.print(maximumSpeedProfileScale, 3);
  SerialDebug.print("/");
  SerialDebug.print(accelerationProfileScale, 3);
  SerialDebug.print("/");
  SerialDebug.print(limitedAccelerationProfileScale, 3);
  SerialDebug.print(", vmax/acc=");
  SerialDebug.print(commandedMaximumStepRate, 1);
  SerialDebug.print("/");
  SerialDebug.println(commandedAcceleration, 1);

  setRouteDriveMotionProfile(
      commandedMaximumStepRate,
      commandedAcceleration);
  startRouteVehicleDisplacement(
      0.0f, commandedRightMm, 0.0f);
  beginRouteCoarseMotion(false);
}

void startRouteTurn(
    float nominalCounterClockwiseDegrees,
    float motionScale) {
  const float commandedCounterClockwiseDegrees =
      nominalCounterClockwiseDegrees * motionScale;
  activeTurnStartHeadingDegrees =
      currentRouteCounterClockwiseHeading();
  activeTurnCommandDegrees =
      commandedCounterClockwiseDegrees;
  activeTurnCorrectionCount = 0U;
  turnCoarseTelemetryPending = true;
  targetCounterClockwiseHeadingDegrees +=
      commandedCounterClockwiseDegrees;

  SerialDebug.print(
      "[SEGMENT TURN] nominal/commanded/scale/target deg=");
  SerialDebug.print(nominalCounterClockwiseDegrees, 1);
  SerialDebug.print("/");
  SerialDebug.print(commandedCounterClockwiseDegrees, 1);
  SerialDebug.print("/");
  SerialDebug.print(motionScale, 3);
  SerialDebug.print("/");
  SerialDebug.println(
      targetCounterClockwiseHeadingDegrees, 2);

  integratedTurnControlActive = true;
  integratedTurnBrakeCommandIssued = false;
  integratedTurnDirectionSign =
      commandedCounterClockwiseDegrees >= 0.0f ? +1 : -1;
  activeTurnPulsesPerDegree =
      rotationPulsesPerDegree(
          integratedTurnDirectionSign);
  if (activeTurnPulsesPerDegree <= 0.0f) {
    routeFault("Invalid turn pulse/degree conversion");
    return;
  }

  setRouteDriveMotionProfile(
      ROUTE_TURN_MAXIMUM_STEP_RATE * motionScale,
      ROUTE_TURN_STEP_ACCELERATION * motionScale);
  startRouteVehicleDisplacement(
      0.0f, 0.0f,
      commandedCounterClockwiseDegrees);
  beginRouteCoarseMotion(true);
  hmiSetRunStatus("TURNIMU");
}

void beginRouteHeadingLock() {
  routeMotionPhase = ROUTE_MOTION_HEADING_LOCK;
  routeHeadingLockStartMs = millis();
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
  hmiSetRunStatus("IMULOCK");
}

void startRouteHeadingCorrection(float errorDegrees) {
  float correction = errorDegrees;
  if (correction >
      ROUTE_MAXIMUM_HEADING_CORRECTION_DEGREES) {
    correction =
        ROUTE_MAXIMUM_HEADING_CORRECTION_DEGREES;
  } else if (
      correction <
      -ROUTE_MAXIMUM_HEADING_CORRECTION_DEGREES) {
    correction =
        -ROUTE_MAXIMUM_HEADING_CORRECTION_DEGREES;
  }

  if (correction > 0.0f &&
      correction <
          ROUTE_MINIMUM_HEADING_CORRECTION_DEGREES) {
    correction =
        ROUTE_MINIMUM_HEADING_CORRECTION_DEGREES;
  } else if (
      correction < 0.0f &&
      correction >
          -ROUTE_MINIMUM_HEADING_CORRECTION_DEGREES) {
    correction =
        -ROUTE_MINIMUM_HEADING_CORRECTION_DEGREES;
  }

  if (activeRouteTurnCommand) {
    ++activeTurnCorrectionCount;
  }
  SerialDebug.print(
      "[ROUTE IMU] target/actual/error/correction=");
  SerialDebug.print(
      targetCounterClockwiseHeadingDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(
      currentRouteCounterClockwiseHeading(), 2);
  SerialDebug.print("/");
  SerialDebug.print(errorDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.println(correction, 2);

  setRouteDriveMotionProfile(
      ROUTE_HEADING_CORRECTION_MAXIMUM_STEP_RATE,
      ROUTE_HEADING_CORRECTION_STEP_ACCELERATION);
  startRouteVehicleDisplacement(
      0.0f, 0.0f, correction);
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
}

bool serviceRouteHeadingLock() {
  if (!imuIsFresh()) {
    routeFault("IMU data timeout during route heading lock");
    return false;
  }
  if (millis() - routeHeadingLockStartMs >
      ROUTE_HEADING_LOCK_TIMEOUT_MS) {
    routeFault("Route heading lock timeout");
    return false;
  }
  if (!allMotorsArrived()) {
    headingStableStartMs = 0UL;
    motorsArrivedStartMs = 0UL;
    return false;
  }

  const uint32_t nowMs = millis();
  if (motorsArrivedStartMs == 0UL) {
    motorsArrivedStartMs = nowMs;
    return false;
  }

  const float tolerance =
      activeRouteTurnCommand
          ? ROUTE_TURN_HEADING_TOLERANCE_DEGREES
          : ROUTE_TRANSLATION_HEADING_TOLERANCE_DEGREES;
  const float error = headingErrorDegrees();
  if (fabsf(error) <= tolerance) {
    if (headingStableStartMs == 0UL) {
      headingStableStartMs = nowMs;
    }

    return true;
  }

  headingStableStartMs = 0UL;
  startRouteHeadingCorrection(error);
  return false;
}

bool nextRouteCommandIsTurn() {
  return routeIndex + 1U < ROUTE_COMMAND_COUNT &&
         commandIsTurn(route[routeIndex + 1U].type);
}

void beginIntegratedTurnBraking() {

  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->setAcceleration(activeDriveDeceleration);
  }
  driveDecelerationActive = true;

  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->stop();
  }
  integratedTurnBrakeCommandIssued = true;
}

void serviceIntegratedTurnCommand() {
  if (routeMotionPhase != ROUTE_MOTION_COARSE ||
      !integratedTurnControlActive) {
    routeFault("Integrated turn state missing");
    return;
  }

  if (!imuIsFresh()) {
    routeFault("IMU data timeout during integrated turn");
    return;
  }

  if (ENABLE_MOTION_TIMEOUTS &&
      millis() - commandStartMs >
          ROUTE_MOTION_TIMEOUT_MS) {
    routeFault("Integrated turn timeout");
    return;
  }

  const float errorDegrees = headingErrorDegrees();
  if (!integratedTurnBrakeCommandIssued &&
      !allMotorsArrived()) {
    float maximumAbsoluteStepRate = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i) {
      const float stepRate = fabsf(motors[i]->speed());
      if (stepRate > maximumAbsoluteStepRate) {
        maximumAbsoluteStepRate = stepRate;
      }
    }

    const float predictedAngularRateDegreesPerSecond =
        maximumAbsoluteStepRate /
        activeTurnPulsesPerDegree;
    const float predictedBrakingDegrees =
        maximumAbsoluteStepRate *
            maximumAbsoluteStepRate /
        (2.0f * activeDriveDeceleration *
         activeTurnPulsesPerDegree);
    const float predictedLatencyDegrees =
        predictedAngularRateDegreesPerSecond *
        TURN_IMU_CONTROL_LATENCY_SECONDS;
    const float remainingDegrees = fabsf(errorDegrees);
    const bool targetReachedOrPassed =
        errorDegrees *
            static_cast<float>(
                integratedTurnDirectionSign) <=
        0.0f;

    if (targetReachedOrPassed ||
        remainingDegrees <=
            predictedBrakingDegrees +
                predictedLatencyDegrees +
                TURN_PREDICTIVE_BRAKE_MARGIN_DEGREES) {
      beginIntegratedTurnBraking();
    }
  }

  if (!allMotorsArrived()) {
    return;
  }

  if (turnCoarseTelemetryPending) {
    printTurnCoarseTelemetry();
    turnCoarseTelemetryPending = false;
  }

  const float stoppedErrorDegrees = headingErrorDegrees();
  if (fabsf(stoppedErrorDegrees) <=
      ROUTE_TURN_HEADING_TOLERANCE_DEGREES) {
    printTurnLockTelemetry();
    integratedTurnControlActive = false;
    integratedTurnBrakeCommandIssued = false;
    integratedTurnDirectionSign = 0;
    hmiSetRunStatus("RUN");
    advanceRoute();
    return;
  }

  integratedTurnBrakeCommandIssued = false;
  integratedTurnDirectionSign =
      stoppedErrorDegrees >= 0.0f ? +1 : -1;
  activeTurnPulsesPerDegree =
      rotationPulsesPerDegree(
          integratedTurnDirectionSign);
  if (activeTurnPulsesPerDegree <= 0.0f) {
    routeFault("Invalid correction pulse/degree conversion");
    return;
  }
  startRouteHeadingCorrection(stoppedErrorDegrees);
  hmiSetRunStatus("TURNIMU");
}

void serviceRoutePhysicalCommand() {
  if (routeMotionPhase == ROUTE_MOTION_COARSE) {
    if (!allMotorsArrived()) {
      if (ENABLE_MOTION_TIMEOUTS &&
          millis() - commandStartMs >
              ROUTE_MOTION_TIMEOUT_MS) {
        routeFault("Route coarse motion timeout");
      }
      return;
    }

    if (activeRouteTurnCommand &&
        turnCoarseTelemetryPending) {
      printTurnCoarseTelemetry();
      turnCoarseTelemetryPending = false;
    }

    if (nextRouteCommandIsTurn()) {
      SerialDebug.println(
          "[ROUTE IMU] skipped before explicit turn");
      advanceRoute();
      return;
    }

    beginRouteHeadingLock();
    return;
  }

  if (routeMotionPhase ==
          ROUTE_MOTION_HEADING_LOCK &&
      serviceRouteHeadingLock()) {
    if (activeRouteTurnCommand) {
      printTurnLockTelemetry();
    }
    hmiSetRunStatus("RUN");
    advanceRoute();
  }
}

MotorPulses currentDriveMotorPositions() {
  return MotorPulses(
      motor1.currentPosition(),
      motor2.currentPosition(),
      motor3.currentPosition(),
      motor4.currentPosition());
}

float forwardTravelFromOriginMm(
    const MotorPulses &origin) {
  const MotorPulses now = currentDriveMotorPositions();
  const float projectedPulses =
      (static_cast<float>(now.motor1 - origin.motor1) -
       static_cast<float>(now.motor2 - origin.motor2) +
       static_cast<float>(now.motor3 - origin.motor3) -
       static_cast<float>(now.motor4 - origin.motor4)) /
      4.0f;
  return projectedPulses /
         FORWARD_PULSES_PER_METER * 1000.0f;
}

void captureScanDistanceB() {
  if (!scanOriginValid) {
    routeFault("QR scan origin missing");
    return;
  }

  scanDistanceBmm =
      selectedStartZoneDirection() *
      forwardTravelFromOriginMm(
          qrScanOriginMotorPositions);
  if (scanDistanceBmm < 0.0f) {
    scanDistanceBmm = 0.0f;
  } else if (
      scanDistanceBmm >
      scanCommandedMaximumDistanceMm) {
    scanDistanceBmm =
        scanCommandedMaximumDistanceMm;
  }

  SerialDebug.print("[SCAN] captured b=");
  SerialDebug.print(scanDistanceBmm, 1);
  SerialDebug.print(" mm, QR=");
  SerialDebug.println(scanFlag ? 1 : 0);
}

void startQrScanAction() {
  qrScanActionStartMs = millis();
  qrScanOriginMotorPositions =
      currentDriveMotorPositions();
  scanOriginValid = true;
  scanDistanceBmm = 0.0f;
  scanCommandedMaximumDistanceMm = fminf(
      static_cast<float>(
          MAXIMUM_SCAN_DISTANCE_B_MM) *
          activeRouteCommand.motionScale,
      static_cast<float>(
          MAXIMUM_SCAN_DISTANCE_B_MM));
  hmiSetRunStatus("SCAN");

  if (scanFlag) {
    SerialDebug.println(
        "[SCAN] valid code already present, b=0 mm");
    activeRouteTurnCommand = false;
    turnMotionEnabled = false;
    beginRouteHeadingLock();
    qrScanPhase = QR_SCAN_LOCK_AFTER_CODE;
    return;
  }

  const float commandedMaximumStepRate =
      ROUTE_SCAN_MAXIMUM_STEP_RATE *
      activeRouteCommand.motionScale *
      ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;
  const float commandedAcceleration =
      ROUTE_SCAN_STEP_ACCELERATION *
      activeRouteCommand.motionScale *
      ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;

  setRouteDriveMotionProfile(
      commandedMaximumStepRate,
      commandedAcceleration);
  startRouteVehicleDisplacement(
      selectedStartZoneDirection() *
          scanCommandedMaximumDistanceMm,
      0.0f, 0.0f);
  beginRouteCoarseMotion(false);
  qrScanPhase = QR_SCAN_FORWARD;
  SerialDebug.print(
      "[SCAN] maximum/scale/vmax/acc=");
  SerialDebug.print(
      scanCommandedMaximumDistanceMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(
      activeRouteCommand.motionScale, 3);
  SerialDebug.print("/");
  SerialDebug.print(
      commandedMaximumStepRate, 1);
  SerialDebug.print("/");
  SerialDebug.println(commandedAcceleration, 1);
}

bool updateQrScanAction() {
  switch (qrScanPhase) {
    case QR_SCAN_IDLE:
      return false;

    case QR_SCAN_FORWARD:

      if (scanFlag) {
        stopAllMotorsImmediately();
        captureScanDistanceB();
        if (programState != PROGRAM_RUNNING) {
          return false;
        }
        beginRouteHeadingLock();
        qrScanPhase = QR_SCAN_LOCK_AFTER_CODE;
        return false;
      }
      if (!allMotorsArrived()) {
        if (ENABLE_MOTION_TIMEOUTS &&
            millis() - commandStartMs >
                ROUTE_MOTION_TIMEOUT_MS) {
          routeFault("QR slow scan motion timeout");
        }
        return false;
      }

      captureScanDistanceB();
      if (programState != PROGRAM_RUNNING) {
        return false;
      }
      beginRouteHeadingLock();
      qrScanPhase = QR_SCAN_LOCK_AFTER_MOTION;
      return false;

    case QR_SCAN_LOCK_AFTER_MOTION:
      if (serviceRouteHeadingLock()) {
        routeMotionPhase = ROUTE_MOTION_IDLE;
        qrScanPhase = QR_SCAN_WAIT_AT_LIMIT;
        hmiSetRunStatus("SCANMAX");
        SerialDebug.print(
            "[SCAN] ");
        SerialDebug.print(
            scanCommandedMaximumDistanceMm, 1);
        SerialDebug.println(
            " mm limit reached; waiting in place");
      }
      return false;

    case QR_SCAN_WAIT_AT_LIMIT:
      if (scanFlag) {
        beginRouteHeadingLock();
        qrScanPhase = QR_SCAN_LOCK_AFTER_CODE;
      }
      return false;

    case QR_SCAN_LOCK_AFTER_CODE:
      if (serviceRouteHeadingLock()) {
        routeMotionPhase = ROUTE_MOTION_IDLE;
        qrScanPhase = QR_SCAN_COMPLETE;
        SerialDebug.println(
            "[SCAN] code accepted; step 2 complete");
        return true;
      }
      return false;

    case QR_SCAN_COMPLETE:
      return true;
  }

  routeFault("Invalid QR sweep phase");
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
  ARM_STANDARD_WAIT_LIFT,
  ARM_STANDARD_WAIT_RETRACT,
  ARM_STANDARD_WAIT_BASE,
  ARM_STANDARD_WAIT_ENDPOINT_PARALLEL,
  ARM_STANDARD_WAIT_BASE_SETTLE,
  ARM_STANDARD_COMPLETE
};

ArmStandardPhase armStandardPhase = ARM_STANDARD_IDLE;
uint32_t armStandardDeadlineMs = 0UL;
float armStandardBaseTargetDegrees = 0.0f;
float armStandardExtensionTargetMm =
    M6_STANDARD_EXTENSION_MM;
bool armStandardEndpointPreparation = false;
bool armStandardKeepBaseAngle = false;
bool armGripperLiftIsolationEnabled = false;

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
  armStandardBaseTargetDegrees = baseTargetDegrees;
  armStandardExtensionTargetMm = extensionTargetMm;
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
    commandGripperOpen();
    armStandardDeadlineMs =
        millis() + GRIPPER_OPEN_SETTLE_MS;
    armStandardPhase = ARM_STANDARD_WAIT_OPEN;
  }
}

void beginArmEndpointPreparation() {
  useArmBaseEndpointTravelMotionProfile();
  armStandardEndpointPreparation = true;
  armStandardKeepBaseAngle = false;
  armStandardExtensionTargetMm =
      M6_STANDARD_EXTENSION_MM;
  armStandardBaseTargetDegrees =
      ENDPOINT_RING1_SEARCH_SEED_ANGLE_DEGREES;
  SerialDebug.print("[ARM ENDPOINT PREP] t=");
  SerialDebug.print(millis());
  SerialDebug.print(
      " ms, open gripper; parallel targets M7/M5=");
  SerialDebug.print(HOUGH_VISION_HEIGHT_MM, 2);
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
    commandGripperOpen();
    armStandardDeadlineMs =
        millis() + GRIPPER_OPEN_SETTLE_MS;
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
      if (armStandardDeadlineMs == 0UL) {
        armStandardDeadlineMs =
            millis() + WORK_M7_TO_GRIPPER_GAP_MS;
        break;
      }
      if (deadlineReached(armStandardDeadlineMs)) {
        commandGripperOpen();
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
      if (deadlineReached(armStandardDeadlineMs)) {
        if (armStandardEndpointPreparation) {
          if (extensionAxis.active ||
              extensionAxis.recoveryPending ||
              extensionAxis.fault ||
              fabsf(
                  extensionAxis.currentMm -
                  M6_STANDARD_EXTENSION_MM) >
                  ARM_AXIS_POSITION_TOLERANCE_MM) {
            routeFault(
                "Endpoint preparation requires M6 safe working zero");
            break;
          }

          if (!startLiftToHeightMm(
                  HOUGH_VISION_HEIGHT_MM)) {
            break;
          }
          startArmBaseStandardFrameDegrees(
              armStandardBaseTargetDegrees);
          SerialDebug.print(
              "[ARM ENDPOINT PREP] parallel M7/M5 started at t=");
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
            " ms, coarse height and ring-1 angle ready -> settle");
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
uint32_t armTransferM7HandoffStartMs = 0UL;
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
bool armTransferReturnToRawViewAtEnd = false;
bool armTransferSourceRotationStarted = false;
bool armTransferDestinationRotationStarted = false;
bool armTransferFinalRotationStarted = false;
arm_transfer::MotionProfile armTransferMotionProfile =
    arm_transfer::PROFILE_STANDARD;

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
    SerialDebug.println(
        "[DOUBLE GRIPPER] pickup -> 20 ms close command");
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
    bool loadedReturnMotion = false) {
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
    return startLinearAxisMove(
        extensionAxis,
        extensionMm,
        minimumMm,
        M6_MAXIMUM_EXTENSION_MM,
        M6_PULSES_PER_MM,
        M6_EXTEND_DIRECTION,
        M6_RETRACT_DIRECTION,
        loadedReturnMotion
            ? arm_config::M6_LOADED_RETURN_SPEED_RPM
            : RETURN_M6_SPEED_RPM,
        loadedReturnMotion
            ? arm_config::M6_LOADED_RETURN_ACCELERATION
            : RETURN_M6_ACCELERATION);
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
    bool loadedReturnMotion = false) {
  if (!armTransferPlanarMoveIsSafe(
          mappedDepartureSide)) {
    return false;
  }
  if (mappedPose && !ringMapHeadingStillValid()) {
    return false;
  }

  if (armTransferUsesReturnProfile()) {
    if (loadedReturnMotion) {
      useArmBaseLoadedReturnMotionProfile();
    } else {
      useArmBaseReturnMotionProfile();
    }
  }
  startArmBaseStandardFrameDegrees(
      pose.standardFrameAngleDegrees);
  rotationStarted = true;
  const bool extensionStarted =
      sourceExtensionPolicy
          ? startArmTransferSourceExtensionToMm(
                pose.extensionMm,
                mappedPose)
          : startArmTransferDestinationExtensionToMm(
                pose.extensionMm,
                mappedPose,
                loadedReturnMotion);
  if (!extensionStarted) {
    if (programState != PROGRAM_FAULT) {
      stopArmBaseImmediately();
      routeFault("M5/M6 planar parallel start rejected");
    }
    return false;
  }

  SerialDebug.print("[TRANSFER PLANAR] ");
  SerialDebug.print(label);
  SerialDebug.println(
      ": M5 rotation + M6 extension/retraction started together");
  if (loadedReturnMotion) {
    SerialDebug.print(
        "[RING RETURN LOADED] M5/M6 profile matched to tray-to-ring, "
        "speed=");
    SerialDebug.print(
        arm_config::M5_LOADED_RETURN_MAXIMUM_STEP_RATE,
        0);
    SerialDebug.print("/");
    SerialDebug.print(
        arm_config::M6_LOADED_RETURN_SPEED_RPM);
    SerialDebug.print("; acceleration=");
    SerialDebug.print(
        arm_config::M5_LOADED_RETURN_STEP_ACCELERATION,
        0);
    SerialDebug.print("/");
    SerialDebug.println(
        arm_config::M6_LOADED_RETURN_ACCELERATION);
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
    bool mappedRingPose = false) {
  const float minimumExtensionMm =
      mappedRingPose
          ? M6_RING2_MINIMUM_EXTENSION_MM
          : M6_STANDARD_EXTENSION_MM;
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
    bool nextSourceMapped = false) {
  const bool effectiveConcurrentSourcePreparation =
      concurrentSourcePreparation &&
      !armGripperLiftIsolationEnabled;
  if (!armPoseIsValid(source, mapSource) ||
      !armPoseIsValid(destination, mapDestination)) {
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
           nextSourceMapped) ||
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
  armTransferNextSourcePreparedAtEnd = false;
  armTransferReturnToRawViewAtEnd =
      returnToRawViewAtEnd;
  armTransferSourceRotationStarted = false;
  armTransferDestinationRotationStarted = false;
  armTransferFinalRotationStarted = false;

  SerialDebug.print("Transfer source angle/ext/height: ");
  SerialDebug.print(source.standardFrameAngleDegrees, 2);
  SerialDebug.print(", ");
  SerialDebug.print(source.extensionMm, 2);
  SerialDebug.print(", ");
  SerialDebug.println(source.heightMm, 2);
  SerialDebug.print("Transfer destination angle/ext/height: ");
  SerialDebug.print(destination.standardFrameAngleDegrees, 2);
  SerialDebug.print(", ");
  SerialDebug.print(destination.extensionMm, 2);
  SerialDebug.print(", ");
  SerialDebug.println(destination.heightMm, 2);
  SerialDebug.print("Gentle destination release: ");
  SerialDebug.println(
      armTransferGentleDestinationRelease ? 1 : 0);
  SerialDebug.print("Concurrent source preparation: ");
  SerialDebug.println(
      armTransferConcurrentSourcePreparation ? 1 : 0);
  SerialDebug.print("Source already prepared / prepare next: ");
  SerialDebug.print(
      armTransferSourceAlreadyPrepared ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.println(
      armTransferPrepareNextSource ? 1 : 0);
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
    SerialDebug.println(
        "[TRANSFER SAFE] retained M5/M6 source pose and open gripper "
        "at/above M7 clearance; lower directly for next pickup");
    if (!startArmTransferLiftToSourceHeightMm(
            armTransferSourcePose.heightMm)) {
      return;
    }
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
      !armTransferUsesReturnProfile()) {

    commandStorageServoPosition(
        static_cast<uint8_t>(
            armTransferNextStorageSlot));
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
  if (armTransferUsesReturnProfile()) {
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
  armTransferPhase =
      ARM_TRANSFER_WAIT_DESTINATION_OPEN;
}

void serviceArmTransferDeferredStorageCommand() {
  if (!armTransferUsesReturnProfile() ||
      !armTransferPrepareNextSource ||
      armTransferNextStorageCommanded ||
      armTransferStorageCommandDueMs == 0UL ||
      !deadlineReached(armTransferStorageCommandDueMs)) {
    return;
  }

  commandStorageServoPosition(
      static_cast<uint8_t>(armTransferNextStorageSlot));
  armTransferNextStorageDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  armTransferNextStorageCommanded = true;
  SerialDebug.print(
      "[RING RETURN STORAGE] 500 ms post-release delay complete; "
      "turntable commanded, arrival-gate-ms=");
  SerialDebug.println(STORAGE_SERVO_SETTLE_MS);
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
        if (armTransferMapSource &&
            !ringMapHeadingStillValid()) {
          return;
        }
        if (startArmTransferLiftToSourceHeightMm(
                armTransferSourcePose.heightMm)) {
          armTransferPhase =
              ARM_TRANSFER_WAIT_SOURCE_LOWER;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_LOWER:
      if (liftMoveFinished()) {
        if (armGripperLiftIsolationEnabled) {
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
              millis() + armTransferGripperCloseSettleMs();
          armTransferPhase =
              ARM_TRANSFER_WAIT_GRIP_CLOSE;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP:
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
            WORK_GRIPPER_TO_M7_GAP_MS;
        armTransferPhase =
            ARM_TRANSFER_WAIT_GRIP_CLOSE;
        SerialDebug.println(
            "[WORK INTERLOCK] source gap complete -> gripper close; "
            "M7 blocked through close completion + short handoff");
      }
      break;

    case ARM_TRANSFER_WAIT_GRIP_CLOSE:
      if (deadlineReached(armTransferDeadlineMs)) {
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
        if (startArmTransferPlanarMove(
                armTransferDestinationPose,
                armTransferMapDestination,
                armTransferMapSource,
                false,
                armTransferDestinationRotationStarted,
                "loaded clearance -> destination",
                armTransferUsesReturnProfile())) {
          armTransferPhase =
              ARM_TRANSFER_WAIT_DESTINATION_ROTATION;
        }
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION:
      if (!armMotors.isM5Running() &&
          extensionMoveFinished() &&
          liftMoveFinished()) {
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
        if (armTransferPrepareNextSource &&
            !armTransferUsesReturnProfile() &&
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
              "[TRANSFER DIRECT] next pickup pose ready; "
              "M5 never passed through intermediate zero");
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
  armTransferPhase = ARM_TRANSFER_IDLE;
  armTransferM7HandoffStartMs = 0UL;
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
      "[RING MAP] M6 visual raw/-8mm/command=");
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
    ArmPose &pose) {
  const float radialDistanceMm =
      hypotf(point.outwardMm, point.leftMm);
  const float angleDegrees =
      atan2f(point.leftMm, point.outwardMm) *
      180.0f / PI_F;
  const float extensionMm =
      radialDistanceMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;

  if (extensionMm <
          M6_STANDARD_EXTENSION_MM -
              ARM_AXIS_POSITION_TOLERANCE_MM ||
      extensionMm >
          M6_MAXIMUM_EXTENSION_MM +
              ARM_AXIS_POSITION_TOLERANCE_MM ||
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
    routeFault("Endpoint outside safe camera workspace");
    return false;
  }

  pose.standardFrameAngleDegrees = angleDegrees;
  pose.extensionMm =
      extensionMm < M6_STANDARD_EXTENSION_MM
          ? M6_STANDARD_EXTENSION_MM
          : extensionMm;
  pose.heightMm = -lowerMm;
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
          HOUGH_VISION_LOWER_MM,
          pose)) {
    return false;
  }
  const float rawPredictedAngleDegrees =
      pose.standardFrameAngleDegrees;
  if (rawPredictedAngleDegrees >= 0.0f) {
    routeFault("Predicted ring 3 is not on the expected M5 side");
    return false;
  }

  pose.standardFrameAngleDegrees =
      ENDPOINT_RING3_SEARCH_SEED_ANGLE_DEGREES;

  SerialDebug.print(
      "[ENDPOINT HANDOFF] ring1 xy -> predicted ring3 "
      "xy/raw-M5/search-M5/M6=");
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
  SerialDebug.println(pose.extensionMm, 2);
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

bool applyTargetPlacementExtraLower(ArmPose &pose) {
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
      "[TARGET PLACE Z] mapped/extra/corrected-mm=");
  SerialDebug.print(pose.heightMm, 2);
  SerialDebug.print("/-");
  SerialDebug.print(
      arm_config::TARGET_PLACE_EXTRA_LOWER_MM,
      1);
  SerialDebug.print("/");
  SerialDebug.println(correctedHeightMm, 2);
  pose.heightMm = correctedHeightMm;
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
    PlanarPoint &point) {
  if (radiusPixels <
          RING_ENDPOINT_MINIMUM_RADIUS_PIXELS ||
      radiusPixels >
          RING_ENDPOINT_MAXIMUM_RADIUS_PIXELS) {
    SerialDebug.print(
        "[ENDPOINT MAP] rejected radius=");
    SerialDebug.println(radiusPixels);
    return false;
  }

  const float mmPerPixel =
      RING_PHYSICAL_RADIUS_MM /
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

  const float gain =
      fineStage
          ? ENDPOINT_FINE_SERVO_GAIN
          : ENDPOINT_COARSE_SERVO_GAIN;
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
      targetPose);
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
  RAW_TARGET_WAIT_TOO_FAR
};

RawTargetPoseResult rawTargetPose(
    float pixelX,
    float pixelY,
    ArmPose &pose) {

  const float pixelDeltaX =
      pixelX - static_cast<float>(IMAGE_CENTER_X);
  const float pixelDeltaY =
      pixelY - static_cast<float>(IMAGE_CENTER_Y);
  const float pixelRadius =
      hypotf(pixelDeltaX, pixelDeltaY);

  const float rawMmPerPixel =
      pixelRadius <= PIXEL_MAPPING_SWITCH_RADIUS_PIXELS
          ? RAW_NEAR_MM_PER_PIXEL
          : RAW_FAR_MM_PER_PIXEL;
  const float clockwiseTangentMm =
      pixelDeltaX * rawMmPerPixel;
  const float cameraRadiusMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      RAW_VIEW_EXTENSION_MM;
  const float radialMm =
      cameraRadiusMm +
      pixelDeltaY *
          rawMmPerPixel *
          IMAGE_Y_TO_ARM_OUTWARD_SIGN;
  const float targetRadiusMm =
      hypotf(radialMm, clockwiseTangentMm);
  const float extensionMm =
      targetRadiusMm -
      ARM_PIVOT_TO_GRIPPER_CENTER_MM;
  const float standardFrameAngleDegrees =
      -atan2f(clockwiseTangentMm, radialMm) *
      180.0f / PI_F;

  SerialDebug.print(
      "[RAW VISION CAL] xy/dxy/r/scale/tan/rad/target-r/M5/M6=");
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
  SerialDebug.print(clockwiseTangentMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(radialMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(targetRadiusMm, 2);
  SerialDebug.print("/");
  SerialDebug.print(standardFrameAngleDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.println(extensionMm, 2);

  if (extensionMm <
      M6_STANDARD_EXTENSION_MM -
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    SerialDebug.print(
        "[RAW WAIT REACH] target too near, radius/ext=");
    SerialDebug.print(targetRadiusMm, 2);
    SerialDebug.print("/");
    SerialDebug.println(extensionMm, 2);
    return RAW_TARGET_WAIT_TOO_NEAR;
  }
  if (extensionMm >
      M6_MAXIMUM_EXTENSION_MM +
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    SerialDebug.print(
        "[RAW WAIT REACH] target too far, radius/ext=");
    SerialDebug.print(targetRadiusMm, 2);
    SerialDebug.print("/");
    SerialDebug.println(extensionMm, 2);
    return RAW_TARGET_WAIT_TOO_FAR;
  }

  pose.standardFrameAngleDegrees =
      standardFrameAngleDegrees;
  pose.extensionMm =
      extensionMm < M6_STANDARD_EXTENSION_MM
          ? M6_STANDARD_EXTENSION_MM
          : extensionMm;
  pose.heightMm = -RAW_PICK_LOWER_MM;
  return RAW_TARGET_POSE_VALID;
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
  WORK_PHASE_RAW_WAIT_TRAVEL_LINEAR_ZERO,
  WORK_PHASE_RAW_WAIT_TRAVEL_PARK,
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
  WORK_PHASE_ENDPOINT_WAIT_FINE_LOWER,
  WORK_PHASE_ENDPOINT_WAIT_RING3_RAISE,
  WORK_PHASE_CIRCLE_WAIT_ARM_LOWER,
  WORK_PHASE_CIRCLE_WAIT_COORDINATE,
  WORK_PHASE_CIRCLE_WAIT_CHASSIS,
  WORK_PHASE_CIRCLE_WAIT_ARM_STANDARD,
  WORK_PHASE_CIRCLE_WAIT_POST_VISION_HEADING,
  WORK_PHASE_START_UNLOAD,
  WORK_PHASE_START_RELOAD,
  WORK_PHASE_WAIT_TRANSFER,
  WORK_PHASE_WAIT_STORAGE_SERVO,
  WORK_PHASE_WAIT_STORAGE_PARK,
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

uint8_t rawFilledSlotMask = 0U;
uint8_t rawCollectedCount = 0U;
uint8_t rawPendingSlotIndex = 0U;
uint8_t rawPendingColor = 0U;
ArmPose rawPendingSourcePose;

bool rawTravelM5ZeroPending = false;
uint32_t processM5ZeroSettleDeadlineMs = 0UL;

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

uint8_t storageRingForItem(
    uint8_t roundIndex,
    uint8_t itemIndex) {
  if (roundIndex == 0U) {
    return taskPositions[0][itemIndex];
  }

  const uint8_t color = taskColors[1][itemIndex];
  const uint8_t ring =
      competition::stackedRingForColor(taskPlan, color);
  if (ring != 0U) {
    return ring;
  }

  routeFault("Second-batch color has no first-batch stack");
  return 0U;
}

void printMeasuredRingMapSummary() {
  if (!measuredRingPoseValid[1U] ||
      !measuredRingPoseValid[2U] ||
      !measuredRingPoseValid[3U]) {
    SerialDebug.println(
        "[RING MAP SUMMARY] unavailable");
    return;
  }

  const float deltaOutwardMm =
      measuredRingPoints[3U].outwardMm -
      measuredRingPoints[1U].outwardMm;
  const float deltaLeftMm =
      measuredRingPoints[3U].leftMm -
      measuredRingPoints[1U].leftMm;
  const float endpointSpanMm =
      hypotf(deltaOutwardMm, deltaLeftMm);
  const float endpointAxisAngleDegrees =
      atan2f(deltaLeftMm, deltaOutwardMm) *
      180.0f / PI_F;
  const uint32_t mapElapsedMs =
      endpointMapCompleteMs >= endpointMapStartMs
          ? endpointMapCompleteMs - endpointMapStartMs
          : 0UL;

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
  SerialDebug.print(
      "[RING MAP SUMMARY] span/axis-angle(1->3)=");
  SerialDebug.print(endpointSpanMm, 2);
  SerialDebug.print("/");
  SerialDebug.println(endpointAxisAngleDegrees, 2);
  SerialDebug.print(
      "[RING MAP SUMMARY] scan1/scan3/map ms=");
  SerialDebug.print(endpointScanElapsedMs[1U]);
  SerialDebug.print("/");
  SerialDebug.print(endpointScanElapsedMs[3U]);
  SerialDebug.print("/");
  SerialDebug.println(mapElapsedMs);
  SerialDebug.print(
      "[RING MAP SUMMARY] radius/conf 1,3=");
  SerialDebug.print(measuredRingRadiiPixels[1U]);
  SerialDebug.print("/");
  SerialDebug.print(measuredRingConfidence[1U]);
  SerialDebug.print(", ");
  SerialDebug.print(measuredRingRadiiPixels[3U]);
  SerialDebug.print("/");
  SerialDebug.println(measuredRingConfidence[3U]);
  SerialDebug.print(
      "[RING MAP SUMMARY] servo-moves/final-px 1,3=");
  SerialDebug.print(endpointServoMoveCounts[1U]);
  SerialDebug.print("/");
  SerialDebug.print(endpointFinalCenterErrorsPixels[1U], 2);
  SerialDebug.print(", ");
  SerialDebug.print(endpointServoMoveCounts[3U]);
  SerialDebug.print("/");
  SerialDebug.println(endpointFinalCenterErrorsPixels[3U], 2);
}

bool completeEndpointMapAndStartTransfers() {
  if (activeEndpointScanRing != 3U) {
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
      "[ENDPOINT MAP] complete; skip M5 standard 0 deg, "
      "prepare direct transfer from ring 3 to container pickup");
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
  armGripperLiftIsolationEnabled = false;
  workActionPhase = WORK_PHASE_IDLE;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  armStandardPhase = ARM_STANDARD_IDLE;
  armStandardEndpointPreparation = false;
  armStandardKeepBaseAngle = false;
  armStandardExtensionTargetMm =
      M6_STANDARD_EXTENSION_MM;
  armTransferPhase = ARM_TRANSFER_IDLE;
  armTransferM7HandoffStartMs = 0UL;
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
  armTransferMotionProfile =
      arm_transfer::PROFILE_STANDARD;
  endpointDirectContainerPickupPending = false;
  endpointInitialStorageCommanded = false;
  containerPickupPrepositionedPending = false;
  ringPickupPrepositionedPending = false;
  workActionStartMs = 0UL;
  workVisionRequestStartMs = 0UL;
  workVisionRetryCount = 0U;
  markMissionProgress();
  SerialDebug.println("Workstation action complete");

  if (ROUGH_PROCESSING_CALIBRATION_MODE &&
      completedAction == WORK_ACTION_PROCESS) {

    stopAllMotorsImmediately();
    enableDriveMotors();
    disableArmBaseMotor();
    commandGripperClose();
    programState = PROGRAM_FINISHED;
    routeMotionPhase = ROUTE_MOTION_IDLE;
    commandStarted = false;
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

void beginStorageParkingBeforeWorkFinish() {
  commandStorageServoParkingPosition();

  beginArmStandardization(0.0f, true);
  workStorageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  workActionPhase = WORK_PHASE_WAIT_STORAGE_PARK;
  SerialDebug.print("[WORK PARK] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, storage -> 165 deg; M6/M7 -> safe-zero; "
      "M5 holds current angle");
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

  commandStorageServoParkingPosition();
  workStorageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  useArmBaseRawTransferMotionProfile();
  startArmBaseStandardFrameDegrees(0.0f);
  if (programState != PROGRAM_RUNNING) {
    return;
  }
  rawTravelM5ZeroPending = true;
  workActionPhase = WORK_PHASE_RAW_WAIT_TRAVEL_PARK;
  SerialDebug.print("[RAW TRAVEL] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, M6/M7 safe; storage -> 165 deg; "
      "M5 -> standard 0 deg while route is released");
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

  // The transfer state machine finishes a tray release with M7 at the
  // -10 mm rotation-clearance height. The chassis travel gate requires the
  // true 0 mm working zero, so explicitly complete that final lift before
  // applying the strict M6/M7 safe-zero check and releasing the route.
  if (!startLiftToHeightMm(M7_STANDARD_HEIGHT_MM)) {
    return;
  }
  workActionPhase =
      WORK_PHASE_RAW_WAIT_TRAVEL_LINEAR_ZERO;
  SerialDebug.print("[RAW TRAVEL] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, final tray release complete; M7 clearance -> safe zero");
}

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
    routeFault("Gripper/storage servo offline");
    return;
  }

  activeWorkAction = kind;
  // Every workstation transfer can command the same gripper and M7 pair.
  // Keep a short nonzero two-way handoff active for RAW, PROCESS and STORAGE;
  // servo motion completion is covered by the independent settle gates.
  armGripperLiftIsolationEnabled =
      kind != WORK_ACTION_NONE;
  SerialDebug.print(
      "[WORK INTERLOCK] mode=SHORT_HANDOFF_ALL_ACTIONS, "
      "M7->gripper/gripper->M7 gap-ms=");
  SerialDebug.print(WORK_M7_TO_GRIPPER_GAP_MS);
  SerialDebug.print("/");
  SerialDebug.print(WORK_GRIPPER_TO_M7_GAP_MS);
  SerialDebug.println(
      "; legacy direct mode disabled for RAW/PROCESS/STORAGE");
  workActionStartMs = millis();
  workVisionRequestStartMs = 0UL;
  workVisionRetryCount = 0U;
  workRoundIndex =
      static_cast<uint8_t>(roundNumber - 1U);
  workItemIndex = 0U;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  armTransferPhase = ARM_TRANSFER_IDLE;
  armTransferM7HandoffStartMs = 0UL;
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
  endpointDirectContainerPickupPending = false;
  endpointInitialStorageCommanded = false;
  containerPickupPrepositionedPending = false;
  ringPickupPrepositionedPending = false;
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
  endpointCenteredConfirmationCount = 0U;
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
  rawFilledSlotMask = 0U;
  rawCollectedCount = 0U;
  rawPendingSlotIndex = 0U;
  rawPendingColor = 0U;
  rawPendingSourcePose = ArmPose();
  workLastMaixSequence =
      maixCam.latest().sequence;
  resetCircleStabilityWindow();

  if (kind == WORK_ACTION_RAW) {
    beginArmStandardization(
        0.0f,
        false,
        RAW_VIEW_EXTENSION_MM);

    commandStorageServoParkingPosition();
    workStorageServoDeadlineMs =
        millis() + STORAGE_SERVO_SETTLE_MS;
    workActionPhase = WORK_PHASE_PREPARE;
    SerialDebug.print("[WORK PREPARE] t=");
    SerialDebug.print(millis());
    SerialDebug.println(
        " ms, M5 -> standard 0 deg and M6 -> "
        "20 mm RAW view");
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
    processM5ZeroSettleDeadlineMs = 0UL;
    workStorageServoDeadlineMs = 0UL;
    workActionPhase =
        WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO;
    SerialDebug.print("[PROCESS ENTRY] t=");
    SerialDebug.print(millis());
    SerialDebug.println(
        " ms, waiting for standard M5 0 deg before ring 1");
    return;
  }

  beginArmEndpointPreparation();
  workStorageServoDeadlineMs = 0UL;
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

  commandStorageServoPosition(
      static_cast<uint8_t>(mappedSlot));
  workStorageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  workActionPhase =
      WORK_PHASE_RAW_WAIT_EXPECTED_SLOT;
  SerialDebug.print(
      "[RAW PREP] expected color/slot=");
  SerialDebug.print(expectedColor);
  SerialDebug.print("/");
  SerialDebug.print(
      static_cast<unsigned int>(mappedSlot));
  SerialDebug.println(
      "; settle tray before requesting fresh coordinates");
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
  SerialDebug.print("/");
  SerialDebug.println(MAIXCAM_ALL_COLORS_REQUEST);

  // Keep the deployed legacy vision protocol: mode 8 reports any stable
  // visible color. STM32 still enforces the QR order below and never grabs a
  // different color.
  beginMaixRequest(MAIXCAM_ALL_COLORS_REQUEST);
  workVisionRequestStartMs = millis();
  workActionPhase = WORK_PHASE_RAW_WAIT_RESULT;
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
  endpointCenteredConfirmationCount = 0U;
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
  if (extensionAxis.active ||
      liftAxis.active ||
      fabsf(
          extensionAxis.currentMm -
          M6_STANDARD_EXTENSION_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM ||
      fabsf(
          liftAxis.currentMm -
          HOUGH_VISION_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault(
      "Initial endpoint scan requires M6 safe zero and M7 coarse height");
    return;
  }

  ArmPose searchPose;
  if (!nominalRingPose(
          ringPosition,
          HOUGH_VISION_LOWER_MM,
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
          HOUGH_VISION_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault("Ring-3 direct handoff requires M7 coarse height");
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
    const char *reason) {
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

  visualCorrectionAccumulator =
      addMotorPulses(
          visualCorrectionAccumulator,
          correctionPulses);
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
    if (!deadlineReached(workStorageServoDeadlineMs)) {
      return;
    }
    workStorageServoDeadlineMs = 0UL;
    SerialDebug.println(
        "[TRANSFER SAFE] initial storage slot 0 settled; "
        "first pickup may start");
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
  if (!applyTargetPlacementExtraLower(destination)) {
    return;
  }
  const bool concurrentSourcePreparation =
      endpointDirectContainerPickupPending;
  endpointDirectContainerPickupPending = false;
  const bool sourceAlreadyPrepared =
      containerPickupPrepositionedPending;
  containerPickupPrepositionedPending = false;
  const bool prepareNextContainerPickup =
      workItemIndex < 2U;
  const bool prepareFirstPlacedRingPickup =
      activeWorkAction == WORK_ACTION_PROCESS &&
      workItemIndex == 2U;
  const bool prepareNextSource =
      prepareNextContainerPickup ||
      prepareFirstPlacedRingPickup;
  const ArmPose currentContainerPickupPose =
      arm_transfer::containerPickPose(workItemIndex);
  ArmPose nextSourcePose;
  uint8_t nextStorageSlot =
      static_cast<uint8_t>(workItemIndex + 1U);
  if (prepareNextContainerPickup) {
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
    if (!ringPose(
            firstRingPosition,
            PROCESS_PLACE_LOWER_MM +
                arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM,
            nextSourcePose)) {
      return;
    }
    nextStorageSlot = firstTransferItemIndex;
    SerialDebug.print(
        "[PLACE->PICK PIPELINE] final placement immediately "
        "prepositions first ring/slot=");
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
      prepareFirstPlacedRingPickup);
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
              arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM,
          source)) {
    return;
  }
  SerialDebug.print(
      "[RING RETURN PICK] extra M7 lower/target-mm=");
  SerialDebug.print(
      arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM,
      1);
  SerialDebug.print("/");
  SerialDebug.println(source.heightMm, 1);

  const bool sourceAlreadyPrepared =
      ringPickupPrepositionedPending;
  ringPickupPrepositionedPending = false;
  const bool prepareNextRingPickup =
      workItemIndex < 2U;
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
    if (!ringPose(
            nextRingPosition,
            PROCESS_PLACE_LOWER_MM +
                arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM,
            nextRingPickupPose)) {
      return;
    }
    nextStorageSlot = nextTransferItemIndex;
    SerialDebug.print(
        "[RETURN PIPELINE] next ring/slot preposition=");
    SerialDebug.print(nextRingPosition);
    SerialDebug.print("/");
    SerialDebug.println(nextStorageSlot);
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
      true);
  activeTransferPurpose =
      TRANSFER_PURPOSE_RING_TO_CONTAINER;
  workActionPhase = WORK_PHASE_WAIT_TRANSFER;
}

void completeTransferAndRotateStorage() {
  markMissionProgress();
  switch (activeTransferPurpose) {
    case TRANSFER_PURPOSE_RAW_TO_CONTAINER: {
      ++correctGrabCount;
      hmiSetTaskCounts();
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

        beginRawTravelParkingAfterFinalStore();
      } else {
        workVisionRetryCount = 0U;
        beginRawExpectedSlotPreparation();
      }
      return;
    }

    case TRANSFER_PURPOSE_CONTAINER_TO_RING:
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

  if (preparedNextSource) {
    if (preparedFirstPlacedRingPickup) {
      workItemIndex = 0U;
      containerPickupPrepositionedPending = false;
      ringPickupPrepositionedPending = true;
      activeTransferPurpose =
          TRANSFER_PURPOSE_RING_TO_CONTAINER;
      SerialDebug.println(
          "[PLACE->PICK PIPELINE] three placements complete; "
          "first ring pose retained while tray finishes rotating");
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
    workActionPhase =
        WORK_PHASE_WAIT_STORAGE_SERVO;
    SerialDebug.println(
        "[TRANSFER PIPELINE] next source handoff retained; "
        "wait only remaining storage-servo time");
    return;
  }

  const bool finishedFinalContainerSequence =
      workItemIndex >= 3U &&
      (activeTransferPurpose ==
           TRANSFER_PURPOSE_RING_TO_CONTAINER ||
       (activeWorkAction == WORK_ACTION_STORAGE &&
        activeTransferPurpose ==
            TRANSFER_PURPOSE_CONTAINER_TO_RING));
  if (finishedFinalContainerSequence) {
    commandStorageServoParkingPosition();
  } else {
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
      millis() + STORAGE_SERVO_SETTLE_MS;
  workActionPhase = WORK_PHASE_WAIT_STORAGE_SERVO;
}

void startVisualCorrectionRestore() {
  stopMaixRequest();
  if (ROUGH_PROCESSING_CALIBRATION_MODE) {

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

  if (ROUGH_PROCESSING_CALIBRATION_MODE) {
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

  serviceArmTransfer();

  switch (workActionPhase) {
    case WORK_PHASE_IDLE:
      break;

    case WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO:
      if (armMotors.isM5Running() ||
          armBaseMotionWatchdogActive) {
        break;
      }
      processM5ZeroSettleDeadlineMs =
          millis() + ARM_BASE_SETTLE_MS;
      workActionPhase =
          WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO_SETTLE;
      SerialDebug.print("[PROCESS ENTRY] t=");
      SerialDebug.print(millis());
      SerialDebug.println(
          " ms, standard M5 0 deg pulse target reached; "
          "confirming 20 ms vision settle");
      break;

    case WORK_PHASE_PROCESS_WAIT_TRAVEL_M5_ZERO_SETTLE:
      if (!deadlineReached(
              processM5ZeroSettleDeadlineMs) ||
          armMotors.isM5Running() ||
          armBaseMotionWatchdogActive) {
        break;
      }
      rawTravelM5ZeroPending = false;
      processM5ZeroSettleDeadlineMs = 0UL;
      beginArmEndpointPreparation();
      workActionPhase = WORK_PHASE_PREPARE;
      SerialDebug.print("[PROCESS ENTRY] t=");
      SerialDebug.print(millis());
      SerialDebug.println(
          " ms, standard M5 0 deg confirmed -> ring 1");
      break;

    case WORK_PHASE_PREPARE:
      if (activeWorkAction == WORK_ACTION_RAW) {
        if (!serviceArmStandardization() ||
            !deadlineReached(
                workStorageServoDeadlineMs)) {
          break;
        }
        SerialDebug.print("[WORK PREPARE] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, arm and storage settle complete");
        armStandardPhase = ARM_STANDARD_IDLE;
        beginRawExpectedSlotPreparation();
        break;
      }

      if (serviceArmStandardization()) {
        armStandardPhase = ARM_STANDARD_IDLE;

        SerialDebug.print("[WORK PREPARE] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, M5/M7 ring-1 pre-pose ready; "
            "storage remains parked until M6 arrives");

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

      ArmPose source;
      const RawTargetPoseResult poseResult =
          rawTargetPose(
              static_cast<float>(x),
              static_cast<float>(y),
              source);
      if (poseResult != RAW_TARGET_POSE_VALID) {

        beginRawItemVision();
        break;
      }

      stopMaixRequest();
      workVisionRetryCount = 0U;
      // The vision wait can be indefinite. Start a fresh mechanical-action
      // timeout only after an expected, reachable target is accepted.
      workActionStartMs = millis();
      rawPendingColor = detectedColor;
      rawPendingSlotIndex = slotIndex;
      rawPendingSourcePose = source;
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
      beginArmTransfer(
          rawPendingSourcePose,
          arm_transfer::containerPlacePose(),
          false,
          false,
          false,
          false,
          false,
          nullptr,
          -1,
          rawCollectedCount < 2U,
          true,
          true);
      activeTransferPurpose =
          TRANSFER_PURPOSE_RAW_TO_CONTAINER;
      workActionPhase = WORK_PHASE_WAIT_TRANSFER;
      break;
    }

    case WORK_PHASE_RAW_WAIT_EXPECTED_SLOT:
      if (!deadlineReached(
              workStorageServoDeadlineMs)) {
        break;
      }
      beginRawItemVision();
      break;

    case WORK_PHASE_RAW_WAIT_TRAVEL_LINEAR_ZERO:
      if (!liftMoveFinished()) {
        break;
      }
      startRawTravelParkingAtSafeLinearZero();
      break;

    case WORK_PHASE_RAW_WAIT_TRAVEL_PARK:
      if (!deadlineReached(
              workStorageServoDeadlineMs)) {
        break;
      }
      SerialDebug.print("[RAW TRAVEL] t=");
      SerialDebug.print(millis());
      SerialDebug.println(
          " ms, storage parked; release chassis route "
          "without waiting for M5");
      finishActiveWorkAction();
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
        if (!startLiftToHeightMm(
                activeEndpointScanPose.heightMm)) {
          break;
        }
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
        endpointCenteredConfirmationCount = 0U;
        if (activeEndpointStageServoMoveCount >=
                ENDPOINT_MAXIMUM_SERVO_MOVES_PER_STAGE ||
            activeEndpointServoMoveCount >=
                ENDPOINT_MAXIMUM_TOTAL_SERVO_MOVES) {
          SerialDebug.print(
              "[ENDPOINT SERVO] failed ring/stage/"
              "stage-moves/total-moves/error px=");
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
            "[ENDPOINT SERVO] local move ring/stage/"
            "stage-step/total-step/"
            "error-px/measured-mm/command-mm/M5/M6=");
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

      if (!endpointFineVisionActive) {
        stopMaixRequest();
        workVisionRequestStartMs = 0UL;
        workVisionRetryCount = 0U;
        endpointFineVisionActive = true;
        activeEndpointStageServoMoveCount = 0U;
        endpointCenteredConfirmationCount = 0U;
        activeEndpointScanPose.heightMm =
            -ENDPOINT_FINE_VISION_LOWER_MM;
        if (!startLiftToHeightMmWithProfile(
                activeEndpointScanPose.heightMm,
                ENDPOINT_FINE_M7_SPEED_RPM,
                ENDPOINT_FINE_M7_ACCELERATION)) {
          break;
        }
        SerialDebug.print(
            "[ENDPOINT SERVO] coarse centered ring/error px; "
            "lower M7 to fine height=");
        SerialDebug.print(ring);
        SerialDebug.print("/");
        SerialDebug.print(centerErrorPixels, 2);
        SerialDebug.print(" -> ");
        SerialDebug.println(
            activeEndpointScanPose.heightMm,
            2);
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_FINE_LOWER;
        break;
      }

      const bool fastAcceptedStableCenter =
          centerErrorPixels <=
              ENDPOINT_FAST_ACCEPT_CENTER_TOLERANCE_PIXELS &&
          confidence >=
              ENDPOINT_FAST_ACCEPT_MINIMUM_CONFIDENCE;
      if (fastAcceptedStableCenter) {
        endpointCenteredConfirmationCount =
            ENDPOINT_FINAL_CENTER_CONFIRMATIONS;
        SerialDebug.print(
            "[ENDPOINT SERVO] strict fast acceptance "
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
        activeEndpointScanPose.heightMm =
            HOUGH_VISION_HEIGHT_MM;
        if (!startLiftToHeightMm(
                activeEndpointScanPose.heightMm)) {
          break;
        }
        SerialDebug.println(
            "[ENDPOINT HANDOFF] ring 1 accepted; raise M7 only "
            "to coarse height, hold M7, then move M5/M6 to ring 3");
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_RING3_RAISE;
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

    case WORK_PHASE_ENDPOINT_WAIT_FINE_LOWER:
      if (liftMoveFinished()) {
        activeEndpointScanPose.heightMm =
            liftAxis.currentMm;
        SerialDebug.print(
            "[ENDPOINT SERVO] fine height ready ring/M7=");
        SerialDebug.print(activeEndpointScanRing);
        SerialDebug.print("/");
        SerialDebug.println(
            activeEndpointScanPose.heightMm,
            2);
        beginEndpointVision();
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_RING3_RAISE:
      if (liftMoveFinished()) {
        activeEndpointScanPose.heightMm =
            liftAxis.currentMm;
        (void)beginDirectRing3EndpointScan();
      }
      break;

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

    case WORK_PHASE_WAIT_STORAGE_SERVO:
      if (!deadlineReached(
              workStorageServoDeadlineMs)) {
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

      if (activeWorkAction == WORK_ACTION_PROCESS &&
          activeTransferPurpose ==
              TRANSFER_PURPOSE_CONTAINER_TO_RING) {
        workItemIndex = 0U;
        workActionPhase = WORK_PHASE_START_RELOAD;
      } else {
        workActionPhase = WORK_PHASE_START_RESTORE;
      }
      break;

    case WORK_PHASE_WAIT_STORAGE_PARK:
      if (serviceArmStandardization() &&
          deadlineReached(workStorageServoDeadlineMs)) {
        armStandardPhase = ARM_STANDARD_IDLE;
        SerialDebug.print("[WORK TRAVEL] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, COMPLETE: M5 held at current angle, "
            "M6/M7 safe-zero; no M5 zero command");
        finishActiveWorkAction();
      }
      break;

    case WORK_PHASE_START_RESTORE:
      startVisualCorrectionRestore();
      break;

    case WORK_PHASE_WAIT_RESTORE:
      if (updateHeadingLock(MOTION_TIMEOUT_MS)) {
        visualCorrectionAccumulator = MotorPulses();
        visualCorrectionForwardMm = 0.0f;
        visualCorrectionLeftMm = 0.0f;
        visualCorrectionMoveCount = 0U;
        beginStorageParkingBeforeWorkFinish();
      }
      break;
  }
}

void cancelCompetitionAction() {
  activeWorkAction = WORK_ACTION_NONE;
  armGripperLiftIsolationEnabled = false;
  workActionPhase = WORK_PHASE_IDLE;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  armStandardPhase = ARM_STANDARD_IDLE;
  armStandardEndpointPreparation = false;
  armStandardKeepBaseAngle = false;
  armStandardExtensionTargetMm =
      M6_STANDARD_EXTENSION_MM;
  armTransferPhase = ARM_TRANSFER_IDLE;
  armTransferM7HandoffStartMs = 0UL;
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
  armTransferMotionProfile =
      arm_transfer::PROFILE_STANDARD;
  endpointDirectContainerPickupPending = false;
  endpointInitialStorageCommanded = false;
  containerPickupPrepositionedPending = false;
  ringPickupPrepositionedPending = false;
  workActionStartMs = 0UL;
  workVisionRequestStartMs = 0UL;
  workVisionRetryCount = 0U;
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
  endpointCenteredConfirmationCount = 0U;
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
  processM5ZeroSettleDeadlineMs = 0UL;
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

void startMotionCommand(const RouteCommand &command) {

  turnMotionEnabled = false;
  activeRouteTurnCommand = false;
  routeMotionPhase = ROUTE_MOTION_IDLE;
  workstationApproachEnabled = false;
  translationPreciseArrivalEnabled = false;
  translationCentralChannelEnabled = false;

  switch (command.type) {
    case COMMAND_ZONE_LONGITUDINAL_FAST:
      startRouteFastLongitudinalTranslation(
          selectedStartZoneDirection() *
              static_cast<float>(command.value),
          command.motionScale);
      break;

    case COMMAND_SCAN_SLOW:
      startQrScanAction();
      break;

    case COMMAND_ADJUST_TO_POINT_A: {
      if (!scanFlag || !scanOriginValid) {
        routeFault(
            "Step 3 requires valid QR code and scan origin");
        break;
      }
      const float deltaAlongScanDirectionMm =
          static_cast<float>(
              SCAN_START_TO_POINT_A_MM) -
          scanDistanceBmm;
      const float vehicleForwardMm =
          selectedStartZoneDirection() *
          deltaAlongScanDirectionMm;
      SerialDebug.print(
          "[SCAN->A] b/delta/vehicle-forward=");
      SerialDebug.print(scanDistanceBmm, 1);
      SerialDebug.print("/");
      SerialDebug.print(deltaAlongScanDirectionMm, 1);
      SerialDebug.print("/");
      SerialDebug.print(vehicleForwardMm, 1);
      SerialDebug.println(" mm");
      startRouteFastLongitudinalTranslation(
          vehicleForwardMm,
          command.motionScale);
      break;
    }

    case COMMAND_FORWARD_FAST:
      startRouteFastLongitudinalTranslation(
          static_cast<float>(command.value),
          command.motionScale);
      break;

    case COMMAND_BACKWARD_FAST:
      startRouteFastLongitudinalTranslation(
          -static_cast<float>(command.value),
          command.motionScale);
      break;

    case COMMAND_RIGHT_FAST: {
      float speedProfileScale =
          ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;
      float accelerationProfileScale =
          ROUTE_NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;
      if (command.specificationStep == 7U) {
        speedProfileScale =
            STEP_07_LATERAL_MAX_SPEED_SCALE;
        accelerationProfileScale =
            STEP_07_LATERAL_ACCELERATION_SCALE;
      } else if (command.specificationStep == 15U) {
        speedProfileScale =
            STEP_15_LATERAL_MAX_SPEED_SCALE;
        accelerationProfileScale =
            STEP_15_LATERAL_ACCELERATION_SCALE;
      }
      startRouteFastLateralTranslation(
          static_cast<float>(command.value),
          command.motionScale,
          speedProfileScale,
          accelerationProfileScale);
      break;
    }

    case COMMAND_TURN_COUNTERCLOCKWISE:
      startRouteTurn(
          static_cast<float>(command.value),
          command.motionScale);
      break;

    case COMMAND_TURN_CLOCKWISE:
      startRouteTurn(
          -static_cast<float>(command.value),
          command.motionScale);
      break;

    case COMMAND_RAW_ACTION:

      activeCompetitionRound =
          static_cast<uint8_t>(command.value);
      rawActionFinished = false;
      hmiSetRunStatus(
          command.value == 1 ? "RAW1" : "RAW2");
      beginWorkAction(
          WORK_ACTION_RAW,
          static_cast<uint8_t>(command.value));
      break;

    case COMMAND_PROCESS_ACTION:

      activeCompetitionRound =
          static_cast<uint8_t>(command.value);
      processActionFinished = false;
      hmiSetRunStatus(
          command.value == 1 ? "PROCESS1" : "PROCESS2");
      beginWorkAction(
          WORK_ACTION_PROCESS,
          static_cast<uint8_t>(command.value));
      break;

    case COMMAND_STORAGE_ACTION:

      activeCompetitionRound =
          static_cast<uint8_t>(command.value);
      storageActionFinished = false;
      hmiSetRunStatus(
          command.value == 1 ? "STORAGE1" : "STORAGE2");
      beginWorkAction(
          WORK_ACTION_STORAGE,
          static_cast<uint8_t>(command.value));
      break;

    case COMMAND_FINISH:
      break;
  }
}

void startCurrentCommand() {
  if (routeIndex >= ROUTE_COMMAND_COUNT) {
    return;
  }

  activeRouteCommand = route[routeIndex];
  commandStartMs = millis();
  routeHeadingLockStartMs = 0UL;
  headingStableStartMs = 0;
  motorsArrivedStartMs = 0;
  printCurrentCommand(activeRouteCommand);
  startMotionCommand(activeRouteCommand);
  if (programState == PROGRAM_RUNNING) {
    commandStarted = true;
    markMissionProgress();
  }
}

void finishProgram() {
  resetQrScanActionState();
  cancelCompetitionAction();
  emergencyStopArmLinearAxes();
  disableDriveMotors();
  disableArmBaseMotor();
  commandGripperClose();
  commandStorageServoParkingPosition();
  programState = PROGRAM_FINISHED;
  routeMotionPhase = ROUTE_MOTION_IDLE;
  activeRouteTurnCommand = false;
  commandStarted = false;
  hmiSetRunStatus("FINISH");

  if (SHOW_RESULT_PAGE_ON_FINISH) {
    hmiCommand("page page0");
    hmiSetTaskCounts();
  }

  SerialDebug.println("Route finished");
}

void updateRoute() {

  if (ROUGH_PROCESSING_CALIBRATION_MODE) {
    return;
  }

  if (programState != PROGRAM_RUNNING ||
      routeIndex >= ROUTE_COMMAND_COUNT) {
    return;
  }

  if (!commandStarted) {
    startCurrentCommand();
    if (!commandStarted) {
      return;
    }
  }

  const RouteCommand &command = activeRouteCommand;

  switch (command.type) {
    case COMMAND_ZONE_LONGITUDINAL_FAST:
    case COMMAND_ADJUST_TO_POINT_A:
    case COMMAND_FORWARD_FAST:
    case COMMAND_BACKWARD_FAST:
    case COMMAND_RIGHT_FAST:
      serviceRoutePhysicalCommand();
      break;

    case COMMAND_TURN_COUNTERCLOCKWISE:
    case COMMAND_TURN_CLOCKWISE:
      serviceIntegratedTurnCommand();
      break;

    case COMMAND_SCAN_SLOW:
      if (updateQrScanAction()) {
        hmiSetRunStatus("RUN");

        qrScanPhase = QR_SCAN_IDLE;
        qrScanActionStartMs = 0UL;
        advanceRoute();
      }
      break;

    case COMMAND_RAW_ACTION:
      if (rawActionFinished) {
        hmiSetRunStatus("RUN");
        advanceRoute();
      }
      break;

    case COMMAND_PROCESS_ACTION:
      if (processActionFinished) {
        hmiSetRunStatus("RUN");
        advanceRoute();
      }
      break;

    case COMMAND_STORAGE_ACTION:
      if (storageActionFinished) {
        hmiSetRunStatus("RUN");
        advanceRoute();
      }
      break;

    case COMMAND_FINISH:
      finishProgram();
      break;
  }
}

void onStartButtonClick() {
  if (programState != PROGRAM_RUNNING) {
    abortRequested = false;
    startRequested = true;
    imuWaitStatusDisplayed = false;

    startArmBaseRotationToDegrees(
        static_cast<float>(
            ARM_BASE_TRAVEL_OLD_FRAME_DEGREES));
  }
}

void onStartButtonDoubleClick() {
  if (programState != PROGRAM_WAITING || startRequested) {
    return;
  }

  if (ROUGH_PROCESSING_CALIBRATION_MODE) {
    hmiSetRunStatus("CALREADY");
    SerialDebug.println(
        "Calibration mode ignores start-zone selection");
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

void suspendRouteChassisProfileForWorkstation() {
  /*
   * route_chassis::runAllMotors()必须持续产生M1～M4脉冲，但工位暂停后
   * 位置目标和运动参数属于GC视觉修正。把route上一段遗留的S形/制动
   * 包络置为中性，避免它在暂停期间覆盖GC设置的maxSpeed/acceleration。
   * 下一路线段的startCurrentCommand()会通过route原有
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
      (!ROUGH_PROCESSING_CALIBRATION_MODE &&
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

  routeIndex = 0;
  integratedWorkPause = INTEGRATED_WORK_NONE;
  integratedWorkPauseStartMs = 0UL;
  activeCompetitionRound = 0;
  commandStarted = false;
  routeMotionPhase = ROUTE_MOTION_IDLE;
  activeRouteTurnCommand = false;
  resetQrScanActionState();
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
  if (ROUGH_PROCESSING_CALIBRATION_MODE) {
    if (!configureRoughProcessingCalibrationTask()) {
      startRequested = false;
      return;
    }
    hmiSetRunStatus("CALRUN");
  } else {
    hmiSetText("t1", "QRWAIT");
    hmiSetRunStatus("RUN");
  }
  commandGripperClose();
  commandStorageServoParkingPosition();

  if (ROUGH_PROCESSING_CALIBRATION_MODE) {
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
    SerialDebug.println(
        "Rough-processing calibration started: "
        "wheels locked; scan rings 1/3, derive ring 2, "
        "QR place/pick ring order 1/3/2");
    beginWorkAction(WORK_ACTION_PROCESS, 1U);
    return;
  }

  /*
   * 正常比赛的四轮清零、使能、速度档、IMU参考和21步路线状态，只由
   * GongChuang_route建立。当前文件随后只镜像航向供机械臂视觉使用。
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
      route_chassis::START_ZONE_SELECTION == 1U
          ? START_ZONE_1
          : START_ZONE_2;
  programState = PROGRAM_RUNNING;
  startRequested = false;
  imuWaitStatusDisplayed = false;

  SerialDebug.println(
      "GongChuang_route 21-step chassis + GC arm/vision started");
  SerialDebug.print("Locked start zone: ");
  SerialDebug.println(
      static_cast<unsigned int>(selectedStartZone));
}

void abortRoute() {
  if (route_chassis::programState ==
      route_chassis::PROGRAM_RUNNING) {
    route_chassis::abortRoute();
  } else {
    route_chassis::disableDriveMotors();
  }
  route_chassis::startRequested = false;
  route_chassis::abortRequested = false;
  resetQrScanActionState();
  invalidateArmLinearReference();
  cancelCompetitionAction();
  emergencyStopArmLinearAxes();
  disableDriveMotors();
  disableArmBaseMotor();
  programState = PROGRAM_FAULT;
  routeMotionPhase = ROUTE_MOTION_IDLE;
  activeRouteTurnCommand = false;
  commandStarted = false;
  abortRequested = false;
  startRequested = false;
  hmiSetRunStatus("STOP");
  SerialDebug.println("Route aborted by long press");
}

void synchronizeTaskCodeToRouteChassis() {
  /*
   * PE0/PE1由GC任务码解析器读取，因为机械臂需要四组颜色/位置数据；
   * 完整解码后只把锁定结果交给route的第2步扫码状态机。
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
      "[CHASSIS BRIDGE] decoded task code released route scan step");
}

IntegratedWorkPause workPauseAfterRouteStep(
    uint8_t completedStep) {
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
  commandStarted = true;

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
  SerialDebug.print(
      "[CHASSIS BRIDGE] arm/vision complete, resume route after ms=");
  SerialDebug.println(millis() - integratedWorkPauseStartMs);
  integratedWorkPause = INTEGRATED_WORK_NONE;
  integratedWorkPauseStartMs = 0UL;
  commandStarted = false;
  routeImuReferenceDegrees =
      route_chassis::routeImuReferenceDegrees;
  targetCounterClockwiseHeadingDegrees =
      route_chassis::targetCounterClockwiseHeadingDegrees;
  hmiSetRunStatus("RUN");
  markMissionProgress();
}

void serviceIntegratedRoute() {
  if (ROUGH_PROCESSING_CALIBRATION_MODE ||
      programState != PROGRAM_RUNNING) {
    return;
  }

  if (route_chassis::programState ==
      route_chassis::PROGRAM_FAULT) {
    routeFault("GongChuang_route chassis entered FAULT");
    return;
  }

  if (integratedWorkPause != INTEGRATED_WORK_NONE) {
    if (integratedWorkPauseFinished()) {
      finishIntegratedWorkPause();
    }
    return;
  }

  const size_t previousIndex = route_chassis::routeIndex;
  route_chassis::updateRoute();
  routeIndex = route_chassis::routeIndex;

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
  if (programState != PROGRAM_RUNNING) {
    return;
  }

  const uint32_t nowMs = millis();
  const bool routeWaitingForQrCode =
      route_chassis::programState ==
          route_chassis::PROGRAM_RUNNING &&
      route_chassis::routeIndex <
          route_chassis::ROUTE_COMMAND_COUNT &&
      route_chassis::route[
          route_chassis::routeIndex].type ==
          route_chassis::COMMAND_SCAN_SLOW &&
      route_chassis::routePhase ==
          route_chassis::ROUTE_PHASE_WAIT_SCAN_CODE;

  if (routeWaitingForQrCode ||
      qrScanPhase == QR_SCAN_WAIT_AT_LIMIT ||
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
      ROUGH_PROCESSING_CALIBRATION_MODE
          ? "CAL132"
          : "QRWAIT");
  hmiSetText("t3", "000+000+");
  hmiSetText("t8", "000+000");
  hmiSetRunStatus(
      !armLinearReferenceValid
          ? "M6INIT"
          : (ROUGH_PROCESSING_CALIBRATION_MODE
                 ? "CALREADY"
                 : (selectedStartZone == START_ZONE_1
                        ? "READY1"
                        : "READY2")));
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
  writeArmLinearEnable(extensionAxis.address, true);
  SerialArmLinear.flush();
  delay(ARM_AXIS_COMMAND_GUARD_MS);
  clearArmLinearReceiveBuffer();
  writeArmLinearEnable(liftAxis.address, true);
  SerialArmLinear.flush();
  delay(ARM_AXIS_COMMAND_GUARD_MS);
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
  SerialDebug.print(
      "[M7 EXPERIMENT] speed-2x/acceleration-4x flags, speed-cap="
  );
  SerialDebug.print(
      m7_experiment::DOUBLE_SPEED_AND_ACCELERATION ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.print(
      m7_experiment::DOUBLE_ACCELERATION_AGAIN ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.print(
      m7_experiment::EMM42_V5_MAXIMUM_SPEED_RPM);
  SerialDebug.println(" RPM");
}

}

void setup() {
  /*
   * 串口、PB9、M1～M4、IMU、二维码、HMI和电池先由route正式main初始化；
   * GC随后只追加机械臂、舵机、MaixCAM和工位动作。
   */
  route_chassis::setup();
  selectedStartZone =
      route_chassis::START_ZONE_SELECTION == 1U
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
      establishArmLinearSafeWorkingZeros();
  hmiSetRunStatus(
      armWorkingZerosReady
          ? (ROUGH_PROCESSING_CALIBRATION_MODE
                 ? "CALREADY"
                 : (selectedStartZone == START_ZONE_1
                        ? "READY1"
                        : "READY2"))
          : "ARMZEROERR");
  resetQrReceiver();

  if (!armWorkingZerosReady) {
    SerialDebug.println(
        "Controller NOT READY: M6/M7 failed to establish "
        "their 10 mm safe working zeros");
    SerialDebug.println(
        "SAFETY: power off and manually restore both mechanical "
        "start endpoints before another startup");
  } else if (ROUGH_PROCESSING_CALIBRATION_MODE) {
    SerialDebug.println(
        "Rough-processing calibration ready: place the chassis "
        "at D, face the three rings, and load tray slots 0/1/2");
    SerialDebug.println(
        "Click PB9 to detect rings 1/3, derive ring 2, "
        "place three items and pick all three back; "
        "long-press to stop");
  } else {
    SerialDebug.println(
        "GongChuang_route chassis + GC arm/vision ready");
    SerialDebug.println(
        "START_ZONE_SELECTION comes from route main; click PB9 to start; "
        "long-press to stop");
  }
}

void loop() {
  route_chassis::runAllMotors();
  armMotors.serviceM5();
  serviceArmBaseMotionWatchdog();
  route_chassis::startButton.tick();
  route_chassis::receiveImuData();
  serviceArmLinearAxes();
  serviceMaixcam();
  receiveQrData();
  synchronizeTaskCodeToRouteChassis();
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
  serviceIntegratedRoute();
}
