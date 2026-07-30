#ifndef GONGCHUANG_VISION_YANYAN_TEST
#define GONGCHUANG_VISION_YANYAN_TEST 0
#endif

static_assert(
    GONGCHUANG_VISION_YANYAN_TEST == 0 ||
        GONGCHUANG_VISION_YANYAN_TEST == 1,
    "GONGCHUANG_VISION_YANYAN_TEST must be 0 or 1");

#include <AccelStepper.h>
#include <ArmMotorController.h>
#include <Arduino.h>
#include <CompetitionCore.h>
#include <FashionStar_UartServo.h>
#include <JY901.h>
#include <MecanumKinematics.h>
#include <OneButton.h>
#include <VisionProtocol.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

using namespace mecanum;

/*
 * GongChuang：225 mm机械臂中心偏移的省赛物流赛项两批物料移动路径
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
 * 路径完整执行两批“原料区 -> 粗加工区 -> 暂存区”。每个工作区均由
 * 2、4侧面对工位中线。PATH_ONLY_TEST=false时执行本文后半部的MaixCAM
 * 定位、M5～M7、夹爪和载物盘非阻塞动作状态机。
 *
 * 【参赛规则地点对照：附件2，物流赛项决赛第一阶段】
 *   - 场地为2400×2400 mm；灰色十字车道宽400 mm。
 *   - 启停区1在右上角，启停区2在右下角，尺寸均为300×300 mm。
 *   - 原料区在上边中部；粗加工区在下边中部；暂存区在左边中部；
 *     二维码板在右边内侧的规定范围内随机摆放。
 *   - 规则要求：扫码 -> 第一批原料/粗加工/暂存 ->
 *     第二批原料/粗加工/暂存（同色码垛）-> 返回抽签确定的启停区。
 *   - 规则图还画有精加工区、成品区，但决赛第一阶段公布的固定流程
 *     只经过原料区、粗加工区和暂存区；本路径没有访问精加工区、成品区。
 *   - 灰色道路上的黑色障碍物位置可随机变化；本文件是固定命令序列，
 *     没有障碍物检测、绕行或重新规划逻辑。
 *
 * 【三种坐标层次，调试时不能混用】
 *   1. 规则层：原料区和二维码板允许在规定范围内随机摆放，实际位置
 *      以现场为准；粗加工区/暂存区台面名义尺寸为580×150 mm。
 *   2. route-simulator层：旧tmcode1理想路线采用原料/粗加工/暂存车心
 *      (1200,2100)/(1200,300)/(300,1200)，仅用于理解路线形状。
 *   3. 本文件层：当前常量、补偿量和实车标定才决定实际发出的脉冲；
 *      程序不测量世界坐标，平移位置是开环脉冲累计，IMU只闭环航向。
 *      因此下文“坐标”均是设计或开环推算值，不是定位传感器实测值。
 *
 * 注意：代码中的“第1轮/第2轮”表示同一次比赛任务里的第一批/第二批
 * 三个物料，不是规则所说的两次独立运行机会。
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

HardwareSerial SerialDebug(DEBUG_RX_PIN, DEBUG_TX_PIN);
HardwareSerial SerialHmi(HMI_RX_PIN, HMI_TX_PIN);
HardwareSerial SerialImu(IMU_RX_PIN, IMU_TX_PIN);
HardwareSerial SerialQr(QR_RX_PIN, QR_TX_PIN);
HardwareSerial SerialMaixcam(MAIXCAM_RX_PIN, MAIXCAM_TX_PIN);
HardwareSerial SerialArmLinear(
    ARM_LINEAR_RX_PIN, ARM_LINEAR_TX_PIN);
HardwareSerial SerialServo(SERVO_RX_PIN, SERVO_TX_PIN);

constexpr uint8_t GRIPPER_SERVO_ID = 4U;
constexpr uint8_t STORAGE_SERVO_ID = 5U;
FSUS_Protocol servoProtocol;
FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &servoProtocol);
FSUS_Servo storageServo(STORAGE_SERVO_ID, &servoProtocol);

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
 * M5 的200步/圈、16细分、5:1减速比及物理方向统一由
 * ArmMotorController 管理。库角度正值为逆时针、负值为顺时针：
 *   上电/发车/行驶姿态 = 旧坐标0°；
 *   标准作业姿态       = 从上电姿态顺时针90° = 旧坐标-90°。
 * 两种姿态下M6都回到离机械硬限位10 mm的工作零点，M7升到最高。
 */
constexpr int32_t ARM_BASE_TRAVEL_OLD_FRAME_DEGREES = 0;
constexpr int32_t ARM_BASE_STANDARD_OLD_FRAME_DEGREES = -90;
constexpr float ARM_BASE_TRAVEL_STANDARD_FRAME_DEGREES = 90.0f;
constexpr int32_t ARM_BASE_HOME_ANGLE_DEGREES =
    ARM_BASE_TRAVEL_OLD_FRAME_DEGREES;
// 容器夹取/释放位：相对标准状态顺时针102°（标准坐标顺时针为负）。
constexpr float ARM_CONTAINER_CLOCKWISE_DEGREES = -102.0f;
/*
 * yyq5正式流程对M5使用60000 pulse/s、35000 pulse/s²；短行程受加速度
 * 限制，实际不会瞬间达到最高速度。这里只设置M5动作曲线，不设置全程
 * 运行时长停机限制。
 */
const float ARM_BASE_MAXIMUM_STEP_RATE = 60000.0f;
const float ARM_BASE_STEP_ACCELERATION = 35000.0f;
// M5脉冲到位后的环节衔接等待按原值减半，仍保留50 ms机械消振。
constexpr uint32_t ARM_BASE_SETTLE_MS = 50UL;

// M6：模数1、分度圆直径35 mm的齿轮齿条；M7：T8x12，导程12 mm。
constexpr uint8_t ARM_EXTENSION_ADDRESS = 6U;
constexpr uint8_t ARM_LIFT_ADDRESS = 7U;
constexpr float FULL_STEPS_PER_REVOLUTION = 200.0f;
constexpr float ARM_LINEAR_MICROSTEPS = 16.0f;
const float M6_TRAVEL_PER_REVOLUTION_MM = PI_F * 35.0f;
constexpr float M7_TRAVEL_PER_REVOLUTION_MM = 12.0f;
const float M6_PULSES_PER_MM =
    FULL_STEPS_PER_REVOLUTION * ARM_LINEAR_MICROSTEPS /
    M6_TRAVEL_PER_REVOLUTION_MM;
constexpr float M7_PULSES_PER_MM =
    FULL_STEPS_PER_REVOLUTION * ARM_LINEAR_MICROSTEPS /
    M7_TRAVEL_PER_REVOLUTION_MM;
constexpr uint8_t M6_EXTEND_DIRECTION = 0U;
constexpr uint8_t M6_RETRACT_DIRECTION = 1U;
constexpr uint8_t M7_RAISE_DIRECTION = 1U;
constexpr uint8_t M7_LOWER_DIRECTION = 0U;
constexpr uint16_t M6_SPEED_RPM = 290U;
constexpr uint8_t M6_ACCELERATION = 246U;
// 上电前人工把M6推到机械回缩端、M7推到物理最高点；上电后两轴各离开
// 机械端点10 mm，再把到达位置共同设为运行期间的安全工作零点。
constexpr float M6_STARTUP_WORKING_ZERO_OFFSET_MM = 10.0f;
constexpr float M7_STARTUP_WORKING_ZERO_OFFSET_MM = 10.0f;
constexpr uint16_t ARM_LINEAR_STARTUP_ZERO_SPEED_RPM = 120U;
constexpr uint8_t ARM_LINEAR_STARTUP_ZERO_ACCELERATION = 128U;
constexpr uint32_t ARM_LINEAR_POSITION_READ_TIMEOUT_MS = 120UL;
constexpr uint16_t M7_SPEED_RPM = 2496U;
constexpr uint8_t M7_ACCELERATION = 255U;
/*
 * 最终物理下降深度仍是155 mm。只把最后15 mm降速到正常速度约60%，
 * 不再插入额外停顿；既减少物料落下时的冲击，也避免明显增加比赛时间。
 */
constexpr float RING_PLACE_FINAL_DESCENT_MM = 15.0f;
constexpr uint16_t M7_RING_PLACE_SPEED_RPM = 1500U;
constexpr uint8_t M7_RING_PLACE_ACCELERATION = 192U;
constexpr uint32_t RING_PLACE_EXTENSION_SETTLE_MS = 0UL;
constexpr uint32_t RING_PLACE_LOWER_SETTLE_MS = 0UL;
constexpr uint32_t ARM_LINEAR_POWER_ON_SETTLE_MS = 1500UL;
constexpr uint32_t ARM_AXIS_COMMAND_GUARD_MS = 30UL;
constexpr uint32_t ARM_AXIS_STATUS_INTERVAL_MS = 30UL;
constexpr uint32_t ARM_AXIS_MINIMUM_ON_POSITION_MS = 120UL;
constexpr uint32_t ARM_AXIS_MINIMUM_TIMEOUT_MS = 5000UL;
constexpr uint32_t ARM_AXIS_MAXIMUM_TIMEOUT_MS = 30000UL;

constexpr float M6_STANDARD_EXTENSION_MM = 0.0f;
constexpr float M7_STANDARD_HEIGHT_MM = 0.0f;
// 物理总行程仍是150 mm；工作零点已离开硬限位10 mm，因此可用行程为140 mm。
constexpr float M6_MAXIMUM_PHYSICAL_EXTENSION_MM = 150.0f;
constexpr float M6_MAXIMUM_EXTENSION_MM =
    M6_MAXIMUM_PHYSICAL_EXTENSION_MM -
    M6_STARTUP_WORKING_ZERO_OFFSET_MM;
// M7物理总下降行程仍为160 mm；新零点已在最高点下方10 mm，因此工作
// 坐标允许从0继续下降150 mm。
constexpr float M7_MINIMUM_PHYSICAL_HEIGHT_MM = -160.0f;
constexpr float M7_MINIMUM_HEIGHT_MM =
    M7_MINIMUM_PHYSICAL_HEIGHT_MM +
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float ARM_AXIS_POSITION_TOLERANCE_MM = 0.05f;
// 以下数值先保留原来的实际物理高度，再减去M7上电已下降的10 mm，确保
// 更换工作零点后相机、夹取和放料的最终物理高度不变。
constexpr float RAW_PICK_PHYSICAL_LOWER_MM = 73.0f;
constexpr float HOUGH_VISION_PHYSICAL_LOWER_MM = 90.0f;
// 斜着找到端点后再多下降15 mm做最终居中，圆在画面中更大、更稳定。
constexpr float ENDPOINT_FINE_VISION_PHYSICAL_LOWER_MM =
    105.0f;
constexpr float CONTAINER_PHYSICAL_LOWER_MM = 40.0f;
constexpr float PROCESS_PLACE_PHYSICAL_LOWER_MM = 155.0f;
constexpr float STORAGE_ROUND1_PLACE_PHYSICAL_LOWER_MM =
    155.0f;
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
constexpr float CONTAINER_PICK_LOWER_MM =
    CONTAINER_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float CONTAINER_PLACE_LOWER_MM =
    CONTAINER_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
// 临时实车试验的物理下降仍是155 mm；新工作坐标中的命令值为145 mm。
constexpr float PROCESS_PLACE_LOWER_MM =
    PROCESS_PLACE_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float STORAGE_ROUND1_PLACE_LOWER_MM =
    STORAGE_ROUND1_PLACE_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;
constexpr float STORAGE_ROUND2_PLACE_LOWER_MM =
    STORAGE_ROUND2_PLACE_PHYSICAL_LOWER_MM -
    M7_STARTUP_WORKING_ZERO_OFFSET_MM;

// 实机夹爪标定：闭合99°、打开37°；最大张开档暂保留但当前未调用。
constexpr float GRIPPER_CLOSE_ANGLE_DEGREES = 99.0f;
constexpr float GRIPPER_OPEN_ANGLE_DEGREES = 37.0f;
constexpr float GRIPPER_MAX_OPEN_ANGLE_DEGREES = -90.0f;
constexpr uint16_t GRIPPER_INTERVAL_MS = 100U;
constexpr uint16_t GRIPPER_OPEN_POWER_MW = 0U;
constexpr uint16_t GRIPPER_CLOSE_POWER_MW = 2000U;
// 夹爪100 ms角度命令之后的动作衔接等待按原值减半。
constexpr uint32_t GRIPPER_OPEN_SETTLE_MS = 175UL;
constexpr uint32_t GRIPPER_CLOSE_SETTLE_MS = 225UL;
constexpr uint16_t STORAGE_SERVO_INTERVAL_MS = 200U;
constexpr uint32_t STORAGE_SERVO_SETTLE_MS = 300UL;
// 上电和底盘行驶保持165°；作业时依次使用-5°、-95°、-185°。
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

constexpr uint8_t MAIXCAM_STOP_REQUEST = 0x00;
constexpr uint8_t MAIXCAM_ALL_COLORS_REQUEST = 0x08;
constexpr uint8_t MAIXCAM_HOUGH_CIRCLE_REQUEST = 0x09;
// 模式10只在机械臂已移动到1号或3号名义搜索位后寻找唯一主外环。
// 圆号由STM32当前扫描阶段赋予，不依赖圆内印刷数字。
constexpr uint8_t MAIXCAM_ENDPOINT_CIRCLE_REQUEST = 0x0A;
constexpr uint32_t MAIXCAM_REQUEST_REPEAT_MS = 1000UL;
constexpr uint32_t MAIXCAM_MODE_SWITCH_GUARD_MS = 100UL;
constexpr uint32_t MAIXCAM_COORDINATE_STALE_MS = 1500UL;
constexpr size_t MAIXCAM_LINE_CAPACITY = 96U;
constexpr int16_t IMAGE_CENTER_X = 160;
constexpr int16_t IMAGE_CENTER_Y = 120;
constexpr int16_t IMAGE_MAX_X = 319;
constexpr int16_t IMAGE_MAX_Y = 239;
constexpr float PIXEL_MAPPING_SWITCH_RADIUS_PIXELS = 50.0f;
constexpr float RAW_NEAR_MM_PER_PIXEL = 40.0f / 72.0f;
constexpr float RAW_FAR_MM_PER_PIXEL = 70.0f / 72.0f;
// 实测圆环：实线外径85 mm、实线内径82 mm。
// 视觉定位采用两条实线的中心线直径83.5 mm，对应实际半径41.75 mm。
constexpr float RING_OUTER_DIAMETER_MM = 85.0f;
constexpr float RING_INNER_DIAMETER_MM = 82.0f;
constexpr float RING_CENTERLINE_DIAMETER_MM =
    0.5f *
    (RING_OUTER_DIAMETER_MM + RING_INNER_DIAMETER_MM);
constexpr float RING_PHYSICAL_RADIUS_MM =
    0.5f * RING_CENTERLINE_DIAMETER_MM;
constexpr float CIRCLE_MM_PER_PIXEL =
    RING_PHYSICAL_RADIUS_MM / 72.0f;
// 永久实测机械尺寸：M6抵住完全回缩端时，M5转轴到相机光轴为125.74 mm。
constexpr float ARM_PIVOT_TO_CAMERA_FULLY_RETRACTED_MM =
    125.74f;
// 当前仍假设完全回缩时夹爪中心与相机光轴具有相同径向距离。
constexpr float ARM_PIVOT_TO_GRIPPER_FULLY_RETRACTED_MM =
    125.74f;
// 后续运动坐标以“物理回缩端前方10 mm”为M6工作零点，所以视觉和夹爪
// 逆运动学的零伸出基准距离均需加上这10 mm，永久125.74 mm标定未改变。
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
/*
 * 名义±150 mm只负责第一次斜着找到端点。找到后底盘完全冻结，M5/M6不
 * 回-80°、不回零，直接在当前位置原地闭环压中；粗观察压到6 px内后再
 * 下降15 mm，最终要求连续两个稳定结果都在2 px内才记录。
 */
constexpr float ENDPOINT_COARSE_CENTER_TOLERANCE_PIXELS = 6.0f;
constexpr float ENDPOINT_FINAL_CENTER_TOLERANCE_PIXELS = 2.0f;
constexpr uint8_t ENDPOINT_FINAL_CENTER_CONFIRMATIONS = 2U;
constexpr uint8_t ENDPOINT_MAXIMUM_SERVO_MOVES = 5U;
constexpr float ENDPOINT_COARSE_SERVO_GAIN = 0.85f;
constexpr float ENDPOINT_FINE_SERVO_GAIN = 0.55f;
constexpr float ENDPOINT_COARSE_MAXIMUM_CORRECTION_MM = 25.0f;
constexpr float ENDPOINT_FINE_MAXIMUM_CORRECTION_MM = 6.0f;
constexpr uint32_t ENDPOINT_LOCAL_MOVE_SETTLE_MS = 80UL;
constexpr float RING_TARGET_MINIMUM_ANGLE_DEGREES = -75.0f;
constexpr float RING_TARGET_MAXIMUM_ANGLE_DEGREES = 75.0f;
constexpr float RING_MAP_MAXIMUM_HEADING_DRIFT_DEGREES = 1.0f;
constexpr float RING_ENDPOINT_MAXIMUM_SCAN_HEADING_DELTA_DEGREES =
    0.75f;
// 端点搜圆时M5只允许在标准坐标-80°～+80°内运动；搜圆预置从-80°开始。
// 该范围与容器夹取角度独立，避免夹取标定变化影响低头搜圆。
constexpr float RING_SCAN_MINIMUM_ANGLE_DEGREES = -80.0f;
constexpr float RING_SCAN_MAXIMUM_ANGLE_DEGREES = 80.0f;
constexpr float RING_SCAN_PRELOAD_ANGLE_DEGREES =
    RING_SCAN_MINIMUM_ANGLE_DEGREES;
/*
 * 相机安装实机标定：
 *   画面+x（向右）对应M5顺时针；
 *   画面+y（向下）对应机械臂向内、M6缩短，因此到“机械臂向外”的
 *   映射符号为-1。
 */
constexpr float IMAGE_Y_TO_ARM_OUTWARD_SIGN = -1.0f;
/*
 * 标准姿态下机械臂沿车体2、4侧伸出：
 *   画面+x（向右）对应车体3、4侧，故底盘forward为负；
 *   画面纵向先按上面的实机标定换算为“机械臂向外”为正；
 *   机械臂向外对应body-left为负。
 * 若实机图像被镜像，只改这两个符号，不改底盘既有标定参数。
 */
constexpr float IMAGE_X_TO_BODY_FORWARD_SIGN = -1.0f;
constexpr float IMAGE_Y_TO_BODY_LEFT_SIGN = -1.0f;
constexpr float CIRCLE_CENTER_TOLERANCE_PIXELS = 3.0f;
/*
 * 模式9仍要求同一物理圆稳定0.5 s；端点模式10使用180 ms、至少3帧且
 * 坐标/半径跨度不超过2 px的快速反馈。最终是否真正居中由STM32连续两次
 * 2 px门槛确认，运动期间则停止相机请求，避免读取机械臂运动中的旧结果。
 */
constexpr uint32_t CIRCLE_CENTER_STABILITY_MS = 0UL;
constexpr uint8_t CIRCLE_STABILITY_MINIMUM_SAMPLES = 1U;
constexpr uint32_t VISION_STABILITY_MAXIMUM_SAMPLE_GAP_MS =
    1200UL;
constexpr uint8_t RAW_MAIN_CONFIRMATION_SAMPLES = 2U;
constexpr int16_t RAW_MAIN_CONFIRMATION_MAX_DELTA_PIXELS = 4;
constexpr uint32_t RAW_MAIN_CONFIRMATION_MAXIMUM_GAP_MS = 2500UL;
constexpr float MAXIMUM_VISUAL_CORRECTION_MM = 150.0f;
constexpr float MAXIMUM_ACCUMULATED_VISUAL_CORRECTION_MM = 220.0f;
constexpr uint8_t MAXIMUM_VISUAL_CORRECTION_MOVES = 6U;
/*
 * 配套Vision 5模式9优先拟合三圆整体并返回中间2号圆；低头后只完整看到
 * 中间圆时，可使用带中心距离和歧义门槛的受限回退。因此不再通过暂存区
 * 固定-30 mm位移“诱导最近圆”，也不会无条件把画面最近圆当作2号基准。
 */

/*
 * M1～M4 电机安装方向。前进时原始驱动脉冲符号为 [-,+,-,+]。
 * 注意不要混淆两层符号：
 *   运动学“物理轮”[-,+,-,+] = 车体逆时针旋转；
 *   乘安装方向后，逆时针对应的“驱动原始脉冲”是 [+,+,+,+]。
 */
const WheelDirections MOTOR_DIRECTIONS(-1, +1, -1, +1);

/*
 * 场地与车辆均按毫米建模。以下布局采用规则图1的左下角为原点：
 *
 *                         +Y / 北
 *                原料区（上边中部）
 *                         |
 *   暂存区（左边中部）-- 场地中心 -- 二维码板（右边中部）
 *                         |
 *              粗加工区（下边中部）   启停区2（右下）
 *                                           启停区1位于右上
 *
 * 该示意只说明地点关系，不表示代码具备地图定位或随机点位识别能力。
 */
constexpr uint16_t FIELD_SIZE_MM = 2400U;
constexpr uint16_t FIELD_CENTER_MM = FIELD_SIZE_MM / 2U;
constexpr uint16_t START_ZONE_SIZE_MM = 300U;
constexpr uint16_t START_ZONE_MIN_X_MM =
    FIELD_SIZE_MM - START_ZONE_SIZE_MM;                      // 2100
constexpr uint16_t START_ZONE_1_MIN_Y_MM =
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
 * 当前代码把三个目标都固定为每组三个位置中的中间位置，并把机械臂
 * 目标点设置为距对应场地边界40 mm。
 *
 * 规则/仿真差异（仅记录，不在本次改数值）：
 *   - 规则图3的粗加工区、暂存区台面深150 mm，名义圆心线在深度中线，
 *     即距场地边界约75 mm；
 *   - 三个圆环的数字1、2、3均朝向场地中心；检查机械臂放料位序时，
 *     应从场地中心侧读取编号，不能按车体当前朝向自行重排；
 *   - route-simulator也使用75 mm目标点；
 *   - 本文件继续保留40 mm实调假设，因此车心坐标比仿名义点更靠边35 mm。
 * 原料区是转动圆盘且位置允许随机，40 mm同样只是本代码固定测试假设，
 * 不是规则给出的原料固定坐标。
 *
 * 机械臂中心相对车体几何中心沿2、4侧偏移225 mm。
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
 * 规则图把原料区横向位置标在1100~1300 mm范围内，本代码固定取1200 mm，
 * 不会根据现场随机位置自动改X坐标。二维码板也可能随机摆放，但本路径
 * 同样固定在右边中线附近扫码。
 *
 * 两个启停区均已参数化：车体贴各区左边，并分别贴上/下场地边界放置，
 * 设计起点中心为(2215,2250)或(2215,150)。这仍是假定人工摆放正确的
 * 名义坐标，不是上电后的X/Y定位结果。
 */
static_assert(FIELD_CENTER_MM == 1200U, "Field centerline must be 1200 mm");
constexpr uint16_t START_CENTER_X_MM =
    START_ZONE_MIN_X_MM + CHASSIS_FOOTPRINT_X_MM / 2U;       // 2215
constexpr uint16_t START_ZONE_1_CENTER_Y_MM =
    START_ZONE_1_MIN_Y_MM + CHASSIS_FOOTPRINT_Y_MM / 2U;     // 2250
constexpr uint16_t START_ZONE_2_CENTER_Y_MM =
    CHASSIS_FOOTPRINT_Y_MM / 2U;                             // 150
// 回区时瞄准300×300启停区几何中心，而不是沿用贴边发车的X坐标。
constexpr uint16_t FINAL_ZONE_CENTER_X_MM =
    FIELD_SIZE_MM - START_ZONE_SIZE_MM / 2U;                 // 2250

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
 * 启停区1沿3、4侧向南、启停区2沿1、2侧向北，各走1050 mm到达右边
 * 中部二维码区。规则要求先读取任务码再到原料区；二维码是继续路线的
 * 门控。调试版收到一次完整且校验通过的任务码即锁定，不重复扫码；
 * 扫码点若尚无有效码，会沿进入方向低速前探并按
 * 保存的四轮绝对脉冲原路回位；
 * route[]的起终点仍不变，机械臂按四组任务码建立颜色到容器槽位的映射，
 * 并确定粗加工环位及第二批同色码垛位置。
 *
 * route-simulator仍标注“二维码区不扫码、不停车”，这是旧tmcode1仿真行为，
 * 与本文件的COMMAND_QR_ACTION前探/回位逻辑不同；调试应以本文件为准。
 *
 * 回程仍使用X=2150的安全转弯带；若在起点中心X=2215原地转180°，
 * 230x300 mm车体的外接圆会越过场地右边界约4 mm。
 */
constexpr uint16_t QR_PASS_CENTER_X_MM = START_CENTER_X_MM;  // 2215
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
/*
 * 起步Y方向实车手调只改这一行：
 *   走过头多少毫米就减多少，没走够多少毫米就加多少。
 * QR_PASS_TO_RAW_APPROACH_MM会自动反算，原料区最终坐标不会被带偏。
 */
constexpr uint16_t START_TO_QR_PASS_MM = 1050U;
constexpr uint16_t QR_PASS_CENTER_Y_MM = FIELD_CENTER_MM;    // 1200
static_assert(
    START_ZONE_1_CENTER_Y_MM - QR_PASS_CENTER_Y_MM ==
            START_TO_QR_PASS_MM &&
        QR_PASS_CENTER_Y_MM - START_ZONE_2_CENTER_Y_MM ==
            START_TO_QR_PASS_MM,
    "Both start zones must be symmetric around the QR row");
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
constexpr uint16_t RETURN_LANE_TO_FINAL_ZONE_X_MM =
    FINAL_ZONE_CENTER_X_MM - RETURN_LANE_X_MM;               // 100
/*
 * 第二批暂存动作点受两轮既有Y补偿影响，开环推算为1110 mm。
 * 两个回区纵移都从这里计算到启停区几何中心；最终仍必须由真实定位/
 * 边线传感器确认，不能把该推算当作实测位姿。
 */
constexpr uint16_t STORAGE_ROUND2_OPEN_LOOP_Y_MM =
    FIELD_CENTER_MM - 90U;                                   // 1110
constexpr uint16_t RETURN_TO_START_ZONE_1_Y_MM =
    START_ZONE_1_CENTER_Y_MM - STORAGE_ROUND2_OPEN_LOOP_Y_MM; // 1140
constexpr uint16_t RETURN_TO_START_ZONE_2_Y_MM =
    STORAGE_ROUND2_OPEN_LOOP_Y_MM - START_ZONE_2_CENTER_Y_MM; // 960

// 本轮所有底盘直线速度均在修改前的当前值上再提高30%。
const float MAXIMUM_STEP_RATE = 7150.0f;
const float CENTRAL_CHANNEL_MAXIMUM_STEP_RATE = 9295.0f;
// 普通/中央通道等非矫正直线加速度由当前5000减半。
const float STEP_ACCELERATION = 2500.0f;
// 底盘粗转和IMU航向纠偏角速度保持不变；仅粗转角加速度由1600减半。
const float TURN_MAXIMUM_STEP_RATE = 3000.0f;
const float TURN_STEP_ACCELERATION = 800.0f;
const float HEADING_CORRECTION_MAXIMUM_STEP_RATE = 1500.0f;
const float HEADING_CORRECTION_STEP_ACCELERATION = 450.0f;
const float WORKSTATION_MAXIMUM_STEP_RATE = 2080.0f;
const float WORKSTATION_STEP_ACCELERATION = 700.0f;
const float FINAL_MAXIMUM_STEP_RATE = 1040.0f; // 最后进入300×300启停区
const float FINAL_STEP_ACCELERATION = 400.0f;
// 扫码区低速前探约104 mm/s，最大只走500 mm。
constexpr uint16_t QR_SCAN_SWEEP_MAXIMUM_MM = 500U;
const float QR_SCAN_MAXIMUM_STEP_RATE = 1040.0f;
const float QR_SCAN_STEP_ACCELERATION = 400.0f;
const uint16_t MINIMUM_STEP_WIDTH_US = 2U;

/*
 * 非中央通道仍保留原1.1 m分段上限。中央通道允许单段最多2.0 m，
 * 覆盖当前最长的1885 mm同向直线，使原料区到粗加工区的1740/1760 mm
 * 以及最终1885 mm回程均不中途停车；转角处仍按独立路线命令停稳。
 */
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
const uint32_t MOTION_TIMEOUT_MS = 20000UL;
const uint32_t TURN_TIMEOUT_MS = 16000UL;
const uint32_t IMU_STALE_TIMEOUT_MS = 600UL;
const bool ENABLE_MOTION_TIMEOUTS = true;
constexpr uint8_t MAXIMUM_TURN_CORRECTIONS = 12U;
constexpr uint32_t QR_SCAN_ACTION_TIMEOUT_MS = 20000UL;
// 当前扫码器按用户要求单帧锁定；如现场串口干扰明显可改回2。
constexpr uint8_t QR_REQUIRED_MATCHING_FRAMES = 1U;
// false：任务内哪个未抓颜色先稳定就先抓，并放入该颜色的二维码槽位。
// true：严格等待二维码颜色顺序；正式赛前按当届规则确认。
constexpr bool REQUIRE_RAW_PICK_QR_ORDER = false;
constexpr uint32_t VISION_RESULT_TIMEOUT_MS = 12000UL;
constexpr uint8_t VISION_MAXIMUM_RETRIES = 2U;
constexpr uint32_t RAW_ACTION_TIMEOUT_MS = 45000UL;
constexpr uint32_t PROCESS_ACTION_TIMEOUT_MS = 65000UL;
constexpr uint32_t STORAGE_ACTION_TIMEOUT_MS = 45000UL;
constexpr uint32_t MISSION_PROGRESS_TIMEOUT_MS = 45000UL;
// 调试版关闭整场180秒硬停；正式比赛前改为true。
constexpr bool ENABLE_COMPETITION_TIME_LIMIT = false;
constexpr uint32_t COMPETITION_TIME_LIMIT_MS = 180000UL;
constexpr uint32_t COMPETITION_HARD_STOP_MARGIN_MS = 1500UL;
static_assert(
    COMPETITION_HARD_STOP_MARGIN_MS < COMPETITION_TIME_LIMIT_MS,
    "Competition hard-stop margin must be smaller than the time limit");

// +1：传感器逆时针角度增加；若以后更换安装方向，可改为 -1。
const int8_t IMU_COUNTERCLOCKWISE_SIGN = +1;

/*
 * “只检查路径”模式：原料区、粗加工区、暂存区动作都只等待750 ms，
 * 不会实际完成规则要求的抓取、随车承载、放置和第二批同色码垛。
 * COMMAND_FINAL_ALIGN也会直接判定完成。因此该模式只能调底盘路线，
 * 不能用来验证完整物流任务。
 */
const bool PATH_ONLY_TEST = false;

/*
 * src/main.cpp或visionyanyan构建环境把该宏设为1。测试模式跳过扫码和
 * 整场路线，默认料盘槽0/1/2已装料，只做粗加工区1/3端点建图、推算2号
 * 并依次放置，随后停机等待人工测量。正式比赛入口必须把该宏设为0。
 */
constexpr bool VISION_YANYAN_TEST_MODE =
    GONGCHUANG_VISION_YANYAN_TEST != 0;

// 恢复二维码接收：到达扫码位置后，必须收到有效任务码才继续路线。
const bool ENABLE_QR_RECEIVER = true;
const bool REQUIRE_QR_SUCCESS = true;

// 仅用于环节衔接/路径测试的等待按原值减半。
const uint32_t QR_TEST_HOLD_MS = 500UL;
const uint32_t WORKSTATION_TEST_HOLD_MS = 750UL;
const uint32_t FINAL_HOLD_MS = 1500UL;

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
ArmMotorController armMotors;
/*
 * 一次运行结束或急停后禁止再次给M5发运动命令，直到MCU重新上电。
 * 这样可以保留原PB9回调主体，同时避免终点/故障姿态下误触导致扫碰。
 */
bool armBaseMotionLockedUntilReset = false;

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
  // 规则示例完整码：134+123+314+231
  // t3：134+123+（8字符）
  // t8：314+231 （7字符）
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

void startArmBaseLibraryDegrees(float libraryDegrees) {
  if (armBaseMotionLockedUntilReset) {
    SerialDebug.println(
        "M5 command ignored until power-cycle reset");
    return;
  }
  armMotors.moveM5ToDegrees(libraryDegrees, false);

  SerialDebug.print("Arm base library target: ");
  SerialDebug.print(libraryDegrees, 1);
  SerialDebug.print(" deg, pulses=");
  SerialDebug.println(
      armMotors.m5PulsesForDegrees(libraryDegrees));
}

void startArmBaseRotationToDegrees(float oldFrameDegrees) {
  /*
   * M5库已按实机恢复为“正角逆时针、负角顺时针”；旧坐标与库坐标
   * 使用相同符号。旧0°是上电/行驶姿态，旧-90°是标准作业姿态。
   */
  startArmBaseLibraryDegrees(oldFrameDegrees);
}

void startArmBaseStandardFrameDegrees(float angleDegrees) {
  /*
   * 工位机械臂采用“旧-90°=新0°”坐标。
   * 新坐标正角为逆时针、负角为顺时针。
   */
  const float standardLibraryDegrees =
      static_cast<float>(
          ARM_BASE_STANDARD_OLD_FRAME_DEGREES);
  startArmBaseLibraryDegrees(
      standardLibraryDegrees + angleDegrees);
}

void stopArmBaseImmediately() {
  armMotors.stopM5Immediately();
}

void disableArmBaseMotor() {
  stopArmBaseImmediately();
  armMotors.disableM5();
  armBaseMotionLockedUntilReset = true;
}

void setDriveMotionProfile(float maximumStepRate, float acceleration) {
  for (uint8_t i = 0; i < 4; ++i) {
    motors[i]->setMaxSpeed(maximumStepRate);
    motors[i]->setAcceleration(acceleration);
  }
}

// ---------------------------------------------------------------------------
// 省赛物流决赛第一阶段：固定两批路线
// ---------------------------------------------------------------------------

/*
 * 这里不是根据障碍物或现场点位实时搜索得到的动态路径规划，而是：
 *   固定RouteCommand表 -> 逐条启动 -> 等待完成 -> 推进到下一条。
 *
 * 四种平移命令按车体侧面命名，方向会随车体姿态一起旋转：
 *   SIDE_12：朝1、2侧，运动学forward为正；
 *   SIDE_34：朝3、4侧，运动学forward为负；
 *   SIDE_13：朝1、3侧，运动学left为正；
 *   SIDE_24：朝2、4侧，运动学left为负。
 *
 * 所以下面注释中的东/西/南/北，必须结合执行该命令前的车体朝向理解；
 * 不能把SIDE_24永久等同为某个世界坐标方向。
 */

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

enum RouteBinding {
  ROUTE_BINDING_FIXED,
  ROUTE_BINDING_START_TO_QR,
  ROUTE_BINDING_RETURN_TO_START_ROW
};

struct RouteCommand {
  CommandType type;
  int32_t value;
  const char *name;
  bool preciseArrival;
  bool centralChannel;
  RouteBinding binding;

  RouteCommand(
      CommandType commandType,
      int32_t commandValue,
      const char *commandName,
      bool precise = false,
      bool central = false,
      RouteBinding routeBinding = ROUTE_BINDING_FIXED)
      : type(commandType),
        value(commandValue),
        name(commandName),
        preciseArrival(precise),
        centralChannel(central),
        binding(routeBinding) {}
};

/*
 * 几何设计层的关键工位姿态（单位 mm，角度表示2、4侧朝向）：
 *
 *   启停区1起点：(2215,2250)，启停区2起点：(2215,150)，2、4侧朝东
 *   对应终点：  (2250,2250)/(2250,150)，2、4侧朝西
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
 * route-simulator的旧tmcode1名义车心是(1200,2100)、(1200,300)、
 * (300,1200)，与这里的2135/265/265不同；仿真只用于理解运动顺序。
 *
 * 更重要的是，route[]中保留了+20、+40、-20、-40、-50等
 * 实车补偿。若从起点严格按现有命令值开环累计，若无打滑，调试推算点为：
 *   扫码点                 (2215,1200)
 *   第1批原料动作点        (1200,2135)
 *   第1批粗加工动作点      (1200, 245)
 *   第1批暂存动作点        ( 265,1130)
 *   第2批原料动作点        (1200,2105)
 *   第2批粗加工动作点      (1200, 215)
 *   第2批暂存动作点        ( 265,1110)
 *   启停区1/2名义终点       (2250,2250)/(2250,150)
 * 这些是“按代码值推算”，不是修改建议，也不是定位传感器测量结果。
 * 因此命令名称中的field center/start zone表示路线意图，未必等于实测坐标。
 *
 * 新车头是3、4侧。第一段向3、4侧移动就是实车“前进”，但在旧运动学
 * 坐标中等价于负 forward，所以路径命令不再使用 forward/backward 命名。
 */
const RouteCommand route[] = {
    /*
     * PB9后机械臂保持上电/发车姿态：M5旧坐标0°，M6安全工作零点、
     * M7安全工作零点（物理最高点下方10 mm）；
     * 到各工位且底盘停稳后，作业状态机才把M5顺时针转90°进入标准状态。
     */
    /*
     * 规则阶段：从抽签启停区出发，到二维码板读取四组三位任务码。
     * 启动前双击PB9选择启停区1/2；两区以相反方向各走1050 mm到
     * 右边中部扫码点(2215,1200)。若尚未收到一次完整有效任务码，
     * COMMAND_QR_ACTION沿各自进入方向低速前探最多500 mm，锁码后按
     * 四轮实际脉冲回到该停车点，再继续后续路线。
     * 若此时IMU尚无有效帧，机械臂保持行驶姿态，底盘停在IMUWAIT。
     */
    {COMMAND_MOVE_SIDE_34_MM, START_TO_QR_PASS_MM,
     "Selected start zone -> QR area direct", false, false,
     ROUTE_BINDING_START_TO_QR},
    {COMMAND_QR_ACTION, 0, "Scan QR task code"},

    // -------------------- 第一批三个物料（代码中的第1轮） --------------------
    /*
     * 扫码点 -> 原料区：
     * 先向1、3侧横移到X=1200，再向1、2侧到原料区前；逆时针转90°后，
     * 2、4侧朝北，最后150 mm低速进入第1批原料动作点(1200,2135)。
     *
     * 规则动作含义：按任务码第一组三位颜色顺序，每次抓取1个，并先把
     * 物料放到机器人载物位置，之后才能抓下一个；不得用手爪夹持跨区运输。
     * 原料台是每6~10 s旋转一周后随机停止的三工位转盘；
     * COMMAND_RAW_ACTION以模式8请求同时识别四色；MaixCAM确认某一颜色
     * 坐标连续0.3 s横、纵跨度各不超过3像素后，只计算一次M5/M6目标；
     * 载物盘按该颜色在本批二维码序列中的槽位收纳，不移动底盘。
     */
    {COMMAND_MOVE_SIDE_13_MM, QR_PASS_TO_FIELD_CENTER_X_MM,
     "QR area -> raw centerline", false, true},
    {COMMAND_MOVE_SIDE_12_MM,
     QR_PASS_TO_RAW_APPROACH_MM,
     "QR Y=1200 -> raw approach round 1", false, true},
    {COMMAND_TURN_COUNTERCLOCKWISE_DEGREES, 90,
     "Face side 2,4 north"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM,
     "Precise entry to raw round 1", true},
    {COMMAND_RAW_ACTION, 1, "Raw action round 1"},

    /*
     * 第1批原料区 -> 粗加工区：
     * 向1、3侧长距离南移；该1740 mm中央通道直线现在一段直达，
     * 仅在终点IMU回正。随后逆时针180°，让2、4侧朝南，最后150 mm低速进入。
     * 当前+20 mm为已有实调补偿，按命令累计动作点约为(1200,245)。
     *
     * 规则动作含义：把第一批三个物料按任务码第二组三位指定的1/2/3号
     * 圆环位置放到粗加工区；随后还要按任务码顺序重新抓回车上。
     * COMMAND_PROCESS_ACTION同时代表“放下3个并重新抓回3个”的接口。
     */
    {COMMAND_MOVE_SIDE_13_MM,
     RAW_TO_PROCESS_MM - WORKSTATION_APPROACH_MM + 20U,
     "Raw -> process approach round 1", false, true},
    {COMMAND_TURN_COUNTERCLOCKWISE_DEGREES, 180,
     "Face side 2,4 south"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM,
     "Precise entry to process round 1", true},
    {COMMAND_PROCESS_ACTION, 1, "Process action round 1"},

    /*
     * 第1批粗加工区 -> 暂存区：
     * 先向1、3侧北移，再向3、4侧西移至工位前；顺时针90°使2、4侧
     * 朝西，最后150 mm低速进入。受现有-50 mm补偿影响，按命令累计
     * 暂存动作点约为(265,1130)，不是几何注释中的(265,1200)。
     *
     * 规则动作含义：将第一批三个物料按任务码第二组三位规定的位置，
     * 平面放置到暂存区，为第二批同色码垛留下底层。
     */
    {COMMAND_MOVE_SIDE_13_MM, PROCESS_TO_CENTER_MM - 50U,
     "Process -> field center round 1", false, true},
    {COMMAND_MOVE_SIDE_34_MM,
     CENTER_TO_STORAGE_MM - WORKSTATION_APPROACH_MM,
     "Center -> storage approach round 1", false, true},
    {COMMAND_TURN_CLOCKWISE_DEGREES, 90,
     "Face side 2,4 west"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM,
     "Precise entry to storage round 1", true},
    {COMMAND_STORAGE_ACTION, 1, "Storage action round 1"},

    // -------------------- 第二批三个物料（代码中的第2轮） --------------------
    /*
     * 暂存区 -> 原料区：
     * 保持2、4侧朝西，先向1、3侧回到X=1200，再向3、4侧北移；
     * 顺时针90°使2、4侧朝北，最后以150+40 mm进入。+40 mm是现有
     * 实调补偿，按命令累计第2批原料动作点约为(1200,2105)。
     *
     * 规则动作含义：按任务码第三组三位颜色顺序抓取第二批三个物料，
     * 同样必须逐个放到机器人上后才能跨区运输。
     */
    {COMMAND_MOVE_SIDE_13_MM, STORAGE_TO_CENTER_MM,
     "Storage -> field center round 2", false, true},
    {COMMAND_MOVE_SIDE_34_MM,
     CENTER_TO_RAW_MM - WORKSTATION_APPROACH_MM,
     "Field center -> raw approach round 2", false, true},
    {COMMAND_TURN_CLOCKWISE_DEGREES, 90,
     "Face side 2,4 north"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM + 40U,
     "Precise entry to raw round 2", true},
    {COMMAND_RAW_ACTION, 2, "Raw action round 2"},

    /*
     * 第2批原料区 -> 粗加工区：
     * 长移段+40 mm、精靠段-20 mm均为已有实调补偿；逆时针180°后
     * 2、4侧朝南，按命令累计粗加工动作点约为(1200,215)。
     *
     * 规则动作含义：按任务码第四组三位指定的圆环位置放下第二批，
     * 再按搬运顺序重新抓回车上，准备送往暂存区。
     */
    {COMMAND_MOVE_SIDE_13_MM,
     RAW_TO_PROCESS_MM - WORKSTATION_APPROACH_MM + 40U,
     "Raw -> process approach round 2", false, true},
    {COMMAND_TURN_COUNTERCLOCKWISE_DEGREES, 180,
     "Face side 2,4 south"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM - 20U,
     "Precise entry to process round 2", true},
    {COMMAND_PROCESS_ACTION, 2, "Process action round 2"},

    /*
     * 第2批粗加工区 -> 暂存区：
     * 先北移、再西移，顺时针90°让2、4侧朝西后精靠；当前-40 mm
     * 补偿使按命令累计暂存动作点约为(265,1110)。
     *
     * 规则动作含义：第二批只能码垛在第一批已正确放置的同色物料上。
     * COMMAND_STORAGE_ACTION第2批接口应在实际机械臂完成三次码垛后反馈。
     */
    {COMMAND_MOVE_SIDE_13_MM, PROCESS_TO_CENTER_MM - 40U,
     "Process -> field center round 2", false, true},
    {COMMAND_MOVE_SIDE_34_MM,
     CENTER_TO_STORAGE_MM - WORKSTATION_APPROACH_MM,
     "Center -> storage approach round 2", false, true},
    {COMMAND_TURN_CLOCKWISE_DEGREES, 90,
     "Face side 2,4 west"},
    {COMMAND_MOVE_SIDE_24_MM, WORKSTATION_APPROACH_MM,
     "Precise entry to storage round 2", true},
    {COMMAND_STORAGE_ACTION, 2, "Storage action round 2"},

    /*
     * 规则阶段：完成两批后返回启动前锁定的启停区。
     * 两个启停区共用到X=2150回程通道的路线，仅纵向方向/距离参数化：
     *   Zone1：向北1140 mm到Y=2250；
     *   Zone2：向南 960 mm到Y=150。
     * 最后向东100 mm到启停区几何中心X=2250。旧版终点(2255,2210)
     * 会让230×300车体下沿伸出Zone1约40 mm，故删除末尾50/40 mm手调尾巴。
     */
    {COMMAND_MOVE_SIDE_13_MM,
     STORAGE_TO_CENTER_MM + CENTER_TO_RETURN_LANE_MM,
     "Storage -> return lane final direct", false, true},
    {COMMAND_SET_PRECISE_MOTION, 0,
     "Set low speed for selected start-zone entry"},
    {COMMAND_MOVE_SIDE_34_MM, RETURN_TO_START_ZONE_1_Y_MM,
     "Return lane -> selected start-zone row", true, false,
     ROUTE_BINDING_RETURN_TO_START_ROW},
    {COMMAND_MOVE_SIDE_13_MM, RETURN_LANE_TO_FINAL_ZONE_X_MM,
      "Enter selected start-zone center", true},
    {COMMAND_FINAL_ALIGN, 0, "Verify final zone footprint"},
    {COMMAND_HOLD, FINAL_HOLD_MS, "Hold in selected start zone"},
    {COMMAND_ARM_BASE_HOME, ARM_BASE_HOME_ANGLE_DEGREES,
     "Keep arm base at travel old 0"},
    {COMMAND_FINISH, 0, "Finished"}};

const size_t ROUTE_COMMAND_COUNT =
    sizeof(route) / sizeof(route[0]);

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
  QR_SCAN_WAIT_AT_LIMIT,
  QR_SCAN_RETURNING,
  QR_SCAN_COMPLETE
};

ProgramState programState = PROGRAM_WAITING;
StartZone selectedStartZone = START_ZONE_1;
QrScanPhase qrScanPhase = QR_SCAN_IDLE;
MotorPulses qrScanOriginMotorPositions;
uint32_t qrScanActionStartMs = 0UL;
size_t routeIndex = 0;
RouteCommand activeRouteCommand(
    COMMAND_FINISH, 0, "Unresolved route command");
uint8_t activeCompetitionRound = 0;
bool commandStarted = false;
uint32_t commandStartMs = 0;
uint32_t competitionStartMs = 0UL;
uint32_t lastMissionProgressMs = 0UL;
uint32_t headingStableStartMs = 0;
uint32_t motorsArrivedStartMs = 0;
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

// 三个工位状态机完成全部真实动作后设置这些完成标志。
volatile bool rawActionFinished = false;
volatile bool processActionFinished = false;
volatile bool storageActionFinished = false;
volatile bool finalAlignmentFinished = false;

void resetQrScanActionState() {
  qrScanPhase = QR_SCAN_IDLE;
  qrScanOriginMotorPositions = MotorPulses();
  qrScanActionStartMs = 0UL;
}

void markMissionProgress() {
  lastMissionProgressMs = millis();
}

RouteCommand resolveRouteCommand(const RouteCommand &command) {
  RouteCommand resolved = command;
  switch (command.binding) {
    case ROUTE_BINDING_FIXED:
      break;

    case ROUTE_BINDING_START_TO_QR:
      if (selectedStartZone == START_ZONE_2) {
        // Zone2保持与Zone1相同车体朝向，沿1、2侧倒车向北到二维码行。
        resolved.type = COMMAND_MOVE_SIDE_12_MM;
        resolved.name = "Start2 -> QR area direct";
      } else {
        resolved.type = COMMAND_MOVE_SIDE_34_MM;
        resolved.name = "Start1 -> QR area direct";
      }
      resolved.value = START_TO_QR_PASS_MM;
      break;

    case ROUTE_BINDING_RETURN_TO_START_ROW:
      if (selectedStartZone == START_ZONE_2) {
        resolved.type = COMMAND_MOVE_SIDE_12_MM;
        resolved.value = RETURN_TO_START_ZONE_2_Y_MM;
        resolved.name = "Return lane -> Start2 row";
      } else {
        resolved.type = COMMAND_MOVE_SIDE_34_MM;
        resolved.value = RETURN_TO_START_ZONE_1_Y_MM;
        resolved.name = "Return lane -> Start1 row";
      }
      break;
  }
  return resolved;
}

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

void printCurrentCommand(const RouteCommand &command) {
  SerialDebug.print("Route ");
  SerialDebug.print(static_cast<unsigned int>(routeIndex + 1));
  SerialDebug.print("/");
  SerialDebug.print(static_cast<unsigned int>(ROUTE_COMMAND_COUNT));
  SerialDebug.print(": ");
  SerialDebug.println(command.name);
}

void emergencyStopArmLinearAxes();
void stopMaixRequest();
void invalidateArmLinearReference();

void routeFault(const char *reason) {
  resetQrScanActionState();
  invalidateArmLinearReference();
  disableDriveMotors();
  disableArmBaseMotor();
  emergencyStopArmLinearAxes();
  stopMaixRequest();
  programState = PROGRAM_FAULT;
  commandStarted = false;
  hmiSetRunStatus("FAULT");

  SerialDebug.print("FAULT: ");
  SerialDebug.println(reason);
}

// ---------------------------------------------------------------------------
// M6/M7串口步进、舵机与MaixCAM非阻塞底层
// ---------------------------------------------------------------------------

bool deadlineReached(uint32_t deadlineMs) {
  return static_cast<int32_t>(millis() - deadlineMs) >= 0;
}

struct LinearAxisMotion {
  uint8_t address;
  float currentMm;
  float targetMm;
  bool active;
  bool fault;
  bool commandAcknowledged;
  bool motionObserved;
  uint32_t startMs;
  uint32_t lastStatusRequestMs;
  uint32_t timeoutMs;

  LinearAxisMotion(uint8_t axisAddress)
      : address(axisAddress),
        currentMm(0.0f),
        targetMm(0.0f),
        active(false),
        fault(false),
        commandAcknowledged(false),
        motionObserved(false),
        startMs(0UL),
        lastStatusRequestMs(0UL),
        timeoutMs(ARM_AXIS_MINIMUM_TIMEOUT_MS) {}
};

LinearAxisMotion extensionAxis(ARM_EXTENSION_ADDRESS);
LinearAxisMotion liftAxis(ARM_LIFT_ADDRESS);
bool armLinearReferenceValid = false;
bool armLinearSerialInitialized = false;
bool manipulationServosOnline = false;
uint8_t armLinearReceiveWindow[4] = {0U, 0U, 0U, 0U};
uint8_t armLinearReceiveCount = 0U;

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
  /*
   * EMM V5位置帧：
   *   byte10=0 相对运动；byte11=0 非多机同步。
   */
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
      return true;
    }
    delay(1);
  }

  armLinearReceiveCount = 0U;
  return false;
}

void markLinearAxisArrived(LinearAxisMotion &axis) {
  axis.active = false;
  axis.currentMm = axis.targetMm;
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
  axis.active = false;
  axis.fault = true;
  SerialDebug.print("Arm axis M");
  SerialDebug.print(axis.address);
  SerialDebug.print(" fault: ");
  SerialDebug.println(reason);
  routeFault(reason);
}

bool consumeArmLinearEnableResponse(
    LinearAxisMotion &axis) {
  uint8_t window[4] = {0U, 0U, 0U, 0U};
  uint8_t count = 0U;
  bool enableResponseSeen = false;

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
        /*
         * 不在这里自动清堵转，也不把F3 E2直接当作路线故障：
         * 某些固件对“已经使能”重复使能也可能返回条件不满足。继续发送
         * FD，随后由FD结果和3A状态位安全地决定是否停止。
         */
      }
    } else if (
        window[0] == axis.address &&
        window[1] == 0x00U &&
        window[2] == 0xEEU) {
      printArmLinearFrame(
          "enable wrong command", window);
      faultLinearAxis(axis, "EMM enable wrong command");
      armLinearReceiveCount = 0U;
      return false;
    }
    count = 0U;
  }

  armLinearReceiveCount = 0U;
  if (!enableResponseSeen) {
    /*
     * 某些驱动器可配置为不立即回复控制命令；继续发送FD，让FD ACK或
     * 后续3A状态决定成败，但把缺少F3应答明确留在调试串口中。
     */
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
  if (axis == nullptr || !axis->active) {
    return;
  }

  const uint32_t elapsedMs = millis() - axis->startMs;

  if (frame[1] == 0x00U && frame[2] == 0xEEU) {
    printArmLinearFrame("wrong command", frame);
    faultLinearAxis(*axis, "EMM wrong command");
    return;
  }

  if (frame[1] == 0xFDU && frame[2] == 0xE2U) {
    printArmLinearFrame(
        "position condition not met", frame);
    faultLinearAxis(
        *axis,
        "EMM position condition not met");
    return;
  }

  if (frame[1] == 0xFDU && frame[2] == 0x02U) {
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
    if ((flags & 0x01U) == 0U) {
      printArmLinearFrame("axis disabled", frame);
      faultLinearAxis(*axis, "EMM axis is not enabled");
      return;
    }
    if ((flags & 0x0CU) != 0U) {
      printArmLinearFrame("stall/protection", frame);
      faultLinearAxis(*axis, "EMM stall/protection flag");
      return;
    }
    if ((flags & 0x02U) == 0U) {
      axis->motionObserved = true;
      return;
    }
    if ((flags & 0x02U) != 0U &&
        elapsedMs >= ARM_AXIS_MINIMUM_ON_POSITION_MS &&
        (axis->commandAcknowledged ||
         axis->motionObserved)) {
      markLinearAxisArrived(*axis);
    }
    return;
  }

  // 部分EMM固件在相对位置运动结束时主动返回 id FD 9F 6B。
  if (frame[1] == 0xFDU &&
      frame[2] == 0x9FU) {
    markLinearAxisArrived(*axis);
  }
}

void serviceArmLinearAxes() {
  while (SerialArmLinear.available()) {
    const uint8_t incoming =
        static_cast<uint8_t>(SerialArmLinear.read());

    if (armLinearReceiveCount < 4U) {
      armLinearReceiveWindow[armLinearReceiveCount++] =
          incoming;
    } else {
      armLinearReceiveWindow[0] = armLinearReceiveWindow[1];
      armLinearReceiveWindow[1] = armLinearReceiveWindow[2];
      armLinearReceiveWindow[2] = armLinearReceiveWindow[3];
      armLinearReceiveWindow[3] = incoming;
    }

    if (armLinearReceiveCount == 4U) {
      handleArmLinearFrame(armLinearReceiveWindow);
      if (incoming == 0x6BU) {
        armLinearReceiveCount = 0U;
      }
    }
  }

  LinearAxisMotion *const axes[2] = {
      &extensionAxis, &liftAxis};
  const uint32_t nowMs = millis();
  for (uint8_t i = 0U; i < 2U; ++i) {
    LinearAxisMotion &axis = *axes[i];
    if (!axis.active) {
      continue;
    }

    if (nowMs - axis.startMs >= axis.timeoutMs) {
      faultLinearAxis(axis, "EMM motion timeout");
      continue;
    }

    if (nowMs - axis.lastStatusRequestMs >=
        ARM_AXIS_STATUS_INTERVAL_MS) {
      axis.lastStatusRequestMs = nowMs;
      writeArmLinearStatusRequest(axis.address);
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
    uint8_t acceleration) {
  if (axis.fault) {
    routeFault("EMM axis remains faulted");
    return false;
  }
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

  const float deltaMm = targetMm - axis.currentMm;
  axis.targetMm = targetMm;
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

  const float pulsesPerSecond =
      static_cast<float>(speedRpm) *
      FULL_STEPS_PER_REVOLUTION *
      ARM_LINEAR_MICROSTEPS / 60.0f;
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

  /*
   * 与已验证的ArmMotorController时序一致：每次运动前重新使能目标轴，
   * 等待驱动器处理并回复后再发送位置命令。不能在EMM应答前背靠背发送
   * 下一条3A查询，否则部分固件会返回FD E2或漏掉命令。
   */
  clearArmLinearReceiveBuffer();
  writeArmLinearEnable(axis.address, true);
  SerialArmLinear.flush();
  delay(ARM_AXIS_COMMAND_GUARD_MS);
  if (!consumeArmLinearEnableResponse(axis)) {
    return false;
  }
  clearArmLinearReceiveBuffer();

  axis.active = true;
  axis.commandAcknowledged = false;
  axis.motionObserved = false;
  axis.startMs = millis();
  axis.lastStatusRequestMs = axis.startMs;

  writeArmLinearPosition(
      axis.address,
      deltaMm > 0.0f
          ? positiveDirection
          : negativeDirection,
      speedRpm,
      acceleration,
      pulses);
  SerialArmLinear.flush();
  /*
   * 第一条3A状态查询由serviceArmLinearAxes在30 ms保护间隔后发送。
   * 后续只有在收到FD命令ACK、观察过未到位，或收到明确FD 9F到位帧后
   * 才承认完成，防止命令丢失时把旧的on-position状态误当成本次到位。
   */

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

bool startLiftToHeightMm(float heightMm) {
  // 高度以最高点为0，向上为正、向下为负。
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

bool startLiftToHeightMmWithProfile(
    float heightMm,
    uint16_t speedRpm,
    uint8_t acceleration) {
  // 只供需要独立速度曲线的短距离M7动作使用。
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

bool extensionMoveFinished() {
  return !extensionAxis.active && !extensionAxis.fault;
}

bool liftMoveFinished() {
  return !liftAxis.active && !liftAxis.fault;
}

void emergencyStopArmLinearAxes() {
  if (armLinearSerialInitialized) {
    writeArmLinearStop(extensionAxis.address);
    writeArmLinearStop(liftAxis.address);
  }
  extensionAxis.active = false;
  liftAxis.active = false;
}

void resetArmLinearSoftwareOrigin() {
  /*
   * 本机没有M6/M7限位开关寻零：上电前仍需人工确认M6抵住完全回缩端、
   * M7位于物理最高点。这里先把这两个机械端点当作临时零点；只有M6向外
   * 10 mm、M7向下10 mm并分别重新标零成功后，参考才允许置真。
   */
  extensionAxis.currentMm = M6_STANDARD_EXTENSION_MM;
  extensionAxis.targetMm = M6_STANDARD_EXTENSION_MM;
  extensionAxis.active = false;
  extensionAxis.fault = false;
  extensionAxis.commandAcknowledged = false;
  extensionAxis.motionObserved = false;
  liftAxis.currentMm = M7_STANDARD_HEIGHT_MM;
  liftAxis.targetMm = M7_STANDARD_HEIGHT_MM;
  liftAxis.active = false;
  liftAxis.fault = false;
  liftAxis.commandAcknowledged = false;
  liftAxis.motionObserved = false;
  armLinearReferenceValid = false;
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
  SerialDebug.print("[");
  SerialDebug.print(axisLabel);
  SerialDebug.print(
      " ZERO] read driver angle, move from mechanical "
      "endpoint by ");
  SerialDebug.print(fabsf(startupTargetMm), 2);
  SerialDebug.println(" mm, then re-zero");

  float angleBeforeDegrees = 0.0f;
  const bool angleBeforeValid =
      readArmLinearCurrentMotorAngleDegrees(
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
        " ZERO] driver angle read timeout before move; "
        "continue from confirmed mechanical endpoint");
  }

  clearArmLinearReceiveBuffer();
  writeArmLinearResetStallProtection(
      axis.address);
  SerialArmLinear.flush();
  const bool stallResetAcknowledged =
      waitForArmLinearSimpleResponse(
          axis.address,
          0x0EU,
          ARM_LINEAR_POSITION_READ_TIMEOUT_MS);
  if (!stallResetAcknowledged) {
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] no accepted stall-reset ACK; "
        "position command/status will make final decision");
  }
  clearArmLinearReceiveBuffer();

  if (!startLinearAxisMove(
          axis,
          startupTargetMm,
          startupMinimumMm,
          startupMaximumMm,
          pulsesPerMm,
          positiveDirection,
          negativeDirection,
          ARM_LINEAR_STARTUP_ZERO_SPEED_RPM,
          ARM_LINEAR_STARTUP_ZERO_ACCELERATION)) {
    return false;
  }

  while (axis.active && !axis.fault) {
    serviceArmLinearAxes();
    delay(1);
  }
  if (axis.fault ||
      programState == PROGRAM_FAULT) {
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] failed to leave mechanical endpoint");
    return false;
  }

  float angleAfterDegrees = 0.0f;
  const bool angleAfterValid =
      readArmLinearCurrentMotorAngleDegrees(
          axis.address,
          angleAfterDegrees);
  if (angleAfterValid) {
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.print(" ZERO] driver angle after move=");
    SerialDebug.print(angleAfterDegrees, 3);
    SerialDebug.println(" deg");
    if (angleBeforeValid) {
      const float expectedAngleMagnitudeDegrees =
          fabsf(startupTargetMm) /
          travelPerRevolutionMm * 360.0f;
      SerialDebug.print("[");
      SerialDebug.print(axisLabel);
      SerialDebug.print(
          " ZERO] observed/expected angle delta magnitude=");
      SerialDebug.print(
          fabsf(angleAfterDegrees - angleBeforeDegrees),
          3);
      SerialDebug.print("/");
      SerialDebug.print(
          expectedAngleMagnitudeDegrees,
          3);
      SerialDebug.println(" deg");
    }
  } else {
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] driver angle read timeout after move");
  }

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
    /*
     * 后续全是相对位置命令，软件零点仍然有效；驱动器内部清零只用于
     * 诊断读数，因此不因缺少该ACK再次向机械端点运动。
     */
    SerialDebug.print("[");
    SerialDebug.print(axisLabel);
    SerialDebug.println(
        " ZERO] driver-zero ACK missing; "
        "software working zero remains authoritative");
  }
  clearArmLinearReceiveBuffer();

  axis.currentMm = 0.0f;
  axis.targetMm = 0.0f;
  axis.active = false;
  axis.fault = false;
  axis.commandAcknowledged = false;
  axis.motionObserved = false;
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

void commandStorageServoPosition(uint8_t positionIndex) {
  if (positionIndex > 3U) {
    routeFault("Invalid storage servo position");
    return;
  }
  /*
   * 作业序列为-5、-95、-185、-5度，对应顺时针90度、
   * 再顺时针90度、最后逆时针180度回到工作零位。序列会越过
   * 单圈角度边界，因此必须使用多圈绝对角度命令。
   */
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
  /*
   * 两次ping不能用&&短路，否则夹爪掉线时不会检查载物盘。
   * 该检查只在启动前或工位停车后执行，最坏约2×100 ms，不影响
   * 正在运行的软件步进脉冲。
   */
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

struct MaixCoordinate {
  uint8_t targetId;
  uint8_t requestSequence;
  uint8_t mode;
  int16_t x;
  int16_t y;
  uint16_t metric;
  uint16_t confidence;
  uint32_t cameraTimestampMs;
  uint32_t sequence;
  uint32_t receivedMs;

  MaixCoordinate()
      : targetId(0U),
        requestSequence(0U),
        mode(0U),
        x(0),
        y(0),
        metric(0U),
        confidence(0U),
        cameraTimestampMs(0UL),
        sequence(0UL),
        receivedMs(0UL) {}
};

bool maixcamSerialInitialized = false;
uint8_t maixRequestedMode = MAIXCAM_STOP_REQUEST;
uint8_t maixRequestSequence = 0U;
bool maixModeCommandSent = false;
uint32_t maixModeSwitchStartMs = 0UL;
uint32_t maixLastRequestMs = 0UL;
char maixReceiveLine[MAIXCAM_LINE_CAPACITY] = {0};
size_t maixReceiveLength = 0U;
bool maixReceiveOverflow = false;
MaixCoordinate latestMaixCoordinate;

void writeMaixRequestFrame(uint8_t request) {
  uint8_t frame[vision_protocol::REQUEST_FRAME_SIZE] = {0U};
  if (!vision_protocol::buildRequest(
          maixRequestSequence,
          request,
          frame,
          sizeof(frame))) {
    routeFault("Vision request frame build failed");
    return;
  }
  const size_t queuedBytes =
      SerialMaixcam.write(frame, sizeof(frame));

  /*
   * 诊断日志记录真正执行SerialMaixcam.write()的时刻。
   * “MaixCAM mode request”只代表状态机计划切换模式，不能证明已经走到TX。
   */
  SerialDebug.print("[MAIX TX] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, v2 seq/mode=");
  SerialDebug.print(maixRequestSequence);
  SerialDebug.print("/");
  if (request < 0x10U) {
    SerialDebug.print("0");
  }
  SerialDebug.print(request, HEX);
  SerialDebug.print(", crc=");
  if (frame[4] < 0x10U) {
    SerialDebug.print("0");
  }
  SerialDebug.print(frame[4], HEX);
  SerialDebug.print(", queued=");
  SerialDebug.println(
      static_cast<unsigned int>(queuedBytes));
}

void stopMaixRequest() {
  if (maixcamSerialInitialized) {
    writeMaixRequestFrame(MAIXCAM_STOP_REQUEST);
  }
  maixRequestedMode = MAIXCAM_STOP_REQUEST;
  maixModeCommandSent = false;
  maixReceiveLength = 0U;
  maixReceiveOverflow = false;
}

void beginMaixRequest(uint8_t request) {
  if (request != MAIXCAM_ALL_COLORS_REQUEST &&
      request != MAIXCAM_HOUGH_CIRCLE_REQUEST &&
      request != MAIXCAM_ENDPOINT_CIRCLE_REQUEST) {
    routeFault("Invalid MaixCAM request");
    return;
  }

  stopMaixRequest();
  while (SerialMaixcam.available()) {
    SerialMaixcam.read();
  }
  maixRequestSequence =
      static_cast<uint8_t>(maixRequestSequence + 1U);
  maixRequestedMode = request;
  maixModeSwitchStartMs = millis();
  maixLastRequestMs = 0UL;
  maixModeCommandSent = false;

  SerialDebug.print("[MAIX PLAN] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, mode=");
  SerialDebug.print(request);
  SerialDebug.print(", seq=");
  SerialDebug.print(maixRequestSequence);
  SerialDebug.print(", TX after guard=");
  SerialDebug.print(MAIXCAM_MODE_SWITCH_GUARD_MS);
  SerialDebug.println(" ms");
}

void finishMaixCoordinateLine() {
  if (maixReceiveOverflow) {
    SerialDebug.println(
        "MaixCAM response discarded: line too long");
    maixReceiveLength = 0U;
    maixReceiveOverflow = false;
    return;
  }

  maixReceiveLine[maixReceiveLength] = '\0';
  vision_protocol::VisionResponse response;
  const vision_protocol::ParseError parseError =
      vision_protocol::parseResponse(
          maixReceiveLine,
          maixReceiveLength,
          response);
  maixReceiveLength = 0U;
  if (parseError != vision_protocol::PARSE_OK) {
    SerialDebug.print(
        "MaixCAM response rejected: ");
    SerialDebug.println(
        vision_protocol::parseErrorText(parseError));
    return;
  }

  if (!maixModeCommandSent ||
      maixRequestedMode == MAIXCAM_STOP_REQUEST ||
      response.sequence != maixRequestSequence ||
      response.mode != maixRequestedMode) {
    SerialDebug.print(
        "MaixCAM stale/mismatched seq/mode received=");
    SerialDebug.print(response.sequence);
    SerialDebug.print("/");
    SerialDebug.print(response.mode);
    SerialDebug.print(", expected=");
    SerialDebug.print(maixRequestSequence);
    SerialDebug.print("/");
    SerialDebug.println(maixRequestedMode);
    return;
  }

  if (response.status != vision_protocol::STATUS_OK) {
    SerialDebug.print("MaixCAM status=");
    SerialDebug.println(response.status);
    if (response.status ==
        vision_protocol::STATUS_CAMERA_ERROR) {
      routeFault("MaixCAM reported camera error");
    }
    return;
  }

  const bool targetMatchesMode =
      (maixRequestedMode ==
           MAIXCAM_ALL_COLORS_REQUEST &&
       response.target >= 1U &&
       response.target <= 4U) ||
      (maixRequestedMode ==
           MAIXCAM_HOUGH_CIRCLE_REQUEST &&
       response.target == 2U) ||
      (maixRequestedMode ==
           MAIXCAM_ENDPOINT_CIRCLE_REQUEST &&
       response.target == 1U);
  if (!targetMatchesMode) {
    SerialDebug.println(
        "MaixCAM target identity does not match mode");
    return;
  }

  latestMaixCoordinate.targetId = response.target;
  latestMaixCoordinate.requestSequence =
      response.sequence;
  latestMaixCoordinate.mode = response.mode;
  latestMaixCoordinate.x =
      static_cast<int16_t>(response.x);
  latestMaixCoordinate.y =
      static_cast<int16_t>(response.y);
  latestMaixCoordinate.metric = response.metric;
  latestMaixCoordinate.confidence =
      response.confidence;
  latestMaixCoordinate.cameraTimestampMs =
      response.timestamp;
  ++latestMaixCoordinate.sequence;
  latestMaixCoordinate.receivedMs = millis();

  SerialDebug.print("MaixCAM v2 seq/mode/target=");
  SerialDebug.print(response.sequence);
  SerialDebug.print("/");
  SerialDebug.print(response.mode);
  SerialDebug.print("/");
  SerialDebug.print(response.target);
  SerialDebug.print(", xy=");
  SerialDebug.print(response.x);
  SerialDebug.print(",");
  SerialDebug.print(response.y);
  SerialDebug.print(", metric/confidence=");
  SerialDebug.print(response.metric);
  SerialDebug.print("/");
  SerialDebug.println(response.confidence);
}

void serviceMaixcam() {
  while (SerialMaixcam.available()) {
    const char incoming =
        static_cast<char>(SerialMaixcam.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      finishMaixCoordinateLine();
      continue;
    }
    if (maixReceiveOverflow) {
      continue;
    }
    if (maixReceiveLength < MAIXCAM_LINE_CAPACITY - 1U) {
      maixReceiveLine[maixReceiveLength++] = incoming;
    } else {
      maixReceiveOverflow = true;
      maixReceiveLength = 0U;
    }
  }

  if (maixRequestedMode == MAIXCAM_STOP_REQUEST) {
    return;
  }

  const uint32_t nowMs = millis();
  if (!maixModeCommandSent) {
    if (nowMs - maixModeSwitchStartMs <
        MAIXCAM_MODE_SWITCH_GUARD_MS) {
      return;
    }
    writeMaixRequestFrame(maixRequestedMode);
    maixModeCommandSent = true;
    maixLastRequestMs = nowMs;
    return;
  }

  if (nowMs - maixLastRequestMs >=
      MAIXCAM_REQUEST_REPEAT_MS) {
    writeMaixRequestFrame(maixRequestedMode);
    maixLastRequestMs = nowMs;
  }
}

bool readNewMaixCoordinate(
    uint32_t &lastSequence,
    uint8_t &targetId,
    int16_t &x,
    int16_t &y) {
  if (latestMaixCoordinate.sequence == lastSequence ||
      millis() - latestMaixCoordinate.receivedMs >
          MAIXCAM_COORDINATE_STALE_MS) {
    return false;
  }
  lastSequence = latestMaixCoordinate.sequence;
  targetId = latestMaixCoordinate.targetId;
  x = latestMaixCoordinate.x;
  y = latestMaixCoordinate.y;
  return true;
}

void advanceRoute() {
  stopAllMotorsImmediately();
  ++routeIndex;
  commandStarted = false;
  commandStartMs = millis();
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

  if (turnMotionEnabled &&
      activeTurnCorrectionCount >= MAXIMUM_TURN_CORRECTIONS) {
    routeFault("Turn correction limit exceeded");
    return false;
  }

  startHeadingCorrection(error);
  return false;
}

MotorPulses currentDriveMotorPositions() {
  return MotorPulses(
      motor1.currentPosition(),
      motor2.currentPosition(),
      motor3.currentPosition(),
      motor4.currentPosition());
}

void startQrScanReturnToOrigin() {
  SerialDebug.println(
      "[QR SWEEP] code acquired; returning to saved origin");

  /*
   * 直接把四轮绝对目标改回各自扫码点脉冲原点。AccelStepper会按当前
   * 低速参数先减速、再反转；不能固定反走500 mm，因为可能在途中扫码。
   */
  setDriveMotionProfile(
      QR_SCAN_MAXIMUM_STEP_RATE,
      QR_SCAN_STEP_ACCELERATION);
  motor1.moveTo(qrScanOriginMotorPositions.motor1);
  motor2.moveTo(qrScanOriginMotorPositions.motor2);
  motor3.moveTo(qrScanOriginMotorPositions.motor3);
  motor4.moveTo(qrScanOriginMotorPositions.motor4);
  commandStartMs = millis();
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
  hmiSetRunStatus("QRBACK");
  qrScanPhase = QR_SCAN_RETURNING;
}

void startQrScanAction() {
  qrScanActionStartMs = millis();
  qrScanOriginMotorPositions =
      currentDriveMotorPositions();
  hmiSetRunStatus("SCAN");

  /*
   * 二维码串口在行驶途中也一直监听。若到扫码点前已经得到并校验了完整
   * 任务码，就保留该有效结果，不再做没有必要的附加位移。
   */
  if (scanFlag) {
    SerialDebug.println(
        "[QR SWEEP] valid code already available; no sweep needed");
    qrScanPhase = QR_SCAN_COMPLETE;
    return;
  }

  // 测试旁路保持原行为：只定时停车，不额外产生扫码位移。
  if (!REQUIRE_QR_SUCCESS) {
    qrScanPhase = QR_SCAN_IDLE;
    return;
  }

  setDriveMotionProfile(
      QR_SCAN_MAXIMUM_STEP_RATE,
      QR_SCAN_STEP_ACCELERATION);
  /*
   * 扫码前探延续所选启停区到二维码行的进入方向，避免Start2到Y=1200
   * 后折返、只重复搜索南半段。回位仍保存四轮绝对脉冲原点，与方向无关。
   */
  const float qrSweepForwardSign =
      selectedStartZone == START_ZONE_1
          ? -1.0f
          : 1.0f;
  startBodyDisplacement(
      qrSweepForwardSign *
          static_cast<float>(
          QR_SCAN_SWEEP_MAXIMUM_MM) /
          1000.0f,
      0.0f,
      0.0f);
  commandStartMs = millis();
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
  qrScanPhase = QR_SCAN_FORWARD;
  SerialDebug.print("[QR SWEEP] signed forward limit=");
  SerialDebug.print(
      qrSweepForwardSign *
      static_cast<float>(QR_SCAN_SWEEP_MAXIMUM_MM),
      0);
  SerialDebug.print(", magnitude=");
  SerialDebug.print(QR_SCAN_SWEEP_MAXIMUM_MM);
  SerialDebug.println(" mm");
}

bool updateQrScanAction() {
  if (REQUIRE_QR_SUCCESS &&
      !scanFlag &&
      (qrScanPhase == QR_SCAN_FORWARD ||
       qrScanPhase == QR_SCAN_WAIT_AT_LIMIT) &&
      qrScanActionStartMs != 0UL &&
      millis() - qrScanActionStartMs >=
          QR_SCAN_ACTION_TIMEOUT_MS) {
    routeFault("QR scan action timeout");
    return false;
  }

  if (!REQUIRE_QR_SUCCESS &&
      (scanFlag ||
       millis() - qrScanActionStartMs >=
           QR_TEST_HOLD_MS)) {
    qrScanPhase = QR_SCAN_COMPLETE;
  }

  switch (qrScanPhase) {
    case QR_SCAN_IDLE:
      return false;

    case QR_SCAN_FORWARD:
      if (scanFlag) {
        startQrScanReturnToOrigin();
        return false;
      }
      if (!imuIsFresh()) {
        routeFault("IMU data timeout during QR sweep");
        return false;
      }
      if (!allMotorsArrived()) {
        if (ENABLE_MOTION_TIMEOUTS &&
            millis() - commandStartMs >=
                MOTION_TIMEOUT_MS) {
          routeFault("QR sweep motion timeout");
        }
        return false;
      }

      qrScanPhase = QR_SCAN_WAIT_AT_LIMIT;
      hmiSetRunStatus("SCANMAX");
      SerialDebug.println(
          "[QR SWEEP] 500 mm limit reached; waiting for valid code");
      return false;

    case QR_SCAN_WAIT_AT_LIMIT:
      if (scanFlag) {
        startQrScanReturnToOrigin();
      }
      return false;

    case QR_SCAN_RETURNING:
      if (updateHeadingLock(MOTION_TIMEOUT_MS)) {
        qrScanPhase = QR_SCAN_COMPLETE;
        SerialDebug.println(
            "[QR SWEEP] returned to scan origin");
        return true;
      }
      return false;

    case QR_SCAN_COMPLETE:
      return true;
  }

  routeFault("Invalid QR sweep phase");
  return false;
}

bool isTranslationCommand(CommandType type) {
  return type == COMMAND_MOVE_SIDE_12_MM ||
         type == COMMAND_MOVE_SIDE_34_MM ||
         type == COMMAND_MOVE_SIDE_13_MM ||
         type == COMMAND_MOVE_SIDE_24_MM;
}

void startTranslationSegment(CommandType type) {
  /*
   * 中央通道同一直线最多2000 mm一段直达，非中央通道仍保留原1100 mm
   * 上限。remaining只是“尚未发送的命令距离”，不是里程计或世界坐标
   * 反馈；轮胎打滑不会自动修改它。
   */
  const uint16_t maximumSegmentMm =
      translationCentralChannelEnabled
          ? CENTRAL_CHANNEL_MAX_TRANSLATION_SEGMENT_MM
          : MAX_TRANSLATION_SEGMENT_MM;
  const uint16_t segmentMm =
      translationRemainingMm > maximumSegmentMm
          ? maximumSegmentMm
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

  const float translationMaximumStepRate =
      preciseMotionEnabled
          ? FINAL_MAXIMUM_STEP_RATE
          : (workstationApproachEnabled
                 ? WORKSTATION_MAXIMUM_STEP_RATE
                 : (translationCentralChannelEnabled
                        ? CENTRAL_CHANNEL_MAXIMUM_STEP_RATE
                        : MAXIMUM_STEP_RATE));
  const float translationStepAcceleration =
      preciseMotionEnabled
          ? FINAL_STEP_ACCELERATION
          : (workstationApproachEnabled
                 ? WORKSTATION_STEP_ACCELERATION
                 : STEP_ACCELERATION);
  setDriveMotionProfile(
      translationMaximumStepRate,
      translationStepAcceleration);

  switch (type) {
    case COMMAND_MOVE_SIDE_12_MM:
      // 朝当前车体1、2侧移动；世界方向取决于当前车身姿态。
      startBodyDisplacement(distanceMeters, 0.0f, 0.0f);
      break;
    case COMMAND_MOVE_SIDE_34_MM:
      // 朝当前车体3、4侧移动；起步时该方向为场地南侧。
      startBodyDisplacement(-distanceMeters, 0.0f, 0.0f);
      break;
    case COMMAND_MOVE_SIDE_13_MM:
      // 朝当前车体1、3侧横移。
      startBodyDisplacement(0.0f, distanceMeters, 0.0f);
      break;
    case COMMAND_MOVE_SIDE_24_MM:
      // 朝当前车体2、4侧横移；也是机械臂伸出侧的工位精靠方向。
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

// ---------------------------------------------------------------------------
// 机械臂动作与三个工作区流程
// ---------------------------------------------------------------------------

struct ArmPose {
  float standardFrameAngleDegrees;
  float extensionMm;
  float heightMm;

  ArmPose(
      float angleDegrees = 0.0f,
      float extension = 0.0f,
      float height = 0.0f)
      : standardFrameAngleDegrees(angleDegrees),
        extensionMm(extension),
        heightMm(height) {}
};

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
  ARM_STANDARD_WAIT_OPEN,
  ARM_STANDARD_WAIT_LIFT,
  ARM_STANDARD_WAIT_RETRACT,
  ARM_STANDARD_WAIT_BASE,
  ARM_STANDARD_WAIT_BASE_SETTLE,
  ARM_STANDARD_COMPLETE
};

ArmStandardPhase armStandardPhase = ARM_STANDARD_IDLE;
uint32_t armStandardDeadlineMs = 0UL;

void beginArmStandardization() {
  SerialDebug.print("[ARM STD] t=");
  SerialDebug.print(millis());
  SerialDebug.print(
      " ms, start: open gripper; M6 current=");
  SerialDebug.print(extensionAxis.currentMm, 2);
  SerialDebug.print(" mm, M7 current=");
  SerialDebug.print(liftAxis.currentMm, 2);
  SerialDebug.println(" mm");
  commandGripperOpen();
  armStandardDeadlineMs =
      millis() + GRIPPER_OPEN_SETTLE_MS;
  armStandardPhase = ARM_STANDARD_WAIT_OPEN;
}

bool serviceArmStandardization() {
  switch (armStandardPhase) {
    case ARM_STANDARD_IDLE:
      return false;

    case ARM_STANDARD_WAIT_OPEN:
      if (deadlineReached(armStandardDeadlineMs)) {
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
        SerialDebug.println(
            " ms, M7 complete -> M6 safe working zero");
        startExtensionToMm(M6_STANDARD_EXTENSION_MM);
        armStandardPhase = ARM_STANDARD_WAIT_RETRACT;
      }
      break;

    case ARM_STANDARD_WAIT_RETRACT:
      if (extensionMoveFinished()) {
        SerialDebug.print("[ARM STD] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, M6 complete -> M5 standard 0 deg");
        startArmBaseStandardFrameDegrees(0.0f);
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
  ARM_TRANSFER_WAIT_SOURCE_OPEN,
  ARM_TRANSFER_WAIT_PREPARE_LIFT,
  ARM_TRANSFER_WAIT_PREPARE_RETRACT,
  ARM_TRANSFER_WAIT_SOURCE_ROTATION,
  ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE,
  ARM_TRANSFER_WAIT_SOURCE_EXTENSION,
  ARM_TRANSFER_WAIT_SOURCE_LOWER,
  ARM_TRANSFER_WAIT_GRIP_CLOSE,
  ARM_TRANSFER_WAIT_LOADED_LIFT,
  ARM_TRANSFER_WAIT_LOADED_RETRACT,
  ARM_TRANSFER_WAIT_DESTINATION_ROTATION,
  ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE,
  ARM_TRANSFER_WAIT_DESTINATION_EXTENSION,
  ARM_TRANSFER_WAIT_DESTINATION_EXTENSION_SETTLE,
  ARM_TRANSFER_WAIT_DESTINATION_APPROACH,
  ARM_TRANSFER_WAIT_DESTINATION_LOWER,
  ARM_TRANSFER_WAIT_DESTINATION_LOWER_SETTLE,
  ARM_TRANSFER_WAIT_DESTINATION_OPEN,
  ARM_TRANSFER_WAIT_FINAL_LIFT,
  ARM_TRANSFER_WAIT_FINAL_RETRACT,
  ARM_TRANSFER_WAIT_STANDARD_ROTATION,
  ARM_TRANSFER_WAIT_STANDARD_SETTLE,
  ARM_TRANSFER_COMPLETE
};

ArmTransferPhase armTransferPhase = ARM_TRANSFER_IDLE;
ArmPose armTransferSourcePose;
ArmPose armTransferDestinationPose;
uint32_t armTransferDeadlineMs = 0UL;
bool armTransferGentleDestinationRelease = false;
bool armTransferMapSource = false;
bool armTransferMapDestination = false;

bool ringMapHeadingStillValid();

bool armPoseIsValid(const ArmPose &pose) {
  return pose.extensionMm >=
             M6_STANDARD_EXTENSION_MM -
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
    bool mapDestination = false) {
  if (!armPoseIsValid(source) ||
      !armPoseIsValid(destination)) {
    routeFault("Transfer pose outside arm travel");
    return;
  }

  armTransferSourcePose = source;
  armTransferDestinationPose = destination;
  armTransferGentleDestinationRelease =
      gentleDestinationRelease;
  armTransferMapSource = mapSource;
  armTransferMapDestination = mapDestination;
  commandGripperOpen();
  armTransferDeadlineMs =
      millis() + GRIPPER_OPEN_SETTLE_MS;
  armTransferPhase = ARM_TRANSFER_WAIT_SOURCE_OPEN;

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
}

void startArmTransferDestinationDescent() {
  if (!armTransferGentleDestinationRelease) {
    startLiftToHeightMm(
        armTransferDestinationPose.heightMm);
    armTransferPhase =
        ARM_TRANSFER_WAIT_DESTINATION_LOWER;
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
  startLiftToHeightMm(approachHeightMm);
  armTransferPhase =
      ARM_TRANSFER_WAIT_DESTINATION_APPROACH;
}

void releaseArmTransferDestination() {
  commandGripperOpen();
  armTransferDeadlineMs =
      millis() + GRIPPER_OPEN_SETTLE_MS;
  armTransferPhase =
      ARM_TRANSFER_WAIT_DESTINATION_OPEN;
}

void serviceArmTransfer() {
  switch (armTransferPhase) {
    case ARM_TRANSFER_IDLE:
    case ARM_TRANSFER_COMPLETE:
      return;

    case ARM_TRANSFER_WAIT_SOURCE_OPEN:
      if (deadlineReached(armTransferDeadlineMs)) {
        startLiftToHeightMm(M7_STANDARD_HEIGHT_MM);
        armTransferPhase =
            ARM_TRANSFER_WAIT_PREPARE_LIFT;
      }
      break;

    case ARM_TRANSFER_WAIT_PREPARE_LIFT:
      if (liftMoveFinished()) {
        startExtensionToMm(M6_STANDARD_EXTENSION_MM);
        armTransferPhase =
            ARM_TRANSFER_WAIT_PREPARE_RETRACT;
      }
      break;

    case ARM_TRANSFER_WAIT_PREPARE_RETRACT:
      if (extensionMoveFinished()) {
        startArmBaseStandardFrameDegrees(
            armTransferSourcePose
                .standardFrameAngleDegrees);
        armTransferPhase =
            ARM_TRANSFER_WAIT_SOURCE_ROTATION;
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_ROTATION:
      if (!armMotors.isM5Running()) {
        armTransferDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        armTransferPhase =
            ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE;
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE:
      if (deadlineReached(armTransferDeadlineMs)) {
        startExtensionToMm(
            armTransferSourcePose.extensionMm);
        armTransferPhase =
            ARM_TRANSFER_WAIT_SOURCE_EXTENSION;
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_EXTENSION:
      if (extensionMoveFinished()) {
        if (armTransferMapSource &&
            !ringMapHeadingStillValid()) {
          return;
        }
        startLiftToHeightMm(
            armTransferSourcePose.heightMm);
        armTransferPhase =
            ARM_TRANSFER_WAIT_SOURCE_LOWER;
      }
      break;

    case ARM_TRANSFER_WAIT_SOURCE_LOWER:
      if (liftMoveFinished()) {
        commandGripperClose();
        armTransferDeadlineMs =
            millis() + GRIPPER_CLOSE_SETTLE_MS;
        armTransferPhase =
            ARM_TRANSFER_WAIT_GRIP_CLOSE;
      }
      break;

    case ARM_TRANSFER_WAIT_GRIP_CLOSE:
      if (deadlineReached(armTransferDeadlineMs)) {
        startLiftToHeightMm(M7_STANDARD_HEIGHT_MM);
        armTransferPhase =
            ARM_TRANSFER_WAIT_LOADED_LIFT;
      }
      break;

    case ARM_TRANSFER_WAIT_LOADED_LIFT:
      if (liftMoveFinished()) {
        startExtensionToMm(M6_STANDARD_EXTENSION_MM);
        armTransferPhase =
            ARM_TRANSFER_WAIT_LOADED_RETRACT;
      }
      break;

    case ARM_TRANSFER_WAIT_LOADED_RETRACT:
      if (extensionMoveFinished()) {
        startArmBaseStandardFrameDegrees(
            armTransferDestinationPose
                .standardFrameAngleDegrees);
        armTransferPhase =
            ARM_TRANSFER_WAIT_DESTINATION_ROTATION;
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION:
      if (!armMotors.isM5Running()) {
        armTransferDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        armTransferPhase =
            ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE;
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE:
      if (deadlineReached(armTransferDeadlineMs)) {
        startExtensionToMm(
            armTransferDestinationPose.extensionMm);
        armTransferPhase =
            ARM_TRANSFER_WAIT_DESTINATION_EXTENSION;
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_EXTENSION:
      if (extensionMoveFinished()) {
        if (armTransferMapDestination &&
            !ringMapHeadingStillValid()) {
          return;
        }
        if (armTransferGentleDestinationRelease) {
          armTransferDeadlineMs =
              millis() +
              RING_PLACE_EXTENSION_SETTLE_MS;
          armTransferPhase =
              ARM_TRANSFER_WAIT_DESTINATION_EXTENSION_SETTLE;
        } else {
          startArmTransferDestinationDescent();
        }
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_EXTENSION_SETTLE:
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
        startLiftToHeightMmWithProfile(
            armTransferDestinationPose.heightMm,
            M7_RING_PLACE_SPEED_RPM,
            M7_RING_PLACE_ACCELERATION);
        armTransferPhase =
            ARM_TRANSFER_WAIT_DESTINATION_LOWER;
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_LOWER:
      if (liftMoveFinished()) {
        if (armTransferGentleDestinationRelease) {
          SerialDebug.println(
              "[RING PLACE] final height reached; "
              "settle before release");
          armTransferDeadlineMs =
              millis() +
              RING_PLACE_LOWER_SETTLE_MS;
          armTransferPhase =
              ARM_TRANSFER_WAIT_DESTINATION_LOWER_SETTLE;
        } else {
          releaseArmTransferDestination();
        }
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_LOWER_SETTLE:
      if (deadlineReached(armTransferDeadlineMs)) {
        releaseArmTransferDestination();
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_OPEN:
      if (deadlineReached(armTransferDeadlineMs)) {
        startLiftToHeightMm(M7_STANDARD_HEIGHT_MM);
        armTransferPhase =
            ARM_TRANSFER_WAIT_FINAL_LIFT;
      }
      break;

    case ARM_TRANSFER_WAIT_FINAL_LIFT:
      if (liftMoveFinished()) {
        startExtensionToMm(M6_STANDARD_EXTENSION_MM);
        armTransferPhase =
            ARM_TRANSFER_WAIT_FINAL_RETRACT;
      }
      break;

    case ARM_TRANSFER_WAIT_FINAL_RETRACT:
      if (extensionMoveFinished()) {
        startArmBaseStandardFrameDegrees(0.0f);
        armTransferPhase =
            ARM_TRANSFER_WAIT_STANDARD_ROTATION;
      }
      break;

    case ARM_TRANSFER_WAIT_STANDARD_ROTATION:
      if (!armMotors.isM5Running()) {
        armTransferDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        armTransferPhase =
            ARM_TRANSFER_WAIT_STANDARD_SETTLE;
      }
      break;

    case ARM_TRANSFER_WAIT_STANDARD_SETTLE:
      if (deadlineReached(armTransferDeadlineMs)) {
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
  armTransferMapSource = false;
  armTransferMapDestination = false;
}

ArmPose containerPose(float lowerMm) {
  return ArmPose(
      ARM_CONTAINER_CLOCKWISE_DEGREES,
      M6_STANDARD_EXTENSION_MM,
      -lowerMm);
}

bool nominalRingPose(
    uint8_t ringPosition,
    float lowerMm,
    ArmPose &pose) {
  if (ringPosition < 1U || ringPosition > 3U) {
    routeFault("Invalid ring position");
    return false;
  }

  // ±150 mm只用于第一次斜着找到端点；最终坐标必须由原地压中获得。
  float imageLeftMm = 0.0f;
  if (ringPosition == 1U) {
    imageLeftMm = 150.0f;
  } else if (ringPosition == 3U) {
    imageLeftMm = -150.0f;
  }

  const float radialDistanceMm =
      hypotf(
          ARM_PIVOT_TO_CAMERA_CENTER_MM,
          imageLeftMm);
  pose.standardFrameAngleDegrees =
      atan2f(
          imageLeftMm,
          ARM_PIVOT_TO_CAMERA_CENTER_MM) *
      180.0f / PI_F;
  if (pose.standardFrameAngleDegrees <
          RING_SCAN_MINIMUM_ANGLE_DEGREES ||
      pose.standardFrameAngleDegrees >
          RING_SCAN_MAXIMUM_ANGLE_DEGREES) {
    routeFault("Nominal endpoint outside -80..80 scan range");
    return false;
  }
  pose.extensionMm =
      radialDistanceMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;
  pose.heightMm = -lowerMm;
  return true;
}

bool planarPointToRingPose(
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
      ARM_PIVOT_TO_GRIPPER_CENTER_MM;

  if (extensionMm <
          M6_STANDARD_EXTENSION_MM -
              ARM_AXIS_POSITION_TOLERANCE_MM ||
      extensionMm >
          M6_MAXIMUM_EXTENSION_MM +
              ARM_AXIS_POSITION_TOLERANCE_MM ||
      angleDegrees <
          RING_TARGET_MINIMUM_ANGLE_DEGREES ||
      angleDegrees >
          RING_TARGET_MAXIMUM_ANGLE_DEGREES) {
    SerialDebug.print(
        "[RING MAP] unreachable xy/r/angle/ext=");
    SerialDebug.print(point.outwardMm, 2);
    SerialDebug.print(",");
    SerialDebug.print(point.leftMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(radialDistanceMm, 2);
    SerialDebug.print("/");
    SerialDebug.print(angleDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.println(extensionMm, 2);
    routeFault("Measured ring outside safe arm workspace");
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

  /*
   * 圆环中心线实测半径41.75 mm，因此同一帧的mm/px可由41.75/r
   * 自适应得到。
   * 局部坐标：图像+y向下对应机械臂向内，图像+x向右对应顺时针，
   * 所以标准坐标的向外/逆时针切向偏移都取负号。
   */
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
      "[ENDPOINT MAP] image xy/r, scale, arm xy=");
  SerialDebug.print(imageX);
  SerialDebug.print(",");
  SerialDebug.print(imageY);
  SerialDebug.print("/");
  SerialDebug.print(radiusPixels);
  SerialDebug.print(", ");
  SerialDebug.print(mmPerPixel, 4);
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

  for (uint8_t ring = 1U; ring <= 3U; ++ring) {
    ArmPose pose;
    if (!planarPointToRingPose(
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

bool rawTargetPose(
    float pixelX,
    float pixelY,
    ArmPose &pose) {
  /*
   * 只在稳定窗口完成后计算一次：
   *   图像是x向右、y向下为正的左手系；
   *   x>160时，为使目标横坐标减小到160，M5必须顺时针，故标准角取负；
   *   y>120时，为使目标纵坐标减小到120，M6应缩短；
   *   y<120时，为使目标纵坐标增大到120，M6应伸长。
   *   因此图像+y与机械臂向外方向相反，使用
   *   IMAGE_Y_TO_ARM_OUTWARD_SIGN=-1换算。
   */
  const float pixelDeltaX =
      pixelX - static_cast<float>(IMAGE_CENTER_X);
  const float pixelDeltaY =
      pixelY - static_cast<float>(IMAGE_CENTER_Y);
  const float pixelRadius =
      hypotf(pixelDeltaX, pixelDeltaY);
  /*
   * 横纵偏差共用同一档比例，避免分别选档后改变偏移向量方向。
   * 到画面中心的距离不超过50像素用40/72，超过50像素用70/72。
   * 这仍是两档实调模型；正式版应由标定数据替换为连续单应性/多项式。
   */
  const float rawMmPerPixel =
      pixelRadius <= PIXEL_MAPPING_SWITCH_RADIUS_PIXELS
          ? RAW_NEAR_MM_PER_PIXEL
          : RAW_FAR_MM_PER_PIXEL;
  const float clockwiseTangentMm =
      pixelDeltaX * rawMmPerPixel;
  const float radialMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      pixelDeltaY *
          rawMmPerPixel *
          IMAGE_Y_TO_ARM_OUTWARD_SIGN;
  const float targetRadiusMm =
      hypotf(radialMm, clockwiseTangentMm);
  const float extensionMm =
      targetRadiusMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;

  if (extensionMm <
          M6_STANDARD_EXTENSION_MM -
              ARM_AXIS_POSITION_TOLERANCE_MM ||
      extensionMm >
          M6_MAXIMUM_EXTENSION_MM +
              ARM_AXIS_POSITION_TOLERANCE_MM) {
    SerialDebug.print("Raw target unreachable: radius=");
    SerialDebug.print(targetRadiusMm, 2);
    SerialDebug.print(", extension=");
    SerialDebug.println(extensionMm, 2);
    routeFault("Raw target outside M6 reach");
    return false;
  }

  pose.standardFrameAngleDegrees =
      -atan2f(clockwiseTangentMm, radialMm) *
      180.0f / PI_F;
  pose.extensionMm =
      extensionMm < M6_STANDARD_EXTENSION_MM
          ? M6_STANDARD_EXTENSION_MM
          : extensionMm;
  pose.heightMm = -RAW_PICK_LOWER_MM;
  return true;
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
  WORK_PHASE_RAW_WAIT_STORAGE_POSITION,
  WORK_PHASE_ENDPOINT_WAIT_PRE_SCAN_HEADING,
  WORK_PHASE_ENDPOINT_WAIT_PRELOAD_BASE,
  WORK_PHASE_ENDPOINT_WAIT_PRELOAD_SETTLE,
  WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE,
  WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE_SETTLE,
  WORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION,
  WORK_PHASE_ENDPOINT_WAIT_ARM_LOWER,
  WORK_PHASE_ENDPOINT_WAIT_COORDINATE,
  WORK_PHASE_ENDPOINT_WAIT_LOCAL_MOVE,
  WORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE,
  WORK_PHASE_ENDPOINT_WAIT_FINE_LOWER,
  WORK_PHASE_ENDPOINT_WAIT_ARM_STANDARD,
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
  WORK_PHASE_WAIT_TRAVEL_BASE,
  WORK_PHASE_WAIT_TRAVEL_BASE_SETTLE,
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

bool visionYanyanPlacementSequenceComplete = false;
uint32_t visionYanyanTestStartMs = 0UL;
uint32_t visionYanyanPositioningCompleteMs = 0UL;

uint8_t rawFilledSlotMask = 0U;
uint8_t rawCollectedCount = 0U;
uint8_t rawPendingSlotIndex = 0U;
uint8_t rawPendingColor = 0U;
ArmPose rawPendingSourcePose;
uint8_t rawConfirmationSampleCounts[4] = {
    0U, 0U, 0U, 0U};
int16_t rawConfirmationX[4] = {0, 0, 0, 0};
int16_t rawConfirmationY[4] = {0, 0, 0, 0};
uint32_t rawConfirmationLastMs[4] = {
    0UL, 0UL, 0UL, 0UL};

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

void finishActiveWorkAction() {
  const WorkActionKind completedAction = activeWorkAction;
  const uint32_t completedAtMs = millis();
  const uint32_t workElapsedMs =
      workActionStartMs == 0UL
          ? 0UL
          : completedAtMs - workActionStartMs;
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
  workActionPhase = WORK_PHASE_IDLE;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  armStandardPhase = ARM_STANDARD_IDLE;
  armTransferPhase = ARM_TRANSFER_IDLE;
  armTransferMapSource = false;
  armTransferMapDestination = false;
  workActionStartMs = 0UL;
  workVisionRequestStartMs = 0UL;
  workVisionRetryCount = 0U;
  markMissionProgress();
  SerialDebug.println("Workstation action complete");

  if (VISION_YANYAN_TEST_MODE &&
      visionYanyanPlacementSequenceComplete &&
      completedAction == WORK_ACTION_PROCESS) {
    disableDriveMotors();
    disableArmBaseMotor();
    commandGripperClose();
    programState = PROGRAM_FINISHED;
    commandStarted = false;
    startRequested = false;
    hmiSetRunStatus("MEASURE");
    hmiSetText("t1", "RING123");
    hmiSetTaskCounts();

    const uint32_t positioningElapsedMs =
        visionYanyanPositioningCompleteMs >=
                visionYanyanTestStartMs
            ? visionYanyanPositioningCompleteMs -
                  visionYanyanTestStartMs
            : 0UL;
    SerialDebug.println(
        "===== VISION YANYAN MEASUREMENT READY =====");
    printMeasuredRingMapSummary();
    SerialDebug.print(
        "[VISION YANYAN SUMMARY] positioning/work/total ms=");
    SerialDebug.print(positioningElapsedMs);
    SerialDebug.print("/");
    SerialDebug.print(workElapsedMs);
    SerialDebug.print("/");
    SerialDebug.println(
        completedAtMs - visionYanyanTestStartMs);
    SerialDebug.print(
        "[VISION YANYAN SUMMARY] final xy/r/confidence=");
    SerialDebug.print(latestMaixCoordinate.x);
    SerialDebug.print(",");
    SerialDebug.print(latestMaixCoordinate.y);
    SerialDebug.print("/");
    SerialDebug.print(latestMaixCoordinate.metric);
    SerialDebug.print("/");
    SerialDebug.println(latestMaixCoordinate.confidence);
    SerialDebug.print(
        "[VISION YANYAN SUMMARY] correction moves/fwd/left mm=");
    SerialDebug.print(visualCorrectionMoveCount);
    SerialDebug.print("/");
    SerialDebug.print(visualCorrectionForwardMm, 2);
    SerialDebug.print("/");
    SerialDebug.println(visualCorrectionLeftMm, 2);
    SerialDebug.println(
        "Measure ring 1/2/3 offsets now; power-cycle before "
        "another run.");
  }
}

void resetRawConfirmationWindow() {
  memset(
      rawConfirmationSampleCounts,
      0,
      sizeof(rawConfirmationSampleCounts));
  memset(
      rawConfirmationX,
      0,
      sizeof(rawConfirmationX));
  memset(
      rawConfirmationY,
      0,
      sizeof(rawConfirmationY));
  memset(
      rawConfirmationLastMs,
      0,
      sizeof(rawConfirmationLastMs));
}

bool confirmRawCoordinate(
    uint8_t color,
    int16_t &x,
    int16_t &y) {
  if (color < 1U || color > 4U) {
    return false;
  }
  const uint8_t colorIndex =
      static_cast<uint8_t>(color - 1U);
  const uint32_t nowMs = millis();
  const bool startsNewWindow =
      rawConfirmationSampleCounts[colorIndex] == 0U ||
      nowMs - rawConfirmationLastMs[colorIndex] >
          RAW_MAIN_CONFIRMATION_MAXIMUM_GAP_MS ||
      abs(static_cast<int>(
          x - rawConfirmationX[colorIndex])) >
          RAW_MAIN_CONFIRMATION_MAX_DELTA_PIXELS ||
      abs(static_cast<int>(
          y - rawConfirmationY[colorIndex])) >
          RAW_MAIN_CONFIRMATION_MAX_DELTA_PIXELS;

  if (startsNewWindow) {
    rawConfirmationSampleCounts[colorIndex] = 1U;
    rawConfirmationX[colorIndex] = x;
    rawConfirmationY[colorIndex] = y;
    rawConfirmationLastMs[colorIndex] = nowMs;
    return RAW_MAIN_CONFIRMATION_SAMPLES <= 1U;
  }

  if (rawConfirmationSampleCounts[colorIndex] < UINT8_MAX) {
    ++rawConfirmationSampleCounts[colorIndex];
  }
  x = static_cast<int16_t>(
      (static_cast<int32_t>(
           rawConfirmationX[colorIndex]) +
       x) /
      2);
  y = static_cast<int16_t>(
      (static_cast<int32_t>(
           rawConfirmationY[colorIndex]) +
       y) /
      2);
  rawConfirmationX[colorIndex] = x;
  rawConfirmationY[colorIndex] = y;
  rawConfirmationLastMs[colorIndex] = nowMs;
  return rawConfirmationSampleCounts[colorIndex] >=
         RAW_MAIN_CONFIRMATION_SAMPLES;
}

void beginStorageParkingBeforeWorkFinish() {
  commandStorageServoParkingPosition();
  /*
   * 结束标志只能在机械臂先完整回到标准状态后产生：M7安全零点、M6安全
   * 零点、M5标准0°。随后再单独把M5逆时针90°转到上电/行驶姿态。
   */
  beginArmStandardization();
  workStorageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  workActionPhase = WORK_PHASE_WAIT_STORAGE_PARK;
  SerialDebug.print("[WORK PARK] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, storage -> 165 deg; arm -> standard "
      "(M6/M7 safe-zero)");
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
  workActionStartMs = millis();
  workVisionRequestStartMs = 0UL;
  workVisionRetryCount = 0U;
  workRoundIndex =
      static_cast<uint8_t>(roundNumber - 1U);
  workItemIndex = 0U;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  visualCorrectionAccumulator = MotorPulses();
  visualCorrectionForwardMm = 0.0f;
  visualCorrectionLeftMm = 0.0f;
  visualCorrectionMoveCount = 0U;
  activeEndpointScanRing = 0U;
  activeEndpointScanPose = ArmPose();
  endpointFineVisionActive = false;
  activeEndpointServoMoveCount = 0U;
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
  resetRawConfirmationWindow();
  workLastMaixSequence =
      latestMaixCoordinate.sequence;
  resetCircleStabilityWindow();

  beginArmStandardization();
  if (kind == WORK_ACTION_RAW) {
    // 原料识别结果尚未知，载物盘先保持行驶位置；识别后再转到颜色对应槽位。
    commandStorageServoParkingPosition();
  } else {
    // 粗加工/暂存卸料始终从二维码序列的第一个槽位开始。
    commandStorageServoPosition(0U);
  }
  workStorageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  workActionPhase = WORK_PHASE_PREPARE;
  SerialDebug.print("[WORK PREPARE] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, waiting for arm standardization");
}

void beginRawItemVision() {
  workLastMaixSequence =
      latestMaixCoordinate.sequence;
  SerialDebug.print("[RAW VISION] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, collected=");
  SerialDebug.print(
      static_cast<unsigned int>(rawCollectedCount));
  SerialDebug.println(
      ", protocol-v2 request mode=8 (all colors)");
  beginMaixRequest(MAIXCAM_ALL_COLORS_REQUEST);
  workVisionRequestStartMs = millis();
  workActionPhase = WORK_PHASE_RAW_WAIT_RESULT;
}

void beginCircleVision() {
  /*
   * 配套Vision 5在多个候选圆中先检查三圆共线、近似等间距和等半径，
   * 协议v2以mode=9、target=2明确返回该三圆整体的中间成员。
   */
  resetCircleStabilityWindow();
  workLastMaixSequence =
      latestMaixCoordinate.sequence;
  beginMaixRequest(MAIXCAM_HOUGH_CIRCLE_REQUEST);
  workVisionRequestStartMs = millis();
  workActionPhase =
      WORK_PHASE_CIRCLE_WAIT_COORDINATE;
}

void beginEndpointVision() {
  workLastMaixSequence =
      latestMaixCoordinate.sequence;
  SerialDebug.print("[ENDPOINT VISION] t/ring=");
  SerialDebug.print(millis());
  SerialDebug.print("/");
  SerialDebug.println(activeEndpointScanRing);
  beginMaixRequest(MAIXCAM_ENDPOINT_CIRCLE_REQUEST);
  workVisionRequestStartMs = millis();
  workActionPhase =
      WORK_PHASE_ENDPOINT_WAIT_COORDINATE;
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
          M7_STANDARD_HEIGHT_MM) >
          ARM_AXIS_POSITION_TOLERANCE_MM) {
    routeFault(
      "Endpoint scan requires M6 at safe zero and M7 raised");
    return;
  }

  ArmPose nominalPose;
  if (!nominalRingPose(
          ringPosition,
          HOUGH_VISION_LOWER_MM,
          nominalPose)) {
    return;
  }

  activeEndpointScanRing = ringPosition;
  activeEndpointScanPose = nominalPose;
  endpointFineVisionActive = false;
  activeEndpointServoMoveCount = 0U;
  endpointCenteredConfirmationCount = 0U;
  endpointLocalSettleDeadlineMs = 0UL;
  endpointScanStartMs[ringPosition] = millis();
  endpointScanElapsedMs[ringPosition] = 0UL;
  workVisionRetryCount = 0U;
  workVisionRequestStartMs = 0UL;
  /*
   * 每个端点都先到-80°预置角，再从相同的逆时针方向逼近目标；
   * 整个端点搜索过程限制在标准坐标-80°～+80°。
   * M6也从完全回缩侧向外伸，使扫描和后续放料的回差方向一致。
   */
  startArmBaseStandardFrameDegrees(
      RING_SCAN_PRELOAD_ANGLE_DEGREES);
  workActionPhase =
      WORK_PHASE_ENDPOINT_WAIT_PRELOAD_BASE;
  SerialDebug.print(
      "[ENDPOINT SCAN] start ring/nominal M5/M6=");
  SerialDebug.print(ringPosition);
  SerialDebug.print("/");
  SerialDebug.print(
      nominalPose.standardFrameAngleDegrees,
      2);
  SerialDebug.print("/");
  SerialDebug.println(nominalPose.extensionMm, 2);
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
      "[RING MAP] invalidated by heading drift=");
  SerialDebug.println(driftDegrees, 2);
  routeFault("Chassis moved after endpoint mapping");
  return false;
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
  // 霍夫圆全部距离统一按72像素=41.75 mm换算；保留参数以维持调用接口。
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
  /*
   * 霍夫闭环已经满足像素条件，且机械臂已恢复标准状态。这里不改变XY
   * 累计量，只按当前路线目标航向进行一次静止IMU锁向。
   */
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

  ArmPose destination;
  uint8_t ringPosition = 0U;
  float lowerMm = PROCESS_PLACE_LOWER_MM;

  if (activeWorkAction == WORK_ACTION_PROCESS) {
    ringPosition =
        taskPositions[workRoundIndex][workItemIndex];
    lowerMm = PROCESS_PLACE_LOWER_MM;
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

  beginArmTransfer(
      containerPose(CONTAINER_PICK_LOWER_MM),
      destination,
      true,
      false,
      true);
  activeTransferPurpose =
      TRANSFER_PURPOSE_CONTAINER_TO_RING;
  workActionPhase = WORK_PHASE_WAIT_TRANSFER;
}

void beginReloadingTransfer() {
  if (!ringMapHeadingStillValid()) {
    return;
  }

  ArmPose source;
  const uint8_t ringPosition =
      taskPositions[workRoundIndex][workItemIndex];
  if (!ringPose(
          ringPosition,
          PROCESS_PLACE_LOWER_MM,
          source)) {
    return;
  }

  beginArmTransfer(
      source,
      containerPose(CONTAINER_PLACE_LOWER_MM),
      false,
      true,
      false);
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
        // 第三件放好后不再经过-5°，立即命令载物盘回到行驶位置165°。
        beginStorageParkingBeforeWorkFinish();
      } else {
        workVisionRetryCount = 0U;
        beginRawItemVision();
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

  consumeArmTransferCompletion();
  ++workItemIndex;
  if (VISION_YANYAN_TEST_MODE &&
      activeWorkAction == WORK_ACTION_PROCESS &&
      activeTransferPurpose ==
          TRANSFER_PURPOSE_CONTAINER_TO_RING &&
      workItemIndex >= 3U) {
    /*
     * 测量模式只放下三个物料。第三次传送已经把M5/M6/M7收回标准状态；
     * 再沿正式收尾状态机停好料盘并把M5转到行驶姿态，随后停在MEASURE。
     * 不执行正式粗加工流程中的“三件重新抓回”。
     */
    visionYanyanPlacementSequenceComplete = true;
    SerialDebug.println(
        "[VISION YANYAN] three placements complete; "
        "park arm/storage and hold for measurement");
    beginStorageParkingBeforeWorkFinish();
    return;
  }

  /*
   * 粗加工卸完第三件后还要从槽0开始重新装盘，因此仅该情况回槽0。
   * 粗加工重新装盘的第三件以及暂存最终卸料的第三件完成后，立即转165°；
   * 下一工位仍会显式转到槽0并等待，不依赖此处留下的角度。
   */
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
    commandStorageServoPosition(workItemIndex);
  }
  workStorageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  workActionPhase = WORK_PHASE_WAIT_STORAGE_SERVO;
}

void startVisualCorrectionRestore() {
  stopMaixRequest();
  if (motorPulsesAreZero(
          visualCorrectionAccumulator)) {
    /*
     * 即使没有发生视觉平移，也在机械臂三次搬运后重新锁定路线目标航向。
     * 否则臂的惯性造成的车体偏航会被带入下一段长距离开环平移。
     */
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
  switch (activeWorkAction) {
    case WORK_ACTION_RAW:
      return RAW_ACTION_TIMEOUT_MS;
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

  if ((workActionPhase == WORK_PHASE_RAW_WAIT_RESULT ||
       workActionPhase == WORK_PHASE_CIRCLE_WAIT_COORDINATE ||
       workActionPhase ==
           WORK_PHASE_ENDPOINT_WAIT_COORDINATE) &&
      workVisionRequestStartMs != 0UL &&
      nowMs - workVisionRequestStartMs >=
          VISION_RESULT_TIMEOUT_MS) {
    if (workVisionRetryCount >= VISION_MAXIMUM_RETRIES) {
      routeFault("Vision result timeout");
      return;
    }

    ++workVisionRetryCount;
    SerialDebug.print("[VISION RETRY] ");
    SerialDebug.print(workVisionRetryCount);
    SerialDebug.print("/");
    SerialDebug.println(VISION_MAXIMUM_RETRIES);
    if (workActionPhase == WORK_PHASE_RAW_WAIT_RESULT) {
      beginRawItemVision();
    } else if (
        workActionPhase ==
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

    case WORK_PHASE_PREPARE:
      if (serviceArmStandardization() &&
          deadlineReached(
              workStorageServoDeadlineMs)) {
        SerialDebug.print("[WORK PREPARE] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, arm and storage settle complete");
        armStandardPhase = ARM_STANDARD_IDLE;
        if (activeWorkAction == WORK_ACTION_RAW) {
          beginRawItemVision();
        } else {
          /*
           * 粗加工区和暂存区都冻结底盘后顺序实测1、3号端点，
           * 由两端中点计算2号。正常流程不再进入旧mode9中圆
           * 定位，也不在端点建图后修正底盘。
           */
          startPreEndpointHeadingCorrection();
        }
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

      if (REQUIRE_RAW_PICK_QR_ORDER) {
        const uint8_t expectedColor =
            taskColors[workRoundIndex][rawCollectedCount];
        if (detectedColor != expectedColor) {
          /*
           * 严格顺序模式下，非目标色只丢弃本次结果，不能清掉尚未过期的
           * 目标色首样本。
           */
          SerialDebug.print(
              "[RAW WAIT] expected/detected color=");
          SerialDebug.print(expectedColor);
          SerialDebug.print("/");
          SerialDebug.println(detectedColor);
          beginRawItemVision();
          break;
        }
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
      if (REQUIRE_RAW_PICK_QR_ORDER &&
          slotIndex != rawCollectedCount) {
        routeFault("Raw color-to-slot order mismatch");
        break;
      }
      const uint8_t slotBit =
          static_cast<uint8_t>(1U << slotIndex);
      if ((rawFilledSlotMask & slotBit) != 0U) {
        /*
         * 任意顺序模式下，同一颜色可能继续可见；槽位掩码保证每个任务色
         * 只抓一次，并保持“颜色 -> 二维码槽位”的后续加工映射。
         */
        SerialDebug.print(
            "[RAW IGNORE] slot already filled, color/slot=");
        SerialDebug.print(detectedColor);
        SerialDebug.print("/");
        SerialDebug.println(slotIndex);
        beginRawItemVision();
        break;
      }

      if (!confirmRawCoordinate(
              detectedColor, x, y)) {
        SerialDebug.println(
            "[RAW CONFIRM] first stable sample accepted; "
            "requesting independent confirmation");
        beginRawItemVision();
        break;
      }
      resetRawConfirmationWindow();

      ArmPose source;
      if (!rawTargetPose(
              static_cast<float>(x),
              static_cast<float>(y),
              source)) {
        break;
      }

      stopMaixRequest();
      workVisionRetryCount = 0U;
      rawPendingColor = detectedColor;
      rawPendingSlotIndex = slotIndex;
      rawPendingSourcePose = source;
      commandStorageServoPosition(rawPendingSlotIndex);
      workStorageServoDeadlineMs =
          millis() + STORAGE_SERVO_SETTLE_MS;
      workActionPhase =
          WORK_PHASE_RAW_WAIT_STORAGE_POSITION;
      SerialDebug.print(
          "[RAW SLOT] color/QR slot/angle=");
      SerialDebug.print(rawPendingColor);
      SerialDebug.print("/");
      SerialDebug.print(rawPendingSlotIndex);
      SerialDebug.print("/");
      SerialDebug.println(
          STORAGE_SERVO_POSITIONS_DEGREES[
              rawPendingSlotIndex],
          1);
      break;
    }

    case WORK_PHASE_RAW_WAIT_STORAGE_POSITION:
      if (!deadlineReached(
              workStorageServoDeadlineMs)) {
        break;
      }
      beginArmTransfer(
          rawPendingSourcePose,
          containerPose(CONTAINER_PLACE_LOWER_MM),
          false);
      activeTransferPurpose =
          TRANSFER_PURPOSE_RAW_TO_CONTAINER;
      workActionPhase = WORK_PHASE_WAIT_TRANSFER;
      break;

    case WORK_PHASE_ENDPOINT_WAIT_PRE_SCAN_HEADING:
      if (updateHeadingLock(MOTION_TIMEOUT_MS)) {
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

    case WORK_PHASE_ENDPOINT_WAIT_PRELOAD_BASE:
      if (!armMotors.isM5Running()) {
        armStandardDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_PRELOAD_SETTLE;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_PRELOAD_SETTLE:
      if (deadlineReached(armStandardDeadlineMs)) {
        startArmBaseStandardFrameDegrees(
            activeEndpointScanPose
                .standardFrameAngleDegrees);
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE:
      if (!armMotors.isM5Running()) {
        armStandardDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE_SETTLE;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_SEARCH_BASE_SETTLE:
      if (deadlineReached(armStandardDeadlineMs) &&
          startExtensionToMm(
              activeEndpointScanPose.extensionMm)) {
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_SEARCH_EXTENSION:
      if (extensionMoveFinished() &&
          startLiftToHeightMm(
              activeEndpointScanPose.heightMm)) {
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
        beginEndpointVision();
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
      workVisionRequestStartMs = millis();
      markMissionProgress();
      (void)targetId;

      if (activeEndpointScanRing != 1U &&
          activeEndpointScanRing != 3U) {
        routeFault("Endpoint response has no active ring");
        break;
      }

      const uint16_t radiusPixels =
          latestMaixCoordinate.metric;
      const uint16_t confidence =
          latestMaixCoordinate.confidence;
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
        if (activeEndpointServoMoveCount >=
            ENDPOINT_MAXIMUM_SERVO_MOVES) {
          SerialDebug.print(
              "[ENDPOINT SERVO] failed ring/stage/error px=");
          SerialDebug.print(ring);
          SerialDebug.print("/");
          SerialDebug.print(
              endpointFineVisionActive ? "fine" : "coarse");
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
        if (!startExtensionToMm(targetPose.extensionMm)) {
          break;
        }
        startArmBaseStandardFrameDegrees(
            targetPose.standardFrameAngleDegrees);
        activeEndpointScanPose = targetPose;
        ++activeEndpointServoMoveCount;

        SerialDebug.print(
            "[ENDPOINT SERVO] local move ring/stage/step/"
            "error-px/measured-mm/command-mm/M5/M6=");
        SerialDebug.print(ring);
        SerialDebug.print("/");
        SerialDebug.print(
            endpointFineVisionActive ? "fine" : "coarse");
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
            WORK_PHASE_ENDPOINT_WAIT_LOCAL_MOVE;
        break;
      }

      if (!endpointFineVisionActive) {
        stopMaixRequest();
        workVisionRequestStartMs = 0UL;
        workVisionRetryCount = 0U;
        endpointFineVisionActive = true;
        endpointCenteredConfirmationCount = 0U;
        activeEndpointScanPose.heightMm =
            -ENDPOINT_FINE_VISION_LOWER_MM;
        if (!startLiftToHeightMm(
                activeEndpointScanPose.heightMm)) {
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

      if (endpointCenteredConfirmationCount < 255U) {
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

      beginArmStandardization();
      workActionPhase =
          WORK_PHASE_ENDPOINT_WAIT_ARM_STANDARD;
      break;
    }

    case WORK_PHASE_ENDPOINT_WAIT_LOCAL_MOVE:
      if (!armMotors.isM5Running() &&
          extensionMoveFinished()) {
        activeEndpointScanPose.extensionMm =
            extensionAxis.currentMm;
        endpointLocalSettleDeadlineMs =
            millis() + ENDPOINT_LOCAL_MOVE_SETTLE_MS;
        workActionPhase =
            WORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE;
      }
      break;

    case WORK_PHASE_ENDPOINT_WAIT_LOCAL_SETTLE:
      if (deadlineReached(
              endpointLocalSettleDeadlineMs)) {
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

    case WORK_PHASE_ENDPOINT_WAIT_ARM_STANDARD:
      if (serviceArmStandardization()) {
        armStandardPhase = ARM_STANDARD_IDLE;
        if (activeEndpointScanRing == 1U) {
          beginEndpointScan(3U);
          break;
        }
        if (activeEndpointScanRing != 3U) {
          routeFault("Endpoint scan sequence corrupted");
          break;
        }
        if (!buildMeasuredRingMap()) {
          break;
        }

        endpointMapCompleteMs = millis();
        workItemIndex = 0U;
        if (VISION_YANYAN_TEST_MODE &&
            visionYanyanPositioningCompleteMs == 0UL) {
          visionYanyanPositioningCompleteMs =
              endpointMapCompleteMs;
        }
        printMeasuredRingMapSummary();
        SerialDebug.println(
            "[ENDPOINT MAP] complete; chassis remains "
            "frozen, start ring transfers");
        workActionPhase = WORK_PHASE_START_UNLOAD;
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
            " ms, positioning complete; restore arm standard");
        beginArmStandardization();
        workActionPhase =
            WORK_PHASE_CIRCLE_WAIT_ARM_STANDARD;
      }
      break;
    }

    case WORK_PHASE_CIRCLE_WAIT_CHASSIS:
      if (updateHeadingLock(MOTION_TIMEOUT_MS)) {
        beginCircleVision();
      }
      break;

    case WORK_PHASE_CIRCLE_WAIT_ARM_STANDARD:
      if (serviceArmStandardization()) {
        armStandardPhase = ARM_STANDARD_IDLE;
        SerialDebug.print("[HOUGH ARM] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, arm standard restored; correct heading");
        startPostVisionHeadingCorrection();
      }
      break;

    case WORK_PHASE_CIRCLE_WAIT_POST_VISION_HEADING:
      if (updateHeadingLock(MOTION_TIMEOUT_MS)) {
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
        if (VISION_YANYAN_TEST_MODE &&
            visionYanyanPositioningCompleteMs == 0UL) {
          visionYanyanPositioningCompleteMs = millis();
        }
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
            " ms, standard confirmed; M5 CCW 90 deg "
            "to travel pose");
        startArmBaseStandardFrameDegrees(
            ARM_BASE_TRAVEL_STANDARD_FRAME_DEGREES);
        workActionPhase = WORK_PHASE_WAIT_TRAVEL_BASE;
      }
      break;

    case WORK_PHASE_WAIT_TRAVEL_BASE:
      if (!armMotors.isM5Running()) {
        armStandardDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        workActionPhase =
            WORK_PHASE_WAIT_TRAVEL_BASE_SETTLE;
      }
      break;

    case WORK_PHASE_WAIT_TRAVEL_BASE_SETTLE:
      if (deadlineReached(armStandardDeadlineMs)) {
        SerialDebug.print("[WORK TRAVEL] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, COMPLETE: M5 travel 0 deg, "
            "M6/M7 safe-zero");
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
  workActionPhase = WORK_PHASE_IDLE;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  armStandardPhase = ARM_STANDARD_IDLE;
  armTransferPhase = ARM_TRANSFER_IDLE;
  armTransferMapSource = false;
  armTransferMapDestination = false;
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
  resetRawConfirmationWindow();
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
  /*
   * 回区时2、4侧朝西（180°），因此外廓X/Y半尺寸仍分别为115/150 mm。
   * 这是名义路线门禁，不是实际定位反馈；接入位置/边线传感器后必须用
   * 实测位姿替换centerX/centerY。
   */
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
  /*
   * 路线命令只在这里转换为“开始做什么”。函数不会阻塞等待完成；
   * loop()持续服务四轮、IMU和二维码，updateRoute()负责判断何时推进。
   */
  turnMotionEnabled = false;
  workstationApproachEnabled = false;
  translationPreciseArrivalEnabled = false;
  translationCentralChannelEnabled = false;

  if (isTranslationCommand(command.type)) {
    translationRemainingMm = command.value;
    translationPreciseArrivalEnabled =
        command.preciseArrival;
    translationCentralChannelEnabled =
        command.centralChannel;
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
      // 停稳后最多前探500 mm；只有扫码并回到停车点后才允许推进路线。
      startQrScanAction();
      break;

    case COMMAND_RAW_ACTION:
      // value=1/2表示同一次比赛的第一批/第二批。
      activeCompetitionRound =
          static_cast<uint8_t>(command.value);
      rawActionFinished = false;
      hmiSetRunStatus(
          command.value == 1 ? "RAW1" : "RAW2");
      if (!PATH_ONLY_TEST) {
        beginWorkAction(
            WORK_ACTION_RAW,
            static_cast<uint8_t>(command.value));
      }
      break;

    case COMMAND_PROCESS_ACTION:
      // 霍夫圆定位后，按任务码放3个，再按同一顺序抓回载物盘。
      activeCompetitionRound =
          static_cast<uint8_t>(command.value);
      processActionFinished = false;
      hmiSetRunStatus(
          command.value == 1 ? "PROCESS1" : "PROCESS2");
      if (!PATH_ONLY_TEST) {
        beginWorkAction(
            WORK_ACTION_PROCESS,
            static_cast<uint8_t>(command.value));
      }
      break;

    case COMMAND_STORAGE_ACTION:
      // 第一批按第二组位置平放；第二批按颜色映射到第一批上码垛。
      activeCompetitionRound =
          static_cast<uint8_t>(command.value);
      storageActionFinished = false;
      hmiSetRunStatus(
          command.value == 1 ? "STORAGE1" : "STORAGE2");
      if (!PATH_ONLY_TEST) {
        beginWorkAction(
            WORK_ACTION_STORAGE,
            static_cast<uint8_t>(command.value));
      }
      break;

    case COMMAND_FINAL_ALIGN:
      /*
       * 先把旧版“无条件完成”升级为完整外廓的名义几何门禁。当前硬件
       * 未给出向下红外/XY协议，故不能伪称实测闭环；正式比赛接入后应在
       * 此状态持续微调，并仅在实测四角均入区时置完成。
       */
      hmiSetRunStatus("ALIGN");
      if (!plannedFinalFootprintIsInsideSelectedZone()) {
        routeFault("Planned final footprint outside selected zone");
      } else {
        finalAlignmentFinished = true;
        SerialDebug.println(
            "Final planned footprint is inside selected start zone");
      }
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

  activeRouteCommand = resolveRouteCommand(route[routeIndex]);
  commandStartMs = millis();
  headingStableStartMs = 0;
  motorsArrivedStartMs = 0;
  printCurrentCommand(activeRouteCommand);
  startMotionCommand(activeRouteCommand);
  if (programState == PROGRAM_RUNNING) {
    commandStarted = true;
    markMissionProgress();
  }
}

bool timedActionFinished(
    bool externalFeedback,
    uint32_t testDurationMs) {
  /*
   * 路径测试模式按固定时间伪造工位完成；正式接入机械臂后必须由对应
   * 动作状态机设置externalFeedback，不能把固定测试等待当作规则任务完成。
   */
  if (PATH_ONLY_TEST) {
    return millis() - commandStartMs >= testDurationMs;
  }
  return externalFeedback;
}

void finishRawActionIfNeeded() {
  if (PATH_ONLY_TEST) {
    // 每批原料区抓取3个；这里只更新显示计数，没有验证颜色或抓取成功。
    correctGrabCount += 3;
    hmiSetTaskCounts();
  }
}

void finishProcessActionIfNeeded() {
  if (PATH_ONLY_TEST) {
    // 每批粗加工区先放下3个、再抓回3个；这里只模拟统计结果。
    correctPlacementCount += 3;
    correctGrabCount += 3;
    hmiSetTaskCounts();
  }
}

void finishStorageActionIfNeeded() {
  if (PATH_ONLY_TEST) {
    // 每批暂存区放置3个；第二批的同色码垛正确性在此模式下未验证。
    correctPlacementCount += 3;
    hmiSetTaskCounts();
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
  commandStarted = false;
  hmiSetRunStatus("FINISH");

  if (SHOW_RESULT_PAGE_ON_FINISH) {
    hmiCommand("page page0");
    hmiSetTaskCounts();
  }

  SerialDebug.println("Route finished");
}

void updateRoute() {
  /*
   * 固定路线状态机的统一“完成判定”：
   *   平移/转向 -> 四轮到达脉冲目标且IMU航向稳定；
   *   二维码     -> scanFlag有效（或明确关闭强制扫码）；
   *   三个工位   -> 路径测试定时，或正式机械臂反馈；
   *   最终对齐   -> 不增加位移的启停区同步点；
   *   HOLD       -> 定时结束。
   * advanceRoute()只推进数组下标，不做坐标重定位。
   */
  if (VISION_YANYAN_TEST_MODE) {
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
    case COMMAND_MOVE_SIDE_12_MM:
    case COMMAND_MOVE_SIDE_34_MM:
    case COMMAND_MOVE_SIDE_13_MM:
    case COMMAND_MOVE_SIDE_24_MM:
      // 中央直线最多2000 mm一段，非中央最多1100 mm；段末只锁航向。
      if (updateHeadingLock(MOTION_TIMEOUT_MS)) {
        if (translationRemainingMm > 0) {
          startTranslationSegment(command.type);
        } else {
          advanceRoute();
        }
      }
      break;

    case COMMAND_ARM_BASE_HOME:
      if (!armMotors.isM5Running()) {
        SerialDebug.println(
            "Arm base confirmed at travel old 0");
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
      if (updateQrScanAction()) {
        hmiSetRunStatus("RUN");
        resetQrScanActionState();
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
    /*
     * 对应规则的“一键式启动”。机械臂保持上电/行驶姿态（旧坐标0°）；
     * 实际摆放正确时该命令不产生M5位移，只使库目标与行驶姿态一致。
     * beginRoute()仍要求先收到有效IMU帧。
     */
    startArmBaseRotationToDegrees(
        static_cast<float>(
            ARM_BASE_TRAVEL_OLD_FRAME_DEGREES));
  }
}

void onStartButtonDoubleClick() {
  if (programState != PROGRAM_WAITING || startRequested) {
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

bool configureVisionYanyanTask() {
  /*
   * 槽0/1/2依次放到环1/2/3；颜色值只用于满足正式TaskPlan契约，
   * 测量模式不进行颜色识别。使用正式解析器可避免测试路径绕过数组边界。
   */
  constexpr char TEST_TASK_CODE[] =
      "123+123+123+123";
  competition::TaskPlan testPlan;
  if (competition::parseTaskCode(
          TEST_TASK_CODE, testPlan) !=
      competition::TASK_CODE_OK) {
    routeFault("Vision Yanyan test task is invalid");
    return false;
  }

  taskPlan = testPlan;
  memcpy(
      qrData,
      TEST_TASK_CODE,
      sizeof(TEST_TASK_CODE));
  scanFlag = true;
  taskCodeDecoded = true;
  hmiSetText("t1", "LOAD123");
  hmiShowTaskCode(TEST_TASK_CODE);
  SerialDebug.println(
      "[VISION YANYAN] fixed mapping: tray slot 0/1/2 "
      "-> ring 1/2/3");
  return true;
}

void beginRoute() {
  /*
   * PB9回调保持原一键启动主体；真正的单次比赛门控放在这里。FAULT/FINISHED
   * 状态不能把当前终点姿态重新当作所选启停区的发车姿态，必须断电并人工恢复上电
   * 行驶姿态（M5旧0°、M6抵住回缩端、M7位于物理最高点）；上电后程序
   * 会自动将M6向外、M7向下各移动10 mm并建立两个安全工作零点。
   */
  if (programState != PROGRAM_WAITING) {
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

  /*
   * 这里把命令下标、IMU相对航向和软件脉冲位置清零，默认机器人已经按
   * 注释放在已选择启停区的设计起点。它不是回零/定位过程，不会测量当前世界坐标；
   * 若实际摆放位置或朝向不同，整条开环路线会整体带偏。
   */
  routeIndex = 0;
  activeCompetitionRound = 0;
  commandStarted = false;
  resetQrScanActionState();
  competitionStartMs = millis();
  commandStartMs = competitionStartMs;
  lastMissionProgressMs = competitionStartMs;
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

  routeImuReferenceDegrees = imuCounterClockwiseDegrees;
  targetCounterClockwiseHeadingDegrees = 0.0f;

  correctGrabCount = 0;
  correctPlacementCount = 0;
  rawActionFinished = false;
  processActionFinished = false;
  storageActionFinished = false;
  finalAlignmentFinished = false;

  if (ENABLE_QR_RECEIVER &&
      !VISION_YANYAN_TEST_MODE) {
    resetQrReceiver();
  }
  hmiSetTaskCounts();
  if (VISION_YANYAN_TEST_MODE) {
    if (!configureVisionYanyanTask()) {
      startRequested = false;
      return;
    }
    hmiSetRunStatus("VTEST");
  } else {
    hmiSetText(
        "t1",
        ENABLE_QR_RECEIVER ? "QRWAIT" : "BYPASS");
    hmiSetText("t3", "000+000+");
    hmiSetText("t8", "000+000");
    hmiSetRunStatus("RUN");
  }
  commandGripperClose();
  commandStorageServoParkingPosition();

  for (uint8_t i = 0; i < 4; ++i) {
    motors[i]->setCurrentPosition(0);
  }

  // 上一次运行结束时可能停留在最终低速档，每次启动都恢复巡航参数。
  setDriveMotionProfile(MAXIMUM_STEP_RATE, STEP_ACCELERATION);
  enableDriveMotors();
  programState = PROGRAM_RUNNING;
  startRequested = false;
  imuWaitStatusDisplayed = false;

  if (VISION_YANYAN_TEST_MODE) {
    visionYanyanPlacementSequenceComplete = false;
    visionYanyanTestStartMs = millis();
    visionYanyanPositioningCompleteMs = 0UL;
    SerialDebug.println(
        "===== VISION YANYAN TEST START =====");
    SerialDebug.println(
        "Assumption: M5 old 0 deg / M6 safe zero "
        "at retract stop / M7 physical highest; startup will "
        "move both axes 10 mm, slots loaded");
    SerialDebug.println(
        "Vision 5.2.1 mode10: scan ring 1, scan ring 3, "
        "derive ring 2; chassis stays frozen");
    beginWorkAction(WORK_ACTION_PROCESS, 1U);
    return;
  }

  SerialDebug.println("Route started");
  SerialDebug.print("Locked start zone: ");
  SerialDebug.println(
      static_cast<unsigned int>(selectedStartZone));
}

void abortRoute() {
  resetQrScanActionState();
  invalidateArmLinearReference();
  cancelCompetitionAction();
  emergencyStopArmLinearAxes();
  disableDriveMotors();
  disableArmBaseMotor();
  programState = PROGRAM_FAULT;
  commandStarted = false;
  abortRequested = false;
  startRequested = false;
  hmiSetRunStatus("STOP");
  SerialDebug.println("Route aborted by long press");
}

void serviceCompetitionWatchdogs() {
  if (programState != PROGRAM_RUNNING) {
    return;
  }

  const uint32_t nowMs = millis();
  if (ENABLE_COMPETITION_TIME_LIMIT &&
      nowMs - competitionStartMs >=
          COMPETITION_TIME_LIMIT_MS -
              COMPETITION_HARD_STOP_MARGIN_MS) {
    routeFault("Competition hard time limit");
    return;
  }

  if (nowMs - lastMissionProgressMs >=
      MISSION_PROGRESS_TIMEOUT_MS) {
    routeFault("Mission progress watchdog timeout");
  }
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

  // 初始化期间先保持所有步进驱动器失能。
  digitalWrite(DRIVE_ENABLE_PIN, HIGH);

  for (uint8_t i = 0; i < 4; ++i) {
    // AccelStepper 的构造函数会配置引脚，但这里在全部串口初始化后
    // 再明确配置一次，防止任何外设复用覆盖 STEP/DIR GPIO。
    motors[i]->enableOutputs();
    motors[i]->setMinPulseWidth(MINIMUM_STEP_WIDTH_US);
  }

  /*
   * 仅初始化库中的 M5，明确不启动库内的 M6/M7 串口，避免与当前
   * 非阻塞 M6/M7 状态机共用 PA2/PA3 时产生双重串口对象冲突。
   */
  armMotors.beginM5();
  armMotors.setM5MotionProfile(
      ARM_BASE_MAXIMUM_STEP_RATE,
      ARM_BASE_STEP_ACCELERATION);
  /*
   * 上电时必须人工保证M5位于旧坐标0°行驶姿态、M6抵住回缩端、M7升到
   * 最高；稍后程序会令M6前伸10 mm并建立安全工作零点。
   */
  armMotors.setM5CurrentAngle(0.0f);
  armMotors.disableM5();

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
      VISION_YANYAN_TEST_MODE
          ? "LOAD123"
          : (ENABLE_QR_RECEIVER ? "QRWAIT" : "BYPASS"));
  hmiSetText("t3", "000+000+");
  hmiSetText("t8", "000+000");
  hmiSetRunStatus(
      !armLinearReferenceValid
          ? "M6INIT"
          : (VISION_YANYAN_TEST_MODE
                 ? "VREADY"
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
  SerialMaixcam.begin(MAIXCAM_BAUDRATE);
  maixcamSerialInitialized = true;
  while (SerialMaixcam.available()) {
    SerialMaixcam.read();
  }
  stopMaixRequest();

  /*
   * M6/M7驱动器与STM32通常同时上电；已验证的独立例程会先等待
   * 1500 ms。保证驱动器完成自检后再发第一次使能命令。
   */
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

  SerialDebug.print("M6 pulses/mm=");
  SerialDebug.println(M6_PULSES_PER_MM, 4);
  SerialDebug.print("M7 pulses/mm=");
  SerialDebug.println(M7_PULSES_PER_MM, 4);
}

} // namespace

void setup() {
  SerialDebug.begin(DEBUG_BAUDRATE);
  SerialHmi.begin(HMI_BAUDRATE);
  SerialImu.begin(IMU_BAUDRATE);
  if (ENABLE_QR_RECEIVER) {
    SerialQr.begin(QR_BAUDRATE);
  }
  initializeManipulationHardware();

  analogReadResolution(12);
  pinMode(BATTERY_ADC_PIN, INPUT_ANALOG);

  // 串口初始化完成后最后配置电机 GPIO，确保 PA1 保持为 M4 STEP 输出。
  initializeMotorOutputs();

  startButton.reset();
  startButton.attachClick(onStartButtonClick);
  startButton.attachDoubleClick(onStartButtonDoubleClick);
  startButton.attachLongPressStart(onStartButtonLongPress);

  initializeHmi();
  const bool armWorkingZerosReady =
      establishArmLinearSafeWorkingZeros();
  hmiSetRunStatus(
      armWorkingZerosReady
          ? (VISION_YANYAN_TEST_MODE
                 ? "VREADY"
                 : (selectedStartZone == START_ZONE_1
                        ? "READY1"
                        : "READY2"))
          : "ARMZEROERR");
  if (ENABLE_QR_RECEIVER) {
    resetQrReceiver();
  }

  if (!armWorkingZerosReady) {
    SerialDebug.println(
        "Controller NOT READY: M6/M7 failed to establish "
        "their 10 mm safe working zeros");
  } else if (VISION_YANYAN_TEST_MODE) {
    SerialDebug.println(
        "VISION YANYAN rough-station placement test ready");
    SerialDebug.println(
        "Load tray slots 0/1/2; face the middle ring; "
        "click PB9 to scan 1/3 and direct-place 1/2/3; "
        "long-press to stop");
  } else {
    SerialDebug.println("GongChuang route controller ready");
    SerialDebug.println(
        "Double-click PB9 to select zone; click to start; "
        "long-press to stop");
  }
  if (!ENABLE_COMPETITION_TIME_LIMIT) {
    SerialDebug.println(
        "DEBUG: 180-second competition hard stop is disabled");
  }
}

void loop() {
  /*
   * 电机 run()、按钮、IMU、二维码和状态机都采用非阻塞服务。
   * 运行过程中不要加入 delay()、舵机 wait() 或 runToPosition()。
   */
  runAllMotors();
  armMotors.serviceM5();
  startButton.tick();
  receiveImuData();
  serviceArmLinearAxes();
  serviceMaixcam();
  if (ENABLE_QR_RECEIVER) {
    receiveQrData();
  }
  updateHmiYaw();
  serviceBatteryVoltage();

  if (abortRequested) {
    /*
     * RUN阶段及点击后的IMUWAIT阶段立即急停；尚未点击、已完赛或已故障时
     * 忽略长按，避免把FINISH结果页覆盖为STOP。
     */
    if (programState == PROGRAM_RUNNING ||
        (programState == PROGRAM_WAITING &&
         startRequested)) {
      abortRoute();
    } else {
      abortRequested = false;
    }
  }

  if (startRequested && programState != PROGRAM_RUNNING) {
    beginRoute();
  }

  serviceCompetitionWatchdogs();
  serviceCompetitionAction();
  updateRoute();
}
