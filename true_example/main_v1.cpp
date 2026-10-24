#include <AccelStepper.h>
#include <ArmMotorController.h>
#include <Arduino.h>
#include <FashionStar_UartServo.h>
#include <JY901.h>
#include <MecanumKinematics.h>
#include <OneButton.h>

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
 * 两种姿态下M6都必须缩回最短、M7都必须升到最高。
 */
constexpr int32_t ARM_BASE_TRAVEL_OLD_FRAME_DEGREES = 0;
constexpr int32_t ARM_BASE_STANDARD_OLD_FRAME_DEGREES = -90;
constexpr float ARM_BASE_TRAVEL_STANDARD_FRAME_DEGREES = 90.0f;
constexpr int32_t ARM_BASE_HOME_ANGLE_DEGREES =
    ARM_BASE_TRAVEL_OLD_FRAME_DEGREES;
// 容器夹取/释放位：相对标准状态顺时针100°（标准坐标顺时针为负）。
constexpr float ARM_CONTAINER_CLOCKWISE_DEGREES = -100.0f;
/*
 * M5最高速度保持60000 pulse/s，加速度由35000减半为17500 pulse/s²；
 * 短行程受加速度限制，实际不会瞬间达到最高速度。这里只设置M5动作
 * 曲线，不设置全程运行时长停机限制。
 */
const float ARM_BASE_MAXIMUM_STEP_RATE = 60000.0f;
const float ARM_BASE_STEP_ACCELERATION = 17500.0f;
// M5脉冲到位后的环节衔接等待按原值减半，仍保留50 ms机械消振。
constexpr uint32_t ARM_BASE_SETTLE_MS = 50UL;

// M6：模数1、分度圆直径35 mm的齿轮齿条；M7：T8x12，导程12 mm。
constexpr uint8_t ARM_EXTENSION_ADDRESS = 6U;
constexpr uint8_t ARM_LIFT_ADDRESS = 7U;
constexpr float FULL_STEPS_PER_REVOLUTION = 200.0f;
// M6、M7驱动器均已设置为256细分；M5仍由独立库按其原16细分管理。
constexpr float ARM_LINEAR_MICROSTEPS = 256.0f;
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
/*
 * EMM使用0~255加速度档位。机械臂相关电机加速度统一减半：
 * M6由235减半并四舍五入为118，M7由250减半为125；最高转速不变。
 */
constexpr uint8_t M6_ACCELERATION = 118U;
constexpr uint16_t M7_SPEED_RPM = 2496U;
constexpr uint8_t M7_ACCELERATION = 125U;
constexpr uint32_t ARM_LINEAR_POWER_ON_SETTLE_MS = 1500UL;
constexpr uint32_t ARM_AXIS_COMMAND_GUARD_MS = 30UL;
constexpr uint32_t ARM_AXIS_STATUS_INTERVAL_MS = 30UL;
constexpr uint32_t ARM_AXIS_MINIMUM_ON_POSITION_MS = 120UL;
constexpr uint32_t ARM_AXIS_MINIMUM_TIMEOUT_MS = 5000UL;
constexpr uint32_t ARM_AXIS_MAXIMUM_TIMEOUT_MS = 30000UL;
constexpr uint32_t ARM_AXIS_STARTUP_RESPONSE_TIMEOUT_MS = 150UL;
// 0x04堵转位或0x08堵转保护位都必须连续出现3帧才判定为持续堵转。
constexpr uint8_t ARM_AXIS_STALL_CONFIRMATION_SAMPLES = 3U;
static_assert(
    ARM_AXIS_STALL_CONFIRMATION_SAMPLES >= 3U,
    "Arm stall shutdown requires at least 3 consecutive samples");

constexpr float M6_STANDARD_EXTENSION_MM = 0.0f;
constexpr float M7_STANDARD_HEIGHT_MM = 0.0f;
constexpr float M6_MAXIMUM_EXTENSION_MM = 150.0f;
constexpr float M7_MINIMUM_HEIGHT_MM = -160.0f;
constexpr float ARM_AXIS_POSITION_TOLERANCE_MM = 0.05f;
constexpr float RAW_PICK_LOWER_MM = 73.0f;
constexpr float HOUGH_VISION_LOWER_MM = 90.0f;
constexpr float HOUGH_VISION_HEIGHT_MM =
    M7_STANDARD_HEIGHT_MM - HOUGH_VISION_LOWER_MM;
// 容器抓取/释放时M6由原来的0 mm再伸长10 mm。
constexpr float CONTAINER_EXTENSION_MM = 10.0f;
// 从容器取料及向容器放料均相对最高点下降40 mm。
constexpr float CONTAINER_PICK_LOWER_MM = 40.0f;
constexpr float CONTAINER_PLACE_LOWER_MM = 40.0f;
constexpr float PROCESS_PLACE_LOWER_MM = 150.0f;
constexpr float STORAGE_ROUND1_PLACE_LOWER_MM = 150.0f;
constexpr float STORAGE_ROUND2_PLACE_LOWER_MM = 90.0f;
// 只调整向粗加工区/暂存区释放时的高度；随后从粗加工区抓回仍用原高度。
constexpr float RING_RELEASE_LOWER_REDUCTION_MM = 5.0f;

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

constexpr uint8_t MAIXCAM_FRAME_HEADER = 0xAA;
constexpr uint8_t MAIXCAM_FRAME_TAIL = 0xBB;
constexpr uint8_t MAIXCAM_STOP_REQUEST = 0x00;
constexpr uint8_t MAIXCAM_ALL_COLORS_REQUEST = 0x08;
constexpr uint8_t MAIXCAM_HOUGH_CIRCLE_REQUEST = 0x09;
constexpr uint32_t MAIXCAM_REQUEST_REPEAT_MS = 1000UL;
constexpr uint32_t MAIXCAM_MODE_SWITCH_GUARD_MS = 100UL;
constexpr uint32_t MAIXCAM_COORDINATE_STALE_MS = 1500UL;
constexpr size_t MAIXCAM_LINE_CAPACITY = 32U;
constexpr int16_t IMAGE_CENTER_X = 160;
constexpr int16_t IMAGE_CENTER_Y = 120;
constexpr int16_t IMAGE_MAX_X = 319;
constexpr int16_t IMAGE_MAX_Y = 239;
constexpr float PIXEL_MAPPING_SWITCH_RADIUS_PIXELS = 50.0f;
constexpr float RAW_NEAR_MM_PER_PIXEL = 40.0f / 72.0f;
constexpr float RAW_FAR_MM_PER_PIXEL = 70.0f / 72.0f;
constexpr float CIRCLE_MM_PER_PIXEL = 35.0f / 72.0f;
constexpr float ARM_PIVOT_TO_CAMERA_CENTER_MM = 135.0f;
constexpr float SIDE_RING_CENTER_OFFSET_MM = 150.0f;
// 1、3号侧圈在同类动作中始终比2号中圈少下降5 mm。
constexpr float SIDE_RING_LOWER_REDUCTION_MM = 5.0f;
/*
 * 相机安装实机标定：
 *   画面+x（向右）对应M5顺时针；
 *   画面+y（向下）对应机械臂向内、M6缩短，因此到“机械臂向外”的
 *   映射符号为-1。
 */
constexpr float IMAGE_Y_TO_ARM_OUTWARD_SIGN = -1.0f;
/*
 * 霍夫闭环不再移动底盘。画面+x换算为机械臂局部顺时针切向，
 * 画面+y先按上面的实机标定换算为机械臂径向伸缩，再由M5/M6完成对中。
 */
constexpr float CIRCLE_CENTER_TOLERANCE_PIXELS = 3.0f;
constexpr uint32_t CIRCLE_CENTER_STABILITY_MS = 500UL;
constexpr uint8_t CIRCLE_STABILITY_MINIMUM_SAMPLES = 2U;
constexpr uint32_t VISION_STABILITY_MAXIMUM_SAMPLE_GAP_MS =
    1200UL;

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
 * 本路线只实现从启停区1出发的版本：车体贴启停区1左边和下边放置，
 * 因此设计起点中心为(2215,2250)。规则允许抽签到启停区2；若抽到区2，
 * 不能直接把本路线注释理解为已覆盖该出发位置。
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
 * 启停区1先沿3、4侧直走1050 mm到达右边中部二维码区，收到有效任务码后
 * 再继续横移。规则要求先读取任务码再到原料区；当前代码把二维码作为
 * 继续路线的门控。扫码点若尚无有效码，会低速前探并按脉冲原路回位；
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

/*
 * 起步Y方向实车手调只改这一行：
 *   走过头多少毫米就减多少，没走够多少毫米就加多少。
 * QR_PASS_TO_RAW_MM会自动反算，原料区最终坐标不会被带偏。
 */
constexpr uint16_t START_TO_QR_PASS_MM = 1050U;
constexpr uint16_t QR_PASS_CENTER_Y_MM =
    START_CENTER_Y_MM - START_TO_QR_PASS_MM;                 // 1200
constexpr uint16_t QR_PASS_TO_FIELD_CENTER_X_MM =
    QR_PASS_CENTER_X_MM - FIELD_CENTER_MM;                   // 1015
constexpr uint16_t QR_PASS_TO_RAW_MM =
    RAW_CENTER_Y_MM - QR_PASS_CENTER_Y_MM;                   // 935
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
// 二维码前靠速度/加速度由原1040/400翻倍；返程仍保持原参数。
constexpr uint16_t QR_SCAN_SWEEP_MAXIMUM_MM = 500U;
const float QR_SCAN_FORWARD_MAXIMUM_STEP_RATE = 2080.0f;
const float QR_SCAN_FORWARD_STEP_ACCELERATION = 800.0f;
const float QR_SCAN_RETURN_MAXIMUM_STEP_RATE = 1040.0f;
const float QR_SCAN_RETURN_STEP_ACCELERATION = 400.0f;
const uint16_t MINIMUM_STEP_WIDTH_US = 2U;

/*
 * 非中央通道仍保留原1.1 m分段上限。中央通道允许单段最多2.0 m，
 * 覆盖当前最长的1890 mm工位直达段和最终1885 mm回程，使它们均不在
 * 直线中途停车；转角处仍按独立路线命令停稳。
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
/*
 * 到达二维码扫码点后、进入前靠前的固定停顿减半：
 * IMU机械消振120->60 ms，航向稳定确认200->100 ms。
 * 只作用于下一条命令是COMMAND_QR_ACTION的路线平移。
 */
const uint32_t QR_SCAN_PRE_SWEEP_IMU_SETTLE_TIME_MS =
    IMU_POST_MOTION_SETTLE_TIME_MS / 2U;
const uint32_t QR_SCAN_PRE_SWEEP_HEADING_STABLE_TIME_MS =
    TRANSLATION_HEADING_STABLE_TIME_MS / 2U;
const uint32_t MOTION_TIMEOUT_MS = 20000UL;
const uint32_t TURN_TIMEOUT_MS = 16000UL;
const uint32_t IMU_STALE_TIMEOUT_MS = 600UL;
const bool ENABLE_MOTION_TIMEOUTS = false;

// +1：传感器逆时针角度增加；若以后更换安装方向，可改为 -1。
const int8_t IMU_COUNTERCLOCKWISE_SIGN = +1;

/*
 * “只检查路径”模式：原料区、粗加工区、暂存区动作都只等待750 ms，
 * 不会实际完成规则要求的抓取、随车承载、放置和第二批同色码垛。
 * COMMAND_FINAL_ALIGN也会直接判定完成。因此该模式只能调底盘路线，
 * 不能用来验证完整物流任务。
 */
const bool PATH_ONLY_TEST = false;

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
// 仅在状态发生变化时输出，不在loop()中周期刷屏。
const bool ENABLE_STATE_TRANSITION_FEEDBACK = true;

// true_example uses a 10 kOhm + 1 kOhm divider on PC1 and sends V * 100 to x1.
// Adjust BATTERY_CALIBRATION_SCALE after comparing x1 with a multimeter.
const float BATTERY_ADC_REFERENCE_VOLTAGE = 3.3f;
const float BATTERY_DIVIDER_RATIO = 11.0f;
const float BATTERY_CALIBRATION_SCALE = 1.0f;
const float BATTERY_FILTER_ALPHA = 0.10f;
const uint32_t BATTERY_SAMPLE_INTERVAL_MS = 50UL;
const uint32_t BATTERY_HMI_INTERVAL_MS = 500UL;
const uint32_t BATTERY_DEBUG_INTERVAL_MS = 2000UL;
const float BATTERY_SAG_WARNING_VOLTS = 1.5f;

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
uint32_t lastBatteryDebugMs = 0;
uint32_t latestBatteryRawAdc = 0U;
float latestBatteryVoltage = 0.0f;
float filteredBatteryVoltage = 0.0f;
float minimumBatteryVoltage = 0.0f;
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
    latestBatteryRawAdc = rawAdc;
    latestBatteryVoltage = sampledVoltage;

    if (!batteryVoltageInitialized) {
      filteredBatteryVoltage = sampledVoltage;
      minimumBatteryVoltage = sampledVoltage;
      batteryVoltageInitialized = true;
    } else {
      if (filteredBatteryVoltage - sampledVoltage >=
          BATTERY_SAG_WARNING_VOLTS) {
        SerialDebug.print("[POWER WARN] t=");
        SerialDebug.print(nowMs);
        SerialDebug.print(" ms, battery instant/filtered=");
        SerialDebug.print(sampledVoltage, 2);
        SerialDebug.print("/");
        SerialDebug.print(filteredBatteryVoltage, 2);
        SerialDebug.println(" V");
      }
      filteredBatteryVoltage +=
          BATTERY_FILTER_ALPHA *
          (sampledVoltage - filteredBatteryVoltage);
      if (sampledVoltage < minimumBatteryVoltage) {
        minimumBatteryVoltage = sampledVoltage;
      }
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

  if (batteryVoltageInitialized &&
      nowMs - lastBatteryDebugMs >=
          BATTERY_DEBUG_INTERVAL_MS) {
    lastBatteryDebugMs = nowMs;
    SerialDebug.print("[BATTERY] t=");
    SerialDebug.print(nowMs);
    SerialDebug.print(" ms, adc/instant/filtered/min=");
    SerialDebug.print(latestBatteryRawAdc);
    SerialDebug.print("/");
    SerialDebug.print(latestBatteryVoltage, 2);
    SerialDebug.print("/");
    SerialDebug.print(filteredBatteryVoltage, 2);
    SerialDebug.print("/");
    SerialDebug.print(minimumBatteryVoltage, 2);
    SerialDebug.println(" V");
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

const size_t QR_CODE_LENGTH = 15;
const size_t QR_BUFFER_SIZE = 32;

char qrData[QR_BUFFER_SIZE] = {0};
size_t qrDataIndex = 0;
bool qrOverflow = false;
bool scanFlag = false;
bool taskCodeDecoded = false;
uint8_t taskColors[2][3] = {{0U, 0U, 0U}, {0U, 0U, 0U}};
uint8_t taskPositions[2][3] = {{0U, 0U, 0U}, {0U, 0U, 0U}};

void commandGripperOpen();

bool characterInRange(char value, char minimum, char maximum) {
  return value >= minimum && value <= maximum;
}

uint8_t threeDigitMask(
    const char *digits, char minimum, char maximum) {
  uint8_t mask = 0U;
  for (uint8_t i = 0U; i < 3U; ++i) {
    if (!characterInRange(digits[i], minimum, maximum)) {
      return 0U;
    }

    const uint8_t bit =
        static_cast<uint8_t>(1U << (digits[i] - minimum));
    if ((mask & bit) != 0U) {
      return 0U;
    }
    mask = static_cast<uint8_t>(mask | bit);
  }
  return mask;
}

bool isValidTaskCode(const char *code) {
  if (strlen(code) != QR_CODE_LENGTH) {
    return false;
  }

  if (code[3] != '+' || code[7] != '+' || code[11] != '+') {
    return false;
  }

  // 赛题颜色编号只有1红、2黄、3蓝、4绿；每批三种颜色不能重复。
  const uint8_t firstColorMask =
      threeDigitMask(code, '1', '4');
  const uint8_t secondColorMask =
      threeDigitMask(code + 8, '1', '4');
  if (firstColorMask == 0U ||
      secondColorMask == 0U ||
      firstColorMask != secondColorMask) {
    return false;
  }

  // 两个位置组三位都必须恰好是1、2、3的一个排列。
  constexpr uint8_t POSITION_MASK_123 = 0x07U;
  return threeDigitMask(code + 4, '1', '3') ==
             POSITION_MASK_123 &&
         threeDigitMask(code + 12, '1', '3') ==
             POSITION_MASK_123;
}

void decodeTaskCode(const char *code) {
  for (uint8_t item = 0U; item < 3U; ++item) {
    taskColors[0][item] =
        static_cast<uint8_t>(code[item] - '0');
    taskPositions[0][item] =
        static_cast<uint8_t>(code[4U + item] - '0');
    taskColors[1][item] =
        static_cast<uint8_t>(code[8U + item] - '0');
    taskPositions[1][item] =
        static_cast<uint8_t>(code[12U + item] - '0');
  }
  taskCodeDecoded = true;
}

void printDecodedTaskCode() {
  SerialDebug.print("[QR PLAN] round1 colors=");
  SerialDebug.print(taskColors[0][0]);
  SerialDebug.print("/");
  SerialDebug.print(taskColors[0][1]);
  SerialDebug.print("/");
  SerialDebug.print(taskColors[0][2]);
  SerialDebug.print(", positions=");
  SerialDebug.print(taskPositions[0][0]);
  SerialDebug.print("/");
  SerialDebug.print(taskPositions[0][1]);
  SerialDebug.print("/");
  SerialDebug.println(taskPositions[0][2]);

  SerialDebug.print("[QR PLAN] round2 colors=");
  SerialDebug.print(taskColors[1][0]);
  SerialDebug.print("/");
  SerialDebug.print(taskColors[1][1]);
  SerialDebug.print("/");
  SerialDebug.print(taskColors[1][2]);
  SerialDebug.print(", positions=");
  SerialDebug.print(taskPositions[1][0]);
  SerialDebug.print("/");
  SerialDebug.print(taskPositions[1][1]);
  SerialDebug.print("/");
  SerialDebug.println(taskPositions[1][2]);
}

void resetQrReceiver() {
  qrDataIndex = 0;
  qrOverflow = false;
  scanFlag = false;
  taskCodeDecoded = false;
  qrData[0] = '\0';
  memset(taskColors, 0, sizeof(taskColors));
  memset(taskPositions, 0, sizeof(taskPositions));

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
    decodeTaskCode(qrData);
    scanFlag = true;
    commandGripperOpen();
    hmiSetText("t1", "QROK");
    hmiShowTaskCode(qrData);
    SerialDebug.print("[QR] accepted code=");
    SerialDebug.println(qrData);
    printDecodedTaskCode();
  } else {
    hmiSetText("t1", "QRERR");
    SerialDebug.print("[QR] rejected: overflow=");
    SerialDebug.print(qrOverflow ? 1 : 0);
    SerialDebug.print(", length=");
    SerialDebug.print(static_cast<unsigned int>(qrDataIndex));
    SerialDebug.print(", data=");
    SerialDebug.println(qrData);
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
    SerialDebug.print("[IMU] first valid angle frame: raw_ccw=");
    SerialDebug.print(counterClockwiseSignedRaw, 2);
    SerialDebug.println(" deg, route heading zero pending");
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

const char *commandTypeName(CommandType type) {
  switch (type) {
    case COMMAND_ARM_BASE_HOME:
      return "ARM_BASE_HOME";
    case COMMAND_MOVE_SIDE_12_MM:
      return "MOVE_SIDE_12";
    case COMMAND_MOVE_SIDE_34_MM:
      return "MOVE_SIDE_34";
    case COMMAND_MOVE_SIDE_13_MM:
      return "MOVE_SIDE_13";
    case COMMAND_MOVE_SIDE_24_MM:
      return "MOVE_SIDE_24";
    case COMMAND_TURN_COUNTERCLOCKWISE_DEGREES:
      return "TURN_CCW";
    case COMMAND_TURN_CLOCKWISE_DEGREES:
      return "TURN_CW";
    case COMMAND_SET_PRECISE_MOTION:
      return "SET_PRECISE";
    case COMMAND_QR_ACTION:
      return "QR_ACTION";
    case COMMAND_RAW_ACTION:
      return "RAW_ACTION";
    case COMMAND_PROCESS_ACTION:
      return "PROCESS_ACTION";
    case COMMAND_STORAGE_ACTION:
      return "STORAGE_ACTION";
    case COMMAND_FINAL_ALIGN:
      return "FINAL_ALIGN";
    case COMMAND_HOLD:
      return "HOLD";
    case COMMAND_FINISH:
      return "FINISH";
  }
  return "UNKNOWN";
}

const char *commandValueUnit(CommandType type) {
  switch (type) {
    case COMMAND_MOVE_SIDE_12_MM:
    case COMMAND_MOVE_SIDE_34_MM:
    case COMMAND_MOVE_SIDE_13_MM:
    case COMMAND_MOVE_SIDE_24_MM:
      return " mm";
    case COMMAND_ARM_BASE_HOME:
    case COMMAND_TURN_COUNTERCLOCKWISE_DEGREES:
    case COMMAND_TURN_CLOCKWISE_DEGREES:
      return " deg";
    case COMMAND_RAW_ACTION:
    case COMMAND_PROCESS_ACTION:
    case COMMAND_STORAGE_ACTION:
      return " round";
    case COMMAND_HOLD:
      return " ms";
    default:
      return "";
  }
}

struct RouteCommand {
  CommandType type;
  int32_t value;
  const char *name;
  bool preciseArrival;
  bool centralChannel;
};

/*
 * 几何设计层的关键工位姿态（单位 mm，角度表示2、4侧朝向）：
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
 * route-simulator的旧tmcode1名义车心是(1200,2100)、(1200,300)、
 * (300,1200)，与这里的2135/265/265不同；仿真只用于理解运动顺序。
 *
 * 更重要的是，route[]中保留了+20、+40、-20、-40、-50、50、40等
 * 实车补偿。若从起点严格按现有命令值开环累计，若无打滑，调试推算点为：
 *   扫码点                 (2215,1200)
 *   第1批原料动作点        (1200,2135)
 *   第1批粗加工动作点      (1200, 245)
 *   第1批暂存动作点        ( 265,1130)
 *   第2批原料动作点        (1200,2105)
 *   第2批粗加工动作点      (1200, 215)
 *   第2批暂存动作点        ( 265,1110)
 *   Enter Start1之后       (2215,2160)
 *   最后50/40 mm实调后     (2255,2210)
 * 这些是“按代码值推算”，不是修改建议，也不是定位传感器测量结果。
 * 因此命令名称中的field center/Start1表示路线意图，未必等于累计坐标。
 *
 * 新车头是3、4侧。第一段向3、4侧移动就是实车“前进”，但在旧运动学
 * 坐标中等价于负 forward，所以路径命令不再使用 forward/backward 命名。
 */
const RouteCommand route[] = {
    /*
     * PB9后机械臂保持上电/发车姿态：M5旧坐标0°，M6最短、M7最高；
     * 到各工位且底盘停稳后，作业状态机才把M5顺时针转90°进入标准状态。
     */
    /*
     * 规则阶段：从抽签启停区出发，到二维码板读取四组三位任务码。
     * 本实现固定为启停区1和右边中部扫码点。初始3、4侧朝南，一次走
     * 1050 mm到(2215,1200)；到点后停稳。若尚未收到有效任务码，
     * COMMAND_QR_ACTION会沿车头低速前探最多500 mm，扫码后按四轮实际
     * 脉冲回到该停车点，再继续后续路线。
     * 若此时IMU尚无有效帧，机械臂保持行驶姿态，底盘停在IMUWAIT。
     */
    {COMMAND_MOVE_SIDE_34_MM, START_TO_QR_PASS_MM,
     "Start1 -> QR area direct"},
    {COMMAND_QR_ACTION, 0, "Scan QR task code"},

    // -------------------- 第一批三个物料（代码中的第1轮） --------------------
    /*
     * 扫码点 -> 原料区：
     * 先向1、3侧横移到X=1200，再向1、2侧一步直达原料动作点；
     * 到点后逆时针转90°使2、4侧朝北，随后直接开始工作。
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
     QR_PASS_TO_RAW_MM,
     "QR Y=1200 -> raw work point round 1 direct", false, true},
    {COMMAND_TURN_COUNTERCLOCKWISE_DEGREES, 90,
     "Face side 2,4 north"},
    {COMMAND_RAW_ACTION, 1, "Raw action round 1"},

    /*
     * 第1批原料区 -> 粗加工区：
     * 向1、3侧长距离南移并一步直达动作点，仅在终点IMU回正。
     * 随后逆时针180°让2、4侧朝南，直接开始工作，不再低速精靠。
     * 当前+20 mm为已有实调补偿，按命令累计动作点约为(1200,245)。
     *
     * 规则动作含义：把第一批三个物料按任务码第二组三位指定的1/2/3号
     * 圆环位置放到粗加工区；随后还要按任务码顺序重新抓回车上。
     * COMMAND_PROCESS_ACTION同时代表“放下3个并重新抓回3个”的接口。
     */
    {COMMAND_MOVE_SIDE_13_MM,
     RAW_TO_PROCESS_MM + 20U,
     "Raw -> process work point round 1 direct", false, true},
    {COMMAND_TURN_COUNTERCLOCKWISE_DEGREES, 180,
     "Face side 2,4 south"},
    {COMMAND_PROCESS_ACTION, 1, "Process action round 1"},

    /*
     * 第1批粗加工区 -> 暂存区：
     * 先向1、3侧北移，再向3、4侧西移并一步直达动作点；顺时针90°
     * 使2、4侧朝西后直接工作。受现有-50 mm补偿影响，按命令累计
     * 暂存动作点约为(265,1130)，不是几何注释中的(265,1200)。
     *
     * 规则动作含义：将第一批三个物料按任务码第二组三位规定的位置，
     * 平面放置到暂存区，为第二批同色码垛留下底层。
     */
    {COMMAND_MOVE_SIDE_13_MM, PROCESS_TO_CENTER_MM - 50U,
     "Process -> field center round 1", false, true},
    {COMMAND_MOVE_SIDE_34_MM,
     CENTER_TO_STORAGE_MM,
     "Center -> storage work point round 1 direct", false, true},
    {COMMAND_TURN_CLOCKWISE_DEGREES, 90,
     "Face side 2,4 west"},
    {COMMAND_STORAGE_ACTION, 1, "Storage action round 1"},

    // -------------------- 第二批三个物料（代码中的第2轮） --------------------
    /*
     * 暂存区 -> 原料区：
     * 保持2、4侧朝西，先向1、3侧回到X=1200，再向3、4侧一步直达；
     * 顺时针90°使2、4侧朝北后直接工作。+40 mm是现有
     * 实调补偿，按命令累计第2批原料动作点约为(1200,2105)。
     *
     * 规则动作含义：按任务码第三组三位颜色顺序抓取第二批三个物料，
     * 同样必须逐个放到机器人上后才能跨区运输。
     */
    {COMMAND_MOVE_SIDE_13_MM, STORAGE_TO_CENTER_MM,
     "Storage -> field center round 2", false, true},
    {COMMAND_MOVE_SIDE_34_MM,
     CENTER_TO_RAW_MM + 40U,
     "Field center -> raw work point round 2 direct", false, true},
    {COMMAND_TURN_CLOCKWISE_DEGREES, 90,
     "Face side 2,4 north"},
    {COMMAND_RAW_ACTION, 2, "Raw action round 2"},

    /*
     * 第2批原料区 -> 粗加工区：
     * 原长移段+40 mm与精靠段-20 mm合并后净补偿为+20 mm；
     * 逆时针180°后2、4侧朝南，直接开始工作。
     *
     * 规则动作含义：按任务码第四组三位指定的圆环位置放下第二批，
     * 再按搬运顺序重新抓回车上，准备送往暂存区。
     */
    {COMMAND_MOVE_SIDE_13_MM,
     RAW_TO_PROCESS_MM + 20U,
     "Raw -> process work point round 2 direct", false, true},
    {COMMAND_TURN_COUNTERCLOCKWISE_DEGREES, 180,
     "Face side 2,4 south"},
    {COMMAND_PROCESS_ACTION, 2, "Process action round 2"},

    /*
     * 第2批粗加工区 -> 暂存区：
     * 先北移、再向西一步直达动作点，顺时针90°让2、4侧朝西后直接工作；
     * 当前-40 mm补偿使按命令累计暂存动作点约为(265,1110)。
     *
     * 规则动作含义：第二批只能码垛在第一批已正确放置的同色物料上。
     * COMMAND_STORAGE_ACTION第2批接口应在实际机械臂完成三次码垛后反馈。
     */
    {COMMAND_MOVE_SIDE_13_MM, PROCESS_TO_CENTER_MM - 40U,
     "Process -> field center round 2", false, true},
    {COMMAND_MOVE_SIDE_34_MM,
     CENTER_TO_STORAGE_MM,
     "Center -> storage work point round 2 direct", false, true},
    {COMMAND_TURN_CLOCKWISE_DEGREES, 90,
     "Face side 2,4 west"},
    {COMMAND_STORAGE_ACTION, 2, "Storage action round 2"},

    /*
     * 规则阶段：完成两批后返回本次抽签确定的启停区。
     * 本实现固定返回启停区1：全程保持2、4侧朝西，经场地中部和
     * X=2150右侧回程通道进入右上角，不再做最后一次180°原地转圈。
     *
     * 与旧route-simulator的差异：仿真在(2215,2250)直接归零；本文件
     * Enter Start1后还执行FINAL_ALIGN同步点、保持，以及50 mm/40 mm两段
     * 原有实调位移。因未提供终点红外协议，FINAL_ALIGN不新增底盘位移。
     */
    {COMMAND_MOVE_SIDE_13_MM,
     STORAGE_TO_CENTER_MM + CENTER_TO_RETURN_LANE_MM,
     "Storage -> return lane final direct", false, true},
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

const char *programStateName(ProgramState state) {
  switch (state) {
    case PROGRAM_WAITING:
      return "WAITING";
    case PROGRAM_RUNNING:
      return "RUNNING";
    case PROGRAM_FINISHED:
      return "FINISHED";
    case PROGRAM_FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

enum QrScanPhase {
  QR_SCAN_IDLE,
  QR_SCAN_FORWARD,
  QR_SCAN_WAIT_AT_LIMIT,
  QR_SCAN_RETURNING,
  QR_SCAN_COMPLETE
};

const char *qrScanPhaseName(QrScanPhase phase) {
  switch (phase) {
    case QR_SCAN_IDLE:
      return "IDLE";
    case QR_SCAN_FORWARD:
      return "FORWARD";
    case QR_SCAN_WAIT_AT_LIMIT:
      return "WAIT_AT_LIMIT";
    case QR_SCAN_RETURNING:
      return "RETURNING";
    case QR_SCAN_COMPLETE:
      return "COMPLETE";
  }
  return "UNKNOWN";
}

ProgramState programState = PROGRAM_WAITING;
QrScanPhase qrScanPhase = QR_SCAN_IDLE;
MotorPulses qrScanOriginMotorPositions;
uint32_t qrScanActionStartMs = 0UL;
size_t routeIndex = 0;
uint8_t activeCompetitionRound = 0;
bool commandStarted = false;
uint32_t commandStartMs = 0;
uint32_t routeCommandStartMs = 0;
uint32_t routeRunStartMs = 0;
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
  const RouteCommand &command = route[routeIndex];
  SerialDebug.print("[ROUTE START] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, step=");
  SerialDebug.print(static_cast<unsigned int>(routeIndex + 1));
  SerialDebug.print("/");
  SerialDebug.print(static_cast<unsigned int>(ROUTE_COMMAND_COUNT));
  SerialDebug.print(", type=");
  SerialDebug.print(commandTypeName(command.type));
  SerialDebug.print(", value=");
  SerialDebug.print(command.value);
  SerialDebug.print(commandValueUnit(command.type));
  SerialDebug.print(", precise=");
  SerialDebug.print(command.preciseArrival ? 1 : 0);
  SerialDebug.print(", central=");
  SerialDebug.print(command.centralChannel ? 1 : 0);
  SerialDebug.print(", name=");
  SerialDebug.println(command.name);
}

void emergencyStopArmLinearAxes();
void stopMaixRequest();
void invalidateArmLinearReference();

void routeFault(const char *reason) {
  const ProgramState stateBeforeFault = programState;
  resetQrScanActionState();
  invalidateArmLinearReference();
  disableDriveMotors();
  disableArmBaseMotor();
  emergencyStopArmLinearAxes();
  stopMaixRequest();
  programState = PROGRAM_FAULT;
  commandStarted = false;
  hmiSetRunStatus("FAULT");

  SerialDebug.print("[FAULT] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, state=");
  SerialDebug.print(programStateName(stateBeforeFault));
  if (routeIndex < ROUTE_COMMAND_COUNT) {
    SerialDebug.print(", step=");
    SerialDebug.print(static_cast<unsigned int>(routeIndex + 1));
    SerialDebug.print("/");
    SerialDebug.print(static_cast<unsigned int>(ROUTE_COMMAND_COUNT));
    SerialDebug.print(", command=");
    SerialDebug.print(commandTypeName(route[routeIndex].type));
    SerialDebug.print(", elapsed=");
    SerialDebug.print(millis() - routeCommandStartMs);
    SerialDebug.print(" ms");
  }
  SerialDebug.print(", reason=");
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
  uint8_t stallSampleCount;
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
        stallSampleCount(0U),
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

void writeArmLinearClearStallProtection(uint8_t address) {
  // EMM V5“解除堵转保护”：地址 + 0E + 52 + 6B。
  const uint8_t frame[4] = {
      address, 0x0EU, 0x52U, 0x6BU};
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

bool waitForArmLinearFrame(
    uint8_t address,
    uint8_t functionCode,
    uint8_t *frame,
    uint32_t timeoutMs) {
  uint8_t window[4] = {0U, 0U, 0U, 0U};
  uint8_t count = 0U;
  const uint32_t startMs = millis();

  do {
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
      if (window[0] == address &&
          window[1] == functionCode) {
        memcpy(frame, window, sizeof(window));
        armLinearReceiveCount = 0U;
        return true;
      }
      count = 0U;
    }
    delay(1);
  } while (millis() - startMs < timeoutMs);

  armLinearReceiveCount = 0U;
  return false;
}

bool recoverAndVerifyArmLinearAxis(
    LinearAxisMotion &axis) {
  uint8_t response[4] = {0U, 0U, 0U, 0U};

  /*
   * MCU复位不一定会让EMM驱动器掉电；先停止，再清除上一次运行锁存的
   * 堵转保护。这里只在上电初始化、轴未运动时清除；运行中的真实堵转
   * 仍由状态机停机，绝不边运动边清故障后继续。
   */
  SerialDebug.print("[EMM BOOT] M");
  SerialDebug.print(axis.address);
  SerialDebug.println(" stop, clear latched stall protection");
  clearArmLinearReceiveBuffer();
  writeArmLinearStop(axis.address);
  SerialArmLinear.flush();
  delay(ARM_AXIS_COMMAND_GUARD_MS);
  clearArmLinearReceiveBuffer();

  writeArmLinearClearStallProtection(axis.address);
  SerialArmLinear.flush();
  if (waitForArmLinearFrame(
          axis.address,
          0x0EU,
          response,
          ARM_AXIS_STARTUP_RESPONSE_TIMEOUT_MS)) {
    printArmLinearFrame("startup clear response", response);
    if (response[2] != 0x02U) {
      SerialDebug.print("[EMM BOOT] M");
      SerialDebug.print(axis.address);
      SerialDebug.println(
          " clear command was not accepted");
    }
  } else {
    SerialDebug.print("[EMM BOOT] M");
    SerialDebug.print(axis.address);
    SerialDebug.println(
        " no clear ACK; verify with 3A status");
  }

  clearArmLinearReceiveBuffer();
  writeArmLinearEnable(axis.address, true);
  SerialArmLinear.flush();
  if (waitForArmLinearFrame(
          axis.address,
          0xF3U,
          response,
          ARM_AXIS_STARTUP_RESPONSE_TIMEOUT_MS)) {
    printArmLinearFrame("startup enable response", response);
  } else {
    SerialDebug.print("[EMM BOOT] M");
    SerialDebug.print(axis.address);
    SerialDebug.println(
        " no enable ACK; verify with 3A status");
  }

  axis.stallSampleCount = 0U;
  for (uint8_t attempt = 0U;
       attempt <
       ARM_AXIS_STALL_CONFIRMATION_SAMPLES;
       ++attempt) {
    clearArmLinearReceiveBuffer();
    writeArmLinearStatusRequest(axis.address);
    SerialArmLinear.flush();
    if (!waitForArmLinearFrame(
            axis.address,
            0x3AU,
            response,
            ARM_AXIS_STARTUP_RESPONSE_TIMEOUT_MS)) {
      SerialDebug.print("[EMM BOOT ERROR] M");
      SerialDebug.print(axis.address);
      SerialDebug.println(" no 3A status response");
      axis.fault = true;
      return false;
    }

    printArmLinearFrame("startup status", response);
    const uint8_t flags = response[2];
    SerialDebug.print("[EMM BOOT] M");
    SerialDebug.print(axis.address);
    SerialDebug.print(" enabled/on-position/stall/protect=");
    SerialDebug.print((flags & 0x01U) != 0U ? 1 : 0);
    SerialDebug.print("/");
    SerialDebug.print((flags & 0x02U) != 0U ? 1 : 0);
    SerialDebug.print("/");
    SerialDebug.print((flags & 0x04U) != 0U ? 1 : 0);
    SerialDebug.print("/");
    SerialDebug.println((flags & 0x08U) != 0U ? 1 : 0);

    if ((flags & 0x01U) == 0U) {
      SerialDebug.print("[EMM BOOT ERROR] M");
      SerialDebug.print(axis.address);
      SerialDebug.println(" axis disabled after enable");
      axis.fault = true;
      return false;
    }
    if ((flags & 0x0CU) == 0U) {
      if (axis.stallSampleCount != 0U) {
        SerialDebug.print("[EMM BOOT] M");
        SerialDebug.print(axis.address);
        SerialDebug.println(
            " stall indication recovered before 3 samples");
      }
      axis.stallSampleCount = 0U;
      return true;
    }

    ++axis.stallSampleCount;
    SerialDebug.print("[EMM BOOT STALL] M");
    SerialDebug.print(axis.address);
    SerialDebug.print(", sample=");
    SerialDebug.print(axis.stallSampleCount);
    SerialDebug.print("/");
    SerialDebug.println(
        ARM_AXIS_STALL_CONFIRMATION_SAMPLES);
    if (axis.stallSampleCount >=
        ARM_AXIS_STALL_CONFIRMATION_SAMPLES) {
      SerialDebug.print("[EMM BOOT ERROR] M");
      SerialDebug.print(axis.address);
      SerialDebug.println(
          " persistent stall after clear/enable");
      axis.fault = true;
      axis.stallSampleCount = 0U;
      return false;
    }
    delay(ARM_AXIS_STATUS_INTERVAL_MS);
  }

  axis.fault = true;
  axis.stallSampleCount = 0U;
  return false;
}

void markLinearAxisArrived(LinearAxisMotion &axis) {
  axis.active = false;
  axis.currentMm = axis.targetMm;
  axis.stallSampleCount = 0U;
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
  axis.stallSampleCount = 0U;
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
    const bool stallReported =
        (flags & 0x04U) != 0U;
    const bool stallProtectionActive =
        (flags & 0x08U) != 0U;
    if (stallReported || stallProtectionActive) {
      if (axis->stallSampleCount < UINT8_MAX) {
        ++axis->stallSampleCount;
      }
      SerialDebug.print("[EMM STALL] t=");
      SerialDebug.print(millis());
      SerialDebug.print(" ms, M");
      SerialDebug.print(axis->address);
      SerialDebug.print(", sample=");
      SerialDebug.print(axis->stallSampleCount);
      SerialDebug.print("/");
      SerialDebug.print(ARM_AXIS_STALL_CONFIRMATION_SAMPLES);
      SerialDebug.print(", stall/protection=");
      SerialDebug.print(stallReported ? 1 : 0);
      SerialDebug.print("/");
      SerialDebug.print(
          stallProtectionActive ? 1 : 0);
      SerialDebug.println("; wait for confirmation");
      if (axis->stallSampleCount >=
          ARM_AXIS_STALL_CONFIRMATION_SAMPLES) {
        printArmLinearFrame(
            "persistent stall indication", frame);
        faultLinearAxis(
            *axis,
            "EMM persistent stall indication");
      }
      return;
    }
    if (axis->stallSampleCount != 0U) {
      SerialDebug.print("[EMM STALL] t=");
      SerialDebug.print(millis());
      SerialDebug.print(" ms, M");
      SerialDebug.print(axis->address);
      SerialDebug.print(" recovered after ");
      SerialDebug.print(axis->stallSampleCount);
      SerialDebug.println(" sample(s)");
      axis->stallSampleCount = 0U;
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
  axis.stallSampleCount = 0U;
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
   * 本机没有M6/M7限位开关寻零：上电/开赛前必须人工确认M6完全回缩、
   * M7位于最高点。这里只重置控制量累计值，不会驱动机械臂寻找端点。
   */
  extensionAxis.currentMm = M6_STANDARD_EXTENSION_MM;
  extensionAxis.targetMm = M6_STANDARD_EXTENSION_MM;
  extensionAxis.active = false;
  extensionAxis.fault = false;
  extensionAxis.commandAcknowledged = false;
  extensionAxis.motionObserved = false;
  extensionAxis.stallSampleCount = 0U;
  liftAxis.currentMm = M7_STANDARD_HEIGHT_MM;
  liftAxis.targetMm = M7_STANDARD_HEIGHT_MM;
  liftAxis.active = false;
  liftAxis.fault = false;
  liftAxis.commandAcknowledged = false;
  liftAxis.motionObserved = false;
  liftAxis.stallSampleCount = 0U;
  armLinearReferenceValid = true;
}

void invalidateArmLinearReference() {
  armLinearReferenceValid = false;
}

void commandGripperOpen() {
  gripperServo.setRawAngle(
      GRIPPER_OPEN_ANGLE_DEGREES,
      GRIPPER_INTERVAL_MS,
      GRIPPER_OPEN_POWER_MW);
  SerialDebug.print("[SERVO] gripper(ID4)=OPEN, angle=");
  SerialDebug.println(GRIPPER_OPEN_ANGLE_DEGREES, 1);
}

void commandGripperMaxOpen() {
  gripperServo.setRawAngle(
      GRIPPER_MAX_OPEN_ANGLE_DEGREES,
      GRIPPER_INTERVAL_MS,
      GRIPPER_OPEN_POWER_MW);
  SerialDebug.print("[SERVO] gripper(ID4)=MAX_OPEN, angle=");
  SerialDebug.println(GRIPPER_MAX_OPEN_ANGLE_DEGREES, 1);
}

void commandGripperClose() {
  gripperServo.setRawAngle(
      GRIPPER_CLOSE_ANGLE_DEGREES,
      GRIPPER_INTERVAL_MS,
      GRIPPER_CLOSE_POWER_MW);
  SerialDebug.print("[SERVO] gripper(ID4)=CLOSE, angle=");
  SerialDebug.println(GRIPPER_CLOSE_ANGLE_DEGREES, 1);
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
  SerialDebug.print("[SERVO] storage(ID5) index=");
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
  SerialDebug.print("[SERVO] storage(ID5)=PARK, angle=");
  SerialDebug.println(
      STORAGE_SERVO_PARK_ANGLE_DEGREES,
      1);
}

bool verifyManipulationServosOnline() {
  /*
   * 两次ping不能用&&短路，否则夹爪掉线时不会检查载物盘。
   * 该检查只在工位停车后执行，最坏约2×100 ms，不影响底盘脉冲输出。
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
  uint8_t resultId;
  int16_t x;
  int16_t y;
  uint32_t sequence;
  uint32_t receivedMs;

  MaixCoordinate()
      : resultId(0U),
        x(0),
        y(0),
        sequence(0UL),
        receivedMs(0UL) {}
};

bool maixcamSerialInitialized = false;
uint8_t maixRequestedMode = MAIXCAM_STOP_REQUEST;
bool maixModeCommandSent = false;
uint32_t maixModeSwitchStartMs = 0UL;
uint32_t maixLastRequestMs = 0UL;
char maixReceiveLine[MAIXCAM_LINE_CAPACITY] = {0};
size_t maixReceiveLength = 0U;
MaixCoordinate latestMaixCoordinate;

void writeMaixRequestFrame(uint8_t request) {
  const uint8_t frame[3] = {
      MAIXCAM_FRAME_HEADER, request, MAIXCAM_FRAME_TAIL};
  const size_t queuedBytes =
      SerialMaixcam.write(frame, sizeof(frame));

  /*
   * 诊断日志记录真正执行SerialMaixcam.write()的时刻。
   * “MaixCAM mode request”只代表状态机计划切换模式，不能证明已经走到TX。
   */
  SerialDebug.print("[MAIX TX] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, frame=AA ");
  if (request < 0x10U) {
    SerialDebug.print("0");
  }
  SerialDebug.print(request, HEX);
  SerialDebug.print(" BB, queued=");
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
}

void beginMaixRequest(uint8_t request) {
  if (request != MAIXCAM_ALL_COLORS_REQUEST &&
      request != MAIXCAM_HOUGH_CIRCLE_REQUEST) {
    routeFault("Invalid MaixCAM request");
    return;
  }

  stopMaixRequest();
  while (SerialMaixcam.available()) {
    SerialMaixcam.read();
  }
  maixRequestedMode = request;
  maixModeSwitchStartMs = millis();
  maixLastRequestMs = 0UL;
  maixModeCommandSent = false;

  SerialDebug.print("[MAIX PLAN] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, mode=");
  SerialDebug.print(request);
  SerialDebug.print(", TX after guard=");
  SerialDebug.print(MAIXCAM_MODE_SWITCH_GUARD_MS);
  SerialDebug.println(" ms");
}

bool parseMaixCoordinateLine(
    const char *line,
    uint8_t &resultId,
    int16_t &x,
    int16_t &y) {
  if (line == nullptr || line[0] == '\0') {
    return false;
  }

  char *idEnd = nullptr;
  const long parsedId = strtol(line, &idEnd, 10);
  if (idEnd == line || idEnd == nullptr || *idEnd != ',') {
    return false;
  }

  const char *xStart = idEnd + 1;
  char *xEnd = nullptr;
  const long parsedX = strtol(xStart, &xEnd, 10);
  if (xEnd == xStart || xEnd == nullptr || *xEnd != ',') {
    return false;
  }

  const char *yStart = xEnd + 1;
  char *yEnd = nullptr;
  const long parsedY = strtol(yStart, &yEnd, 10);
  if (yEnd == yStart || yEnd == nullptr || *yEnd != '\0') {
    return false;
  }

  if (parsedId < 0L || parsedId > 255L ||
      parsedX < 0L || parsedX > IMAGE_MAX_X ||
      parsedY < 0L || parsedY > IMAGE_MAX_Y) {
    return false;
  }

  resultId = static_cast<uint8_t>(parsedId);
  x = static_cast<int16_t>(parsedX);
  y = static_cast<int16_t>(parsedY);
  return true;
}

void finishMaixCoordinateLine() {
  maixReceiveLine[maixReceiveLength] = '\0';
  uint8_t resultId = 0U;
  int16_t x = 0;
  int16_t y = 0;
  if (maixModeCommandSent &&
      maixRequestedMode != MAIXCAM_STOP_REQUEST &&
      parseMaixCoordinateLine(
          maixReceiveLine, resultId, x, y) &&
      ((maixRequestedMode == MAIXCAM_ALL_COLORS_REQUEST &&
        resultId >= 1U && resultId <= 4U) ||
       (maixRequestedMode == MAIXCAM_HOUGH_CIRCLE_REQUEST &&
        resultId == MAIXCAM_HOUGH_CIRCLE_REQUEST))) {
    latestMaixCoordinate.resultId = resultId;
    latestMaixCoordinate.x = x;
    latestMaixCoordinate.y = y;
    ++latestMaixCoordinate.sequence;
    latestMaixCoordinate.receivedMs = millis();

    SerialDebug.print("MaixCAM ");
    SerialDebug.print(maixRequestedMode);
    SerialDebug.print(" result ");
    SerialDebug.print(resultId);
    SerialDebug.print(": ");
    SerialDebug.print(x);
    SerialDebug.print(",");
    SerialDebug.println(y);
  }
  maixReceiveLength = 0U;
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
    if (maixReceiveLength < MAIXCAM_LINE_CAPACITY - 1U) {
      maixReceiveLine[maixReceiveLength++] = incoming;
    } else {
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
    uint8_t &resultId,
    int16_t &x,
    int16_t &y) {
  if (latestMaixCoordinate.sequence == lastSequence ||
      millis() - latestMaixCoordinate.receivedMs >
          MAIXCAM_COORDINATE_STALE_MS) {
    return false;
  }
  lastSequence = latestMaixCoordinate.sequence;
  resultId = latestMaixCoordinate.resultId;
  x = latestMaixCoordinate.x;
  y = latestMaixCoordinate.y;
  return true;
}

void printRouteCommandCompletion() {
  if (routeIndex >= ROUTE_COMMAND_COUNT) {
    return;
  }

  SerialDebug.print("[ROUTE DONE] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, step=");
  SerialDebug.print(static_cast<unsigned int>(routeIndex + 1));
  SerialDebug.print("/");
  SerialDebug.print(static_cast<unsigned int>(ROUTE_COMMAND_COUNT));
  SerialDebug.print(", type=");
  SerialDebug.print(commandTypeName(route[routeIndex].type));
  SerialDebug.print(", elapsed=");
  SerialDebug.print(millis() - routeCommandStartMs);
  SerialDebug.print(" ms, heading target/actual/error=");
  SerialDebug.print(targetCounterClockwiseHeadingDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(currentRouteCounterClockwiseHeading(), 2);
  SerialDebug.print("/");
  SerialDebug.println(headingErrorDegrees(), 2);
}

void advanceRoute() {
  printRouteCommandCompletion();
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
  SerialDebug.print("[IMU LOCK] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, target/actual/error/correction=");
  SerialDebug.print(targetCounterClockwiseHeadingDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(currentRouteCounterClockwiseHeading(), 2);
  SerialDebug.print("/");
  SerialDebug.print(errorCounterClockwiseDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(correction, 2);
  SerialDebug.println(" deg");
  setDriveMotionProfile(
      HEADING_CORRECTION_MAXIMUM_STEP_RATE,
      HEADING_CORRECTION_STEP_ACCELERATION);
  startBodyDisplacement(
      0.0f, 0.0f, correctionCounterClockwiseRadians);
}

bool updateHeadingLock(uint32_t timeoutMs) {
  const bool enteringQrSweepNext =
      routeIndex + 1U < ROUTE_COMMAND_COUNT &&
      route[routeIndex + 1U].type == COMMAND_QR_ACTION;
  const uint32_t postMotionSettleTimeMs =
      enteringQrSweepNext
          ? QR_SCAN_PRE_SWEEP_IMU_SETTLE_TIME_MS
          : IMU_POST_MOTION_SETTLE_TIME_MS;

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
    SerialDebug.print("[MOTION] t=");
    SerialDebug.print(motorsArrivedStartMs);
    SerialDebug.print(
        " ms, pulse targets reached; IMU settle=");
    SerialDebug.print(postMotionSettleTimeMs);
    SerialDebug.println(" ms");
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
      enteringQrSweepNext
          ? QR_SCAN_PRE_SWEEP_HEADING_STABLE_TIME_MS
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
      SerialDebug.print("[IMU LOCK] t=");
      SerialDebug.print(headingStableStartMs);
      SerialDebug.print(
          " ms, inside tolerance; error/tolerance/stable=");
      SerialDebug.print(error, 2);
      SerialDebug.print("/");
      SerialDebug.print(tolerance, 2);
      SerialDebug.print("/");
      SerialDebug.print(stableTimeMs);
      SerialDebug.println(" ms");
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
      QR_SCAN_RETURN_MAXIMUM_STEP_RATE,
      QR_SCAN_RETURN_STEP_ACCELERATION);
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
      QR_SCAN_FORWARD_MAXIMUM_STEP_RATE,
      QR_SCAN_FORWARD_STEP_ACCELERATION);
  /*
   * 新车头是3、4侧，与起点到扫码区的第一段方向相同；在旧运动学坐标
   * 中对应负forward。500 mm名义脉冲为+5000,-5000,+5000,-5000。
   */
  startBodyDisplacement(
      -static_cast<float>(
          QR_SCAN_SWEEP_MAXIMUM_MM) /
          1000.0f,
      0.0f,
      0.0f);
  commandStartMs = millis();
  headingStableStartMs = 0UL;
  motorsArrivedStartMs = 0UL;
  qrScanPhase = QR_SCAN_FORWARD;
  SerialDebug.print("[QR SWEEP] forward limit=");
  SerialDebug.print(QR_SCAN_SWEEP_MAXIMUM_MM);
  SerialDebug.print(" mm, vmax/acc=");
  SerialDebug.print(QR_SCAN_FORWARD_MAXIMUM_STEP_RATE, 0);
  SerialDebug.print("/");
  SerialDebug.println(QR_SCAN_FORWARD_STEP_ACCELERATION, 0);
}

bool updateQrScanAction() {
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

  SerialDebug.print("[MOTION] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, type=");
  SerialDebug.print(commandTypeName(type));
  SerialDebug.print(", segment=");
  SerialDebug.print(segmentMm);
  SerialDebug.print(" mm, remaining=");
  SerialDebug.print(translationRemainingMm);
  SerialDebug.print(" mm, vmax/acc=");
  SerialDebug.print(translationMaximumStepRate, 0);
  SerialDebug.print("/");
  SerialDebug.print(translationStepAcceleration, 0);
  SerialDebug.print(", precise/central=");
  SerialDebug.print(
      translationPreciseArrivalEnabled ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.println(
      translationCentralChannelEnabled ? 1 : 0);

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

enum ArmStandardPhase {
  ARM_STANDARD_IDLE,
  ARM_STANDARD_WAIT_OPEN,
  ARM_STANDARD_WAIT_LIFT,
  ARM_STANDARD_WAIT_RETRACT,
  ARM_STANDARD_WAIT_BASE,
  ARM_STANDARD_WAIT_BASE_SETTLE,
  ARM_STANDARD_COMPLETE
};

const char *armStandardPhaseName(ArmStandardPhase phase) {
  switch (phase) {
    case ARM_STANDARD_IDLE:
      return "IDLE";
    case ARM_STANDARD_WAIT_OPEN:
      return "WAIT_GRIPPER_OPEN";
    case ARM_STANDARD_WAIT_LIFT:
      return "WAIT_M7_HIGHEST";
    case ARM_STANDARD_WAIT_RETRACT:
      return "WAIT_M6_RETRACT";
    case ARM_STANDARD_WAIT_BASE:
      return "WAIT_M5_STANDARD";
    case ARM_STANDARD_WAIT_BASE_SETTLE:
      return "WAIT_M5_SETTLE";
    case ARM_STANDARD_COMPLETE:
      return "COMPLETE";
  }
  return "UNKNOWN";
}

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
            " ms, gripper settled -> M7 highest");
        startLiftToHeightMm(M7_STANDARD_HEIGHT_MM);
        armStandardPhase = ARM_STANDARD_WAIT_LIFT;
      }
      break;

    case ARM_STANDARD_WAIT_LIFT:
      if (liftMoveFinished()) {
        SerialDebug.print("[ARM STD] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, M7 complete -> M6 fully retracted");
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
  ARM_TRANSFER_WAIT_DESTINATION_LOWER,
  ARM_TRANSFER_WAIT_DESTINATION_OPEN,
  ARM_TRANSFER_WAIT_FINAL_LIFT,
  ARM_TRANSFER_WAIT_FINAL_RETRACT,
  ARM_TRANSFER_WAIT_STANDARD_ROTATION,
  ARM_TRANSFER_WAIT_STANDARD_SETTLE,
  ARM_TRANSFER_COMPLETE
};

const char *armTransferPhaseName(ArmTransferPhase phase) {
  switch (phase) {
    case ARM_TRANSFER_IDLE:
      return "IDLE";
    case ARM_TRANSFER_WAIT_SOURCE_OPEN:
      return "WAIT_SOURCE_OPEN";
    case ARM_TRANSFER_WAIT_PREPARE_LIFT:
      return "WAIT_PREPARE_LIFT";
    case ARM_TRANSFER_WAIT_PREPARE_RETRACT:
      return "WAIT_PREPARE_RETRACT";
    case ARM_TRANSFER_WAIT_SOURCE_ROTATION:
      return "WAIT_SOURCE_ROTATION";
    case ARM_TRANSFER_WAIT_SOURCE_ROTATION_SETTLE:
      return "WAIT_SOURCE_ROTATION_SETTLE";
    case ARM_TRANSFER_WAIT_SOURCE_EXTENSION:
      return "WAIT_SOURCE_EXTENSION";
    case ARM_TRANSFER_WAIT_SOURCE_LOWER:
      return "WAIT_SOURCE_LOWER";
    case ARM_TRANSFER_WAIT_GRIP_CLOSE:
      return "WAIT_GRIP_CLOSE";
    case ARM_TRANSFER_WAIT_LOADED_LIFT:
      return "WAIT_LOADED_LIFT";
    case ARM_TRANSFER_WAIT_LOADED_RETRACT:
      return "WAIT_LOADED_RETRACT";
    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION:
      return "WAIT_DEST_ROTATION";
    case ARM_TRANSFER_WAIT_DESTINATION_ROTATION_SETTLE:
      return "WAIT_DEST_ROTATION_SETTLE";
    case ARM_TRANSFER_WAIT_DESTINATION_EXTENSION:
      return "WAIT_DEST_EXTENSION";
    case ARM_TRANSFER_WAIT_DESTINATION_LOWER:
      return "WAIT_DEST_LOWER";
    case ARM_TRANSFER_WAIT_DESTINATION_OPEN:
      return "WAIT_DEST_OPEN";
    case ARM_TRANSFER_WAIT_FINAL_LIFT:
      return "WAIT_FINAL_LIFT";
    case ARM_TRANSFER_WAIT_FINAL_RETRACT:
      return "WAIT_FINAL_RETRACT";
    case ARM_TRANSFER_WAIT_STANDARD_ROTATION:
      return "WAIT_STANDARD_ROTATION";
    case ARM_TRANSFER_WAIT_STANDARD_SETTLE:
      return "WAIT_STANDARD_SETTLE";
    case ARM_TRANSFER_COMPLETE:
      return "COMPLETE";
  }
  return "UNKNOWN";
}

ArmTransferPhase armTransferPhase = ARM_TRANSFER_IDLE;
ArmPose armTransferSourcePose;
ArmPose armTransferDestinationPose;
uint32_t armTransferDeadlineMs = 0UL;
uint32_t armTransferStartMs = 0UL;

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
    const ArmPose &destination) {
  if (!armPoseIsValid(source) ||
      !armPoseIsValid(destination)) {
    routeFault("Transfer pose outside arm travel");
    return;
  }

  armTransferSourcePose = source;
  armTransferDestinationPose = destination;
  armTransferStartMs = millis();
  commandGripperOpen();
  armTransferDeadlineMs =
      millis() + GRIPPER_OPEN_SETTLE_MS;
  armTransferPhase = ARM_TRANSFER_WAIT_SOURCE_OPEN;

  SerialDebug.print("[ARM TRANSFER] t=");
  SerialDebug.print(armTransferStartMs);
  SerialDebug.print(" ms, source angle/ext/height=");
  SerialDebug.print(source.standardFrameAngleDegrees, 2);
  SerialDebug.print(", ");
  SerialDebug.print(source.extensionMm, 2);
  SerialDebug.print(", ");
  SerialDebug.println(source.heightMm, 2);
  SerialDebug.print(
      "[ARM TRANSFER] destination angle/ext/height=");
  SerialDebug.print(destination.standardFrameAngleDegrees, 2);
  SerialDebug.print(", ");
  SerialDebug.print(destination.extensionMm, 2);
  SerialDebug.print(", ");
  SerialDebug.println(destination.heightMm, 2);
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
        startLiftToHeightMm(
            armTransferDestinationPose.heightMm);
        armTransferPhase =
            ARM_TRANSFER_WAIT_DESTINATION_LOWER;
      }
      break;

    case ARM_TRANSFER_WAIT_DESTINATION_LOWER:
      if (liftMoveFinished()) {
        commandGripperOpen();
        armTransferDeadlineMs =
            millis() + GRIPPER_OPEN_SETTLE_MS;
        armTransferPhase =
            ARM_TRANSFER_WAIT_DESTINATION_OPEN;
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
        SerialDebug.print("[ARM TRANSFER] t=");
        SerialDebug.print(millis());
        SerialDebug.print(" ms, COMPLETE, elapsed=");
        SerialDebug.print(millis() - armTransferStartMs);
        SerialDebug.println(" ms");
      }
      break;
  }
}

bool armTransferFinished() {
  return armTransferPhase == ARM_TRANSFER_COMPLETE;
}

void consumeArmTransferCompletion() {
  armTransferPhase = ARM_TRANSFER_IDLE;
}

ArmPose containerPose(float lowerMm) {
  return ArmPose(
      ARM_CONTAINER_CLOCKWISE_DEGREES,
      CONTAINER_EXTENSION_MM,
      -lowerMm);
}

bool nominalRingHorizontalPose(
    uint8_t ringPosition,
    ArmPose &pose) {
  if (ringPosition < 1U || ringPosition > 3U) {
    routeFault("Invalid ring position");
    return false;
  }

  /*
   * 1号圈：逆时针 atan(150/135)，3号圈：顺时针同角度。
   * 侧圈伸长量按勾股定理：
   *   sqrt(150^2 + 135^2) - 135 = 66.804361 mm。
   */
  float imageLeftMm = 0.0f;
  if (ringPosition == 1U) {
    imageLeftMm = SIDE_RING_CENTER_OFFSET_MM;
  } else if (ringPosition == 3U) {
    imageLeftMm = -SIDE_RING_CENTER_OFFSET_MM;
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
  pose.extensionMm =
      radialDistanceMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;
  pose.heightMm = M7_STANDARD_HEIGHT_MM;
  return true;
}

float ringWorkingHeightMm(
    uint8_t ringPosition,
    float lowerMm) {
  const bool isSideRing =
      ringPosition == 1U || ringPosition == 3U;
  return -(lowerMm -
           (isSideRing
                ? SIDE_RING_LOWER_REDUCTION_MM
                : 0.0f));
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
   * 到画面中心的距离不超过50像素用70/72，超过50像素用40/72。
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

const char *workActionKindName(WorkActionKind kind) {
  switch (kind) {
    case WORK_ACTION_NONE:
      return "NONE";
    case WORK_ACTION_RAW:
      return "RAW";
    case WORK_ACTION_PROCESS:
      return "PROCESS";
    case WORK_ACTION_STORAGE:
      return "STORAGE";
  }
  return "UNKNOWN";
}

enum WorkActionPhase {
  WORK_PHASE_IDLE,
  WORK_PHASE_PREPARE,
  WORK_PHASE_RAW_WAIT_RESULT,
  WORK_PHASE_RAW_WAIT_STORAGE_POSITION,
  WORK_PHASE_RING_CALIBRATION,
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

const char *workActionPhaseName(WorkActionPhase phase) {
  switch (phase) {
    case WORK_PHASE_IDLE:
      return "IDLE";
    case WORK_PHASE_PREPARE:
      return "PREPARE";
    case WORK_PHASE_RAW_WAIT_RESULT:
      return "RAW_WAIT_RESULT";
    case WORK_PHASE_RAW_WAIT_STORAGE_POSITION:
      return "RAW_WAIT_STORAGE_POSITION";
    case WORK_PHASE_RING_CALIBRATION:
      return "RING_CALIBRATION";
    case WORK_PHASE_CIRCLE_WAIT_ARM_STANDARD:
      return "CIRCLE_WAIT_ARM_STANDARD";
    case WORK_PHASE_CIRCLE_WAIT_POST_VISION_HEADING:
      return "CIRCLE_WAIT_HEADING";
    case WORK_PHASE_START_UNLOAD:
      return "START_UNLOAD";
    case WORK_PHASE_START_RELOAD:
      return "START_RELOAD";
    case WORK_PHASE_WAIT_TRANSFER:
      return "WAIT_TRANSFER";
    case WORK_PHASE_WAIT_STORAGE_SERVO:
      return "WAIT_STORAGE_SERVO";
    case WORK_PHASE_WAIT_STORAGE_PARK:
      return "WAIT_STORAGE_PARK";
    case WORK_PHASE_WAIT_TRAVEL_BASE:
      return "WAIT_TRAVEL_BASE";
    case WORK_PHASE_WAIT_TRAVEL_BASE_SETTLE:
      return "WAIT_TRAVEL_BASE_SETTLE";
    case WORK_PHASE_START_RESTORE:
      return "START_RESTORE";
    case WORK_PHASE_WAIT_RESTORE:
      return "WAIT_RESTORE";
  }
  return "UNKNOWN";
}

enum TransferPurpose {
  TRANSFER_PURPOSE_NONE,
  TRANSFER_PURPOSE_RAW_TO_CONTAINER,
  TRANSFER_PURPOSE_CONTAINER_TO_RING,
  TRANSFER_PURPOSE_RING_TO_CONTAINER
};

const char *transferPurposeName(TransferPurpose purpose) {
  switch (purpose) {
    case TRANSFER_PURPOSE_NONE:
      return "NONE";
    case TRANSFER_PURPOSE_RAW_TO_CONTAINER:
      return "RAW_TO_CONTAINER";
    case TRANSFER_PURPOSE_CONTAINER_TO_RING:
      return "CONTAINER_TO_RING";
    case TRANSFER_PURPOSE_RING_TO_CONTAINER:
      return "RING_TO_CONTAINER";
  }
  return "UNKNOWN";
}

WorkActionKind activeWorkAction = WORK_ACTION_NONE;
WorkActionPhase workActionPhase = WORK_PHASE_IDLE;
TransferPurpose activeTransferPurpose = TRANSFER_PURPOSE_NONE;
uint8_t workRoundIndex = 0U;
uint8_t workItemIndex = 0U;
uint32_t workStorageServoDeadlineMs = 0UL;
uint32_t workLastMaixSequence = 0UL;

uint8_t rawFilledSlotMask = 0U;
uint8_t rawCollectedCount = 0U;
uint8_t rawPendingSlotIndex = 0U;
uint8_t rawPendingColor = 0U;
ArmPose rawPendingSourcePose;

uint32_t circleStableStartMs = 0UL;
uint32_t circleLastStableSampleMs = 0UL;
uint8_t circleStableSampleCount = 0U;

enum RingCalibrationPhase {
  RING_CALIBRATION_IDLE,
  RING_CALIBRATION_WAIT_BASE,
  RING_CALIBRATION_WAIT_BASE_SETTLE,
  RING_CALIBRATION_WAIT_EXTENSION,
  RING_CALIBRATION_WAIT_LOWER,
  RING_CALIBRATION_WAIT_COORDINATE,
  RING_CALIBRATION_WAIT_CORRECTION_BASE,
  RING_CALIBRATION_WAIT_CORRECTION_BASE_SETTLE,
  RING_CALIBRATION_WAIT_CORRECTION_EXTENSION,
  RING_CALIBRATION_WAIT_SWITCH_LIFT,
  RING_CALIBRATION_WAIT_SWITCH_RETRACT,
  RING_CALIBRATION_COMPLETE
};

RingCalibrationPhase ringCalibrationPhase =
    RING_CALIBRATION_IDLE;
uint8_t calibratingRingPosition = 0U;
uint32_t ringCalibrationDeadlineMs = 0UL;
ArmPose ringCalibrationCurrentPose;
ArmPose ringCalibrationTargetPose;
ArmPose calibratedRingHorizontalPoses[3];
bool calibratedRingPosesValid = false;

bool beginRingCalibration();
bool serviceRingCalibration();

void resetRingCalibration() {
  ringCalibrationPhase = RING_CALIBRATION_IDLE;
  calibratingRingPosition = 0U;
  ringCalibrationDeadlineMs = 0UL;
  ringCalibrationCurrentPose = ArmPose();
  ringCalibrationTargetPose = ArmPose();
  for (uint8_t i = 0U; i < 3U; ++i) {
    calibratedRingHorizontalPoses[i] = ArmPose();
  }
  calibratedRingPosesValid = false;
}

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
  for (uint8_t firstItem = 0U;
       firstItem < 3U;
       ++firstItem) {
    if (taskColors[0][firstItem] == color) {
      return taskPositions[0][firstItem];
    }
  }

  routeFault("Second-batch color has no first-batch stack");
  return 0U;
}

void finishActiveWorkAction() {
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
  SerialDebug.println("Workstation action complete");
}

void beginStorageParkingBeforeWorkFinish() {
  commandStorageServoParkingPosition();
  /*
   * 结束标志只能在机械臂先完整回到标准状态后产生：M7最高、M6最短、
   * M5标准0°。随后再单独把M5逆时针90°转到上电/行驶姿态。
   */
  beginArmStandardization();
  workStorageServoDeadlineMs =
      millis() + STORAGE_SERVO_SETTLE_MS;
  workActionPhase = WORK_PHASE_WAIT_STORAGE_PARK;
  SerialDebug.print("[WORK PARK] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, storage -> 165 deg; arm -> standard "
      "(M6 shortest/M7 highest)");
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
  workRoundIndex =
      static_cast<uint8_t>(roundNumber - 1U);
  workItemIndex = 0U;
  activeTransferPurpose = TRANSFER_PURPOSE_NONE;
  rawFilledSlotMask = 0U;
  rawCollectedCount = 0U;
  rawPendingSlotIndex = 0U;
  rawPendingColor = 0U;
  rawPendingSourcePose = ArmPose();
  workLastMaixSequence =
      latestMaixCoordinate.sequence;
  resetCircleStabilityWindow();
  resetRingCalibration();

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
      ", request=8 (AA 08 BB, all colors)");
  beginMaixRequest(MAIXCAM_ALL_COLORS_REQUEST);
  workActionPhase = WORK_PHASE_RAW_WAIT_RESULT;
}

void beginCircleVision() {
  /*
   * 配套vision-3.py在多个圆中返回距离画面中心最近者。机械臂已经移动到
   * 1号或3号圈的原始水平位置并下降到HOUGH_VISION_HEIGHT_MM后才会调用
   * 本函数；请求9的返回格式为9,x,y。
   */
  resetCircleStabilityWindow();
  workLastMaixSequence =
      latestMaixCoordinate.sequence;
  beginMaixRequest(MAIXCAM_HOUGH_CIRCLE_REQUEST);
}

const char *ringCalibrationPhaseName(
    RingCalibrationPhase phase) {
  switch (phase) {
    case RING_CALIBRATION_IDLE:
      return "IDLE";
    case RING_CALIBRATION_WAIT_BASE:
      return "WAIT_BASE";
    case RING_CALIBRATION_WAIT_BASE_SETTLE:
      return "WAIT_BASE_SETTLE";
    case RING_CALIBRATION_WAIT_EXTENSION:
      return "WAIT_EXTENSION";
    case RING_CALIBRATION_WAIT_LOWER:
      return "WAIT_LOWER";
    case RING_CALIBRATION_WAIT_COORDINATE:
      return "WAIT_COORDINATE";
    case RING_CALIBRATION_WAIT_CORRECTION_BASE:
      return "WAIT_CORRECTION_BASE";
    case RING_CALIBRATION_WAIT_CORRECTION_BASE_SETTLE:
      return "WAIT_CORRECTION_BASE_SETTLE";
    case RING_CALIBRATION_WAIT_CORRECTION_EXTENSION:
      return "WAIT_CORRECTION_EXTENSION";
    case RING_CALIBRATION_WAIT_SWITCH_LIFT:
      return "WAIT_SWITCH_LIFT";
    case RING_CALIBRATION_WAIT_SWITCH_RETRACT:
      return "WAIT_SWITCH_RETRACT";
    case RING_CALIBRATION_COMPLETE:
      return "COMPLETE";
  }
  return "UNKNOWN";
}

void setRingCalibrationPhase(
    RingCalibrationPhase nextPhase) {
  if (ringCalibrationPhase == nextPhase) {
    return;
  }
  SerialDebug.print("[STATE] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, RING_CAL: ");
  SerialDebug.print(
      ringCalibrationPhaseName(ringCalibrationPhase));
  SerialDebug.print(" -> ");
  SerialDebug.println(
      ringCalibrationPhaseName(nextPhase));
  ringCalibrationPhase = nextPhase;
}

bool startRingCalibrationApproach(
    uint8_t ringPosition) {
  ArmPose nominalPose;
  if (!nominalRingHorizontalPose(
          ringPosition, nominalPose)) {
    return false;
  }

  calibratingRingPosition = ringPosition;
  ringCalibrationTargetPose = nominalPose;
  SerialDebug.print("[RING CAL] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, approach nominal ring ");
  SerialDebug.print(
      static_cast<unsigned int>(ringPosition));
  SerialDebug.print(", angle/ext=");
  SerialDebug.print(
      nominalPose.standardFrameAngleDegrees, 3);
  SerialDebug.print("/");
  SerialDebug.print(nominalPose.extensionMm, 3);
  SerialDebug.println(" deg/mm");

  startArmBaseStandardFrameDegrees(
      nominalPose.standardFrameAngleDegrees);
  setRingCalibrationPhase(
      RING_CALIBRATION_WAIT_BASE);
  return true;
}

bool beginRingCalibration() {
  if (activeWorkAction != WORK_ACTION_PROCESS &&
      activeWorkAction != WORK_ACTION_STORAGE) {
    routeFault(
        "Ring calibration requested outside ring station");
    return false;
  }

  resetRingCalibration();
  SerialDebug.print("[RING CAL] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, chassis fixed; calibrate ring 1 then ring 3");
  return startRingCalibrationApproach(1U);
}

bool calculateArmCircleCorrection(
    int16_t imageX,
    int16_t imageY,
    ArmPose &targetPose) {
  const float pixelDeltaX =
      static_cast<float>(imageX - IMAGE_CENTER_X);
  const float outwardCorrectionMm =
      static_cast<float>(imageY - IMAGE_CENTER_Y) *
      CIRCLE_MM_PER_PIXEL *
      IMAGE_Y_TO_ARM_OUTWARD_SIGN;
  const float currentRadiusMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      ringCalibrationCurrentPose.extensionMm;
  const float correctedLocalRadialMm =
      currentRadiusMm + outwardCorrectionMm;
  const float correctedLocalClockwiseMm =
      pixelDeltaX * CIRCLE_MM_PER_PIXEL;

  if (correctedLocalRadialMm <= 0.0f) {
    routeFault("Circle correction crossed arm pivot");
    return false;
  }

  const float targetRadiusMm =
      hypotf(
          correctedLocalRadialMm,
          correctedLocalClockwiseMm);
  targetPose.standardFrameAngleDegrees =
      ringCalibrationCurrentPose
          .standardFrameAngleDegrees -
      atan2f(
          correctedLocalClockwiseMm,
          correctedLocalRadialMm) *
          180.0f / PI_F;
  targetPose.extensionMm =
      targetRadiusMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;
  targetPose.heightMm = HOUGH_VISION_HEIGHT_MM;

  if (!armPoseIsValid(targetPose)) {
    SerialDebug.print(
        "[RING CAL] correction outside reach, angle/ext=");
    SerialDebug.print(
        targetPose.standardFrameAngleDegrees, 3);
    SerialDebug.print("/");
    SerialDebug.println(targetPose.extensionMm, 3);
    routeFault("Circle correction outside arm reach");
    return false;
  }
  return true;
}

void recordCalibratedSideRing() {
  ArmPose recordedPose = ringCalibrationCurrentPose;
  recordedPose.heightMm = M7_STANDARD_HEIGHT_MM;
  calibratedRingHorizontalPoses[
      calibratingRingPosition - 1U] = recordedPose;

  SerialDebug.print("[RING CAL] ring ");
  SerialDebug.print(
      static_cast<unsigned int>(
          calibratingRingPosition));
  SerialDebug.print(
      " centered; recorded angle/ext=");
  SerialDebug.print(
      recordedPose.standardFrameAngleDegrees, 4);
  SerialDebug.print("/");
  SerialDebug.print(recordedPose.extensionMm, 4);
  SerialDebug.println(" deg/mm");
}

bool calculateCalibratedMiddleRing() {
  const ArmPose &ring1 =
      calibratedRingHorizontalPoses[0];
  const ArmPose &ring3 =
      calibratedRingHorizontalPoses[2];
  const float ring1AngleRadians =
      ring1.standardFrameAngleDegrees *
      PI_F / 180.0f;
  const float ring3AngleRadians =
      ring3.standardFrameAngleDegrees *
      PI_F / 180.0f;
  const float ring1RadiusMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      ring1.extensionMm;
  const float ring3RadiusMm =
      ARM_PIVOT_TO_CAMERA_CENTER_MM +
      ring3.extensionMm;

  // 先把两个侧圈圆柱坐标投影到平面直角坐标，再取两圆心的中点。
  const float middleLeftMm =
      (ring1RadiusMm * sinf(ring1AngleRadians) +
       ring3RadiusMm * sinf(ring3AngleRadians)) *
      0.5f;
  const float middleOutwardMm =
      (ring1RadiusMm * cosf(ring1AngleRadians) +
       ring3RadiusMm * cosf(ring3AngleRadians)) *
      0.5f;
  const float middleRadiusMm =
      hypotf(middleLeftMm, middleOutwardMm);

  ArmPose middlePose;
  middlePose.standardFrameAngleDegrees =
      atan2f(middleLeftMm, middleOutwardMm) *
      180.0f / PI_F;
  middlePose.extensionMm =
      middleRadiusMm -
      ARM_PIVOT_TO_CAMERA_CENTER_MM;
  middlePose.heightMm = M7_STANDARD_HEIGHT_MM;
  if (!armPoseIsValid(middlePose)) {
    routeFault(
        "Calculated middle ring outside arm reach");
    return false;
  }

  calibratedRingHorizontalPoses[1] = middlePose;
  calibratedRingPosesValid = true;
  SerialDebug.print(
      "[RING CAL] ring 2 calculated from cylindrical midpoint, angle/ext=");
  SerialDebug.print(
      middlePose.standardFrameAngleDegrees, 4);
  SerialDebug.print("/");
  SerialDebug.print(middlePose.extensionMm, 4);
  SerialDebug.println(" deg/mm");
  return true;
}

bool finishCenteredRingCalibration() {
  stopMaixRequest();
  resetCircleStabilityWindow();
  recordCalibratedSideRing();

  if (calibratingRingPosition == 1U) {
    SerialDebug.println(
        "[RING CAL] ring 1 complete; lift and retract before ring 3");
    if (startLiftToHeightMm(
            M7_STANDARD_HEIGHT_MM)) {
      setRingCalibrationPhase(
          RING_CALIBRATION_WAIT_SWITCH_LIFT);
      return true;
    }
    return false;
  }

  if (calibratingRingPosition != 3U) {
    routeFault("Unexpected calibrated ring number");
    return false;
  }
  if (!calculateCalibratedMiddleRing()) {
    return false;
  }
  setRingCalibrationPhase(
      RING_CALIBRATION_COMPLETE);
  return true;
}

bool serviceRingCalibration() {
  switch (ringCalibrationPhase) {
    case RING_CALIBRATION_IDLE:
      return false;

    case RING_CALIBRATION_WAIT_BASE:
      if (!armMotors.isM5Running()) {
        ringCalibrationDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_BASE_SETTLE);
      }
      break;

    case RING_CALIBRATION_WAIT_BASE_SETTLE:
      if (deadlineReached(
              ringCalibrationDeadlineMs) &&
          startExtensionToMm(
              ringCalibrationTargetPose.extensionMm)) {
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_EXTENSION);
      }
      break;

    case RING_CALIBRATION_WAIT_EXTENSION:
      if (extensionMoveFinished()) {
        ringCalibrationCurrentPose =
            ringCalibrationTargetPose;
        ringCalibrationCurrentPose.heightMm =
            M7_STANDARD_HEIGHT_MM;
        /*
         * 必须先下降M7，再发送AA 09 BB。1号圈和3号圈每次请求都经过
         * 这一状态，不能把请求提前到水平运动过程中。
         */
        if (startLiftToHeightMm(
                HOUGH_VISION_HEIGHT_MM)) {
          setRingCalibrationPhase(
              RING_CALIBRATION_WAIT_LOWER);
        }
      }
      break;

    case RING_CALIBRATION_WAIT_LOWER:
      if (liftMoveFinished()) {
        ringCalibrationCurrentPose.heightMm =
            HOUGH_VISION_HEIGHT_MM;
        SerialDebug.print("[RING CAL] ring ");
        SerialDebug.print(
            static_cast<unsigned int>(
                calibratingRingPosition));
        SerialDebug.print(
            " M7 lowered to ");
        SerialDebug.print(
            HOUGH_VISION_HEIGHT_MM, 2);
        SerialDebug.println(
            " mm; request Hough circle");
        beginCircleVision();
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_COORDINATE);
      }
      break;

    case RING_CALIBRATION_WAIT_COORDINATE: {
      uint8_t resultId = 0U;
      int16_t x = 0;
      int16_t y = 0;
      if (!readNewMaixCoordinate(
              workLastMaixSequence,
              resultId,
              x,
              y)) {
        break;
      }
      (void)resultId;

      const int32_t dx =
          static_cast<int32_t>(x - IMAGE_CENTER_X);
      const int32_t dy =
          static_cast<int32_t>(y - IMAGE_CENTER_Y);
      const float pixelRadius =
          hypotf(
              static_cast<float>(dx),
              static_cast<float>(dy));
      if (pixelRadius >
          CIRCLE_CENTER_TOLERANCE_PIXELS) {
        resetCircleStabilityWindow();
        stopMaixRequest();
        if (!calculateArmCircleCorrection(
                x,
                y,
                ringCalibrationTargetPose)) {
          break;
        }

        SerialDebug.print("[RING CAL] ring ");
        SerialDebug.print(
            static_cast<unsigned int>(
                calibratingRingPosition));
        SerialDebug.print(", circle=");
        SerialDebug.print(x);
        SerialDebug.print("/");
        SerialDebug.print(y);
        SerialDebug.print(
            ", correct M5/M6 to angle/ext=");
        SerialDebug.print(
            ringCalibrationTargetPose
                .standardFrameAngleDegrees,
            4);
        SerialDebug.print("/");
        SerialDebug.println(
            ringCalibrationTargetPose.extensionMm,
            4);
        startArmBaseStandardFrameDegrees(
            ringCalibrationTargetPose
                .standardFrameAngleDegrees);
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_CORRECTION_BASE);
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
        finishCenteredRingCalibration();
      }
      break;
    }

    case RING_CALIBRATION_WAIT_CORRECTION_BASE:
      if (!armMotors.isM5Running()) {
        ringCalibrationDeadlineMs =
            millis() + ARM_BASE_SETTLE_MS;
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_CORRECTION_BASE_SETTLE);
      }
      break;

    case RING_CALIBRATION_WAIT_CORRECTION_BASE_SETTLE:
      if (deadlineReached(
              ringCalibrationDeadlineMs) &&
          startExtensionToMm(
              ringCalibrationTargetPose.extensionMm)) {
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_CORRECTION_EXTENSION);
      }
      break;

    case RING_CALIBRATION_WAIT_CORRECTION_EXTENSION:
      if (extensionMoveFinished()) {
        ringCalibrationCurrentPose =
            ringCalibrationTargetPose;
        beginCircleVision();
        setRingCalibrationPhase(
            RING_CALIBRATION_WAIT_COORDINATE);
      }
      break;

    case RING_CALIBRATION_WAIT_SWITCH_LIFT:
      if (liftMoveFinished()) {
        ringCalibrationCurrentPose.heightMm =
            M7_STANDARD_HEIGHT_MM;
        if (startExtensionToMm(
                M6_STANDARD_EXTENSION_MM)) {
          setRingCalibrationPhase(
              RING_CALIBRATION_WAIT_SWITCH_RETRACT);
        }
      }
      break;

    case RING_CALIBRATION_WAIT_SWITCH_RETRACT:
      if (extensionMoveFinished()) {
        ringCalibrationCurrentPose.extensionMm =
            M6_STANDARD_EXTENSION_MM;
        startRingCalibrationApproach(3U);
      }
      break;

    case RING_CALIBRATION_COMPLETE:
      return true;
  }
  return ringCalibrationPhase ==
         RING_CALIBRATION_COMPLETE;
}

bool calibratedRingPose(
    uint8_t ringPosition,
    float lowerMm,
    ArmPose &pose) {
  if (!calibratedRingPosesValid ||
      ringPosition < 1U ||
      ringPosition > 3U) {
    routeFault("Ring pose requested before calibration");
    return false;
  }

  pose = calibratedRingHorizontalPoses[
      ringPosition - 1U];
  pose.heightMm =
      ringWorkingHeightMm(
          ringPosition, lowerMm);
  if (!armPoseIsValid(pose)) {
    routeFault("Calibrated ring pose outside arm travel");
    return false;
  }
  return true;
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

  /*
   * 用户要求只在释放时少下降5 mm。侧圈原有的5 mm高度补偿仍由
   * ringWorkingHeightMm()叠加处理；粗加工区随后抓回时保持原抓取高度。
   */
  const float releaseLowerMm =
      lowerMm - RING_RELEASE_LOWER_REDUCTION_MM;
  SerialDebug.print("[RING RELEASE] station/ring/lower=");
  SerialDebug.print(
      activeWorkAction == WORK_ACTION_PROCESS
          ? "PROCESS"
          : "STORAGE");
  SerialDebug.print("/");
  SerialDebug.print(ringPosition);
  SerialDebug.print("/");
  SerialDebug.print(releaseLowerMm, 2);
  SerialDebug.println(" mm");
  if (!calibratedRingPose(
          ringPosition, releaseLowerMm, destination)) {
    return;
  }

  beginArmTransfer(
      containerPose(CONTAINER_PICK_LOWER_MM),
      destination);
  activeTransferPurpose =
      TRANSFER_PURPOSE_CONTAINER_TO_RING;
  workActionPhase = WORK_PHASE_WAIT_TRANSFER;
}

void beginReloadingTransfer() {
  ArmPose source;
  const uint8_t ringPosition =
      taskPositions[workRoundIndex][workItemIndex];
  if (!calibratedRingPose(
          ringPosition,
          PROCESS_PLACE_LOWER_MM,
          source)) {
    return;
  }

  beginArmTransfer(
      source,
      containerPose(CONTAINER_PLACE_LOWER_MM));
  activeTransferPurpose =
      TRANSFER_PURPOSE_RING_TO_CONTAINER;
  workActionPhase = WORK_PHASE_WAIT_TRANSFER;
}

void completeTransferAndRotateStorage() {
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
  SerialDebug.print("[TRANSFER DONE] action/round/item/purpose=");
  SerialDebug.print(workActionKindName(activeWorkAction));
  SerialDebug.print("/");
  SerialDebug.print(
      static_cast<unsigned int>(workRoundIndex + 1U));
  SerialDebug.print("/");
  SerialDebug.print(
      static_cast<unsigned int>(workItemIndex + 1U));
  SerialDebug.print("/");
  SerialDebug.print(
      transferPurposeName(activeTransferPurpose));
  SerialDebug.print(", grabs/placements=");
  SerialDebug.print(correctGrabCount);
  SerialDebug.print("/");
  SerialDebug.println(correctPlacementCount);

  consumeArmTransferCompletion();
  ++workItemIndex;
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
  /*
   * 霍夫定位只移动机械臂，不再产生底盘XY累计量。三次搬运结束后仅重新
   * 锁定路线目标航向，消除机械臂惯性带来的车体偏航。
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
  SerialDebug.print("[VISION IMU] t=");
  SerialDebug.print(millis());
  SerialDebug.println(
      " ms, final heading lock; chassis XY unchanged");
}

void serviceCompetitionAction() {
  if (programState != PROGRAM_RUNNING ||
      activeWorkAction == WORK_ACTION_NONE) {
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
        } else if (beginRingCalibration()) {
          workActionPhase =
              WORK_PHASE_RING_CALIBRATION;
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
      if (!rawTargetPose(
              static_cast<float>(x),
              static_cast<float>(y),
              source)) {
        break;
      }

      stopMaixRequest();
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
          containerPose(CONTAINER_PLACE_LOWER_MM));
      activeTransferPurpose =
          TRANSFER_PURPOSE_RAW_TO_CONTAINER;
      workActionPhase = WORK_PHASE_WAIT_TRANSFER;
      break;

    case WORK_PHASE_RING_CALIBRATION:
      if (serviceRingCalibration()) {
        workItemIndex = 0U;
        SerialDebug.print("[RING CAL] t=");
        SerialDebug.print(millis());
        SerialDebug.println(
            " ms, rings 1/2/3 ready; restore arm standard");
        beginArmStandardization();
        workActionPhase =
            WORK_PHASE_CIRCLE_WAIT_ARM_STANDARD;
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
            "M6 shortest, M7 highest");
        finishActiveWorkAction();
      }
      break;

    case WORK_PHASE_START_RESTORE:
      startVisualCorrectionRestore();
      break;

    case WORK_PHASE_WAIT_RESTORE:
      if (updateHeadingLock(MOTION_TIMEOUT_MS)) {
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
  resetRingCalibration();
  stopMaixRequest();
}

void printStateTransition(
    const char *stateMachine,
    const char *previousState,
    const char *nextState) {
  SerialDebug.print("[STATE] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, ");
  SerialDebug.print(stateMachine);
  SerialDebug.print(": ");
  SerialDebug.print(previousState);
  SerialDebug.print(" -> ");
  SerialDebug.println(nextState);
}

void serviceStateTransitionFeedback() {
  if (!ENABLE_STATE_TRANSITION_FEEDBACK) {
    return;
  }

  static bool initialized = false;
  static ProgramState previousProgramState =
      PROGRAM_WAITING;
  static QrScanPhase previousQrScanPhase =
      QR_SCAN_IDLE;
  static ArmStandardPhase previousArmStandardPhase =
      ARM_STANDARD_IDLE;
  static ArmTransferPhase previousArmTransferPhase =
      ARM_TRANSFER_IDLE;
  static WorkActionKind previousWorkAction =
      WORK_ACTION_NONE;
  static WorkActionPhase previousWorkActionPhase =
      WORK_PHASE_IDLE;
  static TransferPurpose previousTransferPurpose =
      TRANSFER_PURPOSE_NONE;

  if (!initialized) {
    previousProgramState = programState;
    previousQrScanPhase = qrScanPhase;
    previousArmStandardPhase = armStandardPhase;
    previousArmTransferPhase = armTransferPhase;
    previousWorkAction = activeWorkAction;
    previousWorkActionPhase = workActionPhase;
    previousTransferPurpose = activeTransferPurpose;
    initialized = true;

    SerialDebug.print("[STATE] baseline: program=");
    SerialDebug.print(programStateName(programState));
    SerialDebug.print(", qr=");
    SerialDebug.print(qrScanPhaseName(qrScanPhase));
    SerialDebug.print(", work=");
    SerialDebug.println(workActionKindName(activeWorkAction));
    return;
  }

  if (previousProgramState != programState) {
    printStateTransition(
        "PROGRAM",
        programStateName(previousProgramState),
        programStateName(programState));
    previousProgramState = programState;
  }
  if (previousQrScanPhase != qrScanPhase) {
    printStateTransition(
        "QR_SCAN",
        qrScanPhaseName(previousQrScanPhase),
        qrScanPhaseName(qrScanPhase));
    previousQrScanPhase = qrScanPhase;
  }
  if (previousArmStandardPhase != armStandardPhase) {
    printStateTransition(
        "ARM_STANDARD",
        armStandardPhaseName(previousArmStandardPhase),
        armStandardPhaseName(armStandardPhase));
    previousArmStandardPhase = armStandardPhase;
  }
  if (previousArmTransferPhase != armTransferPhase) {
    printStateTransition(
        "ARM_TRANSFER",
        armTransferPhaseName(previousArmTransferPhase),
        armTransferPhaseName(armTransferPhase));
    previousArmTransferPhase = armTransferPhase;
  }
  if (previousWorkAction != activeWorkAction) {
    printStateTransition(
        "WORK_ACTION",
        workActionKindName(previousWorkAction),
        workActionKindName(activeWorkAction));
    previousWorkAction = activeWorkAction;
  }
  if (previousWorkActionPhase != workActionPhase) {
    printStateTransition(
        "WORK_PHASE",
        workActionPhaseName(previousWorkActionPhase),
        workActionPhaseName(workActionPhase));
    SerialDebug.print("[WORK CONTEXT] round/item=");
    SerialDebug.print(
        static_cast<unsigned int>(workRoundIndex + 1U));
    SerialDebug.print("/");
    SerialDebug.print(
        static_cast<unsigned int>(workItemIndex + 1U));
    SerialDebug.print(", purpose=");
    SerialDebug.println(
        transferPurposeName(activeTransferPurpose));
    previousWorkActionPhase = workActionPhase;
  }
  if (previousTransferPurpose != activeTransferPurpose) {
    printStateTransition(
        "TRANSFER_PURPOSE",
        transferPurposeName(previousTransferPurpose),
        transferPurposeName(activeTransferPurpose));
    previousTransferPurpose = activeTransferPurpose;
  }
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
      SerialDebug.print("[ARM M5] route home target=");
      SerialDebug.print(command.value);
      SerialDebug.println(" old-frame deg");
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
      SerialDebug.print("[TURN] start/command/target=");
      SerialDebug.print(activeTurnStartHeadingDegrees, 2);
      SerialDebug.print("/");
      SerialDebug.print(counterClockwiseDegrees, 2);
      SerialDebug.print("/");
      SerialDebug.print(
          targetCounterClockwiseHeadingDegrees, 2);
      SerialDebug.println(" deg");
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
      SerialDebug.print("[MOTION] precise profile enabled: vmax/acc=");
      SerialDebug.print(FINAL_MAXIMUM_STEP_RATE, 0);
      SerialDebug.print("/");
      SerialDebug.println(FINAL_STEP_ACCELERATION, 0);
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
       * 用户要求保留原底盘路线/参数，且未给出终点红外协议；因此此命令
       * 只作为“已进入启停区”的同步点，不额外伪造一次底盘修正。
       */
      hmiSetRunStatus("ALIGN");
      finalAlignmentFinished = true;
      SerialDebug.println(
          "[FINAL ALIGN] synchronization only; no sensor correction configured");
      break;

    case COMMAND_HOLD:
      hmiSetRunStatus("HOLD");
      SerialDebug.print("[HOLD] duration=");
      SerialDebug.print(command.value);
      SerialDebug.println(" ms");
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

  routeCommandStartMs = millis();
  commandStartMs = routeCommandStartMs;
  headingStableStartMs = 0;
  motorsArrivedStartMs = 0;
  printCurrentCommand();
  startMotionCommand(route[routeIndex]);
  commandStarted = true;
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
  printRouteCommandCompletion();
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

  SerialDebug.print("[RUN COMPLETE] total=");
  SerialDebug.print(millis() - routeRunStartMs);
  SerialDebug.print(" ms, QR=");
  SerialDebug.print(scanFlag ? 1 : 0);
  SerialDebug.print(", grabs/placements=");
  SerialDebug.print(correctGrabCount);
  SerialDebug.print("/");
  SerialDebug.println(correctPlacementCount);
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

  const RouteCommand &command = route[routeIndex];

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
  SerialDebug.print("[INPUT] PB9 click, program=");
  SerialDebug.println(programStateName(programState));
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
  } else {
    SerialDebug.println(
        "[INPUT] start ignored: route already running");
  }
}

void onStartButtonLongPress() {
  SerialDebug.print("[INPUT] PB9 long press, program=");
  SerialDebug.println(programStateName(programState));
  abortRequested = true;
}

void beginRoute() {
  /*
   * PB9回调保持原一键启动主体；真正的单次比赛门控放在这里。FAULT/FINISHED
   * 状态不能把当前终点姿态重新当作启停区1，必须断电并人工恢复上电
   * 行驶姿态（M5旧0°、M6最短、M7最高）。
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
        "Restart refused: before power-cycle manually restore "
        "M5 old-frame 0 / M6 fully retracted / M7 highest");
    return;
  }

  /*
   * 这里把命令下标、IMU相对航向和软件脉冲位置清零，默认机器人已经按
   * 注释放在启停区1设计起点。它不是回零/定位过程，不会测量当前世界坐标；
   * 若实际摆放位置或朝向不同，整条开环路线会整体带偏。
   */
  routeIndex = 0;
  activeCompetitionRound = 0;
  commandStarted = false;
  resetQrScanActionState();
  commandStartMs = millis();
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
  commandGripperClose();
  commandStorageServoParkingPosition();

  for (uint8_t i = 0; i < 4; ++i) {
    motors[i]->setCurrentPosition(0);
  }

  // 上一次运行结束时可能停留在最终低速档，每次启动都恢复巡航参数。
  setDriveMotionProfile(MAXIMUM_STEP_RATE, STEP_ACCELERATION);
  enableDriveMotors();
  routeRunStartMs = millis();
  programState = PROGRAM_RUNNING;
  startRequested = false;
  imuWaitStatusDisplayed = false;

  SerialDebug.print("[RUN START] t=");
  SerialDebug.print(routeRunStartMs);
  SerialDebug.print(" ms, IMU reference=");
  SerialDebug.print(routeImuReferenceDegrees, 2);
  SerialDebug.print(" deg, route steps=");
  SerialDebug.println(
      static_cast<unsigned int>(ROUTE_COMMAND_COUNT));
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
  SerialDebug.print("[RUN ABORT] t=");
  SerialDebug.print(millis());
  SerialDebug.print(" ms, step=");
  SerialDebug.print(static_cast<unsigned int>(routeIndex + 1));
  SerialDebug.print("/");
  SerialDebug.print(static_cast<unsigned int>(ROUTE_COMMAND_COUNT));
  SerialDebug.println(", reason=PB9 long press");
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

void printStartupConfiguration() {
  SerialDebug.println("=== GongChuang controller boot ===");
  SerialDebug.println(
      "[BOOT] debug UART PB12/PB13 @ 115200");
  SerialDebug.println(
      "[BOOT] HMI PB15/PB14 @ 115200; IMU PD9/PD8 @ 115200");
  SerialDebug.println(
      "[BOOT] QR PE0/PE1 @ 9600; MaixCAM PE7/PE8 @ 115200");
  SerialDebug.println(
      "[BOOT] EMM M6/M7 PA3/PA2 @ 115200; servos PC7/PC6 @ 115200");
  SerialDebug.print(
      "[BOOT] path_only/QR_rx/QR_required/motion_timeouts=");
  SerialDebug.print(PATH_ONLY_TEST ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.print(ENABLE_QR_RECEIVER ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.print(REQUIRE_QR_SUCCESS ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.println(ENABLE_MOTION_TIMEOUTS ? 1 : 0);
  if (!ENABLE_MOTION_TIMEOUTS) {
    SerialDebug.println(
        "[BOOT WARN] chassis motion timeouts are disabled");
  }
}

void printResetCause() {
  const bool brownoutReset =
      __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET;
  const bool pinReset =
      __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET;
  const bool powerOnReset =
      __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET;
  const bool softwareReset =
      __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET;
  const bool independentWatchdogReset =
      __HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != RESET;
  const bool windowWatchdogReset =
      __HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST) != RESET;
  const bool lowPowerReset =
      __HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST) != RESET;

  SerialDebug.print(
      "[RESET] BOR/PIN/POR/SOFT/IWDG/WWDG/LPWR=");
  SerialDebug.print(brownoutReset ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.print(pinReset ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.print(powerOnReset ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.print(softwareReset ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.print(independentWatchdogReset ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.print(windowWatchdogReset ? 1 : 0);
  SerialDebug.print("/");
  SerialDebug.println(lowPowerReset ? 1 : 0);
  if (brownoutReset && !powerOnReset) {
    SerialDebug.println(
        "[RESET DIAG] brownout flag set; inspect battery, regulator and connectors");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

void initializeMotorOutputs() {
  SerialDebug.println(
      "[BOOT] initialize M1-M4 and M5 outputs (disabled)");
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
   * 上电时必须人工保证M5位于旧坐标0°行驶姿态、M6缩回最短、M7升到
   * 最高；本程序只建立软件零点，不含任何限位开关寻零。
   */
  armMotors.setM5CurrentAngle(0.0f);
  armMotors.disableM5();

  setDriveMotionProfile(MAXIMUM_STEP_RATE, STEP_ACCELERATION);
  SerialDebug.println("[BOOT] motor outputs ready");
}

void initializeHmi() {
  SerialDebug.println("[BOOT] initialize HMI");
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
  SerialDebug.println("[BOOT] HMI ready");
}

void initializeManipulationHardware() {
  SerialDebug.println("[BOOT] initialize MaixCAM UART");
  SerialMaixcam.begin(MAIXCAM_BAUDRATE);
  maixcamSerialInitialized = true;
  while (SerialMaixcam.available()) {
    SerialMaixcam.read();
  }
  stopMaixRequest();
  SerialDebug.println("[BOOT] MaixCAM UART ready");

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
  SerialDebug.println(
      "[BOOT] EMM UART ready; recover M6/M7 and set software origins");
  clearArmLinearReceiveBuffer();
  resetArmLinearSoftwareOrigin();
  const bool extensionReady =
      recoverAndVerifyArmLinearAxis(extensionAxis);
  const bool liftReady =
      recoverAndVerifyArmLinearAxis(liftAxis);
  if (!extensionReady || !liftReady) {
    armLinearReferenceValid = false;
    SerialDebug.println(
        "[BOOT ERROR] M6/M7 recovery failed; PB9 start will be refused");
  } else {
    SerialDebug.println(
        "[BOOT] M6/M7 clear/enable/status verification passed");
  }
  clearArmLinearReceiveBuffer();

  SerialDebug.println("[BOOT] initialize servo bus IDs 4/5");
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
  SerialDebug.println("[BOOT] manipulation hardware initialized");
}

} // namespace

void setup() {
  SerialDebug.begin(DEBUG_BAUDRATE);
  printResetCause();
  printStartupConfiguration();
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
  startButton.attachLongPressStart(onStartButtonLongPress);

  initializeHmi();
  if (ENABLE_QR_RECEIVER) {
    resetQrReceiver();
  }

  SerialDebug.println("[READY] controller initialization complete");
  SerialDebug.println(
      "[READY] click PB9 to start; long-press PB9 to stop");
  serviceStateTransitionFeedback();
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

  serviceCompetitionAction();
  updateRoute();
  serviceStateTransitionFeedback();
}
