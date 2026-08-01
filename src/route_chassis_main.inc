#include <AccelStepper.h>
#include <Arduino.h>
#include <MecanumKinematics.h>
#include <OneButton.h>

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

using namespace mecanum;

/*
 * 平移控制策略由编译单元在包含本文件前选择：
 *
 *   1（默认main）：位移和IMU航向均为硬约束。使用有符号四轮速度直接
 *      合成平移与旋转，但行程中先完整保留速度规划给出的平移轮速，IMU
 *      只使用剩余轮速余量，不能令平移减速或暂停。位移到位后平移轮速
 *      自然为0，IMU再使用完整余量完成终点航向；合格后立即进入下一段，
 *      不增加稳定计时。
 *   0（main_v0）：平移过程中不做IMU纠偏；平移脉冲到位后，在本段内部
 *      原地闭环修正航向。明确旋转仍使用实时IMU预测制动。
 *
 * src/main_v0.cpp会先定义为0再包含本文件；普通src/main.cpp保持默认1。
 */
#ifndef GONGCHUANG_TRANSLATION_CONTROL_MODE
#define GONGCHUANG_TRANSLATION_CONTROL_MODE 1
#endif
static_assert(
    GONGCHUANG_TRANSLATION_CONTROL_MODE == 0 ||
        GONGCHUANG_TRANSLATION_CONTROL_MODE == 1,
    "GONGCHUANG_TRANSLATION_CONTROL_MODE must be 0 or 1");

/*
 * GongChuang_route：只保留麦克纳姆底盘的比赛路线
 * ==================================================
 *
 * 本文件已经移除全部机械臂相关内容：
 *   - 不初始化、不驱动 M5/M6/M7；
 *   - 不初始化夹爪、载物盘舵机；
 *   - 不请求 MaixCAM 颜色/圆环识别；
 *   - 原料区、粗加工区和暂存区均只作为底盘路径点，不执行取放动作。
 *
 * 仍然保留：
 *   - PB9 一键启动和长按急停；
 *   - PE0/PE1 二维码扫码器；
 *   - PD9/PD8 JY901/HWT101 航向数据；
 *   - Nextion HMI 的状态、任务码、航向和电池电压显示；
 *   - 四路 STEP/DIR 麦克纳姆底盘。
 *
 * 【为什么没有直接使用 PulseMecanumChassis::drive()】
 * lib/PulseMecanumChassis 的 drive()/run() 使用 AccelStepper::runSpeed()
 * 连续速度模式，只能给出速度，不能直接保证“走 953 mm 后停下”。
 * 该库 README 也明确要求不能让同一组电机同时混用 runSpeed() 速度模式和
 * move()/run() 位置模式。本路线全部是有限距离，因此仍由运动学定义目标：
 *
 *   lib/MecanumKinematics 逆运动学 -> 将车体位移换算为四轮脉冲；
 *   main用runSpeed()执行有符号轮速，投影脉冲进度保证有限距离；
 *   main_v0用move()/run()执行平移位置段，段末再原地修正航向。
 *
 * 同一时刻绝不让一台电机同时运行run()和runSpeed()，两套程序都能保留
 * 有限距离、加减速和段末停车。
 *
 * 【车体方向】
 * 实车以 3、4 号轮所在端为“车头”：
 *   - 用户所说“前进”：向 3、4 侧；
 *   - 用户所说“后退”：向 1、2 侧；
 *   - 用户所说“向右平移”：向 1、3 侧；
 *   - 用户所说“向左平移”：向 2、4 侧。
 *
 * MecanumKinematics 库的 +forward 却沿 1、2 侧，+left 沿 1、3 侧。
 * 所以本文件对用户坐标做如下固定映射：
 *
 *   用户前进 mm -> 库 forward = 负值
 *   用户向右 mm -> 库 left    = 正值
 *   用户逆时针  -> 库 counter-clockwise = 正值
 *
 * 所有未明确要求旋转的路线都以 forward/right 平移为主体，不会先把底盘
 * 转向。main在行程内实时叠加IMU轮速；main_v0在平移脉冲到位后才原地
 * 修正本段航向。明确的90°/180°旋转在两套程序中都持续读取IMU并提前制动。
 */

namespace {

// ===========================================================================
// 1. 用户选择项
// ===========================================================================

/*
 * 启停区选择标志位：
 *   1：从启停区1出发；第1段叠加向右150 mm，步骤1、2、3、21的
 *      纵向分量按原描述方向执行；
 *   2：从启停区2出发；第1段同样叠加向右150 mm，只把步骤
 *      1、2、3、21的前进/后退互换。
 *
 * 切换起点时只改这一行，不要改 route[] 中的其他方向。
 */
constexpr uint8_t START_ZONE_SELECTION = 1U;
static_assert(
    START_ZONE_SELECTION == 1U || START_ZONE_SELECTION == 2U,
    "START_ZONE_SELECTION must be 1 or 2");

/*
 * 分段调试停止位：
 *
 *   0：正常执行完整的1～21段；
 *   1～21：完成所填数字对应的路线段后停止，不再进入下一段。
 *
 * “完成该段”包括运动到位和该段应有的IMU控制。停止后会关闭
 * 底盘使能并进入FINISHED状态；如需重跑，沿用现有安全逻辑重新上电。
 * 逐段调试时只改下面这一行。
 */
constexpr uint8_t STOP_AFTER_STEP = 0U;
static_assert(
    STOP_AFTER_STEP <= 21U,
    "STOP_AFTER_STEP must be from 0 to 21");

/*
 * 第1～21段独立动作系数：
 *
 * 以后调试哪一段，就只修改对应的STEP_xx_MOTION_SCALE；a～g等距离常量
 * 始终继续填写真实/名义值，不要预先乘系数。
 *
 * 快速直行/横移：距离、最大速度和加速度同时乘该段系数；所有横移段的
 * 加速度还会被限制为同档直行加速度的1/2。第7、15段继续额外使用下面
 * 独立的防打滑速度/加速度倍率，但不会再次改变距离。
 * 步骤2扫码慢行：最大慢行距离、慢行速度和加速度使用第2段系数，但
 * 为满足原始要求，慢行距离无论如何都不会超过200 mm。
 * 步骤3扫码后补偿：由b计算出的补偿距离、快速速度和加速度乘第3段系数。
 * 明确旋转：初始脉冲目标、旋转速度和加速度乘该段系数；实时IMU目标也
 * 累加这个“名义转角×本段系数”的设定角度，不会再强行吸附回未乘系数
 * 的90°/180°。旋转内部的残差微调档本身不乘路线系数。
 *
 * 系数只要求大于0，可以根据实车标定填写小于、等于或大于1的数值。
 */
constexpr float STEP_01_MOTION_SCALE = 1.00f; // 快速到扫码慢行起点
constexpr float STEP_02_MOTION_SCALE = 1.00f; // 扫码前慢行
constexpr float STEP_03_MOTION_SCALE = 1.00f; // 扫码后快速补偿到A
constexpr float STEP_04_MOTION_SCALE = 1.07f; // A向右横移到B
constexpr float STEP_05_MOTION_SCALE = 1.00f; // 在B逆时针旋转90°（原第6段动作）
constexpr float STEP_06_MOTION_SCALE = 0.997f; // B向左横移到原料区C（沿用原第5段标定）
constexpr float STEP_07_MOTION_SCALE = 1.02f; // C向右横移到D
constexpr float STEP_08_MOTION_SCALE = 0.99f; // D顺时针旋转180°
constexpr float STEP_09_MOTION_SCALE = 0.9867f; // D前进到E
constexpr float STEP_10_MOTION_SCALE = 0.99f; // E顺时针旋转90°
constexpr float STEP_11_MOTION_SCALE = 1.01f; // E前进到F
constexpr float STEP_12_MOTION_SCALE = 0.977f; // F前进到G
constexpr float STEP_13_MOTION_SCALE = 1.00f; // G顺时针旋转90°
constexpr float STEP_14_MOTION_SCALE = 0.995f; // G前进到C
constexpr float STEP_15_MOTION_SCALE = 1.02f; // 重复7：C向右横移到D
constexpr float STEP_16_MOTION_SCALE = 1.00f; // 重复8：顺时针180°
constexpr float STEP_17_MOTION_SCALE = 0.995f; // 重复9：D前进到E
constexpr float STEP_18_MOTION_SCALE = 0.99f; // 重复10：顺时针90°
constexpr float STEP_19_MOTION_SCALE = 0.98f; // 重复11：E前进到F
constexpr float STEP_20_MOTION_SCALE = 1.015f; // F向右横移到A
constexpr float STEP_21_MOTION_SCALE = 1.077f; // A返回所选启停区

/*
 * 第2、3段独立速度规划倍率：
 *
 *   0.70 = 在这两段各自当前速度上限和加速度基础上降低30%。
 *
 * 这里与STEP_02/03_MOTION_SCALE分开，保证只改变运动快慢：第2段扫码距离
 * 仍最多200 mm，第3段仍严格执行92-b的有符号补偿距离。第2、3段分别列出，
 * 后续可以独立实车调节，其他路线段不会随之改变。
 */
constexpr float STEP_02_MAXIMUM_SPEED_PROFILE_SCALE = 0.70f;
constexpr float STEP_02_ACCELERATION_PROFILE_SCALE = 0.70f;
constexpr float STEP_03_MAXIMUM_SPEED_PROFILE_SCALE = 0.70f;
constexpr float STEP_03_ACCELERATION_PROFILE_SCALE = 0.70f;
static_assert(
    STEP_02_MAXIMUM_SPEED_PROFILE_SCALE > 0.0f &&
        STEP_02_ACCELERATION_PROFILE_SCALE > 0.0f &&
        STEP_03_MAXIMUM_SPEED_PROFILE_SCALE > 0.0f &&
        STEP_03_ACCELERATION_PROFILE_SCALE > 0.0f,
    "Step 2/3 speed-profile scales must be greater than zero");

/*
 * 全部平移段的“行程内IMU纠偏”总开关：
 *
 *   false（当前）：第1～21段的平移位移尚未完成时，不输出任何IMU共同
 *                  旋转轮速；原有纠偏计算和轮速合成代码全部保留；
 *   true：恢复行程内纠偏，并继续受下面第1段专用开关控制。
 *
 * 无论该开关为何值，每段位移完成后的终点IMU纠偏始终启用，航向仍是
 * 进入下一段前的硬约束。需要恢复时只把false改成true，不必恢复其他代码。
 */
constexpr bool ENABLE_IN_MOTION_IMU_CORRECTION = false;

/*
 * 第1段IMU策略专用开关（仅在上方总开关为true时生效）：
 *
 *   false：第1段斜线平移期间完全不输出IMU共同旋转轮速；位移完成后立即
 *          恢复终点航向闭环，继续以设定航向为硬约束，合格后才进入第2段；
 *   true：恢复其他普通平移段所用的“行程内使用剩余轮速纠偏”策略。
 *
 * 该开关只影响第1段的行程内纠偏，不改变第1段距离、斜线右移150 mm、
 * 速度规划、启停区方向，也不影响第2～21段。
 */
constexpr bool STEP_01_ENABLE_IN_MOTION_IMU_CORRECTION = false;

/*
 * 第7、15段长距离横移防打滑档位：
 *
 * 麦轮横移时滚子产生的侧向力较大，打滑通常主要出现在起步加速和停车减速
 * 阶段。因此不改已经标定的横移距离系数，只在公共快速档和本段动作系数
 * 之后，再单独降低最高速度，并更明显地降低加速阶段的加速度。
 *
 * 历史防打滑档在提高50%后为：
 *   最大速度倍率：0.65×1.50 = 0.975；
 *   加速度倍率：0.40×1.50 = 0.60。
 *
 * 本轮只把第7、15段当前最大速度上限降低25%，因此各自最终速度倍率为：
 *   0.975×0.75 = 0.73125。
 * 加速度倍率仍为0.60，减速阶段峰值加速度仍是加速阶段的2/3；距离系数、
 * S形包络和其他路线段均不受本轮速度调整影响。
 *
 * 第7、15段分开列出，后续可根据两次实际横移分别调节。数值越小越不容易
 * 打滑，但耗时会增加；这两个倍率只影响运动曲线，不影响目标距离。
 * 文件后面的全局速度倍率和非对称减速倍率仍会作用于第7、15段。
 */
constexpr float STEP_07_LATERAL_MAX_SPEED_SCALE = 0.73125f;
constexpr float STEP_07_LATERAL_ACCELERATION_SCALE = 0.60f;
constexpr float STEP_15_LATERAL_MAX_SPEED_SCALE = 0.73125f;
constexpr float STEP_15_LATERAL_ACCELERATION_SCALE = 0.60f;

/*
 * 所有横移段的加速度硬上限，相对于同一路段系数下的普通直行加速度：
 *
 *   lateral acceleration <= longitudinal acceleration × 0.50
 *
 * 普通横移原本使用1.50档位倍率，因此会被限制为0.75；第7、15段现有
 * 防打滑倍率为0.60，本来就小于0.75，所以仍保留更柔和的0.60。
 * 此限制不改变横移距离和速度上限；普通横移减速再乘下方梯形2/3比例，
 * 第7、15段则使用单独的S形减速比例。
 */
constexpr float LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL =
    0.50f;
static_assert(
    LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL > 0.0f &&
        LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL <= 1.0f,
    "Lateral acceleration limit must be in (0, 1]");

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
    "Every step motion scale must be greater than zero");
static_assert(
    STEP_07_LATERAL_MAX_SPEED_SCALE > 0.0f &&
        STEP_07_LATERAL_ACCELERATION_SCALE > 0.0f &&
        STEP_15_LATERAL_MAX_SPEED_SCALE > 0.0f &&
        STEP_15_LATERAL_ACCELERATION_SCALE > 0.0f,
    "Step 7/15 anti-slip profile scales must be greater than zero");

// ===========================================================================
// 2. 硬件引脚与串口
// ===========================================================================

// M1～M4 公共使能，低电平有效。
constexpr uint8_t DRIVE_ENABLE_PIN = PE13;

// 四路 STEP/DIR 与现有底盘接线保持一致。
constexpr uint8_t M1_STEP_PIN = PD4;
constexpr uint8_t M1_DIRECTION_PIN = PD6;
constexpr uint8_t M2_STEP_PIN = PE11;
constexpr uint8_t M2_DIRECTION_PIN = PE9;
constexpr uint8_t M3_STEP_PIN = PD15;
constexpr uint8_t M3_DIRECTION_PIN = PD14;
constexpr uint8_t M4_STEP_PIN = PA1;
constexpr uint8_t M4_DIRECTION_PIN = PC3_C;

// 一键启动/长按停止按键，低电平按下，使用内部上拉。
constexpr uint8_t START_BUTTON_PIN = PB9;

/*
 * 默认 Serial 的 RX 会与 M4 STEP 的 PA1 冲突，因此继续使用独立调试串口。
 * HardwareSerial 构造参数顺序是 RX、TX。
 */
constexpr uint8_t DEBUG_RX_PIN = PB12;
constexpr uint8_t DEBUG_TX_PIN = PB13;
constexpr uint8_t HMI_RX_PIN = PB15;
constexpr uint8_t HMI_TX_PIN = PB14;
constexpr uint8_t IMU_RX_PIN = PD9;
constexpr uint8_t IMU_TX_PIN = PD8;
constexpr uint8_t QR_RX_PIN = PE0;
constexpr uint8_t QR_TX_PIN = PE1;
constexpr uint8_t BATTERY_ADC_PIN = PC1;

constexpr uint32_t DEBUG_BAUDRATE = 115200UL;
constexpr uint32_t HMI_BAUDRATE = 115200UL;
constexpr uint32_t IMU_BAUDRATE = 115200UL;
constexpr uint32_t QR_BAUDRATE = 9600UL;

/*
 * 运动串口回传周期。遥测使用不足64字节的紧凑单行，并且只在调试串口
 * TX缓冲区能一次容纳整行时发送；缓冲区忙就丢弃本次采样，不等待串口。
 * 这样不会把Serial.print()的阻塞时间插入高速AccelStepper::run()循环。
 */
constexpr uint32_t MOTION_TELEMETRY_INTERVAL_MS = 500UL;

HardwareSerial SerialDebug(DEBUG_RX_PIN, DEBUG_TX_PIN);
HardwareSerial SerialHmi(HMI_RX_PIN, HMI_TX_PIN);
HardwareSerial SerialImu(IMU_RX_PIN, IMU_TX_PIN);
HardwareSerial SerialQr(QR_RX_PIN, QR_TX_PIN);

OneButton startButton(START_BUTTON_PIN, true, true);

// ===========================================================================
// 3. 用户给出的 a～g 路径尺寸
// ===========================================================================

/*
 * 所有算式都保留原始写法，便于直接与赛道图和用户的 21 个步骤核对。
 * 290 mm 是车体前后方向尺寸，234 mm 是车体左右方向尺寸。
 */
constexpr int32_t CHASSIS_LENGTH_MM = 290;
constexpr int32_t CHASSIS_WIDTH_MM = 234;

// 步骤1：快速到扫码慢行段起点。
constexpr int32_t DISTANCE_A_MM =
    1100 - 300 + CHASSIS_LENGTH_MM / 2 + 8;                 // 953

/*
 * 步骤1在原纵向位移上同步叠加向右150 mm横向分量，麦轮一次合成
 * 斜线，不先旋转底盘。启停区1和启停区2都向右；只有纵向分量随启停区
 * 反转。该数值是名义真实值，和纵向953 mm一样再乘第1段独立系数。
 */
constexpr int32_t STEP_01_ADDITIONAL_LATERAL_MM = 150;

/*
 * 步骤3：扫码慢行段起点到扫码区中点 A 的距离。
 * 用户原式：(900 + 290/2 - a) = 1045 - 953 = 92 mm。
 */
constexpr int32_t SCAN_START_TO_POINT_A_MM =
    900 + CHASSIS_LENGTH_MM / 2 - DISTANCE_A_MM;            // 92

// 步骤2：即使还未扫码成功，慢行最多只能走 200 mm。
constexpr int32_t MAXIMUM_SCAN_DISTANCE_B_MM = 200;

// 步骤4：A 向右横移到中心点 B。
constexpr int32_t DISTANCE_C_MM =
    900 + CHASSIS_WIDTH_MM / 2;                             // 1017
/*
 * 第4段按追加要求固定为向右，并从1017 mm减为867 mm。两个启停区的
 * 第1段都已向右叠加150 mm，因此这里都使用同一个扣减后的距离。
 */
constexpr int32_t STEP_04_RIGHT_DISTANCE_MM =
    DISTANCE_C_MM - STEP_01_ADDITIONAL_LATERAL_MM;          // 867

// 步骤6：先在B逆时针旋转90°，再向车体左侧横移到原料区C。
// 路场中的实际位移长度仍等于原第5段的d，只改变动作顺序和车体运动方向。
constexpr int32_t DISTANCE_D_MM =
    1200 - 85 - CHASSIS_LENGTH_MM / 2 - 20;                // 950

// 步骤7：C 向右横移到粗加工区 D。
constexpr int32_t DISTANCE_E_MM =
    DISTANCE_D_MM + 1200 - 150 -
    CHASSIS_LENGTH_MM / 2 - 20;                            // 1835

// 步骤9：D 前进到左下角 E。
constexpr int32_t DISTANCE_F_MM =
    1200 - 150 - 40 - CHASSIS_WIDTH_MM / 2;                // 893

// 步骤11：E 前进到暂存区 F。
constexpr int32_t DISTANCE_G_MM =
    1200 - 150 - CHASSIS_LENGTH_MM / 2 - 20;               // 885

// 步骤20：暂存区 F 向右横移回扫码区中点 A。
constexpr int32_t STORAGE_F_TO_SCAN_A_MM =
    DISTANCE_F_MM + DISTANCE_C_MM;                          // 1910

// 步骤21：A 直行进入对应启停区；启停区2时由标志位反向。
constexpr int32_t POINT_A_TO_START_ZONE_MM =
    1100 - 300 + CHASSIS_LENGTH_MM / 2;                     // 945

static_assert(DISTANCE_A_MM == 953, "Distance a must be 953 mm");
static_assert(
    SCAN_START_TO_POINT_A_MM == 92,
    "Scan start to point A must be 92 mm");
static_assert(DISTANCE_C_MM == 1017, "Distance c must be 1017 mm");
static_assert(
    STEP_01_ADDITIONAL_LATERAL_MM == 150,
    "Step 1 additional lateral displacement must be 150 mm");
static_assert(
    STEP_04_RIGHT_DISTANCE_MM == 867,
    "Step 4 right displacement must be 867 mm");
static_assert(DISTANCE_D_MM == 950, "Distance d must be 950 mm");
static_assert(DISTANCE_E_MM == 1835, "Distance e must be 1835 mm");
static_assert(DISTANCE_F_MM == 893, "Distance f must be 893 mm");
static_assert(DISTANCE_G_MM == 885, "Distance g must be 885 mm");
static_assert(
    STORAGE_F_TO_SCAN_A_MM == 1910,
    "F to A lateral distance must be f+c=1910 mm");
static_assert(
    POINT_A_TO_START_ZONE_MM == 945,
    "Point A to start zone must be 945 mm");
static_assert(
    SCAN_START_TO_POINT_A_MM < MAXIMUM_SCAN_DISTANCE_B_MM,
    "The slow scan limit must pass point A so step 3 can correct either way");

// ===========================================================================
// 4. 底盘几何、脉冲标定与速度档
// ===========================================================================

constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 2.0f * PI_F;

// lib/MecanumKinematics 当前工程使用的实测轮心几何。
constexpr float WHEELBASE_MM = 187.5f;
constexpr float TRACK_WIDTH_MM = 195.0f;
constexpr float WHEEL_DIAMETER_MM = 100.0f;

/*
 * 平移继续沿用旧底盘实测值 10000 pulse/m。
 * 横移单独保留标定常量，便于以后针对麦轮打滑单独修改。
 */
constexpr float FORWARD_PULSES_PER_METER = 10000.0f;
constexpr float LATERAL_PULSES_PER_METER = 10000.0f;
constexpr float PULSES_PER_WHEEL_REVOLUTION = 3200.0f;

// 顺/逆时针可以在实车标定后分别修正，当前均为 1。
constexpr float COUNTERCLOCKWISE_ROTATION_PULSE_SCALE = 1.0f;
constexpr float CLOCKWISE_ROTATION_PULSE_SCALE = 1.0f;

// M1～M4 电机安装方向；实际前进的驱动脉冲为 +、-、+、-。
const WheelDirections MOTOR_DIRECTIONS(-1, +1, -1, +1);

/*
 * 第一轮公共运动档位统一提高50%：
 *
 *   快速平移：9295/2500   -> 13942.5/3750
 *   扫码慢行：4160/1600   -> 6240/2400
 *   粗旋转：  3750/1000   -> 5625/1500
 *   旋转残差： 1500/450    -> 2250/675
 *
 * 用户追加要求：
 *   - 除第7、15段外，线速度上限和线加速度在上述结果上再乘1.50；
 *   - 粗旋转和旋转残差的角速度上限、角加速度都再乘1.50；
 *   - 第7、15段保持独立防打滑档，不使用第二轮线性提速。
 *
 * 最终追加要求作用于当前全部档位（包括第7、15段和旋转残差）：
 *   - 所有线速度/角速度上限在上一版基础上再乘2/3；
 *   - 加速阶段继续使用当前加速度；
 *   - 制动基础值为当前加速度的2/3；非对称S曲线再用1.50平缓系数降低
 *     实际峰值减速度并提前预留制动距离。
 *
 * 单位分别为pulse/s和pulse/s²。快速平移、扫码慢行和粗旋转还会继续
 * 乘各自路线段的STEP_xx_MOTION_SCALE；旋转残差档不乘路线段系数。
 * 上面是调参历史，实际运行值始终以下方常量当前填写值为准。
 */
constexpr float MOTION_PROFILE_INCREASE_SCALE = 1.50f;
constexpr float NON_07_15_LINEAR_PROFILE_INCREASE_SCALE = 1.50f;
constexpr float ADDITIONAL_ANGULAR_MAXIMUM_SPEED_SCALE = 1.50f;
constexpr float ADDITIONAL_ANGULAR_ACCELERATION_SCALE = 1.50f;
constexpr float FINAL_ALL_MAXIMUM_SPEED_SCALE = 2.0f / 3.0f;
constexpr float FINAL_ALL_ACCELERATION_SCALE = 4.00f;
/*
 * 本轮只把当前全部路线运动档位的最大速度上限统一提高50%。
 * 该倍率作用于普通直行、横移、扫码、粗旋转和旋转残差；不改变加速度、
 * IMU纠偏角速度安全上限、S形参数、距离或分段系数。
 */
constexpr float LATEST_MAXIMUM_SPEED_INCREASE_SCALE = 1.50f;

/*
 * 本轮在上面全部既有调参结果的基础上，再统一放大所有底盘运动档位：
 *
 *   - 线速度上限、角速度上限：当前值 x 2；
 *   - 线加速度、角加速度：    当前值 x 4。
 *
 * 该倍率同时作用于普通直行/横移、扫码、粗旋转和旋转残差。第7、15段
 * 仍走独立的非对称S形分支，并继续叠加各自0.975速度倍率和0.60加速度
 * 倍率；也就是说这里只整体抬高档位，不会把第7、15段改回梯形规划。
 * 距离、逐段运动系数、IMU航向参数和S形形状参数均不受这两个倍率影响。
 */
constexpr float LATEST_GLOBAL_MAXIMUM_SPEED_SCALE = 2.00f;
constexpr float LATEST_GLOBAL_ACCELERATION_SCALE = 4.00f;
static_assert(
    LATEST_GLOBAL_MAXIMUM_SPEED_SCALE > 0.0f &&
        LATEST_GLOBAL_ACCELERATION_SCALE > 0.0f,
    "Latest global motion scales must be greater than zero");

/*
 * 本轮只把“当前线运动结果”减半：
 *
 *   - 普通直行、前进/后退、全部横移和扫码的速度上限 x 0.50；
 *   - 上述平移动作的加速度 x 0.50。
 *
 * 该系数放在FAST/SCAN公共线运动档中，因此所有1～5、7、9、11、12、
 * 14、15、17、19～21段都会统一生效。第7、15段仍使用非对称S形，
 * 其独立防打滑倍率继续保留。TURN和TURN_RESIDUAL不乘此系数，所以粗旋转、
 * 旋转残差以及独立IMU角速度/角加速度参数维持上一版数值。
 */
constexpr float LATEST_LINEAR_MAXIMUM_SPEED_SCALE = 0.50f;
constexpr float LATEST_LINEAR_ACCELERATION_SCALE = 0.50f;
static_assert(
    LATEST_LINEAR_MAXIMUM_SPEED_SCALE > 0.0f &&
        LATEST_LINEAR_ACCELERATION_SCALE > 0.0f,
    "Latest linear motion scales must be greater than zero");

/*
 * 本轮在当前全部线运动结果上继续统一乘2/3：
 *
 *   - 普通前进/后退、斜行、横移、扫码的最大线速度 x 2/3；
 *   - 上述动作的线加速度 x 2/3。
 *
 * 该倍率继续放在FAST/SCAN公共档中，所以第7、15段S形以及其他梯形平移
 * 都会生效；只改变线运动，不改变明确旋转、旋转残差或IMU纠偏的角速度、
 * 角加速度。以后若继续整体调整线运动，只需修改下面两个全局量。
 */
constexpr float LATEST_ADDITIONAL_LINEAR_MAXIMUM_SPEED_SCALE =
    2.0f / 3.0f;
constexpr float LATEST_ADDITIONAL_LINEAR_ACCELERATION_SCALE =
    2.0f / 3.0f;
static_assert(
    LATEST_ADDITIONAL_LINEAR_MAXIMUM_SPEED_SCALE > 0.0f &&
        LATEST_ADDITIONAL_LINEAR_ACCELERATION_SCALE > 0.0f,
    "Additional linear motion scales must be greater than zero");

/*
 * 本轮只降低非对称梯形规划的最终档位：
 *
 *   梯形最大线速度/角速度     = 原当前值 x 0.85
 *   梯形最大线加速度/角加速度 = 原当前值 x 0.85
 *
 * 第7、15段仍使用独立S形防打滑曲线，不乘此系数。梯形减速度由降低后的
 * 加速度继续乘2/3，因此加速、制动和明确旋转的档位会同步降低15%。
 */
constexpr float TRAPEZOID_MAXIMUM_SPEED_REDUCTION_SCALE = 0.85f;
constexpr float TRAPEZOID_ACCELERATION_REDUCTION_SCALE = 0.85f;
/*
 * 本轮只进一步降低梯形规划的加速度：所有非对称梯形段在此前已经计算出的
 * 加速度基础上再乘0.80。速度上限不变；梯形减速度仍由降低后的加速度继续
 * 乘2/3，因此加速和制动强度会同步降到上一版的80%。第7、15段使用S形，
 * 不经过该倍率。
 */
constexpr float LATEST_TRAPEZOID_ACCELERATION_SCALE = 0.80f;
constexpr float TRAPEZOID_FINAL_ACCELERATION_SCALE =
    TRAPEZOID_ACCELERATION_REDUCTION_SCALE *
    LATEST_TRAPEZOID_ACCELERATION_SCALE; // 0.85 x 0.80 = 0.68
static_assert(
    TRAPEZOID_MAXIMUM_SPEED_REDUCTION_SCALE > 0.0f &&
        TRAPEZOID_MAXIMUM_SPEED_REDUCTION_SCALE <= 1.0f &&
        TRAPEZOID_ACCELERATION_REDUCTION_SCALE > 0.0f &&
        TRAPEZOID_ACCELERATION_REDUCTION_SCALE <= 1.0f &&
        LATEST_TRAPEZOID_ACCELERATION_SCALE > 0.0f &&
        LATEST_TRAPEZOID_ACCELERATION_SCALE <= 1.0f,
    "Trapezoid reduction scales must be in (0, 1]");

/*
 * 两种规划的减速阶段统一采用加速阶段加速度的2/3：
 *
 *   - 除第7、15段外：按Desktop/route.cpp恢复非对称梯形，
 *     减速度为加速度的2/3；
 *   - 第7、15段：S形速度包络的实际峰值减速度也严格为加速度的2/3。
 *
 * S形在起止两端仍会按正弦连续改变加速度，所以即使峰值比例与梯形相同，
 * 冲击仍比梯形小；这里统一的是“峰值减速度”，不是把S形改成梯形。
 */
constexpr float TRAPEZOID_DECELERATION_ACCELERATION_SCALE =
    2.0f / 3.0f;
constexpr float S_CURVE_DECELERATION_ACCELERATION_SCALE =
    2.0f / 3.0f;

/*
 * 非对称S形速度包络：
 *
 * 加速侧按已走脉冲s采用正弦速度包络：
 *   v(s) = vmax × sin(PI × s / (2 × Sacc))
 *
 * 减速侧按剩余脉冲r采用正弦速度包络：
 *   v(r) = vstart × sin(PI × r / (2 × Sdec))
 *
 * 两式对应的实际加速度呈正弦形，起点和终点都从接近0平滑过渡。按距离
 * 而非固定时间推进，可避免主循环没有完全跟上理论步频时S曲线提前结束。
 * AccelStepper原生只有梯形规划：main按投影距离直接更新runSpeed()基础
 * 步频；main_v0每2 ms更新moveTo()/run()的速度上限。两者都保留终点距离。
 *
 * 当前减速侧平缓系数设为1.00（中性），因此不会在上面的2/3峰值比例之外
 * 再次降低减速度。S形本身仍通过正弦包络让加速度在起止端连续过渡。
 * 最小速度/加速度比例用于避免0导致库内除零，同时让最后少量取整脉冲
 * 低速完成。
 *
 * 【实车调参入口和影响】
 *   S_CURVE_DECELERATION_GENTLENESS
 *     保持1.00：实际峰值减速度严格等于加速阶段的2/3；
 *     增大：会在2/3基础上继续降低峰值、提前制动，不再满足“恰为2/3”。
 *   S_CURVE_MINIMUM_SPEED_SCALE
 *     增大：末端最后几脉冲更快，但停车瞬间冲击更大；
 *     减小：末端更柔和，但低速收尾时间更长；不能设为0。
 *   S_CURVE_MINIMUM_ACCELERATION_SCALE
 *     只是一项数值安全下限，防止极端平缓设置把库内加速度压得过小；
 *     当前1.05平缓系数下通常不会触发此下限。
 *   S_CURVE_UPDATE_INTERVAL_US
 *     减小：包络更新更细腻但CPU开销更大；
 *     增大：CPU开销更小，但速度上限变化更呈台阶状。
 *   DECELERATION_SWITCH_MARGIN_STEPS
 *     增大：更早进入减速，终点更保守；减小：运行更激进但更易晚刹。
 *
 * FAST/SCAN/TURN各档的最大速度和加速度也会改变S曲线长度，制动距离
 * 近似正比于速度平方、反比于峰值减速度。所有这些参数只影响到达目标的
 * 过程，不会改写普通平移的整数脉冲终点。
 */
constexpr float S_CURVE_DECELERATION_GENTLENESS = 1.00f;
constexpr float S_CURVE_MINIMUM_SPEED_SCALE = 0.05f;
constexpr float S_CURVE_MINIMUM_ACCELERATION_SCALE = 0.05f;
constexpr uint32_t S_CURVE_UPDATE_INTERVAL_US = 2000UL;
static_assert(
    S_CURVE_DECELERATION_GENTLENESS >= 1.0f,
    "S-curve deceleration must not be sharper than acceleration");
static_assert(
    S_CURVE_MINIMUM_SPEED_SCALE > 0.0f &&
        S_CURVE_MINIMUM_SPEED_SCALE < 1.0f,
    "S-curve minimum speed scale must be in (0, 1)");
static_assert(
    S_CURVE_MINIMUM_ACCELERATION_SCALE > 0.0f &&
        S_CURVE_MINIMUM_ACCELERATION_SCALE <= 1.0f,
    "S-curve minimum acceleration scale must be in (0, 1]");

/*
 * 每次loop()都会检查剩余脉冲和当前速度。提前8 pulse进入S形减速，可吸收
 * 主循环一次执行时间和浮点四舍五入造成的边界误差，避免高速时切换过晚。
 */
constexpr float DECELERATION_SWITCH_MARGIN_STEPS = 8.0f;
constexpr float FAST_MAXIMUM_STEP_RATE =
    9295.0f *
    MOTION_PROFILE_INCREASE_SCALE *
    FINAL_ALL_MAXIMUM_SPEED_SCALE *
    LATEST_MAXIMUM_SPEED_INCREASE_SCALE *
    LATEST_GLOBAL_MAXIMUM_SPEED_SCALE *
    LATEST_LINEAR_MAXIMUM_SPEED_SCALE *
    LATEST_ADDITIONAL_LINEAR_MAXIMUM_SPEED_SCALE;
constexpr float FAST_STEP_ACCELERATION =
    2500.0f *
    MOTION_PROFILE_INCREASE_SCALE *
    FINAL_ALL_ACCELERATION_SCALE *
    LATEST_GLOBAL_ACCELERATION_SCALE *
    LATEST_LINEAR_ACCELERATION_SCALE *
    LATEST_ADDITIONAL_LINEAR_ACCELERATION_SCALE;
constexpr float SCAN_MAXIMUM_STEP_RATE =
    4160.0f *
    MOTION_PROFILE_INCREASE_SCALE *
    FINAL_ALL_MAXIMUM_SPEED_SCALE *
    LATEST_MAXIMUM_SPEED_INCREASE_SCALE *
    LATEST_GLOBAL_MAXIMUM_SPEED_SCALE *
    LATEST_LINEAR_MAXIMUM_SPEED_SCALE *
    LATEST_ADDITIONAL_LINEAR_MAXIMUM_SPEED_SCALE;
constexpr float SCAN_STEP_ACCELERATION =
    1600.0f *
    MOTION_PROFILE_INCREASE_SCALE *
    FINAL_ALL_ACCELERATION_SCALE *
    LATEST_GLOBAL_ACCELERATION_SCALE *
    LATEST_LINEAR_ACCELERATION_SCALE *
    LATEST_ADDITIONAL_LINEAR_ACCELERATION_SCALE;
constexpr float TURN_MAXIMUM_STEP_RATE =
    3750.0f *
    MOTION_PROFILE_INCREASE_SCALE *
    ADDITIONAL_ANGULAR_MAXIMUM_SPEED_SCALE *
    FINAL_ALL_MAXIMUM_SPEED_SCALE *
    LATEST_MAXIMUM_SPEED_INCREASE_SCALE *
    LATEST_GLOBAL_MAXIMUM_SPEED_SCALE;
constexpr float TURN_STEP_ACCELERATION =
    1000.0f *
    MOTION_PROFILE_INCREASE_SCALE *
    ADDITIONAL_ANGULAR_ACCELERATION_SCALE *
    FINAL_ALL_ACCELERATION_SCALE *
    LATEST_GLOBAL_ACCELERATION_SCALE;
constexpr float TURN_RESIDUAL_MAXIMUM_STEP_RATE =
    1500.0f *
    MOTION_PROFILE_INCREASE_SCALE *
    ADDITIONAL_ANGULAR_MAXIMUM_SPEED_SCALE *
    FINAL_ALL_MAXIMUM_SPEED_SCALE *
    LATEST_MAXIMUM_SPEED_INCREASE_SCALE *
    LATEST_GLOBAL_MAXIMUM_SPEED_SCALE;
constexpr float TURN_RESIDUAL_STEP_ACCELERATION =
    450.0f *
    MOTION_PROFILE_INCREASE_SCALE *
    ADDITIONAL_ANGULAR_ACCELERATION_SCALE *
    FINAL_ALL_ACCELERATION_SCALE *
    LATEST_GLOBAL_ACCELERATION_SCALE;
constexpr uint16_t MINIMUM_STEP_WIDTH_US = 2U;

// ===========================================================================
// 5. 行程内IMU航向保持与明确旋转参数
// ===========================================================================

/*
 * HWT101 实车约定：逆时针旋转时角度增加。
 * 如果以后安装方向反了，只把 +1 改成 -1，不要改路线中的顺/逆时针。
 */
constexpr int8_t IMU_COUNTERCLOCKWISE_SIGN = +1;
constexpr uint32_t IMU_STALE_TIMEOUT_MS = 600UL;

/*
 * main的直行、后退、横移和扫码慢行使用“行程内IMU航向保持”：
 *   - 每收到一帧新的有效IMU数据，就计算当前航向与路线设定航向的误差；
 *   - 把小角度旋转轮速实时叠加到纯平移轮速，不改写平移目标；
 *   - 补偿角速度采用P控制，并限制角速度变化率，避免纠偏突然跳变；
 *   - 位移完成后可只保留旋转轮速，位移和航向同时合格才进入下一段。
 *
 * main_v0在平移期间不执行上述控制，纯平移到位后再原地读取IMU修正；
 * 因此下面的P控制参数只作用于main的行程内控制。
 *
 * 步骤5/8/10/13/16/18继续使用“旋转中实时IMU闭环”：
 *   - 转动过程中不断用IMU计算距离用户设定角度还有多少；
 *   - 根据当前电机步频和目标减速度预测制动角，提前开始减速；
 *   - 停下后若仍超差，小角度补偿也留在同一次旋转控制内部完成；
 *   - 达到容差后直接进入下一段。
 *
 * 下面的行程内参数均独立列出，便于实车逐项调节：
 *   - DEADBAND：误差在±0.15°内不继续增加纠偏量，避免噪声抖动；
 *   - KP：每1°误差对应的期望纠偏角速度；
 *   - MAX_RATE：纠偏角速度绝对上限；
 *   - RATE_SLEW：纠偏角速度每秒最多变化多少，相当于纠偏角加速度限制；
 *   - MAX_ACCUMULATED：单段允许累计的最大纠偏角，防止IMU方向或数据异常
 *     时控制量无限增长；
 *   - MAX_DT：IMU数据偶发延迟后，单次积分最多采用50 ms，避免目标突跳。
 */
constexpr float TRANSLATION_HEADING_DEADBAND_DEGREES = 0.15f;
constexpr float TRANSLATION_HEADING_KP_PER_SECOND = 3.0f;
constexpr float TRANSLATION_HEADING_MAXIMUM_RATE_DPS = 30.0f;
constexpr float TRANSLATION_HEADING_RATE_SLEW_DPS2 = 120.0f;
constexpr float TRANSLATION_MAXIMUM_ACCUMULATED_CORRECTION_DEGREES =
    20.0f;
constexpr float TRANSLATION_HEADING_MAXIMUM_CONTROL_DT_SECONDS = 0.050f;
constexpr float TRANSLATION_COMPLETION_RATE_TOLERANCE_DPS = 0.25f;

/*
 * 位移完成后的终点专用纠偏参数：
 *
 *   - 行程中仍使用上面的基础Kp=3.0，并且IMU只能使用平移剩余轮速；
 *   - 位移到位后Kp乘2.0，得到终点Kp=6.0，更快消除小角度残差；
 *   - 旧代码没有显式最低纠偏角速度。本轮将1.0°/s列为调参基准，再按
 *     用户要求乘2.0，得到终点最低纠偏角速度2.0°/s；
 *   - 最低角速度只在误差仍超出本段容差时生效。误差合格后目标立即回到0，
 *     再按现有120°/s²角速度斜率降速，不会为了维持最低速度持续旋转；
 *   - 30°/s最大角速度和120°/s²角加速度本轮不变。
 */
constexpr float TRANSLATION_TERMINAL_KP_SCALE = 2.0f;
constexpr float TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_BASE_DPS =
    1.0f;
constexpr float TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_SCALE =
    2.0f;
constexpr float TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_DPS =
    TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_BASE_DPS *
    TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_SCALE;

/*
 * 第4段结束到第5段启动之间的专用IMU主动调整窗口。
 *
 * 计时起点是第4段纯平移位移投影到位，而不是第4段命令开始；300 ms内仍
 * 高频读取IMU并运行终点纠偏控制器，不是关闭电机后的盲等。300 ms只是
 * 最短窗口：到时若航向误差或纠偏角速度仍未合格，会继续调整；总上限仍由
 * MOTION_TIMEOUT_MS约束。
 */
constexpr uint32_t STEP_04_TERMINAL_IMU_MINIMUM_DURATION_MS = 300UL;
/*
 * main平移优先、终点航向硬约束：
 *
 *   - 行程中始终先完整保留S形/梯形规划给出的平移基础轮速；
 *   - IMU共同旋转轮速只使用四轮尚未占用的速度余量，余量不足就削弱
 *     本轮纠偏，绝不反过来削弱、暂停或反转平移；
 *   - 位移投影剩余不超过0.75 pulse就永久关闭该段平移分量，之后只允许
 *     四轮同号的终点旋转分量，并可使用本段完整轮速余量；
 *   - 最后同时满足位移和航向即完成，不要求条件持续保持若干毫秒。
 */
constexpr float TRANSLATION_POSITION_TOLERANCE_STEPS = 0.75f;
constexpr float TRANSLATION_TERMINAL_MINIMUM_STEP_RATE = 80.0f;
/*
 * 第二次原料区停车时使用比普通平移0.15°更严格的0.05°死区，确保用户
 * 指出的0.1°差值会真正产生纠偏轮速，而不是被普通死区直接视为合格。
 */
constexpr float STEP_15_PRE_ALIGNMENT_TOLERANCE_DEGREES = 0.05f;
constexpr float TURN_HEADING_TOLERANCE_DEGREES = 0.15f;
constexpr float MAXIMUM_SINGLE_TURN_RESIDUAL_DEGREES = 12.0f;
constexpr float MINIMUM_TURN_RESIDUAL_DEGREES = 0.15f;
constexpr uint32_t MOTION_TIMEOUT_MS = 30000UL;
static_assert(
    TRANSLATION_HEADING_DEADBAND_DEGREES >= 0.0f &&
        TRANSLATION_HEADING_KP_PER_SECOND > 0.0f &&
        TRANSLATION_HEADING_MAXIMUM_RATE_DPS > 0.0f &&
        TRANSLATION_HEADING_RATE_SLEW_DPS2 > 0.0f &&
        TRANSLATION_MAXIMUM_ACCUMULATED_CORRECTION_DEGREES > 0.0f &&
        TRANSLATION_COMPLETION_RATE_TOLERANCE_DPS >= 0.0f &&
        TRANSLATION_TERMINAL_KP_SCALE > 0.0f &&
        TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_DPS > 0.0f &&
        TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_DPS <=
            TRANSLATION_HEADING_MAXIMUM_RATE_DPS &&
        STEP_04_TERMINAL_IMU_MINIMUM_DURATION_MS >= 300UL &&
        TRANSLATION_POSITION_TOLERANCE_STEPS > 0.0f &&
        TRANSLATION_TERMINAL_MINIMUM_STEP_RATE > 0.0f &&
        STEP_15_PRE_ALIGNMENT_TOLERANCE_DEGREES > 0.0f &&
        STEP_15_PRE_ALIGNMENT_TOLERANCE_DEGREES <
            TRANSLATION_HEADING_DEADBAND_DEGREES,
    "Translation IMU controller parameters must be valid");

/*
 * 明确旋转的提前制动补偿：
 *
 * TURN_IMU_CONTROL_LATENCY_SECONDS用于补偿IMU一帧加主循环调度的总延迟；
 * TURN_PREDICTIVE_BRAKE_MARGIN_DEGREES再留0.10°的小余量，吸收脉冲换算
 * 取整和传感器噪声。这两个量只改变“何时开始刹车”，不改变用户设定目标角。
 */
constexpr float TURN_IMU_CONTROL_LATENCY_SECONDS = 0.015f;
constexpr float TURN_PREDICTIVE_BRAKE_MARGIN_DEGREES = 0.10f;

// ===========================================================================
// 6. HMI 与电池显示参数
// ===========================================================================

constexpr float BATTERY_ADC_REFERENCE_VOLTAGE = 3.3f;
constexpr float BATTERY_DIVIDER_RATIO = 11.0f; // 10 kOhm + 1 kOhm
constexpr float BATTERY_CALIBRATION_SCALE = 1.0f;
constexpr float BATTERY_FILTER_ALPHA = 0.10f;
constexpr uint32_t BATTERY_SAMPLE_INTERVAL_MS = 50UL;
constexpr uint32_t BATTERY_HMI_INTERVAL_MS = 500UL;
constexpr uint32_t HMI_YAW_INTERVAL_MS = 100UL;

uint32_t lastBatterySampleMs = 0UL;
uint32_t lastBatteryHmiMs = 0UL;
uint32_t lastHmiYawMs = 0UL;
float filteredBatteryVoltage = 0.0f;
bool batteryVoltageInitialized = false;

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

void hmiSetRunStatus(const char *status) {
  hmiSetText("t7", status);
}

void hmiShowTaskCode(const char *taskCode) {
  // 示例 134+123+314+231：前 8 个字符放 t3，后 7 个字符放 t8。
  char firstRow[9] = {0};
  char secondRow[8] = {0};
  memcpy(firstRow, taskCode, 8U);
  memcpy(secondRow, taskCode + 8U, 7U);
  hmiSetText("t3", firstRow);
  hmiSetText("t8", secondRow);
}

void serviceBatteryVoltage() {
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
    hmiSetValue(
        "x1",
        static_cast<int32_t>(
            lroundf(filteredBatteryVoltage * 100.0f)));
  }
}

// ===========================================================================
// 7. 二维码接收：有效任务码到达后立即置 scanFlag
// ===========================================================================

constexpr size_t QR_CODE_LENGTH = 15U;
constexpr size_t QR_BUFFER_SIZE = 32U;

char qrData[QR_BUFFER_SIZE] = {0};
size_t qrDataIndex = 0U;
bool qrOverflow = false;
bool scanFlag = false;

bool characterInRange(char value, char minimum, char maximum) {
  return value >= minimum && value <= maximum;
}

/*
 * 检查连续三个数字是否都在给定范围内且不重复。
 * 返回值是数字集合的位掩码；0 表示格式或重复错误。
 */
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

/*
 * 任务码仍按正式格式校验，而不是收到任意串口字节就判定“扫码成功”：
 *   CCC+PPP+CCC+PPP
 *   - 两组 CCC 各为 1～4 中不重复的三种颜色，且两组颜色集合相同；
 *   - 两组 PPP 均为 1、2、3 的排列；
 *   - 总长度固定 15，分隔符固定在 3、7、11。
 *
 * 虽然本版不再用任务码驱动机械臂，严格校验仍能避免噪声误触发底盘离开
 * 慢行扫码段。
 */
bool isValidTaskCode(const char *code) {
  if (strlen(code) != QR_CODE_LENGTH) {
    return false;
  }
  if (code[3] != '+' || code[7] != '+' || code[11] != '+') {
    return false;
  }

  const uint8_t firstColorMask =
      threeDigitMask(code, '1', '4');
  const uint8_t secondColorMask =
      threeDigitMask(code + 8U, '1', '4');
  if (firstColorMask == 0U ||
      secondColorMask == 0U ||
      firstColorMask != secondColorMask) {
    return false;
  }

  constexpr uint8_t POSITION_MASK_123 = 0x07U;
  return threeDigitMask(code + 4U, '1', '3') ==
             POSITION_MASK_123 &&
         threeDigitMask(code + 12U, '1', '3') ==
             POSITION_MASK_123;
}

void resetQrReceiver() {
  qrDataIndex = 0U;
  qrOverflow = false;
  scanFlag = false;
  qrData[0] = '\0';
  while (SerialQr.available()) {
    SerialQr.read();
  }
}

void finishQrFrame() {
  if (qrDataIndex == 0U && !qrOverflow) {
    return;
  }

  qrData[qrDataIndex] = '\0';
  if (!qrOverflow && isValidTaskCode(qrData)) {
    scanFlag = true;
    hmiSetText("t1", "QROK");
    hmiShowTaskCode(qrData);
    SerialDebug.print("[QR] accepted code=");
    SerialDebug.println(qrData);
  } else {
    hmiSetText("t1", "QRERR");
    SerialDebug.print("[QR] rejected, overflow/length/data=");
    SerialDebug.print(qrOverflow ? 1 : 0);
    SerialDebug.print("/");
    SerialDebug.print(static_cast<unsigned int>(qrDataIndex));
    SerialDebug.print("/");
    SerialDebug.println(qrData);
  }

  qrDataIndex = 0U;
  qrOverflow = false;
}

void receiveQrData() {
  /*
   * 一旦已经获得有效码，只清空后续重复输出，不允许第二帧改写本次比赛结果。
   */
  if (scanFlag) {
    while (SerialQr.available()) {
      SerialQr.read();
    }
    return;
  }

  while (SerialQr.available()) {
    const char incoming =
        static_cast<char>(SerialQr.read());

    // 同时兼容 CR、LF 和 CRLF 结尾。
    if (incoming == '\r' || incoming == '\n') {
      finishQrFrame();
      continue;
    }

    if (qrDataIndex < QR_BUFFER_SIZE - 1U) {
      qrData[qrDataIndex++] = incoming;
    } else {
      qrOverflow = true;
    }
  }
}

// ===========================================================================
// 8. IMU：校验 0x53 帧并把 ±180° 展开成连续角度
// ===========================================================================

bool imuInitialized = false;
float imuLastSignedRawDegrees = 0.0f;
float imuCounterClockwiseDegrees = 0.0f;
uint32_t lastImuReceiveMs = 0UL;
/*
 * 每收到一帧校验正确的0x53航向数据就递增。行程内控制器按序号判断是否
 * 真正获得了新测量值，避免在两个IMU帧之间对同一个误差重复积分。
 */
uint32_t imuValidFrameSequence = 0UL;
uint8_t imuFrame[11] = {0};
uint8_t imuFrameIndex = 0U;

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
  ++imuValidFrameSequence;
  if (!imuInitialized) {
    imuInitialized = true;
    imuLastSignedRawDegrees = counterClockwiseSignedRaw;
    imuCounterClockwiseDegrees = 0.0f;
    SerialDebug.print("[IMU] first valid yaw frame=");
    SerialDebug.print(counterClockwiseSignedRaw, 2);
    SerialDebug.println(" deg");
    return;
  }

  /*
   * 对相邻帧先做相位展开。例如 179° 到 -179° 实际是继续转了 +2°，
   * 不能按普通减法当成 -358°。连续累计后可正确跟踪路线中的多次 180°
   * 以及超过一圈的总转角。
   */
  const float delta =
      wrapDeltaDegrees(
          counterClockwiseSignedRaw -
          imuLastSignedRawDegrees);
  imuCounterClockwiseDegrees += delta;
  imuLastSignedRawDegrees = counterClockwiseSignedRaw;
}

void receiveImuData() {
  while (SerialImu.available()) {
    const uint8_t incoming =
        static_cast<uint8_t>(SerialImu.read());

    if (imuFrameIndex == 0U) {
      if (incoming == 0x55U) {
        imuFrame[imuFrameIndex++] = incoming;
      }
      continue;
    }

    imuFrame[imuFrameIndex++] = incoming;
    if (imuFrameIndex < sizeof(imuFrame)) {
      continue;
    }

    uint8_t checksum = 0U;
    for (uint8_t i = 0U; i < 10U; ++i) {
      checksum = static_cast<uint8_t>(
          checksum + imuFrame[i]);
    }

    if (checksum == imuFrame[10] &&
        imuFrame[0] == 0x55U &&
        imuFrame[1] == 0x53U) {
      // 0x53 角度帧的 yaw 位于第 6、7 字节，小端 int16_t。
      const uint16_t yawUnsigned =
          static_cast<uint16_t>(imuFrame[6]) |
          (static_cast<uint16_t>(imuFrame[7]) << 8U);
      updateContinuousImuHeading(
          static_cast<int16_t>(yawUnsigned));
    }

    /*
     * 坏帧末字节若恰好是下一帧 0x55 帧头，就保留它继续同步；
     * 否则从头寻找下一帧。
     */
    if (checksum != imuFrame[10] &&
        imuFrame[10] == 0x55U) {
      imuFrame[0] = 0x55U;
      imuFrameIndex = 1U;
    } else {
      imuFrameIndex = 0U;
    }
  }
}

bool imuIsFresh() {
  return imuInitialized &&
         millis() - lastImuReceiveMs <= IMU_STALE_TIMEOUT_MS;
}

// ===========================================================================
// 9. 麦轮位移到四轮脉冲的换算
// ===========================================================================

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

AccelStepper *const motors[4] = {
    &motor1, &motor2, &motor3, &motor4};

enum DriveProfileShape {
  DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID,
  DRIVE_PROFILE_ASYMMETRIC_S_CURVE
};

/*
 * 当前运动段的双速度规划状态：
 *
 *   1. 非对称梯形：除第7、15段外使用。起步立即给出完整速度上限，
 *      AccelStepper按完整加速度上升；制动区切换到route.cpp的2/3减速度。
 *   2. 非对称S形：只给第7、15段使用。速度上限按已走/剩余脉冲正弦变化，
 *      并使用单独的更平缓减速度，继续抑制长距离横移打滑。
 *
 * 两种规划都只改变过程速度和加速度。main用平移投影到位判定；main_v0
 * 和明确旋转仍要求四轮distanceToGo()==0。
 */
DriveProfileShape activeDriveProfileShape =
    DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID;
float activeDriveAcceleration = 1.0f;
float activeDriveDeceleration = 1.0f;
float activeDriveMaximumStepRate = 1.0f;
float appliedDriveMaximumStepRate = 1.0f;
float appliedDriveAcceleration = 1.0f;
bool driveDecelerationActive = false;
uint32_t driveProfileLastUpdateUs = 0UL;
float driveAccelerationDistanceSteps = 0.0f;
float driveDecelerationStartStepRate = 0.0f;
float driveDecelerationStartRemainingSteps = 0.0f;

/*
 * 明确旋转的实时IMU控制状态。
 *
 * integratedTurnDirectionSign：逆时针为+1，顺时针为-1；
 * activeTurnPulsesPerDegree：按当前旋转方向标定后的平均单轮pulse/°；
 * integratedTurnBrakeCommandIssued：防止同一次旋转重复改写S形停车目标。
 */
bool integratedTurnControlActive = false;
bool integratedTurnBrakeCommandIssued = false;
int8_t integratedTurnDirectionSign = 0;
float activeTurnPulsesPerDegree = 1.0f;

struct MotorPulses {
  long motor1;
  long motor2;
  long motor3;
  long motor4;

  MotorPulses(
      long m1 = 0L, long m2 = 0L,
      long m3 = 0L, long m4 = 0L)
      : motor1(m1), motor2(m2),
        motor3(m3), motor4(m4) {}
};

MotorPulses driveProfileMotionOrigin;

/*
 * 两套平移控制共用的状态。
 *
 * main（模式1）：translationRelativeMotorPulses是纯平移硬约束向量。控制器
 * 从四轮实际位置中减去共同旋转分量，再投影到此向量得到0～1的平移进度；
 * translationBaseStepRate只允许为非负值，所以位移进度不会因IMU重算而倒退。
 * translationBaseStepRate先完整占用速度规划要求的轮速；随后把
 * translationHeadingCorrectionRateDps转换成四轮同号轮速，只使用各轮
 * 剩余余量。translationAppliedHeadingCorrectionRateDps记录本轮真正获准
 * 输出的纠偏角速度，避免“请求了但被平移挤掉”的纠偏量触发虚假安全故障。
 *
 * main_v0（模式0）：startRelativeMotorMove()的目标由move()/run()严格执行；
 * 平移过程中不调用IMU纠偏，四轮到位后才进入本段的原地航向闭环。
 */
bool translationHeadingControlActive = false;
MotorPulses translationRelativeMotorPulses;
MotorPulses translationMotionOrigin;
float activeTranslationForwardMm = 0.0f;
float activeTranslationRightMm = 0.0f;
float translationAccumulatedCorrectionDegrees = 0.0f;
float translationHeadingCorrectionRateDps = 0.0f;
float translationAppliedHeadingCorrectionRateDps = 0.0f;
uint32_t translationLastImuFrameSequence = 0UL;
uint32_t translationLastControlMs = 0UL;
float translationMaximumBasePulses = 0.0f;
float translationBaseStepRate = 0.0f;
float translationAppliedBaseStepRate = 0.0f;
uint32_t translationVelocityProfileLastUpdateUs = 0UL;
bool translationBaseDisplacementComplete = false;
bool translationBaseForcedComplete = false;
bool translationTerminalCorrectionStartValid = false;
uint32_t translationTerminalCorrectionStartMs = 0UL;

/*
 * main_v0段后纠偏状态。动作仍属于刚完成的平移段，因而不会插入纯等待；
 * 每次小角度位置运动停下后立刻读取IMU，合格就立即放行，不设稳定计时。
 */
enum PostTranslationCompletionAction {
  POST_TRANSLATION_ADVANCE_ROUTE,
  POST_TRANSLATION_WAIT_FOR_SCAN_CODE
};
bool postTranslationHeadingCorrectionActive = false;
PostTranslationCompletionAction postTranslationCompletionAction =
    POST_TRANSLATION_ADVANCE_ROUTE;

long roundedPulseCount(float pulses) {
  return static_cast<long>(
      pulses >= 0.0f ? pulses + 0.5f : pulses - 0.5f);
}

/*
 * 把库坐标系中的位移 dx、dy、dHeading 换成四轮相对脉冲。
 *
 * MecanumKinematics::inverse() 本来处理速度。麦轮模型是线性的，把
 * vx/vy/wz 换成 dx/dy/dHeading 后，输出就代表四轮转角。
 * 平移和旋转分别使用实测 pulse/m 与 3200 pulse/rev，再叠加并施加
 * M1～M4 的安装方向符号。
 */
MotorPulses libraryDisplacementToMotorPulses(
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
      geometry.wheelRadiusMeters() *
      FORWARD_PULSES_PER_METER;
  const float lateralFactor =
      geometry.wheelRadiusMeters() *
      LATERAL_PULSES_PER_METER;
  const float rotationScale =
      counterClockwiseRadians >= 0.0f
          ? COUNTERCLOCKWISE_ROTATION_PULSE_SCALE
          : CLOCKWISE_ROTATION_PULSE_SCALE;
  const float rotationFactor =
      PULSES_PER_WHEEL_REVOLUTION /
      TWO_PI_F * rotationScale;

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
      roundedPulseCount(
          physical1 * MOTOR_DIRECTIONS.frontLeft),
      roundedPulseCount(
          physical2 * MOTOR_DIRECTIONS.frontRight),
      roundedPulseCount(
          physical3 * MOTOR_DIRECTIONS.rearLeft),
      roundedPulseCount(
          physical4 * MOTOR_DIRECTIONS.rearRight));
}

/*
 * 用户坐标到库坐标的唯一入口。
 *
 * forwardMm > 0：向实车车头（3、4侧）前进；
 * rightMm   > 0：向实车右侧（1、3侧）横移；
 * ccwDeg    > 0：俯视逆时针旋转。
 *
 * 用户前进 = 库 forward 负值；用户向右 = 库 left 正值。
 */
MotorPulses vehicleDisplacementToMotorPulses(
    float forwardMm,
    float rightMm,
    float counterClockwiseDegrees) {
  return libraryDisplacementToMotorPulses(
      -forwardMm / 1000.0f,
      rightMm / 1000.0f,
       counterClockwiseDegrees * PI_F / 180.0f);
}

float absolutePulseCount(long pulses) {
  return static_cast<float>(
      pulses >= 0L ? pulses : -pulses);
}

/*
 * 用实际麦轮几何、旋转方向标定系数和电机安装符号计算单轮平均pulse/°。
 * 顺、逆时针分别计算，因而以后即使两个旋转标定系数不同，提前制动预测仍
 * 使用正确方向的换算值。100°只是为了减小整数脉冲取整造成的比例误差。
 */
float rotationPulsesPerDegree(int8_t directionSign) {
  const float calibrationDegrees =
      directionSign >= 0 ? 100.0f : -100.0f;
  const MotorPulses pulses =
      vehicleDisplacementToMotorPulses(
          0.0f, 0.0f, calibrationDegrees);
  const float averageAbsolutePulses =
      (absolutePulseCount(pulses.motor1) +
       absolutePulseCount(pulses.motor2) +
       absolutePulseCount(pulses.motor3) +
       absolutePulseCount(pulses.motor4)) /
      4.0f;
  return averageAbsolutePulses /
         fabsf(calibrationDegrees);
}

MotorPulses currentMotorPositions() {
  return MotorPulses(
      motor1.currentPosition(),
      motor2.currentPosition(),
      motor3.currentPosition(),
      motor4.currentPosition());
}

/*
 * 纯前进/后退的四轮脉冲和为0，纯左/右横移的四轮脉冲和也为0；只有
 * 旋转分量会让M1～M4同号。因此用四轮相对起点脉冲的平均值，可以从
 * 平移过程中分离出已经真正执行的旋转脉冲，再换算成实际纠偏角。
 */
float actualTranslationCorrectionDegrees() {
  const MotorPulses current =
      currentMotorPositions();
  const float averageRotationPulses =
      (static_cast<float>(
           current.motor1 -
           translationMotionOrigin.motor1) +
       static_cast<float>(
           current.motor2 -
           translationMotionOrigin.motor2) +
       static_cast<float>(
           current.motor3 -
           translationMotionOrigin.motor3) +
       static_cast<float>(
           current.motor4 -
           translationMotionOrigin.motor4)) /
      4.0f;
  if (averageRotationPulses == 0.0f) {
    return 0.0f;
  }

  const int8_t directionSign =
      averageRotationPulses > 0.0f ? +1 : -1;
  const float pulsesPerDegree =
      rotationPulsesPerDegree(directionSign);
  if (pulsesPerDegree <= 0.0f) {
    return 0.0f;
  }
  return averageRotationPulses / pulsesPerDegree;
}

/*
 * 去掉四轮平均旋转脉冲后，剩余部分就是纯平移已经完成的脉冲。该函数用于
 * 判断名义平移是否已经到位；到位后实时混合不再继续加入平移基础轮速，
 * 只允许剩余IMU旋转分量完成。
 */
float translationProgressFraction() {
  const MotorPulses current =
      currentMotorPositions();
  const float delta1 =
      static_cast<float>(
          current.motor1 -
          translationMotionOrigin.motor1);
  const float delta2 =
      static_cast<float>(
          current.motor2 -
          translationMotionOrigin.motor2);
  const float delta3 =
      static_cast<float>(
          current.motor3 -
          translationMotionOrigin.motor3);
  const float delta4 =
      static_cast<float>(
          current.motor4 -
          translationMotionOrigin.motor4);
  const float averageRotationPulses =
      (delta1 + delta2 + delta3 + delta4) /
      4.0f;

  const float base1 = static_cast<float>(
      translationRelativeMotorPulses.motor1);
  const float base2 = static_cast<float>(
      translationRelativeMotorPulses.motor2);
  const float base3 = static_cast<float>(
      translationRelativeMotorPulses.motor3);
  const float base4 = static_cast<float>(
      translationRelativeMotorPulses.motor4);
  const float denominator =
      base1 * base1 + base2 * base2 +
      base3 * base3 + base4 * base4;
  if (denominator <= 0.0f) {
    return 1.0f;
  }

  /*
   * 最小二乘投影：共同旋转脉冲被averageRotationPulses消去，剩余四轮
   * 位移只在原始平移向量上求进度。这样原地纠偏不会增加或减少距离进度。
   */
  return (
      base1 * (delta1 - averageRotationPulses) +
      base2 * (delta2 - averageRotationPulses) +
      base3 * (delta3 - averageRotationPulses) +
      base4 * (delta4 - averageRotationPulses)) /
      denominator;
}

float maximumTranslationComponentDistanceToGo() {
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  if (translationBaseForcedComplete ||
      translationBaseDisplacementComplete ||
      translationMaximumBasePulses <= 0.0f) {
    return 0.0f;
  }
  const float remainingFraction =
      1.0f - translationProgressFraction();
  return remainingFraction > 0.0f
             ? remainingFraction *
                   translationMaximumBasePulses
             : 0.0f;
#else
  /*
   * main_v0在平移期间没有旋转分量，位置模式的最大剩余脉冲就是硬位移
   * 剩余量。段后原地纠偏开始后，平移位移已经完成，调用者不再使用此值。
   */
  float maximumRemaining = 0.0f;
  for (uint8_t i = 0U; i < 4U; ++i) {
    maximumRemaining = fmaxf(
        maximumRemaining,
        absolutePulseCount(motors[i]->distanceToGo()));
  }
  return maximumRemaining;
#endif
}

void startRelativeMotorMove(const MotorPulses &pulses);
void applyTranslationWheelSpeedMix();
float headingErrorDegrees();
float moveTowardWithMaximumDelta(
    float current,
    float target,
    float maximumDelta);

/*
 * 启动一段纯平移，并按编译策略选择控制方法：
 *
 *   main：每轮速度=非负平移基础轮速+IMU共同旋转轮速；
 *   main_v0：执行不含旋转的四轮位置目标，段内不混入IMU。
 */
void startTranslationDisplacement(
    float forwardMm,
    float rightMm) {
  const MotorPulses translationPulses =
      vehicleDisplacementToMotorPulses(
          forwardMm, rightMm, 0.0f);
  const MotorPulses current =
      currentMotorPositions();

  translationRelativeMotorPulses =
      translationPulses;
  translationMotionOrigin = current;
  activeTranslationForwardMm = forwardMm;
  activeTranslationRightMm = rightMm;
  translationAccumulatedCorrectionDegrees = 0.0f;
  translationHeadingCorrectionRateDps = 0.0f;
  translationAppliedHeadingCorrectionRateDps = 0.0f;
  translationLastImuFrameSequence =
      imuValidFrameSequence;
  translationLastControlMs = millis();
  translationMaximumBasePulses = fmaxf(
      fmaxf(
          absolutePulseCount(translationPulses.motor1),
          absolutePulseCount(translationPulses.motor2)),
      fmaxf(
          absolutePulseCount(translationPulses.motor3),
          absolutePulseCount(translationPulses.motor4)));
  translationBaseStepRate = 0.0f;
  translationAppliedBaseStepRate = 0.0f;
  translationVelocityProfileLastUpdateUs = micros();
  translationBaseDisplacementComplete =
      translationMaximumBasePulses <=
      TRANSLATION_POSITION_TOLERANCE_STEPS;
  translationBaseForcedComplete = false;
  translationTerminalCorrectionStartValid = false;
  translationTerminalCorrectionStartMs = 0UL;
  postTranslationHeadingCorrectionActive = false;

  startRelativeMotorMove(translationPulses);
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  /*
   * move()只留下便于诊断的名义目标；从这一刻起本段只调用runSpeed()。
   * 速度从0按本段软加速度上升，方向完全由平移向量和IMU轮速之和决定。
   *
   * 这里必须把AccelStepper内部的maxSpeed设为本段真实上限，不能保留
   * setDriveMotionProfile()给位置模式S曲线设置的“起步速度上限”。原因是
   * runSpeed()虽然不使用AccelStepper自带的加速度规划，setSpeed()仍会被
   * 该库的maxSpeed钳位。若第7/15段继续保留5%的S形起步上限，则外层
   * updateTranslationBaseRate()即使已经算出更高的S形轮速，
   * 库内仍会把它永久截成vmax*0.05；实车串口表现就是第7段始终只有
   * 约676 pulse/s。
   *
   * main实时平移的S形/梯形包络、加减速斜率和终点制动全部由外层控制器
   * 逐轮计算，因而库内maxSpeed在此仅作为最终安全上限，不会取消第7、
   * 15段的S形规划。main_v0仍走位置模式，不进入本分支，保持原有做法。
   */
  translationHeadingControlActive = true;
  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->setMaxSpeed(activeDriveMaximumStepRate);
    motors[i]->setSpeed(0.0f);
  }
  applyTranslationWheelSpeedMix();
#else
  // main_v0只执行上面的纯平移位置目标，段内不启用IMU控制器。
  translationHeadingControlActive = false;
#endif
}

void setDriveMotionProfile(
    float maximumStepRate,
    float acceleration,
    DriveProfileShape profileShape =
        DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID) {
  /*
   * 梯形段的速度上限继续乘0.85；加速度先乘既有0.85，再乘本轮0.80，
   * 合计为调用方请求值的0.68。随后直接给出完整速度上限，由外层梯形
   * 控制器使用降低后的加速度起步。S形段不乘这些梯形专用系数，仍从
   * 最低速度比例开始并按距离抬高速度上限。
   *
   * 梯形减速度严格采用Desktop/route.cpp的加速度×2/3；第7、15段S形
   * 的实际峰值减速度同样采用加速度×2/3，但仍保留正弦S形包络。扫码
   * 立即停车与长按急停仍属于安全例外，不等待规划减速。
   */
  activeDriveProfileShape = profileShape;
  const float profileMaximumSpeedScale =
      profileShape ==
              DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID
          ? TRAPEZOID_MAXIMUM_SPEED_REDUCTION_SCALE
          : 1.0f;
  const float profileAccelerationScale =
      profileShape ==
              DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID
          ? TRAPEZOID_FINAL_ACCELERATION_SCALE
          : 1.0f;
  activeDriveMaximumStepRate =
      fmaxf(
          maximumStepRate * profileMaximumSpeedScale,
          1.0f);
  activeDriveAcceleration =
      fmaxf(
          acceleration * profileAccelerationScale,
          1.0f);
  activeDriveDeceleration =
      activeDriveAcceleration *
      (profileShape ==
               DRIVE_PROFILE_ASYMMETRIC_S_CURVE
           ? S_CURVE_DECELERATION_ACCELERATION_SCALE
           : TRAPEZOID_DECELERATION_ACCELERATION_SCALE);
  driveDecelerationActive = false;
  driveDecelerationStartStepRate = 0.0f;
  driveDecelerationStartRemainingSteps = 0.0f;
  driveProfileMotionOrigin =
      currentMotorPositions();
  driveAccelerationDistanceSteps =
      profileShape ==
              DRIVE_PROFILE_ASYMMETRIC_S_CURVE
          ? PI_F *
                activeDriveMaximumStepRate *
                activeDriveMaximumStepRate /
                (4.0f * activeDriveAcceleration)
          : 0.0f;
  driveProfileLastUpdateUs = micros();

  appliedDriveMaximumStepRate =
      profileShape ==
              DRIVE_PROFILE_ASYMMETRIC_S_CURVE
          ? fmaxf(
                activeDriveMaximumStepRate *
                    S_CURVE_MINIMUM_SPEED_SCALE,
                1.0f)
          : activeDriveMaximumStepRate;
  appliedDriveAcceleration =
      activeDriveAcceleration;

  SerialDebug.print(
      profileShape ==
              DRIVE_PROFILE_ASYMMETRIC_S_CURVE
          ? "[PROFILE S] vmax/acc/base-decel/S-decel-peak="
          : "[PROFILE T] vmax/acc/decel=");
  SerialDebug.print(activeDriveMaximumStepRate, 1);
  SerialDebug.print("/");
  SerialDebug.print(activeDriveAcceleration, 1);
  SerialDebug.print("/");
  SerialDebug.print(activeDriveDeceleration, 1);
  if (profileShape ==
      DRIVE_PROFILE_ASYMMETRIC_S_CURVE) {
    SerialDebug.print("/");
    SerialDebug.println(
        activeDriveDeceleration /
            S_CURVE_DECELERATION_GENTLENESS,
        1);
  } else {
    SerialDebug.println();
  }

  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->setMaxSpeed(
        appliedDriveMaximumStepRate);
    motors[i]->setAcceleration(
        appliedDriveAcceleration);
  }
}

void startRelativeMotorMove(const MotorPulses &pulses) {
  SerialDebug.print("[PULSES] M1/M2/M3/M4=");
  SerialDebug.print(pulses.motor1);
  SerialDebug.print("/");
  SerialDebug.print(pulses.motor2);
  SerialDebug.print("/");
  SerialDebug.print(pulses.motor3);
  SerialDebug.print("/");
  SerialDebug.println(pulses.motor4);

  motor1.move(pulses.motor1);
  motor2.move(pulses.motor2);
  motor3.move(pulses.motor3);
  motor4.move(pulses.motor4);
}

void startVehicleDisplacement(
    float forwardMm,
    float rightMm,
    float counterClockwiseDegrees) {
  startRelativeMotorMove(
      vehicleDisplacementToMotorPulses(
          forwardMm, rightMm, counterClockwiseDegrees));
}

bool allMotorsArrived() {
  return motor1.distanceToGo() == 0L &&
         motor2.distanceToGo() == 0L &&
         motor3.distanceToGo() == 0L &&
         motor4.distanceToGo() == 0L;
}

float maximumAbsoluteMotorStepRate() {
  float maximumStepRate = 0.0f;
  for (uint8_t i = 0U; i < 4U; ++i) {
    const float stepRate = fabsf(motors[i]->speed());
    if (stepRate > maximumStepRate) {
      maximumStepRate = stepRate;
    }
  }
  return maximumStepRate;
}

float maximumAbsoluteMotorDistanceToGo() {
  float maximumSteps = 0.0f;
  for (uint8_t i = 0U; i < 4U; ++i) {
    const float steps =
        absolutePulseCount(
            motors[i]->distanceToGo());
    if (steps > maximumSteps) {
      maximumSteps = steps;
    }
  }
  return maximumSteps;
}

float maximumMotorTravelFromProfileOrigin() {
  const MotorPulses current =
      currentMotorPositions();
  float maximumSteps =
      absolutePulseCount(
          current.motor1 -
          driveProfileMotionOrigin.motor1);
  maximumSteps =
      fmaxf(
          maximumSteps,
          absolutePulseCount(
              current.motor2 -
              driveProfileMotionOrigin.motor2));
  maximumSteps =
      fmaxf(
          maximumSteps,
          absolutePulseCount(
              current.motor3 -
              driveProfileMotionOrigin.motor3));
  maximumSteps =
      fmaxf(
          maximumSteps,
          absolutePulseCount(
              current.motor4 -
              driveProfileMotionOrigin.motor4));
  return maximumSteps;
}

float effectiveSCurvePeakDeceleration() {
  return activeDriveDeceleration /
         S_CURVE_DECELERATION_GENTLENESS;
}

/*
 * 正弦S形速度包络从速度v降到0所需距离：
 *
 *   distance = PI × v² / (4 × peakDeceleration)
 *
 * 与恒减速的v²/(2a)相比会更早开始制动，换取起止两端更平缓的速度斜率。
 */
float sCurveStoppingSteps(float stepRate) {
  const float peakDeceleration =
      effectiveSCurvePeakDeceleration();
  if (peakDeceleration <= 0.0f) {
    return 0.0f;
  }
  return PI_F * stepRate * stepRate /
         (4.0f * peakDeceleration);
}

float trapezoidStoppingSteps(float stepRate) {
  if (activeDriveDeceleration <= 0.0f) {
    return 0.0f;
  }
  return stepRate * stepRate /
         (2.0f * activeDriveDeceleration);
}

float activeProfileStoppingSteps(float stepRate) {
  return activeDriveProfileShape ==
                 DRIVE_PROFILE_ASYMMETRIC_S_CURVE
             ? sCurveStoppingSteps(stepRate)
             : trapezoidStoppingSteps(stepRate);
}

float signedBaseStepRate(
    long translationPulses,
    float maximumTranslationPulses,
    float baseMagnitude) {
  if (maximumTranslationPulses <= 0.0f) {
    return 0.0f;
  }
  return static_cast<float>(translationPulses) /
         maximumTranslationPulses *
         baseMagnitude;
}

void setMotorMaximumStepRate(
    AccelStepper &motor,
    float requestedMagnitude) {
  const float limitedMagnitude =
      fmaxf(requestedMagnitude, 1.0f);
  if (fabsf(
          motor.maxSpeed() -
          limitedMagnitude) >=
      0.5f) {
    motor.setMaxSpeed(limitedMagnitude);
  }
}

void setMotorAcceleration(
    AccelStepper &motor,
    float requestedAcceleration) {
  const float limitedAcceleration =
      fmaxf(requestedAcceleration, 1.0f);
  if (fabsf(
          motor.acceleration() -
          limitedAcceleration) >=
      0.5f) {
    motor.setAcceleration(limitedAcceleration);
  }
}

/*
 * 根据剩余硬位移生成本轮“希望使用”的平移基础步频。
 *
 * 梯形段使用sqrt(2*a*r)制动上限；第7、15段使用与既有正弦S形相匹配的
 * sqrt(4*a*r/PI)制动上限。加速度和减速度只决定基础步频每轮能变化多少，
 * 是软约束；IMU纠偏不参与本函数，因此不会进一步压低基础平移速度。
 */
void updateTranslationBaseRate() {
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  if (!translationHeadingControlActive) {
    return;
  }

  const uint32_t nowUs = micros();
  float dtSeconds = static_cast<float>(
      nowUs - translationVelocityProfileLastUpdateUs) /
      1000000.0f;
  translationVelocityProfileLastUpdateUs = nowUs;
  if (dtSeconds <= 0.0f) {
    return;
  }
  if (dtSeconds >
      TRANSLATION_HEADING_MAXIMUM_CONTROL_DT_SECONDS) {
    dtSeconds =
        TRANSLATION_HEADING_MAXIMUM_CONTROL_DT_SECONDS;
  }

  float remainingSteps =
      maximumTranslationComponentDistanceToGo();
  if (translationBaseForcedComplete ||
      remainingSteps <=
          TRANSLATION_POSITION_TOLERANCE_STEPS) {
    translationBaseDisplacementComplete = true;
    translationBaseStepRate = 0.0f;
    translationAppliedBaseStepRate = 0.0f;
    driveDecelerationActive = true;
    return;
  }

  float brakingLimitedStepRate =
      activeDriveProfileShape ==
              DRIVE_PROFILE_ASYMMETRIC_S_CURVE
          ? sqrtf(
                4.0f *
                effectiveSCurvePeakDeceleration() *
                remainingSteps / PI_F)
          : sqrtf(
                2.0f * activeDriveDeceleration *
                remainingSteps);
  brakingLimitedStepRate = fmaxf(
      brakingLimitedStepRate,
      TRANSLATION_TERMINAL_MINIMUM_STEP_RATE);

  float targetBaseStepRate = fminf(
      activeDriveMaximumStepRate,
      brakingLimitedStepRate);

  if (activeDriveProfileShape ==
      DRIVE_PROFILE_ASYMMETRIC_S_CURVE) {
    const float traveledSteps = fmaxf(
        translationMaximumBasePulses - remainingSteps,
        0.0f);
    float accelerationFraction =
        driveAccelerationDistanceSteps > 0.0f
            ? traveledSteps /
                  driveAccelerationDistanceSteps
            : 1.0f;
    if (accelerationFraction > 1.0f) {
      accelerationFraction = 1.0f;
    }
    const float minimumSCurveStepRate = fmaxf(
        activeDriveMaximumStepRate *
            S_CURVE_MINIMUM_SPEED_SCALE,
        TRANSLATION_TERMINAL_MINIMUM_STEP_RATE);
    const float accelerationLimitedStepRate =
        minimumSCurveStepRate +
        (activeDriveMaximumStepRate -
         minimumSCurveStepRate) *
            sinf(0.5f * PI_F *
                 accelerationFraction);
    targetBaseStepRate = fminf(
        targetBaseStepRate,
        accelerationLimitedStepRate);
  }

  const bool reducingSpeed =
      targetBaseStepRate < translationBaseStepRate;
  const float rateChangeLimit =
      (reducingSpeed
           ? activeDriveDeceleration
           : activeDriveAcceleration) *
      dtSeconds;
  translationBaseStepRate =
      moveTowardWithMaximumDelta(
          translationBaseStepRate,
          targetBaseStepRate,
          rateChangeLimit);
  if (translationBaseStepRate < 0.0f) {
    translationBaseStepRate = 0.0f;
  }
  driveDecelerationActive =
      brakingLimitedStepRate <
      activeDriveMaximumStepRate;
#endif
}

/*
 * main的平移优先轮速分配：
 *
 *   wheelRate[i] = baseRatio[i] * baseRate + correctionRate
 *
 * 先完整保留速度规划给出的baseRate，再逐轮求在不超过本段速度上限时
 * correctionRate允许落入的交集区间，最后只钳制纠偏轮速。这样IMU误差
 * 无论多大都不能让行程中的平移减速或停车；接近最高平移速度时纠偏可能
 * 暂时变小甚至为0，但减速段会逐渐释放轮速余量。位移到位后baseRate=0，
 * 纠偏可以使用完整余量完成终点航向硬约束。
 */
void applyTranslationWheelSpeedMix() {
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  if (!translationHeadingControlActive) {
    return;
  }

  if (translationBaseForcedComplete ||
      maximumTranslationComponentDistanceToGo() <=
          TRANSLATION_POSITION_TOLERANCE_STEPS) {
    translationBaseDisplacementComplete = true;
    translationBaseStepRate = 0.0f;
  }

  float correctionPulsesPerSecond =
      translationHeadingCorrectionRateDps == 0.0f
          ? 0.0f
          : translationHeadingCorrectionRateDps *
                rotationPulsesPerDegree(
                    translationHeadingCorrectionRateDps >
                            0.0f
                        ? +1
                        : -1);
  float requestedBaseStepRate =
      translationBaseDisplacementComplete
          ? 0.0f
          : translationBaseStepRate;
  requestedBaseStepRate = fminf(
      fmaxf(requestedBaseStepRate, 0.0f),
      activeDriveMaximumStepRate);
  const long basePulses[4] = {
      translationRelativeMotorPulses.motor1,
      translationRelativeMotorPulses.motor2,
      translationRelativeMotorPulses.motor3,
      translationRelativeMotorPulses.motor4};

  translationAppliedBaseStepRate =
      requestedBaseStepRate;

  /*
   * 对每个轮子都有：
   *   -vmax <= baseWheelRate + correctionRate <= vmax
   * 因此该轮允许的纠偏区间是
   *   [-vmax-baseWheelRate, +vmax-baseWheelRate]。
   * 四轮区间取交集后再钳制纠偏，即可数学上保证基础平移轮速完全不变。
   */
  float minimumAllowedCorrectionStepRate =
      -activeDriveMaximumStepRate;
  float maximumAllowedCorrectionStepRate =
      activeDriveMaximumStepRate;
  for (uint8_t i = 0U; i < 4U; ++i) {
    const float baseWheelRate =
        signedBaseStepRate(
            basePulses[i],
            translationMaximumBasePulses,
            translationAppliedBaseStepRate);
    minimumAllowedCorrectionStepRate = fmaxf(
        minimumAllowedCorrectionStepRate,
        -activeDriveMaximumStepRate - baseWheelRate);
    maximumAllowedCorrectionStepRate = fminf(
        maximumAllowedCorrectionStepRate,
        activeDriveMaximumStepRate - baseWheelRate);
  }
  correctionPulsesPerSecond = fminf(
      fmaxf(
          correctionPulsesPerSecond,
          minimumAllowedCorrectionStepRate),
      maximumAllowedCorrectionStepRate);

  if (correctionPulsesPerSecond == 0.0f) {
    translationAppliedHeadingCorrectionRateDps = 0.0f;
  } else {
    const float pulsesPerDegree = rotationPulsesPerDegree(
        correctionPulsesPerSecond > 0.0f ? +1 : -1);
    translationAppliedHeadingCorrectionRateDps =
        pulsesPerDegree > 0.0f
            ? correctionPulsesPerSecond / pulsesPerDegree
            : 0.0f;
  }

  for (uint8_t i = 0U; i < 4U; ++i) {
    const float mixedRate =
        signedBaseStepRate(
            basePulses[i],
            translationMaximumBasePulses,
            translationAppliedBaseStepRate) +
        correctionPulsesPerSecond;
    motors[i]->setSpeed(mixedRate);
  }
#endif
}

void applyDriveMaximumStepRate(float maximumStepRate) {
  const float limitedStepRate =
      fmaxf(maximumStepRate, 1.0f);
  if (fabsf(
          limitedStepRate -
          appliedDriveMaximumStepRate) <
      0.5f) {
    return;
  }

  appliedDriveMaximumStepRate =
      limitedStepRate;
  if (translationHeadingControlActive) {
    applyTranslationWheelSpeedMix();
  } else {
    for (uint8_t i = 0U; i < 4U; ++i) {
      setMotorMaximumStepRate(
          *motors[i],
          appliedDriveMaximumStepRate);
    }
  }
}

/*
 * 进入非对称S形减速。此函数只冻结制动开始时的速度和剩余脉冲，不修改
 * 普通平移的目标位置；
 * 明确旋转会在beginIntegratedTurnBraking()中另外按预计S形制动距离更新
 * 四轮停车目标。
 */
void beginDriveSCurveDeceleration() {
  driveDecelerationActive = true;
  driveProfileLastUpdateUs = micros();
  driveDecelerationStartStepRate =
      maximumAbsoluteMotorStepRate();
  driveDecelerationStartRemainingSteps =
      maximumAbsoluteMotorDistanceToGo();

  const float peakDeceleration =
      effectiveSCurvePeakDeceleration();
  appliedDriveAcceleration =
      fmaxf(
          peakDeceleration,
          activeDriveDeceleration *
              S_CURVE_MINIMUM_ACCELERATION_SCALE);
  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->setAcceleration(
        appliedDriveAcceleration);
  }
  if (translationHeadingControlActive) {
    applyTranslationWheelSpeedMix();
  }
}

/*
 * route.cpp参考的非对称梯形制动：位置目标保持不变，只把四轮加速度参数
 * 同时切换为加速值的2/3。AccelStepper随后按剩余位置自动线性降速。
 */
void beginDriveTrapezoidDeceleration() {
  driveDecelerationActive = true;
  appliedDriveAcceleration =
      activeDriveDeceleration;
  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->setAcceleration(
        appliedDriveAcceleration);
  }
  if (translationHeadingControlActive) {
    applyTranslationWheelSpeedMix();
  }
}

void beginDriveProfileDeceleration() {
  if (activeDriveProfileShape ==
      DRIVE_PROFILE_ASYMMETRIC_S_CURVE) {
    beginDriveSCurveDeceleration();
  } else {
    beginDriveTrapezoidDeceleration();
  }
}

void serviceDriveSCurveSpeedEnvelope() {
  if (activeDriveProfileShape !=
      DRIVE_PROFILE_ASYMMETRIC_S_CURVE) {
    return;
  }

  const uint32_t nowUs = micros();
  if (nowUs - driveProfileLastUpdateUs <
      S_CURVE_UPDATE_INTERVAL_US) {
    return;
  }
  driveProfileLastUpdateUs = nowUs;

  if (!driveDecelerationActive) {
    if (driveAccelerationDistanceSteps <= 0.0f) {
      applyDriveMaximumStepRate(
          activeDriveMaximumStepRate);
      return;
    }

    float normalizedDistance =
        maximumMotorTravelFromProfileOrigin() /
        driveAccelerationDistanceSteps;
    if (normalizedDistance >= 1.0f) {
      applyDriveMaximumStepRate(
          activeDriveMaximumStepRate);
      return;
    }

    const float sCurveFraction =
        sinf(
            0.5f * PI_F *
            normalizedDistance);
    const float minimumStepRate =
        fmaxf(
            activeDriveMaximumStepRate *
                S_CURVE_MINIMUM_SPEED_SCALE,
            1.0f);
    applyDriveMaximumStepRate(
        minimumStepRate +
        (activeDriveMaximumStepRate -
         minimumStepRate) *
            sCurveFraction);
    return;
  }

  if (driveDecelerationStartRemainingSteps <= 0.0f) {
    return;
  }

  float normalizedRemainingDistance =
      maximumAbsoluteMotorDistanceToGo() /
      driveDecelerationStartRemainingSteps;
  if (normalizedRemainingDistance > 1.0f) {
    normalizedRemainingDistance = 1.0f;
  }

  const float sCurveFraction =
      sinf(
          0.5f * PI_F *
          normalizedRemainingDistance);
  const float minimumStepRate =
      fmaxf(
          activeDriveMaximumStepRate *
              S_CURVE_MINIMUM_SPEED_SCALE,
          1.0f);
  const float requestedStepRate =
      minimumStepRate +
      (driveDecelerationStartStepRate -
       minimumStepRate) *
          sCurveFraction;
  /*
   * 此函数只服务位置模式的main_v0平移或明确旋转。减速速度上限只允许
   * 继续下降，禁止因取整造成的剩余量波动而重新抬速，避免二次冲击。
   */
  applyDriveMaximumStepRate(
      fminf(
          requestedStepRate,
          appliedDriveMaximumStepRate));
}

void serviceDriveMotionProfile() {
  serviceDriveSCurveSpeedEnvelope();

  /*
   * 明确旋转由serviceIntegratedTurnCommand()根据实时IMU剩余角决定制动点，
   * 不能再让普通的“剩余脉冲”判据抢先切换并干扰它。
   */
  if (integratedTurnControlActive ||
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

    const float currentSpeed =
        fabsf(motors[i]->speed());
    if (remainingSteps == 0L || currentSpeed <= 0.0f) {
      continue;
    }

    /*
     * 第7/15段使用S形制动距离，其余段使用route.cpp的v²/(2a)梯形制动
     * 距离。任一轮进入制动区就让四轮同轮切换，避免麦轮产生额外偏航。
     */
    const float stoppingSteps =
        activeProfileStoppingSteps(currentSpeed);
    if (static_cast<float>(remainingSteps) <=
        stoppingSteps +
            DECELERATION_SWITCH_MARGIN_STEPS) {
      shouldStartDeceleration = true;
      break;
    }
  }

  if (!shouldStartDeceleration) {
    return;
  }

  beginDriveProfileDeceleration();
}

void runAllMotors() {
  /*
   * 必须先判断是否进入制动区，再执行本轮step，才能按较小减速度提前刹车。
   * 此函数不输出串口，避免在高速运动中因打印阻塞导致脉冲间隔抖动。
   */
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  if (translationHeadingControlActive) {
    /*
     * 平移段只运行速度模式。每一轮step前重新检查硬位移投影，到位后立即
     * 去掉基础平移分量；随后纯旋转纠偏不会改变该投影。四个runSpeed()
     * 必须全部调用，不能用会短路的逻辑表达式连接。
     */
    applyTranslationWheelSpeedMix();
    motor1.runSpeed();
    motor2.runSpeed();
    motor3.runSpeed();
    motor4.runSpeed();
    return;
  }
#endif

  serviceDriveMotionProfile();

  // 明确旋转及main_v0平移使用位置模式。
  motor1.run();
  motor2.run();
  motor3.run();
  motor4.run();
}

/*
 * 扫码成功时要求“立刻退出慢行段”，所以不能等待 AccelStepper 按原目标
 * 继续减速。把每个电机当前位置设为新目标，会立即清除剩余距离。
 * 因为扫码段速度低，该立即停止比在高速段使用同样方式更温和。
 */
void stopAllMotorsImmediately() {
  for (uint8_t i = 0U; i < 4U; ++i) {
    const long position = motors[i]->currentPosition();
    motors[i]->setCurrentPosition(position);
  }
  driveDecelerationActive = false;
  driveAccelerationDistanceSteps = 0.0f;
  driveDecelerationStartStepRate = 0.0f;
  driveDecelerationStartRemainingSteps = 0.0f;
  translationHeadingControlActive = false;
  activeTranslationForwardMm = 0.0f;
  activeTranslationRightMm = 0.0f;
  translationAccumulatedCorrectionDegrees = 0.0f;
  translationHeadingCorrectionRateDps = 0.0f;
  translationAppliedHeadingCorrectionRateDps = 0.0f;
  translationMaximumBasePulses = 0.0f;
  translationBaseStepRate = 0.0f;
  translationAppliedBaseStepRate = 0.0f;
  translationBaseDisplacementComplete = false;
  translationBaseForcedComplete = false;
  translationTerminalCorrectionStartValid = false;
  translationTerminalCorrectionStartMs = 0UL;
  postTranslationHeadingCorrectionActive = false;
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

/*
 * 从四轮实际已经输出的脉冲反算“沿用户前进方向”走了多少毫米。
 * 用户前进时原始脉冲符号是 +、-、+、-，因此用：
 *
 *   forwardPulse = (M1 - M2 + M3 - M4) / 4
 *
 * 再除以 10000 pulse/m。此值只在纯纵向的扫码慢行段中使用。
 */
float forwardTravelFromOriginMm(const MotorPulses &origin) {
  const MotorPulses now = currentMotorPositions();
  const float projectedPulses =
      (static_cast<float>(now.motor1 - origin.motor1) -
       static_cast<float>(now.motor2 - origin.motor2) +
       static_cast<float>(now.motor3 - origin.motor3) -
       static_cast<float>(now.motor4 - origin.motor4)) /
      4.0f;
  return projectedPulses /
         FORWARD_PULSES_PER_METER * 1000.0f;
}

/*
 * 启停区2用-1反转步骤1、2、3、21的纵向分量；第1段附加的向右
 * 150 mm不使用这个符号，其他路线段也不受此符号影响。
 */
float selectedStartZoneDirection() {
  return START_ZONE_SELECTION == 1U ? 1.0f : -1.0f;
}

// ===========================================================================
// 10. 21 步路线表
// ===========================================================================

enum CommandType {
  COMMAND_ZONE_LONGITUDINAL_FAST,
  COMMAND_SCAN_SLOW,
  COMMAND_ADJUST_TO_POINT_A,
  COMMAND_FORWARD_FAST,
  COMMAND_BACKWARD_FAST,
  COMMAND_RIGHT_FAST,
  COMMAND_LEFT_FAST,
  COMMAND_TURN_COUNTERCLOCKWISE,
  COMMAND_TURN_CLOCKWISE
};

struct RouteCommand {
  uint8_t specificationStep;
  CommandType type;
  int32_t value;
  float motionScale;
  const char *name;
};

/*
 * 表中严格保留用户的 1～21 步。
 *
 * 步骤15～19虽然是“重复7～11”，仍显式列出，避免运行时跳转造成难以核对。
 * 表中的FORWARD/BACKWARD/RIGHT/LEFT都是相对当前车体方向的麦轮平移。
 * 只有步骤5、8、10、13、16、18会改变路线目标航向。
 *
 * 第4列motionScale逐段引用程序前部的STEP_xx_MOTION_SCALE。
 * 不要在本路线表中直接写0.95等数字；以后逐段调试只修改前部对应常量，
 * 这样动作顺序、距离算式与标定系数不会混在一起。
 */
const RouteCommand route[] = {
    {1U, COMMAND_ZONE_LONGITUDINAL_FAST, DISTANCE_A_MM,
     STEP_01_MOTION_SCALE,
     "Start zone -> scan start, diagonal a/lateral150"},
    {2U, COMMAND_SCAN_SLOW, MAXIMUM_SCAN_DISTANCE_B_MM,
     STEP_02_MOTION_SCALE,
     "Slow scan until QR success, b <= 200"},
    {3U, COMMAND_ADJUST_TO_POINT_A, SCAN_START_TO_POINT_A_MM,
     STEP_03_MOTION_SCALE,
     "Adjust by 92-b to QR midpoint A"},
    {4U, COMMAND_RIGHT_FAST, STEP_04_RIGHT_DISTANCE_MM,
     STEP_04_MOTION_SCALE,
     "A -> center B, right c-150"},
    {5U, COMMAND_TURN_COUNTERCLOCKWISE, 90,
     STEP_05_MOTION_SCALE,
     "At B turn counter-clockwise 90 (former step 6)"},
    {6U, COMMAND_LEFT_FAST, DISTANCE_D_MM,
     STEP_06_MOTION_SCALE,
     "B -> raw area C, left d after turn"},
    {7U, COMMAND_RIGHT_FAST, DISTANCE_E_MM,
     STEP_07_MOTION_SCALE,
     "C -> rough process D, right e"},
    {8U, COMMAND_TURN_CLOCKWISE, 180,
     STEP_08_MOTION_SCALE,
     "At D turn clockwise 180"},
    {9U, COMMAND_FORWARD_FAST, DISTANCE_F_MM,
     STEP_09_MOTION_SCALE,
     "D -> lower-left E, forward f"},
    {10U, COMMAND_TURN_CLOCKWISE, 90,
     STEP_10_MOTION_SCALE,
     "At E turn clockwise 90"},
    {11U, COMMAND_FORWARD_FAST, DISTANCE_G_MM,
     STEP_11_MOTION_SCALE,
     "E -> temporary storage F, forward g"},
    {12U, COMMAND_FORWARD_FAST, DISTANCE_D_MM,
     STEP_12_MOTION_SCALE,
     "F -> upper-left G, forward d"},
    {13U, COMMAND_TURN_CLOCKWISE, 90,
     STEP_13_MOTION_SCALE,
     "At G turn clockwise 90"},
    {14U, COMMAND_FORWARD_FAST, DISTANCE_F_MM,
     STEP_14_MOTION_SCALE,
     "G -> raw area C, forward f"},
    {15U, COMMAND_RIGHT_FAST, DISTANCE_E_MM,
     STEP_15_MOTION_SCALE,
     "Repeat step 7: C -> D, right e"},
    {16U, COMMAND_TURN_CLOCKWISE, 180,
     STEP_16_MOTION_SCALE,
     "Repeat step 8: at D clockwise 180"},
    {17U, COMMAND_FORWARD_FAST, DISTANCE_F_MM,
     STEP_17_MOTION_SCALE,
     "Repeat step 9: D -> E, forward f"},
    {18U, COMMAND_TURN_CLOCKWISE, 90,
     STEP_18_MOTION_SCALE,
     "Repeat step 10: at E clockwise 90"},
    {19U, COMMAND_FORWARD_FAST, DISTANCE_G_MM,
     STEP_19_MOTION_SCALE,
     "Repeat step 11: E -> F, forward g"},
    {20U, COMMAND_RIGHT_FAST, STORAGE_F_TO_SCAN_A_MM,
     STEP_20_MOTION_SCALE,
     "F -> QR midpoint A, right f+c"},
    {21U, COMMAND_ZONE_LONGITUDINAL_FAST,
     POINT_A_TO_START_ZONE_MM, STEP_21_MOTION_SCALE,
     "A -> selected start zone"}};

constexpr size_t ROUTE_COMMAND_COUNT =
    sizeof(route) / sizeof(route[0]);
static_assert(
    ROUTE_COMMAND_COUNT == 21U,
    "The route must contain exactly the 21 requested steps");

bool commandIsTurn(CommandType type) {
  return type == COMMAND_TURN_COUNTERCLOCKWISE ||
         type == COMMAND_TURN_CLOCKWISE;
}

const char *commandTypeName(CommandType type) {
  switch (type) {
    case COMMAND_ZONE_LONGITUDINAL_FAST:
      return "ZONE_LONGITUDINAL_FAST";
    case COMMAND_SCAN_SLOW:
      return "SCAN_SLOW";
    case COMMAND_ADJUST_TO_POINT_A:
      return "ADJUST_TO_A";
    case COMMAND_FORWARD_FAST:
      return "FORWARD_FAST";
    case COMMAND_BACKWARD_FAST:
      return "BACKWARD_FAST";
    case COMMAND_RIGHT_FAST:
      return "RIGHT_FAST";
    case COMMAND_LEFT_FAST:
      return "LEFT_FAST";
    case COMMAND_TURN_COUNTERCLOCKWISE:
      return "TURN_CCW";
    case COMMAND_TURN_CLOCKWISE:
      return "TURN_CW";
  }
  return "UNKNOWN";
}

// ===========================================================================
// 11. 路线状态、行程内IMU控制和扫码 b 状态
// ===========================================================================

enum ProgramState {
  PROGRAM_WAITING,
  PROGRAM_RUNNING,
  PROGRAM_FINISHED,
  PROGRAM_FAULT
};

enum RoutePhase {
  ROUTE_PHASE_IDLE,
  ROUTE_PHASE_MOTION,
  ROUTE_PHASE_WAIT_SCAN_CODE
};

ProgramState programState = PROGRAM_WAITING;
RoutePhase routePhase = ROUTE_PHASE_IDLE;

size_t routeIndex = 0U;
bool commandStarted = false;
volatile bool startRequested = false;
volatile bool abortRequested = false;

uint32_t routeRunStartMs = 0UL;
uint32_t commandStartMs = 0UL;
uint32_t lastMotionTelemetryMs = 0UL;

float routeImuReferenceDegrees = 0.0f;
float targetCounterClockwiseHeadingDegrees = 0.0f;

/*
 * 第7、15段都从原料区C向粗加工区D横移，二者必须使用完全相同的车头
 * 朝向。第7段开始时保存其累计IMU目标；第14段第二次回到C并完全停车后，
 * 再把目标切换到与该参考相差整数圈的最近等效角，完成纠偏后才进入15段。
 *
 * 不把“-0.1°”写死：若实车当次差值正好为0.1°，就只修正0.1°；若IMU
 * 漂移或前面旋转系数改变，仍会自动修正实际差值并保证两个横移目标同向。
 */
float step7RawAreaHeadingReferenceDegrees = 0.0f;
bool step7RawAreaHeadingReferenceValid = false;
bool step15HeadingMatchAppliedAtSecondRawArea = false;

/*
 * 扫码段 b 的状态：
 * scanMotionOrigin 保存步骤2开始时四轮脉冲；
 * scanDistanceBmm 保存“沿本起点规定方向”慢行的实际软件脉冲距离。
 *
 * 注意：这是已输出步数换算的开环距离，不是外部定位或轮端编码器实测距离；
 * 麦轮打滑不会自动反映到 b 中。
 */
MotorPulses scanMotionOrigin;
float scanDistanceBmm = 0.0f;
float scanCommandedMaximumDistanceMm = 0.0f;
bool scanOriginValid = false;
bool scanDistanceCaptured = false;

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

float currentRouteHeadingDegrees() {
  return imuCounterClockwiseDegrees -
         routeImuReferenceDegrees;
}

float headingErrorDegrees() {
  return targetCounterClockwiseHeadingDegrees -
         currentRouteHeadingDegrees();
}

/*
 * 连续IMU角度会超过±360°，而同一车头朝向每隔360°重复一次。
 * 将第7段保存的目标加减整数圈，选择离第二次到达C时的实际角度最近者，
 * 可以只做最短的小角度修正，而不会让底盘为了“90°等于-270°”多转一圈。
 */
float nearestEquivalentHeadingDegrees(
    float referenceDegrees,
    float nearDegrees) {
  const float wholeTurns =
      roundf(
          (nearDegrees - referenceDegrees) /
          360.0f);
  return referenceDegrees +
         wholeTurns * 360.0f;
}

/*
 * 普通平移沿用0.15°死区；只有第14段已经停车并切换到第15段参考目标后，
 * 临时收紧到0.05°。这不会让其他21段更容易在终点发生反复微调。
 */
float activeTranslationHeadingToleranceDegrees() {
  if (routeIndex < ROUTE_COMMAND_COUNT &&
      route[routeIndex].specificationStep == 14U &&
      step15HeadingMatchAppliedAtSecondRawArea) {
    return STEP_15_PRE_ALIGNMENT_TOLERANCE_DEGREES;
  }
  return TRANSLATION_HEADING_DEADBAND_DEGREES;
}

/*
 * 运行中每500 ms回传一行紧凑遥测：
 *
 * @step,elapsed,maxRemain,maxSpeed,speedCap,heading,error,cmdCorr,doneCorr,decel,imuAge
 *
 * 单位依次为：
 *   路线段号、ms、pulse、pulse/s、pulse/s、0.01°、0.01°、0.01°、
 *   0.01°、0/1、ms。
 *
 * 例如：
 *   @15,4200,9630,8700,13500,-26895,-15,42,38,0,8
 *
 * 表示第15段运行4.2 s，最大剩余9630 pulse，当前最大轮速8700 pulse/s，
 * 当前速度上限13500 pulse/s，航向-268.95°，误差-0.15°，本段命令
 * 纠偏0.42°、已经实际执行0.38°，尚未进入减速，最新IMU帧距今8 ms。
 *
 * 这里不使用连续Serial.print()：只有HardwareSerial的TX缓冲区能一次
 * 放下整行才write()，否则直接丢弃本次遥测，绝不等待串口释放空间。
 */
void serviceMotionTelemetry() {
  if (programState != PROGRAM_RUNNING ||
      !commandStarted ||
      routeIndex >= ROUTE_COMMAND_COUNT) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastMotionTelemetryMs <
      MOTION_TELEMETRY_INTERVAL_MS) {
    return;
  }
  lastMotionTelemetryMs = nowMs;

  char line[64];
  float telemetryRemainingSteps =
      maximumAbsoluteMotorDistanceToGo();
  float telemetrySpeedCap =
      appliedDriveMaximumStepRate;
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  if (translationHeadingControlActive) {
    telemetryRemainingSteps =
        maximumTranslationComponentDistanceToGo();
    telemetrySpeedCap =
        activeDriveMaximumStepRate;
  }
#endif
  const long maximumRemainingSteps =
      roundedPulseCount(telemetryRemainingSteps);
  const long maximumSpeed =
      roundedPulseCount(
          maximumAbsoluteMotorStepRate());
  const long speedCap =
      roundedPulseCount(telemetrySpeedCap);
  const long headingTimes100 =
      lroundf(currentRouteHeadingDegrees() * 100.0f);
  const long errorTimes100 =
      lroundf(headingErrorDegrees() * 100.0f);
  const long correctionTimes100 =
      lroundf(
          translationAccumulatedCorrectionDegrees *
          100.0f);
  const long executedCorrectionTimes100 =
      translationHeadingControlActive
          ? lroundf(
                actualTranslationCorrectionDegrees() *
                100.0f)
          : 0L;
  const uint32_t imuAgeMs =
      imuInitialized
          ? nowMs - lastImuReceiveMs
          : UINT32_MAX;

  const int length = snprintf(
      line,
      sizeof(line),
      "@%u,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%lu\n",
      static_cast<unsigned int>(
          route[routeIndex].specificationStep),
      static_cast<unsigned long>(
          nowMs - commandStartMs),
      maximumRemainingSteps,
      maximumSpeed,
      speedCap,
      headingTimes100,
      errorTimes100,
      correctionTimes100,
      executedCorrectionTimes100,
      driveDecelerationActive ? 1U : 0U,
      static_cast<unsigned long>(imuAgeMs));

  if (length <= 0 ||
      static_cast<size_t>(length) >= sizeof(line)) {
    return;
  }

  if (SerialDebug.availableForWrite() < length) {
    return;
  }

  SerialDebug.write(
      reinterpret_cast<const uint8_t *>(line),
      static_cast<size_t>(length));
}

void routeFault(const char *reason) {
  /*
   * disableDriveMotors()会把目标位置改成当前位置并清除控制状态。必须先
   * 把故障瞬间的数据复制到局部变量，再立即失能，最后才做串口打印；
   * 这样既不会为了日志延迟停车，也不会丢失“为什么在第15段停机”的证据。
   */
  const ProgramState stateAtFault = programState;
  const uint8_t stepAtFault =
      routeIndex < ROUTE_COMMAND_COUNT
          ? route[routeIndex].specificationStep
          : 0U;
  const uint32_t nowMs = millis();
  const uint32_t elapsedMs =
      nowMs - commandStartMs;
  const uint32_t imuAgeMs =
      imuInitialized
          ? nowMs - lastImuReceiveMs
          : UINT32_MAX;
  const float targetHeading =
      targetCounterClockwiseHeadingDegrees;
  const float actualHeading =
      currentRouteHeadingDegrees();
  const float headingError =
      targetHeading - actualHeading;
  const float accumulatedCorrection =
      translationAccumulatedCorrectionDegrees;
  const float executedCorrection =
      translationHeadingControlActive
          ? actualTranslationCorrectionDegrees()
          : 0.0f;
  const float correctionLead =
      accumulatedCorrection -
      executedCorrection;
  const float maximumStepRate =
      maximumAbsoluteMotorStepRate();
  float speedCap =
      appliedDriveMaximumStepRate;
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  if (translationHeadingControlActive) {
    speedCap = activeDriveMaximumStepRate;
  }
#endif
  const bool decelerationWasActive =
      driveDecelerationActive;
  const float commandedForwardMm =
      activeTranslationForwardMm;
  const float commandedRightMm =
      activeTranslationRightMm;
  const MotorPulses positions =
      currentMotorPositions();
  const MotorPulses targets(
      motor1.targetPosition(),
      motor2.targetPosition(),
      motor3.targetPosition(),
      motor4.targetPosition());
  const MotorPulses remaining(
      motor1.distanceToGo(),
      motor2.distanceToGo(),
      motor3.distanceToGo(),
      motor4.distanceToGo());

  disableDriveMotors();
  programState = PROGRAM_FAULT;
  routePhase = ROUTE_PHASE_IDLE;
  commandStarted = false;
  startRequested = false;
  abortRequested = false;
  hmiSetRunStatus("FAULT");

  SerialDebug.print("[FAULT] state/step/reason=");
  SerialDebug.print(programStateName(stateAtFault));
  SerialDebug.print("/");
  SerialDebug.print(stepAtFault);
  SerialDebug.print("/");
  SerialDebug.println(reason);
  SerialDebug.print(
      "[FAULT SNAP] elapsed/imu-age/max-speed/speed-cap/decel=");
  SerialDebug.print(elapsedMs);
  SerialDebug.print("/");
  SerialDebug.print(imuAgeMs);
  SerialDebug.print("/");
  SerialDebug.print(maximumStepRate, 1);
  SerialDebug.print("/");
  SerialDebug.print(speedCap, 1);
  SerialDebug.print("/");
  SerialDebug.println(decelerationWasActive ? 1 : 0);
  SerialDebug.print(
      "[FAULT SNAP] heading target/actual/error/cmd-correction/executed/lead=");
  SerialDebug.print(targetHeading, 2);
  SerialDebug.print("/");
  SerialDebug.print(actualHeading, 2);
  SerialDebug.print("/");
  SerialDebug.print(headingError, 2);
  SerialDebug.print("/");
  SerialDebug.print(accumulatedCorrection, 2);
  SerialDebug.print("/");
  SerialDebug.print(executedCorrection, 2);
  SerialDebug.print("/");
  SerialDebug.println(correctionLead, 2);
  SerialDebug.print(
      "[FAULT SNAP] translation forward/right mm=");
  SerialDebug.print(commandedForwardMm, 1);
  SerialDebug.print("/");
  SerialDebug.println(commandedRightMm, 1);
  SerialDebug.print("[FAULT SNAP] motor position=");
  SerialDebug.print(positions.motor1);
  SerialDebug.print("/");
  SerialDebug.print(positions.motor2);
  SerialDebug.print("/");
  SerialDebug.print(positions.motor3);
  SerialDebug.print("/");
  SerialDebug.println(positions.motor4);
  SerialDebug.print("[FAULT SNAP] motor target=");
  SerialDebug.print(targets.motor1);
  SerialDebug.print("/");
  SerialDebug.print(targets.motor2);
  SerialDebug.print("/");
  SerialDebug.print(targets.motor3);
  SerialDebug.print("/");
  SerialDebug.println(targets.motor4);
  SerialDebug.print("[FAULT SNAP] motor remaining=");
  SerialDebug.print(remaining.motor1);
  SerialDebug.print("/");
  SerialDebug.print(remaining.motor2);
  SerialDebug.print("/");
  SerialDebug.print(remaining.motor3);
  SerialDebug.print("/");
  SerialDebug.println(remaining.motor4);
}

float moveTowardWithMaximumDelta(
    float current,
    float target,
    float maximumDelta) {
  if (target > current + maximumDelta) {
    return current + maximumDelta;
  }
  if (target < current - maximumDelta) {
    return current - maximumDelta;
  }
  return target;
}

/*
 * main平移IMU控制器：每帧更新期望纠偏角速度，轮速合成函数立即把它作为
 * 四轮共同旋转分量输出。这里不再修改moveTo()目标，因此IMU误差减小、
 * 变号或第14段切换目标时，都不可能把纯平移终点回缩并触发倒车。
 *
 * P控制、角速度斜率以及实际纠偏角安全限制仍保留。累计角只积分经过
 * 平移优先分配后真正获准输出的纠偏角速度，未获得轮速余量的请求不会被
 * 误算为已经执行，也不会在第7/15段长行程中触发虚假累计角故障。位移投影
 * 到位以及航向进入容差仍是本段完成的硬条件。
 */
bool serviceTranslationHeadingControl() {
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 0
  // main_v0的平移段禁止进入此函数；段后纠偏由独立状态机处理。
  return true;
#else
  if (!translationHeadingControlActive) {
    routeFault("translation heading control state missing");
    return false;
  }
  if (!imuIsFresh()) {
    routeFault("IMU data timeout during translation");
    return false;
  }

  // 速度规划每轮更新，不依赖IMU帧率；硬位移到位也能及时撤去平移轮速。
  updateTranslationBaseRate();
  applyTranslationWheelSpeedMix();

  /*
   * 位移完成的本轮立即记录终点纠偏阶段起点，不依赖下一帧IMU到达。该时间
   * 对所有段都记录，只有第4段的完成判定会强制使用300 ms最短窗口。
   */
  if ((translationBaseDisplacementComplete ||
       translationBaseForcedComplete) &&
      !translationTerminalCorrectionStartValid) {
    translationTerminalCorrectionStartValid = true;
    translationTerminalCorrectionStartMs = millis();
    if (routeIndex < ROUTE_COMMAND_COUNT &&
        route[routeIndex].specificationStep == 4U) {
      SerialDebug.print(
          "[STEP4->5 IMU WINDOW] start, minimum ms=");
      SerialDebug.println(
          STEP_04_TERMINAL_IMU_MINIMUM_DURATION_MS);
    }
  }

  // 两帧之间不重复积分同一个误差。
  if (imuValidFrameSequence ==
      translationLastImuFrameSequence) {
    return true;
  }

  const uint32_t nowMs = millis();
  float dtSeconds =
      static_cast<float>(
          nowMs - translationLastControlMs) /
      1000.0f;
  translationLastControlMs = nowMs;
  translationLastImuFrameSequence =
      imuValidFrameSequence;

  if (dtSeconds <= 0.0f) {
    return true;
  }
  if (dtSeconds >
      TRANSLATION_HEADING_MAXIMUM_CONTROL_DT_SECONDS) {
    dtSeconds =
        TRANSLATION_HEADING_MAXIMUM_CONTROL_DT_SECONDS;
  }

  const float errorDegrees = headingErrorDegrees();
  const bool terminalHeadingCorrectionActive =
      translationBaseDisplacementComplete ||
      translationBaseForcedComplete;
  const bool activeCommandIsStep01 =
      routeIndex < ROUTE_COMMAND_COUNT &&
      route[routeIndex].specificationStep == 1U;
  const bool headingCorrectionAllowedNow =
      terminalHeadingCorrectionActive ||
      (ENABLE_IN_MOTION_IMU_CORRECTION &&
       (!activeCommandIsStep01 ||
        STEP_01_ENABLE_IN_MOTION_IMU_CORRECTION));
  const float activeHeadingKpPerSecond =
      TRANSLATION_HEADING_KP_PER_SECOND *
      (terminalHeadingCorrectionActive
           ? TRANSLATION_TERMINAL_KP_SCALE
           : 1.0f);
  float desiredCorrectionRateDps = 0.0f;
  /*
   * 总开关关闭时，所有平移段在位移完成前都故意不进入本条件，因此目标
   * 纠偏角速度保持0；IMU仍持续读取并记录航向误差，终点状态一成立便在
   * 同一控制器中立即恢复纠偏，不增加独立的纯等待或稳定计时。代码路径
   * 完整保留，重新打开总开关即可恢复原行程内控制。
   */
  if (headingCorrectionAllowedNow &&
      fabsf(errorDegrees) >
      activeTranslationHeadingToleranceDegrees()) {
    desiredCorrectionRateDps =
        activeHeadingKpPerSecond *
        errorDegrees;

    /*
     * P控制在接近目标时会自然趋近0，过去0.2°误差只有约0.6°/s，容易形成
     * 肉眼可见的低速长尾。现在仅在位移已完成的终点阶段，把仍超差的非零
     * 请求抬到至少2°/s；行程中不套用该下限，因此不会增加途中抢占轮速。
     */
    if (terminalHeadingCorrectionActive &&
        fabsf(desiredCorrectionRateDps) <
            TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_DPS) {
      desiredCorrectionRateDps =
          errorDegrees > 0.0f
              ? TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_DPS
              : -TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_DPS;
    }

    if (desiredCorrectionRateDps >
        TRANSLATION_HEADING_MAXIMUM_RATE_DPS) {
      desiredCorrectionRateDps =
          TRANSLATION_HEADING_MAXIMUM_RATE_DPS;
    } else if (
        desiredCorrectionRateDps <
        -TRANSLATION_HEADING_MAXIMUM_RATE_DPS) {
      desiredCorrectionRateDps =
          -TRANSLATION_HEADING_MAXIMUM_RATE_DPS;
    }
  }

  const float maximumRateChangeDps =
      TRANSLATION_HEADING_RATE_SLEW_DPS2 *
      dtSeconds;
  /*
   * 从“上一轮真正获准输出的角速度”起算角速度斜率，而不是从可能被平移
   * 完全钳掉的请求值起算。否则长直线最高速时请求值会在后台升到30°/s，
   * 位移一结束便可能瞬间全部释放，绕过120°/s²角加速度限制并造成甩动。
   * 现在即使行程中没有纠偏余量，终点独立调整也会从实际0角速度正常加速。
   */
  translationHeadingCorrectionRateDps =
      moveTowardWithMaximumDelta(
          translationAppliedHeadingCorrectionRateDps,
          desiredCorrectionRateDps,
          maximumRateChangeDps);

  // 先完成平移优先分配，才能知道本轮真正获准输出的纠偏角速度。
  applyTranslationWheelSpeedMix();

  translationAccumulatedCorrectionDegrees +=
      translationAppliedHeadingCorrectionRateDps *
      dtSeconds;

  /*
   * 安全限制检查四轮位置表明的“实际已执行纠偏角”，而不是可能因平移占满
   * 轮速而被钳掉的请求角。这样仍能发现IMU方向错误或持续自转，同时不会
   * 把未执行控制量错误累计成第15段中途停机。
   */
  if (fabsf(actualTranslationCorrectionDegrees()) >
      TRANSLATION_MAXIMUM_ACCUMULATED_CORRECTION_DEGREES) {
    routeFault("translation actual IMU correction exceeded safety limit");
    return false;
  }
  return true;
#endif
}

void startTurnResidualCorrection(float errorDegrees) {
  float correction = errorDegrees;
  if (correction >
      MAXIMUM_SINGLE_TURN_RESIDUAL_DEGREES) {
    correction =
        MAXIMUM_SINGLE_TURN_RESIDUAL_DEGREES;
  } else if (correction <
             -MAXIMUM_SINGLE_TURN_RESIDUAL_DEGREES) {
    correction =
        -MAXIMUM_SINGLE_TURN_RESIDUAL_DEGREES;
  }

  /*
   * 误差刚刚超过容差时，过小的脉冲可能克服不了静摩擦。
   * 因此把非零修正提高到最小 0.15°，修正后再由 IMU 重新判断。
   */
  if (correction > 0.0f &&
      correction < MINIMUM_TURN_RESIDUAL_DEGREES) {
    correction = MINIMUM_TURN_RESIDUAL_DEGREES;
  } else if (
      correction < 0.0f &&
      correction > -MINIMUM_TURN_RESIDUAL_DEGREES) {
    correction = -MINIMUM_TURN_RESIDUAL_DEGREES;
  }

  SerialDebug.print(
      "[TURN RESIDUAL] target/actual/error/correction=");
  SerialDebug.print(
      targetCounterClockwiseHeadingDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(currentRouteHeadingDegrees(), 2);
  SerialDebug.print("/");
  SerialDebug.print(errorDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.println(correction, 2);

  setDriveMotionProfile(
      TURN_RESIDUAL_MAXIMUM_STEP_RATE,
      TURN_RESIDUAL_STEP_ACCELERATION);
  startVehicleDisplacement(0.0f, 0.0f, correction);
}

void printCurrentCommand() {
  const RouteCommand &command = route[routeIndex];
  SerialDebug.print("[ROUTE START] step=");
  SerialDebug.print(command.specificationStep);
  SerialDebug.print("/21, type=");
  SerialDebug.print(commandTypeName(command.type));
  SerialDebug.print(", value=");
  SerialDebug.print(command.value);
  SerialDebug.print(", motion scale=");
  SerialDebug.print(command.motionScale, 2);
  SerialDebug.print(", target heading=");
  SerialDebug.print(
      targetCounterClockwiseHeadingDegrees, 2);
  SerialDebug.print(", name=");
  SerialDebug.println(command.name);
}

void printCurrentCommandCompletion() {
  const RouteCommand &command = route[routeIndex];
  SerialDebug.print("[ROUTE DONE] step=");
  SerialDebug.print(command.specificationStep);
  SerialDebug.print("/21, elapsed=");
  SerialDebug.print(millis() - commandStartMs);
  SerialDebug.print(
      " ms, target/actual/error heading=");
  SerialDebug.print(
      targetCounterClockwiseHeadingDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(currentRouteHeadingDegrees(), 2);
  SerialDebug.print("/");
  SerialDebug.println(headingErrorDegrees(), 2);
}

void finishProgram(
    bool stoppedByConfiguredStep,
    uint8_t completedStep) {
  disableDriveMotors();
  programState = PROGRAM_FINISHED;
  routePhase = ROUTE_PHASE_IDLE;
  commandStarted = false;
  startRequested = false;
  abortRequested = false;
  hmiSetRunStatus(
      stoppedByConfiguredStep ? "STEPSTOP" : "FINISH");

  SerialDebug.print(
      stoppedByConfiguredStep
          ? "[RUN STEP STOP] completed step="
          : "[RUN COMPLETE] completed step=");
  SerialDebug.print(completedStep);
  SerialDebug.print("/21, total=");
  SerialDebug.print(millis() - routeRunStartMs);
  SerialDebug.print(" ms, start zone=");
  SerialDebug.print(START_ZONE_SELECTION);
  SerialDebug.print(", scan b=");
  SerialDebug.print(scanDistanceBmm, 1);
  SerialDebug.println(" mm");
}

void advanceRoute() {
  const uint8_t completedStep =
      route[routeIndex].specificationStep;
  printCurrentCommandCompletion();
  stopAllMotorsImmediately();
  commandStarted = false;
  routePhase = ROUTE_PHASE_IDLE;

  /*
   * 调试停止只在当前段正常完成后判断，因此不会在运动中途硬停，也不会
   * 绕过行程内IMU控制。STOP_AFTER_STEP=0时此分支永远不进入。
   */
  if (STOP_AFTER_STEP != 0U &&
      completedStep == STOP_AFTER_STEP) {
    finishProgram(true, completedStep);
    return;
  }

  ++routeIndex;
  if (routeIndex >= ROUTE_COMMAND_COUNT) {
    finishProgram(false, completedStep);
  } else {
    /*
     * 不设置任何定时段间等待。commandStarted已清零，下一次loop()会立即
     * 启动下一段；保留下来的终点IMU纠偏属于实际控制动作，不是纯等待。
     */
    hmiSetRunStatus("RUN");
  }
}

/*
 * main_v0平移段末航向闭环。
 *
 * 位移位置段到位后才进入这里，因此行驶过程中完全没有IMU轮速叠加。
 * 每次用当前实时误差启动一个最多12°的低速原地修正；电机一停就再次读取
 * IMU。误差进入本段容差后立即完成动作，没有“连续稳定N毫秒”的延时。
 * 扫码走满200 mm时使用WAIT动作：先修正航向，再原地等二维码；扫码途中
 * 已成功则使用ADVANCE动作，纠偏完成后立刻进入第3段。
 */
void beginPostTranslationHeadingCorrection(
    PostTranslationCompletionAction completionAction) {
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 0
  translationHeadingControlActive = false;
  translationHeadingCorrectionRateDps = 0.0f;
  postTranslationHeadingCorrectionActive = true;
  postTranslationCompletionAction =
      completionAction;
  hmiSetRunStatus("POSTIMU");
  SerialDebug.print(
      "[POST IMU START] target/actual/error/action=");
  SerialDebug.print(
      targetCounterClockwiseHeadingDegrees, 2);
  SerialDebug.print("/");
  SerialDebug.print(currentRouteHeadingDegrees(), 2);
  SerialDebug.print("/");
  SerialDebug.print(headingErrorDegrees(), 2);
  SerialDebug.print("/");
  SerialDebug.println(
      completionAction ==
              POST_TRANSLATION_ADVANCE_ROUTE
          ? "ADVANCE"
          : "WAIT_SCAN");
#else
  (void)completionAction;
#endif
}

void servicePostTranslationHeadingCorrection() {
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 0
  if (!postTranslationHeadingCorrectionActive) {
    routeFault("post-translation IMU state missing");
    return;
  }
  if (!imuIsFresh()) {
    routeFault("IMU timeout during post-translation correction");
    return;
  }
  if (millis() - commandStartMs > MOTION_TIMEOUT_MS) {
    routeFault("post-translation IMU correction timeout");
    return;
  }

  // 上一次小角度修正尚未走完时继续高频run()，不插入任何delay()。
  if (!allMotorsArrived()) {
    return;
  }

  const float errorDegrees = headingErrorDegrees();
  if (fabsf(errorDegrees) <=
      activeTranslationHeadingToleranceDegrees()) {
    SerialDebug.print(
        "[POST IMU DONE] target/actual/error=");
    SerialDebug.print(
        targetCounterClockwiseHeadingDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.print(currentRouteHeadingDegrees(), 2);
    SerialDebug.print("/");
    SerialDebug.println(errorDegrees, 2);

    postTranslationHeadingCorrectionActive = false;
    if (postTranslationCompletionAction ==
        POST_TRANSLATION_WAIT_FOR_SCAN_CODE) {
      routePhase = ROUTE_PHASE_WAIT_SCAN_CODE;
      hmiSetRunStatus("SCANMAX");
      SerialDebug.println(
          "[SCAN] heading corrected, waiting in place");
    } else {
      advanceRoute();
    }
    return;
  }

  startTurnResidualCorrection(errorDegrees);
  hmiSetRunStatus("POSTIMU");
#endif
}

// ===========================================================================
// 12. 启动每一种路线命令
// ===========================================================================

/*
 * 第1段专用斜线入口：纵向a与向右150 mm在同一个麦轮位移命令中合成。
 * 两个启停区都传入正横移；只有纵向a根据启停区选择正负。两个名义分量
 * 都乘第1段独立系数。速度规划使用普通段非对称梯形；IMU旋转轮速继续
 * 实时叠加，因此底盘朝向保持发车目标，不会先转向再行驶。
 */
void startZoneDiagonalTranslation(
    float nominalForwardMm,
    float nominalRightMm,
    float motionScale) {
  const float commandedForwardMm =
      nominalForwardMm * motionScale;
  const float commandedRightMm =
      nominalRightMm * motionScale;
  const float commandedMaximumStepRate =
      FAST_MAXIMUM_STEP_RATE *
      motionScale *
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;
  const float commandedAcceleration =
      FAST_STEP_ACCELERATION *
      motionScale *
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;

  SerialDebug.print(
      "[SEGMENT DIAGONAL] nominal forward/right=");
  SerialDebug.print(nominalForwardMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(nominalRightMm, 1);
  SerialDebug.print(", commanded=");
  SerialDebug.print(commandedForwardMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(commandedRightMm, 1);
  SerialDebug.print(", scale/requested-vmax/acc=");
  SerialDebug.print(motionScale, 3);
  SerialDebug.print("/");
  SerialDebug.print(commandedMaximumStepRate, 1);
  SerialDebug.print("/");
  SerialDebug.println(commandedAcceleration, 1);

  setDriveMotionProfile(
      commandedMaximumStepRate,
      commandedAcceleration,
      DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID);
  startTranslationDisplacement(
      commandedForwardMm,
      commandedRightMm);
  routePhase = ROUTE_PHASE_MOTION;
}

/*
 * 快速平移入口接收路线真实/名义毫米值和该段动作系数。
 * 所有纵向段都不属于第7、15段，因此除各段motionScale外，最大速度和
 * 加速度还统一乘第二轮1.50提速系数。距离只乘该段motionScale，不会因
 * 第二轮提速而改变。
 */
void startFastLongitudinalTranslation(
    float nominalForwardMm,
    float motionScale,
    float maximumSpeedProfileScale = 1.0f,
    float accelerationProfileScale = 1.0f) {
  const float commandedForwardMm =
      nominalForwardMm * motionScale;
  const float commandedMaximumStepRate =
      FAST_MAXIMUM_STEP_RATE *
      motionScale *
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
      maximumSpeedProfileScale;
  const float commandedAcceleration =
      FAST_STEP_ACCELERATION *
      motionScale *
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
      accelerationProfileScale;

  SerialDebug.print(
      "[SEGMENT LONG] nominal/commanded/scale mm=");
  SerialDebug.print(nominalForwardMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(commandedForwardMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(motionScale, 2);
  SerialDebug.print(", profile-v/a-scale=");
  SerialDebug.print(maximumSpeedProfileScale, 2);
  SerialDebug.print("/");
  SerialDebug.print(accelerationProfileScale, 2);
  SerialDebug.print(", requested-vmax/acc=");
  SerialDebug.print(commandedMaximumStepRate, 1);
  SerialDebug.print("/");
  SerialDebug.println(commandedAcceleration, 1);

  setDriveMotionProfile(
      commandedMaximumStepRate,
      commandedAcceleration);
  startTranslationDisplacement(
      commandedForwardMm, 0.0f);
  routePhase = ROUTE_PHASE_MOTION;
}

/*
 * 横向快速运动：
 *   motionScale继续负责该段距离标定，并按既有规则同步影响基础档位；
 *   maximumSpeedProfileScale和accelerationProfileScale只调整运动曲线。
 *
 * 第7、15段利用后两个参数做柔和起停；其他横移段传第二轮1.50提速
 * 系数。实际使用前，accelerationProfileScale还会被硬限制到普通直行
 * 档位倍率的1/2。正值为向右，负值为向左。
 */
void startFastLateralTranslation(
    float nominalRightMm,
    float motionScale,
    float maximumSpeedProfileScale,
    float accelerationProfileScale,
    DriveProfileShape profileShape) {
  const float commandedRightMm =
      nominalRightMm * motionScale;
  const float commandedMaximumStepRate =
      FAST_MAXIMUM_STEP_RATE *
      motionScale *
      maximumSpeedProfileScale;
  const float lateralAccelerationProfileLimit =
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
      LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL;
  const float limitedAccelerationProfileScale =
      fminf(
          accelerationProfileScale,
          lateralAccelerationProfileLimit);
  const float commandedAcceleration =
      FAST_STEP_ACCELERATION *
      motionScale *
      limitedAccelerationProfileScale;

  SerialDebug.print(
      "[SEGMENT LATERAL] nominal/commanded/distance-scale mm=");
  SerialDebug.print(nominalRightMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(commandedRightMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(motionScale, 2);
  SerialDebug.print(
      ", profile speed/requested-acc/limited-acc scale=");
  SerialDebug.print(maximumSpeedProfileScale, 2);
  SerialDebug.print("/");
  SerialDebug.print(accelerationProfileScale, 2);
  SerialDebug.print("/");
  SerialDebug.print(limitedAccelerationProfileScale, 2);
  SerialDebug.print(", requested-vmax/acc=");
  SerialDebug.print(commandedMaximumStepRate, 1);
  SerialDebug.print("/");
  SerialDebug.println(commandedAcceleration, 1);

  setDriveMotionProfile(
      commandedMaximumStepRate,
      commandedAcceleration,
      profileShape);
  startTranslationDisplacement(
      0.0f, commandedRightMm);
  routePhase = ROUTE_PHASE_MOTION;
}

void startTurn(
    float nominalCounterClockwiseDegrees,
    float motionScale) {
  /*
   * “用户设定转角”就是路线名义转角乘该段独立系数。例如第8段名义
   * 顺时针180°、系数0.98时，本段设定转角就是顺时针176.4°。
   *
   * 累计IMU目标和旋转脉冲的初始目标必须使用同一个设定转角。转动过程中
   * 会实时读取IMU并预测制动位置，使车辆尽可能到达176.4°，不会拉回
   * 180°；后续直行/横移段继续保持这个累计设定航向。
   */
  const float commandedCounterClockwiseDegrees =
      nominalCounterClockwiseDegrees * motionScale;
  targetCounterClockwiseHeadingDegrees +=
      commandedCounterClockwiseDegrees;

  SerialDebug.print(
      "[SEGMENT TURN] nominal/commanded/scale/target deg=");
  SerialDebug.print(
      nominalCounterClockwiseDegrees, 1);
  SerialDebug.print("/");
  SerialDebug.print(
      commandedCounterClockwiseDegrees, 1);
  SerialDebug.print("/");
  SerialDebug.print(motionScale, 3);
  SerialDebug.print("/");
  SerialDebug.println(
      targetCounterClockwiseHeadingDegrees, 2);

  translationHeadingControlActive = false;
  activeTranslationForwardMm = 0.0f;
  activeTranslationRightMm = 0.0f;
  integratedTurnControlActive = true;
  integratedTurnBrakeCommandIssued = false;
  integratedTurnDirectionSign =
      commandedCounterClockwiseDegrees >= 0.0f ? +1 : -1;
  activeTurnPulsesPerDegree =
      rotationPulsesPerDegree(
          integratedTurnDirectionSign);
  if (activeTurnPulsesPerDegree <= 0.0f) {
    routeFault("invalid turn pulse/degree conversion");
    return;
  }

  setDriveMotionProfile(
      TURN_MAXIMUM_STEP_RATE * motionScale,
      TURN_STEP_ACCELERATION * motionScale);
  startVehicleDisplacement(
      0.0f, 0.0f,
      commandedCounterClockwiseDegrees);
  routePhase = ROUTE_PHASE_MOTION;
  hmiSetRunStatus("TURNIMU");
}

void startSlowScan(float motionScale) {
  scanMotionOrigin = currentMotorPositions();
  scanOriginValid = true;
  scanDistanceCaptured = false;
  scanDistanceBmm = 0.0f;
  scanCommandedMaximumDistanceMm = fminf(
      static_cast<float>(MAXIMUM_SCAN_DISTANCE_B_MM) *
          motionScale,
      static_cast<float>(MAXIMUM_SCAN_DISTANCE_B_MM));

  /*
   * 二维码串口在步骤1期间也一直监听。如果进入步骤2时已经有有效码，
   * 则b=0，不再产生无意义慢行。仍建立零位移控制状态：main在本段内部
   * 核验硬航向；main_v0按其规则执行段后原地航向修正。
   */
  if (scanFlag) {
    SerialDebug.println(
        "[SCAN] valid code already present, b=0 mm");
    /*
     * 即使b=0也建立一个零位移平移段：main可在同一步内核验硬航向，
     * main_v0则会立即进入段后原地纠偏。这样状态机不会绕过第2段航向条件。
     */
    setDriveMotionProfile(
        SCAN_MAXIMUM_STEP_RATE *
            STEP_02_MAXIMUM_SPEED_PROFILE_SCALE,
        SCAN_STEP_ACCELERATION *
            STEP_02_ACCELERATION_PROFILE_SCALE);
    startTranslationDisplacement(0.0f, 0.0f);
    routePhase = ROUTE_PHASE_MOTION;
    return;
  }

  /*
   * 只有“尚未获得有效任务码”的前探过程使用扫码慢速档。
   * 最大前探距离、最大速度和加速度都使用独立的第2段系数；速度和
   * 加速度还使用非7/15段的第二轮1.50提速。距离另加200 mm硬上限，
   * 防止把系数调到大于1后违反扫码慢行段限制。
   * scanFlag一旦置位，本段会立即停止平移：main继续完成段内硬航向，
   * main_v0转入段后原地修正；两者合格后才进入步骤3。
   */
  const float commandedMaximumStepRate =
      SCAN_MAXIMUM_STEP_RATE *
      motionScale *
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
      STEP_02_MAXIMUM_SPEED_PROFILE_SCALE;
  const float commandedAcceleration =
      SCAN_STEP_ACCELERATION *
      motionScale *
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
      STEP_02_ACCELERATION_PROFILE_SCALE;

  SerialDebug.print(
      "[SCAN] maximum/scale/requested-vmax/acc=");
  SerialDebug.print(scanCommandedMaximumDistanceMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(motionScale, 2);
  SerialDebug.print("/");
  SerialDebug.print(commandedMaximumStepRate, 1);
  SerialDebug.print("/");
  SerialDebug.println(commandedAcceleration, 1);

  setDriveMotionProfile(
      commandedMaximumStepRate,
      commandedAcceleration);
  startTranslationDisplacement(
      selectedStartZoneDirection() *
          scanCommandedMaximumDistanceMm,
      0.0f);
  routePhase = ROUTE_PHASE_MOTION;
  hmiSetRunStatus("SCAN");
}

void captureScanDistanceB() {
  if (!scanOriginValid) {
    routeFault("scan origin missing");
    return;
  }

  /*
   * 沿启停区规定方向取正后，该值就是步骤3“92-b”使用的开环b。
   * 最后限制到0～第2段实际下发的最大距离，消除脉冲四舍五入带来的
   * 极小边界误差。这里不反除第2段系数，否则会把已输出的距离重复放大。
   */
  scanDistanceBmm =
      selectedStartZoneDirection() *
      forwardTravelFromOriginMm(scanMotionOrigin);
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
  scanDistanceCaptured = true;
}

void startAdjustToPointA(float motionScale) {
  if (!scanFlag || !scanOriginValid) {
    routeFault("step 3 requires a valid QR code and scan origin");
    return;
  }

  /*
   * 用户步骤3的三个分支由同一个有符号 delta 完整表示：
   *
   *   delta = (900 + 290/2 - a) - b = 92 - b
   *
   *   b < 92：delta > 0，沿步骤2方向快速前进 92-b；
   *   b = 92：delta = 0，已经位于 A，不产生平移；
   *   b > 92：delta < 0，沿反方向快速倒退 b-92。
   *
   * 启停区2再整体乘 selectedStartZoneDirection()=-1，因此只反转
   * 步骤1、2、3、21，分支判断本身仍以正的 b 表示。
   */
  const float deltaAlongScanDirectionMm =
      static_cast<float>(SCAN_START_TO_POINT_A_MM) -
      scanDistanceBmm;
  const float vehicleForwardMm =
      selectedStartZoneDirection() *
      deltaAlongScanDirectionMm;

  SerialDebug.print("[SCAN->A] b/delta/vehicle-forward=");
  SerialDebug.print(scanDistanceBmm, 1);
  SerialDebug.print("/");
  SerialDebug.print(deltaAlongScanDirectionMm, 1);
  SerialDebug.print("/");
  SerialDebug.print(vehicleForwardMm, 1);
  SerialDebug.println(" mm");

  if (deltaAlongScanDirectionMm > 0.05f) {
    SerialDebug.println(
        "[SCAN->A] b<92: continue along scan direction");
  } else if (deltaAlongScanDirectionMm < -0.05f) {
    SerialDebug.println(
        "[SCAN->A] b>92: reverse excess distance");
  } else {
    SerialDebug.println(
        "[SCAN->A] b=92: already at point A");
  }

  startFastLongitudinalTranslation(
      vehicleForwardMm,
      motionScale,
      STEP_03_MAXIMUM_SPEED_PROFILE_SCALE,
      STEP_03_ACCELERATION_PROFILE_SCALE);
}

void startCurrentCommand() {
  if (routeIndex >= ROUTE_COMMAND_COUNT ||
      programState != PROGRAM_RUNNING) {
    return;
  }

  const RouteCommand &command = route[routeIndex];
  const float motionScale = command.motionScale;
  commandStartMs = millis();
  printCurrentCommand();

  /*
   * 第7段启动前的累计目标就是第一次在原料区C准备横移时的标准朝向。
   * 只保存目标，不保存带容差的瞬时IMU实测值，避免把一次测量噪声永久
   * 复制到第15段。
   */
  if (command.specificationStep == 7U) {
    step7RawAreaHeadingReferenceDegrees =
        targetCounterClockwiseHeadingDegrees;
    step7RawAreaHeadingReferenceValid = true;
    SerialDebug.print(
        "[STEP7 HEADING REF] unwrapped/normalized deg=");
    SerialDebug.print(
        step7RawAreaHeadingReferenceDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.println(
        wrapDeltaDegrees(
            step7RawAreaHeadingReferenceDegrees),
        2);
  }

  /*
   * 第15段只能在第14段停车位置已经完成同向匹配后启动。这里再校验一次
   * 两个累计目标的模360°差值，防止以后调整状态机时绕过该安全条件。
   */
  if (command.specificationStep == 15U) {
    if (!step7RawAreaHeadingReferenceValid ||
        !step15HeadingMatchAppliedAtSecondRawArea) {
      routeFault("step 15 heading match was not completed");
      return;
    }
    const float targetMismatchDegrees =
        wrapDeltaDegrees(
            targetCounterClockwiseHeadingDegrees -
            step7RawAreaHeadingReferenceDegrees);
    if (fabsf(targetMismatchDegrees) > 0.001f) {
      routeFault("step 15 target differs from step 7");
      return;
    }
  }

  switch (command.type) {
    case COMMAND_ZONE_LONGITUDINAL_FAST:
      /*
       * 步骤1和21共用“随启停区反转纵向”的命令类型，但只有步骤1
       * 需要斜线横移。这里显式按步骤号分开，避免返航的第21段再次
       * 误叠加150 mm横移。
       *
       * 启停区方向符号只乘纵向，第1段横向固定向右：
       *   启停区1：+a前进、+150 mm向右；
       *   启停区2：-a后退、+150 mm向右。
       */
      if (command.specificationStep == 1U) {
        startZoneDiagonalTranslation(
            selectedStartZoneDirection() *
                static_cast<float>(command.value),
            static_cast<float>(
                STEP_01_ADDITIONAL_LATERAL_MM),
            motionScale);
      } else {
        startFastLongitudinalTranslation(
            selectedStartZoneDirection() *
                static_cast<float>(command.value),
            motionScale);
      }
      break;

    case COMMAND_SCAN_SLOW:
      startSlowScan(motionScale);
      break;

    case COMMAND_ADJUST_TO_POINT_A:
      startAdjustToPointA(motionScale);
      break;

    case COMMAND_FORWARD_FAST:
      startFastLongitudinalTranslation(
          static_cast<float>(command.value),
          motionScale);
      break;

    case COMMAND_BACKWARD_FAST:
      startFastLongitudinalTranslation(
          -static_cast<float>(command.value),
          motionScale);
      break;

    case COMMAND_RIGHT_FAST:
    case COMMAND_LEFT_FAST: {
      /*
       * 第7、15段使用独立防打滑曲线，不参加第二轮提速；第4、6、20段及
       * 以后新增的普通横移默认使用第二轮1.50速度/加速度倍率。
       * 横移入口以右为正，因此LEFT命令只在这里把名义距离取负；距离标定、
       * 速度规划以及终点IMU控制仍与其他横移共用同一套实现。
       */
      float maximumSpeedProfileScale =
          NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;
      float accelerationProfileScale =
          NON_07_15_LINEAR_PROFILE_INCREASE_SCALE;
      DriveProfileShape profileShape =
          DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID;
      if (command.specificationStep == 7U) {
        maximumSpeedProfileScale =
            STEP_07_LATERAL_MAX_SPEED_SCALE;
        accelerationProfileScale =
            STEP_07_LATERAL_ACCELERATION_SCALE;
        profileShape =
            DRIVE_PROFILE_ASYMMETRIC_S_CURVE;
      } else if (command.specificationStep == 15U) {
        maximumSpeedProfileScale =
            STEP_15_LATERAL_MAX_SPEED_SCALE;
        accelerationProfileScale =
            STEP_15_LATERAL_ACCELERATION_SCALE;
        profileShape =
            DRIVE_PROFILE_ASYMMETRIC_S_CURVE;
      }

      startFastLateralTranslation(
          (command.type == COMMAND_LEFT_FAST ? -1.0f : 1.0f) *
              static_cast<float>(command.value),
          motionScale,
          maximumSpeedProfileScale,
          accelerationProfileScale,
          profileShape);
      break;
    }

    case COMMAND_TURN_COUNTERCLOCKWISE:
      startTurn(
          static_cast<float>(command.value),
          motionScale);
      break;

    case COMMAND_TURN_CLOCKWISE:
      startTurn(
          -static_cast<float>(command.value),
          motionScale);
      break;
  }

  if (programState == PROGRAM_RUNNING) {
    commandStarted = true;
  }
}

// ===========================================================================
// 13. 扫码、实时IMU旋转与普通步骤的非阻塞完成判定
// ===========================================================================

void serviceScanCommand() {
  switch (routePhase) {
    case ROUTE_PHASE_MOTION:
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
      /*
       * main：二维码成功后立即把非负平移基础步频清零，慢行到此为止；
       * 不调用位置目标重算，也不产生反向平移。随后仍在第2段内只输出共同
       * 旋转轮速，直到硬航向合格。该纯旋转收尾属于段内闭环，不是段后
       * 独立校准，也没有稳定性计时。
       */
      if (scanFlag) {
        if (!translationBaseForcedComplete) {
          translationBaseForcedComplete = true;
          translationBaseDisplacementComplete = true;
          translationBaseStepRate = 0.0f;
          translationAppliedBaseStepRate = 0.0f;
          applyTranslationWheelSpeedMix();
          if (!scanDistanceCaptured) {
            captureScanDistanceB();
          }
          SerialDebug.println(
              "[SCAN] QR acquired: translation stopped, finishing in-segment heading");
        }
      }

      if (!serviceTranslationHeadingControl()) {
        return;
      }
      if (millis() - commandStartMs >
          MOTION_TIMEOUT_MS) {
        routeFault("slow scan motion/heading timeout");
        return;
      }

      if (!translationBaseDisplacementComplete ||
          fabsf(headingErrorDegrees()) >
              activeTranslationHeadingToleranceDegrees() ||
          fabsf(translationHeadingCorrectionRateDps) >
              TRANSLATION_COMPLETION_RATE_TOLERANCE_DPS) {
        return;
      }

      if (!scanDistanceCaptured) {
        captureScanDistanceB();
      }
      if (scanFlag) {
        advanceRoute();
      } else {
        /*
         * 完成硬航向后才进入不定长扫码等待。先把最多0.25°/s的残余纠偏
         * 轮速清零，避免等待二维码期间因状态机不再更新控制器而缓慢自转。
         */
        stopAllMotorsImmediately();
        routePhase = ROUTE_PHASE_WAIT_SCAN_CODE;
        hmiSetRunStatus("SCANMAX");
        SerialDebug.print(
            "[SCAN] configured limit and heading reached at ");
        SerialDebug.print(scanCommandedMaximumDistanceMm, 1);
        SerialDebug.println(" mm, waiting in place");
      }
      return;
#else
      /*
       * main_v0：位置运动期间完全不调用serviceTranslationHeadingControl()。
       * 扫码成功时先立即终止平移并记录b，再进入本段段后原地纠偏；走满
       * 200 mm则先到位、再纠偏、最后等待二维码。
       */
      if (postTranslationHeadingCorrectionActive) {
        if (scanFlag) {
          postTranslationCompletionAction =
              POST_TRANSLATION_ADVANCE_ROUTE;
        }
        servicePostTranslationHeadingCorrection();
        return;
      }

      if (scanFlag) {
        stopAllMotorsImmediately();
        if (!scanDistanceCaptured) {
          captureScanDistanceB();
        }
        beginPostTranslationHeadingCorrection(
            POST_TRANSLATION_ADVANCE_ROUTE);
        servicePostTranslationHeadingCorrection();
        return;
      }

      if (!allMotorsArrived()) {
        if (millis() - commandStartMs >
            MOTION_TIMEOUT_MS) {
          routeFault("slow scan motion timeout");
        }
        return;
      }

      if (!scanDistanceCaptured) {
        captureScanDistanceB();
      }
      beginPostTranslationHeadingCorrection(
          POST_TRANSLATION_WAIT_FOR_SCAN_CODE);
      servicePostTranslationHeadingCorrection();
      return;
#endif

    case ROUTE_PHASE_WAIT_SCAN_CODE:
      if (scanFlag) {
        SerialDebug.println(
            "[SCAN] code acquired at 200 mm limit");
        // 航向已经在进入等待相位前合格，因此这里不再重复校准。
        advanceRoute();
      }
      return;

    case ROUTE_PHASE_IDLE:
      return;
  }
}

/*
 * 明确旋转按当前规划启动制动：
 *
 *   - 当前所有明确旋转均使用route.cpp非对称梯形：先切换2/3减速度，再
 *     调用AccelStepper::stop()按当前步频计算停车目标；
 *   - 保留S形分支仅用于代码完整性，若以后把某个旋转显式设成S形，仍按
 *     正弦包络预计制动脉冲。
 */
void beginIntegratedTurnBraking() {
  if (activeDriveProfileShape ==
      DRIVE_PROFILE_ASYMMETRIC_TRAPEZOID) {
    beginDriveTrapezoidDeceleration();
    for (uint8_t i = 0U; i < 4U; ++i) {
      motors[i]->stop();
    }
    integratedTurnBrakeCommandIssued = true;
    return;
  }

  for (uint8_t i = 0U; i < 4U; ++i) {
    const float signedStepRate =
        motors[i]->speed();
    const long stoppingSteps =
        static_cast<long>(
            sCurveStoppingSteps(
                fabsf(signedStepRate)) +
            0.5f);
    if (signedStepRate > 0.0f) {
      motors[i]->move(stoppingSteps);
    } else if (signedStepRate < 0.0f) {
      motors[i]->move(-stoppingSteps);
    } else {
      motors[i]->moveTo(
          motors[i]->currentPosition());
    }
  }

  beginDriveSCurveDeceleration();
  integratedTurnBrakeCommandIssued = true;
}

/*
 * 明确旋转的实时IMU控制器。
 *
 * 旋转尚未停车时：
 *   1. 实时读取目标角与当前角之差；
 *   2. 从四轮当前最大绝对步频估算底盘当前角速度；
 *   3. 按当前梯形规划的v²/(2a)估算制动脉冲，再换算成制动角；
 *   4. 叠加IMU/循环延迟期间的预计转角和固定小余量，提前开始制动。
 *
 * 四轮停下后直接用IMU判断同一个用户设定目标。若误差仍超过0.15°，
 * startTurnResidualCorrection()发出的低速小角度脉冲仍由本控制器监控和提前
 * 制动；达到容差后直接完成本路线段，不进入独立的段后IMU锁向状态。
 */
void serviceIntegratedTurnCommand() {
  if (routePhase != ROUTE_PHASE_MOTION ||
      !integratedTurnControlActive) {
    routeFault("integrated turn state missing");
    return;
  }

  if (!imuIsFresh()) {
    routeFault("IMU data timeout during integrated turn");
    return;
  }

  if (millis() - commandStartMs > MOTION_TIMEOUT_MS) {
    routeFault("integrated turn timeout");
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

    /*
     * stepRate/pulsesPerDegree是预计角速度；当前规划制动距离先得到pulse，再
     * 除pulsesPerDegree得到预计制动角。使用四轮最大步频可避免其中一轮
     * 需要更长制动距离却被平均值掩盖。
     */
    const float predictedAngularRateDegreesPerSecond =
        maximumAbsoluteStepRate /
        activeTurnPulsesPerDegree;
    const float predictedBrakingDegrees =
        activeProfileStoppingSteps(
            maximumAbsoluteStepRate) /
        activeTurnPulsesPerDegree;
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

  const float stoppedErrorDegrees =
      headingErrorDegrees();
  if (fabsf(stoppedErrorDegrees) <=
      TURN_HEADING_TOLERANCE_DEGREES) {
    SerialDebug.print(
        "[TURN IMU] target reached, target/actual/error=");
    SerialDebug.print(
        targetCounterClockwiseHeadingDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.print(currentRouteHeadingDegrees(), 2);
    SerialDebug.print("/");
    SerialDebug.println(stoppedErrorDegrees, 2);

    integratedTurnControlActive = false;
    integratedTurnBrakeCommandIssued = false;
    integratedTurnDirectionSign = 0;
    advanceRoute();
    return;
  }

  /*
   * 粗旋转已停但还有残差：仍在当前旋转段内部低速补偿。每次最多12°，
   * 停下后本函数会再次读取IMU；目标始终是用户设置的累计角，而不是某个
   * 固定的0°、90°或180°。
   */
  integratedTurnBrakeCommandIssued = false;
  integratedTurnDirectionSign =
      stoppedErrorDegrees >= 0.0f ? +1 : -1;
  activeTurnPulsesPerDegree =
      rotationPulsesPerDegree(
          integratedTurnDirectionSign);
  startTurnResidualCorrection(stoppedErrorDegrees);
  hmiSetRunStatus("TURNIMU");
}

void serviceOrdinaryCommand() {
  if (routePhase != ROUTE_PHASE_MOTION) {
    return;
  }

#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 0
  /*
   * main_v0第一阶段只等待纯平移move()/run()目标到位，期间不读取航向来
   * 修改轮速或位置目标。到位后才启动本段原地IMU闭环；若已在容差内，
   * servicePostTranslationHeadingCorrection()会在同一轮直接完成，不等待。
   */
  if (postTranslationHeadingCorrectionActive) {
    servicePostTranslationHeadingCorrection();
    return;
  }
  if (!allMotorsArrived()) {
    if (millis() - commandStartMs > MOTION_TIMEOUT_MS) {
      routeFault("chassis translation timeout");
    }
    return;
  }
#else
  /*
   * main每轮同时服务软速度规划与实时航向。runSpeed()没有“位置目标到达”
   * 瞬间；完成条件由纯平移投影、IMU误差和纠偏角速度共同决定。
   */
  if (!serviceTranslationHeadingControl()) {
    return;
  }
#endif

  /*
   * 第14段第二次返回原料区C后，先等纯平移硬位移完成，
   * 再把目标航向切换成第7段参考朝向的最近等效角。目标切换后不增加固定
   * 延时：main继续使用共同旋转轮速；main_v0随即进入段后原地闭环。
   *
   *   - 名义平移已经完成，基础平移轮速为0；
   *   - 控制器只叠加所需的小角度旋转轮速；
   *   - 航向误差和纠偏角速度都合格后，下面的普通完成判定才会放行；
   *   - 因此该修正发生在第二次原料区停车内，第15段不会边横移边补差。
   */
  if (route[routeIndex].specificationStep == 14U &&
      !step15HeadingMatchAppliedAtSecondRawArea &&
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
      translationBaseDisplacementComplete) {
#else
      allMotorsArrived()) {
#endif
    if (!step7RawAreaHeadingReferenceValid) {
      routeFault("step 7 heading reference missing");
      return;
    }

    const float previousTargetDegrees =
        targetCounterClockwiseHeadingDegrees;
    const float matchedTargetDegrees =
        nearestEquivalentHeadingDegrees(
            step7RawAreaHeadingReferenceDegrees,
            currentRouteHeadingDegrees());
    targetCounterClockwiseHeadingDegrees =
        matchedTargetDegrees;
    step15HeadingMatchAppliedAtSecondRawArea = true;

    SerialDebug.print(
        "[STEP15 PRE-ALIGN] step7/previous/matched/actual/delta deg=");
    SerialDebug.print(
        step7RawAreaHeadingReferenceDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.print(previousTargetDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.print(matchedTargetDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.print(currentRouteHeadingDegrees(), 2);
    SerialDebug.print("/");
    SerialDebug.println(
        matchedTargetDegrees -
            currentRouteHeadingDegrees(),
        2);
  }

#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 0
  beginPostTranslationHeadingCorrection(
      POST_TRANSLATION_ADVANCE_ROUTE);
  servicePostTranslationHeadingCorrection();
  return;
#else
  const bool activeCommandIsStep04 =
      route[routeIndex].specificationStep == 4U;
  const bool step04TerminalImuMinimumTimeSatisfied =
      !activeCommandIsStep04 ||
      (translationTerminalCorrectionStartValid &&
       millis() - translationTerminalCorrectionStartMs >=
           STEP_04_TERMINAL_IMU_MINIMUM_DURATION_MS);
  if (!translationBaseDisplacementComplete ||
      fabsf(headingErrorDegrees()) >
          activeTranslationHeadingToleranceDegrees() ||
      fabsf(translationHeadingCorrectionRateDps) >
          TRANSLATION_COMPLETION_RATE_TOLERANCE_DPS ||
      !step04TerminalImuMinimumTimeSatisfied) {
    if (millis() - commandStartMs >
        MOTION_TIMEOUT_MS) {
      routeFault("chassis motion timeout");
    }
    return;
  }

  if (activeCommandIsStep04) {
    SerialDebug.print(
        "[STEP4->5 IMU WINDOW] done, elapsed/target/actual/error=");
    SerialDebug.print(
        millis() - translationTerminalCorrectionStartMs);
    SerialDebug.print(" ms/");
    SerialDebug.print(
        targetCounterClockwiseHeadingDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.print(currentRouteHeadingDegrees(), 2);
    SerialDebug.print("/");
    SerialDebug.println(headingErrorDegrees(), 2);
  }

  /*
   * 四轮已经到达“平移终点+实际执行的累计纠偏脉冲”，航向误差在死区内，
   * 且纠偏角速度已经回到近零。本段直接完成，不进入独立IMU矫正阶段。
   */
  if (route[routeIndex].specificationStep == 14U &&
      step15HeadingMatchAppliedAtSecondRawArea) {
    SerialDebug.print(
        "[STEP15 PRE-ALIGN DONE] target/actual/error deg=");
    SerialDebug.print(
        targetCounterClockwiseHeadingDegrees, 2);
    SerialDebug.print("/");
    SerialDebug.print(currentRouteHeadingDegrees(), 2);
    SerialDebug.print("/");
    SerialDebug.println(headingErrorDegrees(), 2);
  }
  advanceRoute();
#endif
}

void updateRoute() {
  if (programState != PROGRAM_RUNNING ||
      routeIndex >= ROUTE_COMMAND_COUNT) {
    return;
  }

  if (!commandStarted) {
    startCurrentCommand();
    return;
  }

  if (route[routeIndex].type == COMMAND_SCAN_SLOW) {
    serviceScanCommand();
  } else if (commandIsTurn(route[routeIndex].type)) {
    serviceIntegratedTurnCommand();
  } else {
    serviceOrdinaryCommand();
  }
}

// ===========================================================================
// 14. 一键启动、长按急停和周期服务
// ===========================================================================

void onStartButtonClick() {
  SerialDebug.print("[INPUT] PB9 click, state=");
  SerialDebug.println(programStateName(programState));

  /*
   * 一次上电只允许从 WAITING 启动一次。FINISHED/FAULT 后若要重跑，
   * 需要人工把车放回选定启停区并重新上电，避免在未知位置误启动路线。
   */
  if (programState == PROGRAM_WAITING) {
    startRequested = true;
    abortRequested = false;
  }
}

void onStartButtonLongPress() {
  abortRequested = true;
}

void beginRoute() {
  if (programState != PROGRAM_WAITING) {
    startRequested = false;
    return;
  }

  /*
   * 路线所有运动段都实时依赖IMU，因此一键启动后必须先等到一帧校验正确
   * 且新鲜的0x53航向数据。等待期间底盘保持失能。
   */
  if (!imuIsFresh()) {
    hmiSetRunStatus("IMUWAIT");
    return;
  }

  // 软件位置清零只建立本次路线相对坐标，不是机械/场地绝对定位。
  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->setCurrentPosition(0L);
  }

  routeIndex = 0U;
  routePhase = ROUTE_PHASE_IDLE;
  commandStarted = false;
  scanOriginValid = false;
  scanDistanceCaptured = false;
  scanDistanceBmm = 0.0f;
  scanCommandedMaximumDistanceMm = 0.0f;
  step7RawAreaHeadingReferenceDegrees = 0.0f;
  step7RawAreaHeadingReferenceValid = false;
  step15HeadingMatchAppliedAtSecondRawArea = false;
  resetQrReceiver();

  /*
   * 发车瞬间的实际IMU航向作为本次路线参考角。下面的目标0°表示“保持
   * 发车时的朝向”，不是强迫IMU回到传感器绝对0°或场地绝对0°。
   * 后续每次明确旋转再把该段实际设定转角累计到目标中。
   */
  routeImuReferenceDegrees =
      imuCounterClockwiseDegrees;
  targetCounterClockwiseHeadingDegrees = 0.0f;

  setDriveMotionProfile(
      FAST_MAXIMUM_STEP_RATE,
      FAST_STEP_ACCELERATION);
  enableDriveMotors();

  routeRunStartMs = millis();
  lastMotionTelemetryMs = routeRunStartMs;
  programState = PROGRAM_RUNNING;
  startRequested = false;
  hmiSetText("t1", "QRWAIT");
  hmiSetText("t3", "000+000+");
  hmiSetText("t8", "000+000");
  hmiSetRunStatus("RUN");

  SerialDebug.print("[RUN START] start zone=");
  SerialDebug.print(START_ZONE_SELECTION);
  SerialDebug.print(", longitudinal sign=");
  SerialDebug.print(selectedStartZoneDirection(), 0);
  SerialDebug.print(", step1 lateral mm=");
  SerialDebug.print(
      static_cast<float>(
          STEP_01_ADDITIONAL_LATERAL_MM),
      1);
  SerialDebug.print(", IMU reference=");
  SerialDebug.print(routeImuReferenceDegrees, 2);
  SerialDebug.println(" deg");
}

void abortRoute() {
  const uint8_t stoppedStep =
      routeIndex < ROUTE_COMMAND_COUNT
          ? route[routeIndex].specificationStep
          : 0U;
  const uint32_t elapsedMs =
      millis() - commandStartMs;
  const float actualHeading =
      currentRouteHeadingDegrees();
  const float headingError =
      headingErrorDegrees();

  disableDriveMotors();
  programState = PROGRAM_FAULT;
  routePhase = ROUTE_PHASE_IDLE;
  commandStarted = false;
  startRequested = false;
  abortRequested = false;
  hmiSetRunStatus("STOP");

  SerialDebug.print("[RUN ABORT] step=");
  SerialDebug.print(stoppedStep);
  SerialDebug.print(
      "/21, reason=PB9 long press, elapsed/heading/error=");
  SerialDebug.print(elapsedMs);
  SerialDebug.print("/");
  SerialDebug.print(actualHeading, 2);
  SerialDebug.print("/");
  SerialDebug.println(headingError, 2);
}

void updateHmiYaw() {
  if (!imuInitialized) {
    return;
  }
  const uint32_t nowMs = millis();
  if (nowMs - lastHmiYawMs < HMI_YAW_INTERVAL_MS) {
    return;
  }
  lastHmiYawMs = nowMs;
  hmiSetValue(
      "x0",
      static_cast<int32_t>(
          lroundf(currentRouteHeadingDegrees() * 100.0f)));
}

void initializeMotorOutputs() {
  /*
   * 公共 EN 必须先写高再设 OUTPUT，确保初始化期间保持失能。
   * setup() 不会触碰任何机械臂引脚或串口。
   */
  digitalWrite(DRIVE_ENABLE_PIN, HIGH);
  pinMode(DRIVE_ENABLE_PIN, OUTPUT);

  for (uint8_t i = 0U; i < 4U; ++i) {
    motors[i]->enableOutputs();
    motors[i]->setMinPulseWidth(MINIMUM_STEP_WIDTH_US);
    motors[i]->setCurrentPosition(0L);
  }

  setDriveMotionProfile(
      FAST_MAXIMUM_STEP_RATE,
      FAST_STEP_ACCELERATION);
  disableDriveMotors();
}

void initializeHmi() {
  delay(300);
  while (SerialHmi.available()) {
    SerialHmi.read();
  }

  hmiCommand("page main");
  delay(50);
  hmiSetText("t1", "QRWAIT");
  hmiSetText("t3", "000+000+");
  hmiSetText("t8", "000+000");
  hmiSetRunStatus(
      START_ZONE_SELECTION == 1U ? "READY1" : "READY2");
  hmiSetValue("x0", 0);
  hmiSetValue("x1", 0);
}

void printStartupConfiguration() {
  SerialDebug.println(
      "=== GongChuang chassis-only route ===");
  SerialDebug.println(
      "[BOOT] arm M5/M6/M7, gripper, tray and MaixCAM are not initialized");
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  SerialDebug.println(
      "[BOOT] firmware=main, translation=hard-distance + hard-heading, signed wheel-rate priority");
  SerialDebug.println(
      "[BOOT] terminal stability delay=0 ms; translation progress never commands reverse");
#else
  SerialDebug.println(
      "[BOOT] firmware=main_v0, translation=no in-motion IMU + post-segment IMU correction");
  SerialDebug.println(
      "[BOOT] terminal stability delay=0 ms; explicit turns keep integrated IMU braking");
#endif
  SerialDebug.print("[BOOT] selected start zone=");
  SerialDebug.println(START_ZONE_SELECTION);
  SerialDebug.print(
      "[BOOT] distances a/Aoffset/c/d/e/f/g/FtoA/final=");
  SerialDebug.print(DISTANCE_A_MM);
  SerialDebug.print("/");
  SerialDebug.print(SCAN_START_TO_POINT_A_MM);
  SerialDebug.print("/");
  SerialDebug.print(DISTANCE_C_MM);
  SerialDebug.print("/");
  SerialDebug.print(DISTANCE_D_MM);
  SerialDebug.print("/");
  SerialDebug.print(DISTANCE_E_MM);
  SerialDebug.print("/");
  SerialDebug.print(DISTANCE_F_MM);
  SerialDebug.print("/");
  SerialDebug.print(DISTANCE_G_MM);
  SerialDebug.print("/");
  SerialDebug.print(STORAGE_F_TO_SCAN_A_MM);
  SerialDebug.print("/");
  SerialDebug.println(POINT_A_TO_START_ZONE_MM);
  SerialDebug.print(
      "[BOOT] step1 right-lateral/step4 right mm=");
  SerialDebug.print(
      static_cast<float>(
          STEP_01_ADDITIONAL_LATERAL_MM),
      1);
  SerialDebug.print("/");
  SerialDebug.println(STEP_04_RIGHT_DISTANCE_MM);
  SerialDebug.print("[BOOT] stop after step=");
  SerialDebug.print(STOP_AFTER_STEP);
  SerialDebug.println(
      STOP_AFTER_STEP == 0U
          ? " (full route)"
          : " (configured step stop)");
  SerialDebug.println("[BOOT] independent step motion scales:");
  for (size_t i = 0U; i < ROUTE_COMMAND_COUNT; ++i) {
    SerialDebug.print("[BOOT]   step ");
    SerialDebug.print(route[i].specificationStep);
    SerialDebug.print(" = ");
    SerialDebug.println(route[i].motionScale, 3);
  }
  SerialDebug.print(
      "[BOOT] step 7 lateral anti-slip speed/acc scale=");
  SerialDebug.print(STEP_07_LATERAL_MAX_SPEED_SCALE, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      STEP_07_LATERAL_ACCELERATION_SCALE, 2);
  SerialDebug.print(
      "[BOOT] step 15 lateral anti-slip speed/acc scale=");
  SerialDebug.print(STEP_15_LATERAL_MAX_SPEED_SCALE, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      STEP_15_LATERAL_ACCELERATION_SCALE, 2);
  SerialDebug.print(
      "[BOOT] non-7/15 second speed/acc scale=");
  SerialDebug.print(
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE, 2);
  SerialDebug.print(
      "[BOOT] second angular speed/acc scale=");
  SerialDebug.print(
      ADDITIONAL_ANGULAR_MAXIMUM_SPEED_SCALE, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      ADDITIONAL_ANGULAR_ACCELERATION_SCALE, 2);
  SerialDebug.print(
      "[BOOT] final all speed/acc scale=");
  SerialDebug.print(FINAL_ALL_MAXIMUM_SPEED_SCALE, 2);
  SerialDebug.print("/");
  SerialDebug.println(FINAL_ALL_ACCELERATION_SCALE, 2);
  SerialDebug.print(
      "[BOOT] latest maximum-speed-only increase scale=");
  SerialDebug.println(
      LATEST_MAXIMUM_SPEED_INCREASE_SCALE, 2);
  SerialDebug.print(
      "[BOOT] latest global speed/acc scale=");
  SerialDebug.print(
      LATEST_GLOBAL_MAXIMUM_SPEED_SCALE, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      LATEST_GLOBAL_ACCELERATION_SCALE, 2);
  SerialDebug.print(
      "[BOOT] latest linear-only speed/acc scale=");
  SerialDebug.print(
      LATEST_LINEAR_MAXIMUM_SPEED_SCALE, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      LATEST_LINEAR_ACCELERATION_SCALE, 2);
  SerialDebug.print(
      "[BOOT] additional linear-only speed/acc scale=");
  SerialDebug.print(
      LATEST_ADDITIONAL_LINEAR_MAXIMUM_SPEED_SCALE, 3);
  SerialDebug.print("/");
  SerialDebug.println(
      LATEST_ADDITIONAL_LINEAR_ACCELERATION_SCALE, 3);
  SerialDebug.print(
      "[BOOT] trapezoid-only final speed/acc scale=");
  SerialDebug.print(
      TRAPEZOID_MAXIMUM_SPEED_REDUCTION_SCALE, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      TRAPEZOID_FINAL_ACCELERATION_SCALE, 2);
  SerialDebug.print(
      "[BOOT] latest trapezoid acceleration multiplier=");
  SerialDebug.println(
      LATEST_TRAPEZOID_ACCELERATION_SCALE, 2);
  SerialDebug.print(
      "[BOOT] trapezoid/S-curve peak deceleration scale=");
  SerialDebug.print(
      TRAPEZOID_DECELERATION_ACCELERATION_SCALE, 3);
  SerialDebug.print("/");
  SerialDebug.println(
      S_CURVE_DECELERATION_ACCELERATION_SCALE, 3);
  SerialDebug.print(
      "[BOOT] S-curve decel-gentleness/min-speed/min-acc=");
  SerialDebug.print(
      S_CURVE_DECELERATION_GENTLENESS, 2);
  SerialDebug.print("/");
  SerialDebug.print(S_CURVE_MINIMUM_SPEED_SCALE, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      S_CURVE_MINIMUM_ACCELERATION_SCALE, 2);
  SerialDebug.print(
      "[BOOT] translation heading deadband/Kp/max-rate/rate-slew/max-total=");
  SerialDebug.print(
      TRANSLATION_HEADING_DEADBAND_DEGREES, 2);
  SerialDebug.print("/");
  SerialDebug.print(
      TRANSLATION_HEADING_KP_PER_SECOND, 2);
  SerialDebug.print("/");
  SerialDebug.print(
      TRANSLATION_HEADING_MAXIMUM_RATE_DPS, 1);
  SerialDebug.print("/");
  SerialDebug.print(
      TRANSLATION_HEADING_RATE_SLEW_DPS2, 1);
  SerialDebug.print("/");
  SerialDebug.print(
      TRANSLATION_MAXIMUM_ACCUMULATED_CORRECTION_DEGREES,
      1);
  SerialDebug.println();
  SerialDebug.print(
      "[BOOT] terminal heading Kp/min-rate/base-min-scale=");
  SerialDebug.print(
      TRANSLATION_HEADING_KP_PER_SECOND *
          TRANSLATION_TERMINAL_KP_SCALE,
      2);
  SerialDebug.print("/");
  SerialDebug.print(
      TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_DPS,
      2);
  SerialDebug.print("/");
  SerialDebug.print(
      TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_BASE_DPS,
      2);
  SerialDebug.print("x");
  SerialDebug.println(
      TRANSLATION_TERMINAL_MINIMUM_CORRECTION_RATE_SCALE,
      2);
  SerialDebug.println(
      "[BOOT] route fixed inter-segment pure wait ms=0");
  SerialDebug.print(
      "[BOOT] step4->5 terminal IMU minimum window ms=");
  SerialDebug.println(
      STEP_04_TERMINAL_IMU_MINIMUM_DURATION_MS);
  SerialDebug.print(
      "[BOOT] all-translation in-motion/end IMU correction=");
  SerialDebug.print(
      ENABLE_IN_MOTION_IMU_CORRECTION
          ? "enabled/terminal"
          : "disabled/terminal");
  SerialDebug.print(", step1 override-if-enabled=");
  SerialDebug.println(
      STEP_01_ENABLE_IN_MOTION_IMU_CORRECTION
          ? "enabled"
          : "disabled");
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  SerialDebug.print(
      "[BOOT] translation-priority/position-tolerance/min-terminal-rate=BASE/");
  SerialDebug.print(
      TRANSLATION_POSITION_TOLERANCE_STEPS, 2);
  SerialDebug.print("/");
  SerialDebug.println(
      TRANSLATION_TERMINAL_MINIMUM_STEP_RATE, 1);
#endif
  SerialDebug.print(
      "[BOOT] step15 pre-alignment heading tolerance deg=");
  SerialDebug.println(
      STEP_15_PRE_ALIGNMENT_TOLERANCE_DEGREES, 2);
  SerialDebug.print(
      "[BOOT] lateral/longitudinal acceleration limit=");
  SerialDebug.println(
      LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL,
      2);
  SerialDebug.print(
      "[BOOT] ordinary lateral trapezoid acc/decel before step scale=");
  const float ordinaryLateralAcceleration =
      FAST_STEP_ACCELERATION *
      NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
      LATERAL_ACCELERATION_LIMIT_RELATIVE_TO_LONGITUDINAL *
      TRAPEZOID_FINAL_ACCELERATION_SCALE;
  SerialDebug.print(ordinaryLateralAcceleration, 1);
  SerialDebug.print("/");
  const float ordinaryLateralBaseDeceleration =
      ordinaryLateralAcceleration *
      TRAPEZOID_DECELERATION_ACCELERATION_SCALE;
  SerialDebug.println(
      ordinaryLateralBaseDeceleration, 1);
  SerialDebug.print(
      "[BOOT] integrated-turn latency/brake-margin=");
  SerialDebug.print(
      TURN_IMU_CONTROL_LATENCY_SECONDS, 3);
  SerialDebug.print(" s/");
  SerialDebug.print(
      TURN_PREDICTIVE_BRAKE_MARGIN_DEGREES, 2);
  SerialDebug.println(" deg");
  SerialDebug.print(
      "[BOOT] step 7 lateral actual vmax/acc=");
  SerialDebug.print(
      FAST_MAXIMUM_STEP_RATE *
          STEP_07_MOTION_SCALE *
          STEP_07_LATERAL_MAX_SPEED_SCALE,
      1);
  SerialDebug.print("/");
  SerialDebug.println(
      FAST_STEP_ACCELERATION *
          STEP_07_MOTION_SCALE *
          STEP_07_LATERAL_ACCELERATION_SCALE,
      1);
  SerialDebug.print(
      "[BOOT] step 15 lateral actual vmax/acc=");
  SerialDebug.print(
      FAST_MAXIMUM_STEP_RATE *
          STEP_15_MOTION_SCALE *
          STEP_15_LATERAL_MAX_SPEED_SCALE,
      1);
  SerialDebug.print("/");
  SerialDebug.println(
      FAST_STEP_ACCELERATION *
          STEP_15_MOTION_SCALE *
          STEP_15_LATERAL_ACCELERATION_SCALE,
      1);
  SerialDebug.print(
      "[BOOT] non-7/15 fast vmax/acc before step scale=");
  SerialDebug.print(
      FAST_MAXIMUM_STEP_RATE *
          NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
          TRAPEZOID_MAXIMUM_SPEED_REDUCTION_SCALE,
      1);
  SerialDebug.print("/");
  SerialDebug.println(
      FAST_STEP_ACCELERATION *
          NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
          TRAPEZOID_FINAL_ACCELERATION_SCALE,
      1);
  SerialDebug.print(
      "[BOOT] scan-before-QR vmax/acc before step scale=");
  SerialDebug.print(
      SCAN_MAXIMUM_STEP_RATE *
          NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
          TRAPEZOID_MAXIMUM_SPEED_REDUCTION_SCALE,
      1);
  SerialDebug.print("/");
  SerialDebug.println(
      SCAN_STEP_ACCELERATION *
          NON_07_15_LINEAR_PROFILE_INCREASE_SCALE *
          TRAPEZOID_FINAL_ACCELERATION_SCALE,
      1);
  SerialDebug.print(
      "[BOOT] coarse-turn vmax/acc before step scale=");
  SerialDebug.print(
      TURN_MAXIMUM_STEP_RATE *
          TRAPEZOID_MAXIMUM_SPEED_REDUCTION_SCALE,
      1);
  SerialDebug.print("/");
  SerialDebug.println(
      TURN_STEP_ACCELERATION *
          TRAPEZOID_FINAL_ACCELERATION_SCALE,
      1);
  SerialDebug.print("[BOOT] turn-residual vmax/acc=");
  SerialDebug.print(
      TURN_RESIDUAL_MAXIMUM_STEP_RATE *
          TRAPEZOID_MAXIMUM_SPEED_REDUCTION_SCALE,
      1);
  SerialDebug.print("/");
  SerialDebug.println(
      TURN_RESIDUAL_STEP_ACCELERATION *
          TRAPEZOID_FINAL_ACCELERATION_SCALE,
      1);
#if GONGCHUANG_TRANSLATION_CONTROL_MODE == 1
  SerialDebug.println(
      "[BOOT] post-translation independent IMU lock=disabled");
#else
  SerialDebug.println(
      "[BOOT] translation in-motion IMU=disabled; post-translation IMU lock=enabled");
#endif
  SerialDebug.print(
      "[BOOT] motion telemetry interval ms=");
  SerialDebug.println(MOTION_TELEMETRY_INTERVAL_MS);
  SerialDebug.println(
      "[BOOT] telemetry @step,elapsed,maxRemain,maxSpeed,speedCap,"
      "heading*100,error*100,cmdCorr*100,doneCorr*100,decel,imuAge");
}

} // namespace

void setup() {
  SerialDebug.begin(DEBUG_BAUDRATE);
  SerialHmi.begin(HMI_BAUDRATE);
  SerialImu.begin(IMU_BAUDRATE);
  SerialQr.begin(QR_BAUDRATE);

  printStartupConfiguration();

  analogReadResolution(12);
  pinMode(BATTERY_ADC_PIN, INPUT_ANALOG);
  initializeMotorOutputs();

  startButton.reset();
  startButton.attachClick(onStartButtonClick);
  startButton.attachLongPressStart(onStartButtonLongPress);

  initializeHmi();
  resetQrReceiver();

  SerialDebug.println(
      "[READY] click PB9 to start; long-press PB9 to stop");
}

void loop() {
  /*
   * runAllMotors()会按当前动作选择runSpeed()（main平移）或run()（明确
   * 旋转及main_v0平移），且必须高频、无条件服务。路线、按钮、二维码和
   * IMU全部采用非阻塞状态机；运行期间不要加入delay()、runToPosition()
   * 或其他长时间阻塞调用。
   */
  runAllMotors();
  startButton.tick();
  receiveImuData();
  receiveQrData();
  updateHmiYaw();
  serviceBatteryVoltage();

  if (abortRequested) {
    if (programState == PROGRAM_RUNNING ||
        (programState == PROGRAM_WAITING &&
         startRequested)) {
      abortRoute();
    } else {
      abortRequested = false;
    }
  }

  if (startRequested) {
    beginRoute();
  }

  updateRoute();
  serviceMotionTelemetry();
}
