#include <Arduino.h>
#include <DDSM210.h>
#include <MecanumKinematics.h>

// 将库中的类型引入当前作用域，后续可直接写 WheelSpeeds 等类型名。
using namespace mecanum;

/*
 * 项目现有 STM32H750 例程统一使用 PA3/PA2 与 DDSM210 通信：
 *
 *   PA3 = STM32 接收 RX
 *   PA2 = STM32 发送 TX
 *
 * HardwareSerial 构造函数的参数顺序是“RX 在前、TX 在后”。
 */
#define M0603C_RX PA3
#define M0603C_TX PA2

// 创建专用于四个 DDSM210 电机的硬件串口。
HardwareSerial Serial_M0603C(M0603C_RX, M0603C_TX);

// DDSM210 串口协议控制对象。
DDSM_CTRL dc;

/*
 * 创建本车的麦轮运动学模型。
 *
 * 三个参数依次为：
 * 1. 前后轮中心距 187.5 mm；
 * 2. 左右轮中心距 195.0 mm；
 * 3. 车轮直径 100.0 mm。
 *
 * 注意第三个参数是直径，不是半径。库会自动换算成米和半径。
 */
MecanumKinematics chassis =
    MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);

/*
 * 电机安装方向修正。
 *
 * 运动学中的正轮速表示“该轮推动底盘向前”，但电机实际安装方向可能相反。
 * 当前方向保留为：1号反向、2号正向、3号反向、4号正向。
 *
 * 正式使用前必须架空底盘，逐轮发送小命令确认方向；如方向不符，
 * 将对应位置的 +1 与 -1 互换。
 */
WheelDirections motorDirections(-1, +1, -1, +1);

/*
 * 控制周期。
 *
 * 项目原例程在 loop() 中连续发送命令；这里改成固定 20 ms 周期，
 * 即约 50 Hz。这样既能持续刷新电机命令，又不会无意义地占满 CPU。
 *
 * DDSM210 库会依次给 1～4号电机发送命令并读取反馈，因此实际一次调用
 * 如果超过 20 ms，下一次循环只会顺延，不会同时堆积多个控制任务。
 */
const uint32_t CONTROL_PERIOD_MS = 20;

// 每 200 ms 输出一次命令和反馈，避免串口打印影响控制周期。
const uint32_t TELEMETRY_PERIOD_MS = 200;

/*
 * 自动动作演示的总开关。
 *
 * 默认必须保持 false。只有在完成以下检查后才改为 true：
 *
 * 1. 将底盘架空，确保轮子不会带动车体突然移动；
 * 2. 确认四个电机 ID 依次为 1、2、3、4；
 * 3. 确认 motorDirections 中的四个方向正确；
 * 4. 确认 PA3/PA2 串口接线与电机供电正常。
 */
const bool ENABLE_AUTOMATIC_DEMO = false;

// 保存控制循环和遥测循环上一次执行的时间。
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;

// 记录自动演示开始时间，用于非阻塞状态切换。
uint32_t demoStartMs = 0;

// 保存最近一次发出的命令，供遥测打印使用。
DDSM210Commands lastCommands;

/**
 * @brief 将一个底盘速度目标完整转换并发送给四个 DDSM210 电机。
 *
 * 调用链：
 *
 * `底盘 vx/vy/wz`
 * → 麦轮逆运动学
 * → 四轮 rad/s
 * → 四轮 RPM
 * → 安装方向修正及 ±2100 等比例限幅
 * → `ddsm210_ctrl_4()` 串口发送。
 *
 * @param target 底盘目标速度。vx/vy 单位 m/s，wz 单位 rad/s。
 * @return 本次实际发送给 ID 1～4 电机的原始命令。
 */
DDSM210Commands sendChassisVelocity(const ChassisVelocity &target) {
  // 第一步：底盘速度通过逆运动学换算为四轮物理角速度。
  const WheelSpeeds wheelRadPerSec = chassis.inverse(target);

  // 第二步：将 rad/s 转为 DDSM210 速度环使用的 RPM。
  const WheelSpeeds wheelRpm =
      MecanumKinematics::radPerSecToRpm(wheelRadPerSec);

  /*
   * 第三步：根据电机安装方向生成整数命令。
   * 如果任一轮超过 ±2100，四轮会被同比例缩小。
   */
  const DDSM210Commands commands =
      MecanumKinematics::toDDSM210Commands(wheelRpm, motorDirections);

  /*
   * 第四步：按照电机 ID 顺序发送。
   *
   * 项目例程原来的 RunMotors() 最终也是调用这个函数。
   * 第五个参数 act 使用库的默认值 1。
   */
  dc.ddsm210_ctrl_4(commands.wheel1, commands.wheel2, commands.wheel3,
                    commands.wheel4);

  return commands;
}

/**
 * @brief 根据自动演示已经运行的时间，返回当前阶段的底盘目标速度。
 *
 * 演示动作只执行一次，不循环：
 *
 * - 0～3 s：停止，留出人员撤离和检查时间；
 * - 3～5 s：以 0.20 m/s 向前；
 * - 5～6 s：停止；
 * - 6～8 s：以 0.20 m/s 向左；
 * - 8～9 s：停止；
 * - 9～11 s：以 0.60 rad/s 逆时针旋转；
 * - 11 s 后：永久停止，直到复位。
 *
 * @param elapsedMs 从演示开始到现在经过的毫秒数。
 * @return 当前阶段的 vx/vy/wz 目标。
 */
ChassisVelocity automaticDemoTarget(uint32_t elapsedMs) {
  if (!ENABLE_AUTOMATIC_DEMO) {
    // 自动演示未开启时始终返回零，底盘保持停止。
    return ChassisVelocity();
  }

  if (elapsedMs < 3000) {
    return ChassisVelocity();
  }
  if (elapsedMs < 5000) {
    return ChassisVelocity(0.20f, 0.0f, 0.0f); // 向前
  }
  if (elapsedMs < 6000) {
    return ChassisVelocity();
  }
  if (elapsedMs < 8000) {
    return ChassisVelocity(0.0f, 0.20f, 0.0f); // 向左
  }
  if (elapsedMs < 9000) {
    return ChassisVelocity();
  }
  if (elapsedMs < 11000) {
    return ChassisVelocity(0.0f, 0.0f, 0.60f); // 逆时针旋转
  }

  // 整套动作完成后不再重复，持续发送零速命令。
  return ChassisVelocity();
}

/**
 * @brief 打印最近一次命令及 DDSM210 库保存的四轮反馈速度。
 *
 * `speed_data_4[]` 由 `ddsm210_ctrl_4()` 在依次控制四个电机时更新。
 * 如果某个电机没有在超时时间内返回有效反馈，当前 DDSM210 库会用
 * 对应目标命令代替该次 speed_data，因此该值不能直接等同于可靠编码器测量。
 */
void printTelemetry() {
  Serial.print("CMD: ");
  Serial.print(lastCommands.wheel1);
  Serial.print(", ");
  Serial.print(lastCommands.wheel2);
  Serial.print(", ");
  Serial.print(lastCommands.wheel3);
  Serial.print(", ");
  Serial.print(lastCommands.wheel4);

  Serial.print(" | FB: ");
  Serial.print(dc.speed_data_4[0]);
  Serial.print(", ");
  Serial.print(dc.speed_data_4[1]);
  Serial.print(", ");
  Serial.print(dc.speed_data_4[2]);
  Serial.print(", ");
  Serial.println(dc.speed_data_4[3]);
}

void setup() {
  // 调试输出串口，仅用于观察命令与反馈，不参与电机通信。
  Serial.begin(115200);

  /*
   * 按项目原有 STM32_M0603C 例程初始化电机串口：
   * 115200 波特率、PA3 RX、PA2 TX。
   */
  Serial_M0603C.begin(DDSM_BAUDRATE);

  // 告诉 DDSM210 库后续所有命令应从哪个串口发送。
  dc.pSerial = &Serial_M0603C;

  // 明确选择 DDSM210 类型；构造函数默认也是 DDSM210。
  dc.set_ddsm_type(TYPE_DDSM210);

  // 等待电机、收发器和串口稳定，与项目原例程保持一致。
  delay(100);

  // 清除上电期间可能残留在接收缓冲区中的无效字节。
  dc.clear_ddsm_buffer();

  // 记录自动演示的时间起点。
  demoStartMs = millis();
  lastControlMs = demoStartMs;
  lastTelemetryMs = demoStartMs;

  /*
   * 上电后立即发送一次零速度，确保四轮处于停止状态。
   * 自动演示默认关闭，因此后续 loop() 也会持续发送零速度。
   */
  lastCommands = sendChassisVelocity(ChassisVelocity());
}

void loop() {
  /*
   * 使用 millis() 实现非阻塞定时。
   *
   * 不使用 delay(20)，这样后续加入 IMU、遥控接收、串口屏或其他状态机时，
   * 这些任务仍能在两个控制周期之间运行。
   */
  const uint32_t nowMs = millis();

  if (static_cast<uint32_t>(nowMs - lastControlMs) >= CONTROL_PERIOD_MS) {
    /*
     * 直接记录当前时间，而不是连续补发历史周期。
     * 如果某次 DDSM 通信耗时较长，控制循环会自然顺延，避免短时间内堆积命令。
     */
    lastControlMs = nowMs;

    // 根据当前演示阶段获得目标速度；演示关闭时该值恒为零。
    const ChassisVelocity target =
        automaticDemoTarget(static_cast<uint32_t>(nowMs - demoStartMs));

    // 完成运动学换算并给四个电机发送本周期命令。
    lastCommands = sendChassisVelocity(target);
  }

  /*
   * 遥测打印使用独立的低频周期，不阻塞或改变电机控制节奏。
   */
  if (static_cast<uint32_t>(nowMs - lastTelemetryMs) >=
      TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry();
  }
}
