#include "MecanumKinematics.h"

#include <math.h>

namespace mecanum {
namespace {

// 单精度圆周率。嵌入式运动学使用 float 即可满足本底盘精度需求。
const float kPi = 3.14159265358979323846f;

// rad/s 转 RPM 的固定换算系数：60 / (2π)。
const float kRadPerSecToRpm = 60.0f / (2.0f * kPi);

// RPM 转 rad/s 的固定换算系数：2π / 60。
const float kRpmToRadPerSec = (2.0f * kPi) / 60.0f;

/**
 * @brief 求单个浮点数的绝对值。
 *
 * 这里使用一个很小的内部函数，避免限幅逻辑重复书写正负判断。
 * 该函数只在本 .cpp 文件内部使用，不属于对外 API。
 */
float absoluteValue(float value) { return value < 0.0f ? -value : value; }

/**
 * @brief 找出四轮数值中的最大绝对值。
 *
 * 用途：判断四轮中是否有任意一个超过电机能力，并计算统一缩放比例。
 *
 * @param values 四轮数值，可以是 rad/s、RPM 或原始电机命令。
 * @return 四个数的最大绝对值。
 */
float largestMagnitude(const WheelSpeeds &values) {
  float largest = absoluteValue(values.frontLeft);
  const float frontRight = absoluteValue(values.frontRight);
  const float rearLeft = absoluteValue(values.rearLeft);
  const float rearRight = absoluteValue(values.rearRight);

  if (frontRight > largest) {
    largest = frontRight;
  }
  if (rearLeft > largest) {
    largest = rearLeft;
  }
  if (rearRight > largest) {
    largest = rearRight;
  }
  return largest;
}

/**
 * @brief 将浮点命令四舍五入成 int16_t，并再次执行最终安全限幅。
 *
 * 正数加 0.5、负数减 0.5 后再转为整数，可以实现对称的四舍五入。
 * 即使上层限幅逻辑将来发生变化，此处仍确保返回值不会超过
 * `[-maximumCommand, +maximumCommand]`。
 *
 * @param value 要转换的浮点命令。
 * @param maximumCommand 最大允许绝对值。
 * @return 四舍五入后的 DDSM210 整数命令。
 */
int16_t roundedCommand(float value, int16_t maximumCommand) {
  float limited = value;
  if (limited > maximumCommand) {
    limited = maximumCommand;
  } else if (limited < -maximumCommand) {
    limited = -maximumCommand;
  }

  return static_cast<int16_t>(limited >= 0.0f ? limited + 0.5f
                                               : limited - 0.5f);
}

} // namespace

/**
 * @brief 使用米制参数初始化运动学模型。
 *
 * `rotationLeverArmMeters_` 是旋转项使用的几何参数：
 *
 * @code
 * L = 轴距 / 2 + 轮距 / 2 = (轴距 + 轮距) / 2
 * @endcode
 *
 * 本车的 L 为 `(0.1875 + 0.195) / 2 = 0.19125 m`。
 */
MecanumKinematics::MecanumKinematics(float wheelbaseMeters,
                                     float trackWidthMeters,
                                     float wheelRadiusMeters)
    : wheelbaseMeters_(wheelbaseMeters),
      trackWidthMeters_(trackWidthMeters),
      wheelRadiusMeters_(wheelRadiusMeters),
      rotationLeverArmMeters_((wheelbaseMeters + trackWidthMeters) * 0.5f) {}

/**
 * @brief 将机械尺寸从毫米转换为米，并使用车轮直径计算半径。
 *
 * - mm 转 m：乘以 0.001；
 * - 直径 mm 转半径 m：乘以 0.001 后再除以 2，即乘以 0.0005。
 */
MecanumKinematics
MecanumKinematics::fromMillimeters(float wheelbaseMm, float trackWidthMm,
                                   float wheelDiameterMm) {
  return MecanumKinematics(wheelbaseMm * 0.001f, trackWidthMm * 0.001f,
                           wheelDiameterMm * 0.0005f);
}

/**
 * @brief 检查三个基础尺寸是否都是有限正数。
 *
 * `isfinite()` 会排除 NaN 和正负无穷，随后再排除 0 与负数。
 * 无效参数会导致除零或没有物理意义，因此正逆运动学会先调用本函数。
 */
bool MecanumKinematics::isValid() const {
  return isfinite(wheelbaseMeters_) && isfinite(trackWidthMeters_) &&
         isfinite(wheelRadiusMeters_) && wheelbaseMeters_ > 0.0f &&
         trackWidthMeters_ > 0.0f && wheelRadiusMeters_ > 0.0f;
}

/** @brief 返回前后轮中心距，单位 m。 */
float MecanumKinematics::wheelbaseMeters() const {
  return wheelbaseMeters_;
}

/** @brief 返回左右轮中心距，单位 m。 */
float MecanumKinematics::trackWidthMeters() const {
  return trackWidthMeters_;
}

/** @brief 返回车轮半径，单位 m。 */
float MecanumKinematics::wheelRadiusMeters() const {
  return wheelRadiusMeters_;
}

/** @brief 返回旋转等效力臂 `(轴距 + 轮距) / 2`，单位 m。 */
float MecanumKinematics::rotationLeverArmMeters() const {
  return rotationLeverArmMeters_;
}

/**
 * @brief 执行 X 型麦轮底盘逆运动学。
 *
 * 对于轮号 1前左、2前右、3后左、4后右，公式为：
 *
 * @code
 * w1 = (vx - vy - L*wz) / r
 * w2 = (vx + vy + L*wz) / r
 * w3 = (vx + vy - L*wz) / r
 * w4 = (vx - vy + L*wz) / r
 * @endcode
 *
 * 其中：
 *
 * - `L` 是旋转等效力臂；
 * - `r` 是车轮半径；
 * - `w1～w4` 的单位为 rad/s。
 *
 * 由公式可验证几个基本动作：
 *
 * - 只前进：四轮全部为正；
 * - 只向左：轮速符号为 `- + + -`；
 * - 只逆时针旋转：轮速符号为 `- + - +`。
 */
WheelSpeeds
MecanumKinematics::inverse(const ChassisVelocity &velocity) const {
  // 参数无效时返回四轮全零，避免除以零并让底盘保持安全停止。
  if (!isValid()) {
    return WheelSpeeds();
  }

  // L*wz 的单位为 m/s，表示旋转在轮子位置产生的切向线速度。
  const float rotation = rotationLeverArmMeters_ * velocity.wz;

  // 最后统一乘以 1/r，将轮缘线速度换算成车轮角速度。
  const float inverseRadius = 1.0f / wheelRadiusMeters_;

  return WheelSpeeds(
      (velocity.vx - velocity.vy - rotation) * inverseRadius,
      (velocity.vx + velocity.vy + rotation) * inverseRadius,
      (velocity.vx + velocity.vy - rotation) * inverseRadius,
      (velocity.vx - velocity.vy + rotation) * inverseRadius);
}

/**
 * @brief 执行 X 型麦轮底盘正运动学。
 *
 * 正运动学是 `inverse()` 的逆变换，公式为：
 *
 * @code
 * vx = r/4    * ( w1 + w2 + w3 + w4)
 * vy = r/4    * (-w1 + w2 + w3 - w4)
 * wz = r/(4L) * (-w1 + w2 - w3 + w4)
 * @endcode
 *
 * 输入必须是经过电机方向修正后的物理轮速，单位 rad/s。
 * 返回 `vx/vy` 为 m/s，`wz` 为 rad/s。
 */
ChassisVelocity
MecanumKinematics::forward(const WheelSpeeds &wheelRadPerSec) const {
  // 无效几何参数无法完成换算，安全返回静止速度。
  if (!isValid()) {
    return ChassisVelocity();
  }

  // 平移速度公式的公共系数 r/4。
  const float linearFactor = wheelRadiusMeters_ * 0.25f;

  // 旋转速度公式的公共系数 r/(4L)。
  const float angularFactor =
      wheelRadiusMeters_ / (4.0f * rotationLeverArmMeters_);

  return ChassisVelocity(
      linearFactor *
          (wheelRadPerSec.frontLeft + wheelRadPerSec.frontRight +
           wheelRadPerSec.rearLeft + wheelRadPerSec.rearRight),
      // 横移项的符号由俯视 X 型滚子方向决定。
      linearFactor *
          (-wheelRadPerSec.frontLeft + wheelRadPerSec.frontRight +
           wheelRadPerSec.rearLeft - wheelRadPerSec.rearRight),
      // 旋转项：左侧轮和右侧轮产生相反方向的旋转贡献。
      angularFactor *
          (-wheelRadPerSec.frontLeft + wheelRadPerSec.frontRight -
           wheelRadPerSec.rearLeft + wheelRadPerSec.rearRight));
}

/**
 * @brief 把四轮角速度从 rad/s 转成 RPM。
 *
 * 四个轮子独立乘以相同换算系数，不改变各轮之间的比例和符号。
 */
WheelSpeeds
MecanumKinematics::radPerSecToRpm(const WheelSpeeds &wheelRadPerSec) {
  return WheelSpeeds(
      wheelRadPerSec.frontLeft * kRadPerSecToRpm,
      wheelRadPerSec.frontRight * kRadPerSecToRpm,
      wheelRadPerSec.rearLeft * kRadPerSecToRpm,
      wheelRadPerSec.rearRight * kRadPerSecToRpm);
}

/**
 * @brief 把四轮转速从 RPM 转成 rad/s。
 *
 * DDSM210 返回 RPM 后，可先调用本函数，再将结果交给 `forward()`。
 */
WheelSpeeds
MecanumKinematics::rpmToRadPerSec(const WheelSpeeds &wheelRpm) {
  return WheelSpeeds(wheelRpm.frontLeft * kRpmToRadPerSec,
                     wheelRpm.frontRight * kRpmToRadPerSec,
                     wheelRpm.rearLeft * kRpmToRadPerSec,
                     wheelRpm.rearRight * kRpmToRadPerSec);
}

/**
 * @brief 对四轮值进行同比例限幅。
 *
 * 举例：原始命令为 `{3000, 1500, -3000, -1500}`，限制为 2100，
 * 最大绝对值为 3000，因此统一缩放系数是 `2100/3000 = 0.7`，
 * 结果为 `{2100, 1050, -2100, -1050}`。
 *
 * 这样保留了所有轮子之间的速度比例，底盘合成方向不会因单轮硬裁剪而改变。
 */
float MecanumKinematics::limitPreservingRatio(WheelSpeeds &values,
                                              float maximumMagnitude) {
  // 限制值必须为有限正数；否则将四轮清零，优先保证安全。
  if (!isfinite(maximumMagnitude) || maximumMagnitude <= 0.0f) {
    values = WheelSpeeds();
    return 0.0f;
  }

  const float largest = largestMagnitude(values);

  // 输入中存在 NaN 或无穷时无法可靠控制，统一清零。
  if (!isfinite(largest)) {
    values = WheelSpeeds();
    return 0.0f;
  }

  // 最大值已经在限制内，或四轮本来就是零，无需缩放。
  if (largest <= maximumMagnitude || largest == 0.0f) {
    return 1.0f;
  }

  // 四轮同时乘以相同系数，保留轮速比例。
  const float scale = maximumMagnitude / largest;
  values.frontLeft *= scale;
  values.frontRight *= scale;
  values.rearLeft *= scale;
  values.rearRight *= scale;
  return scale;
}

/**
 * @brief 将物理轮速 RPM 转换为 DDSM210 原始速度命令。
 *
 * 本函数将“运动学层”和“电机驱动层”连接起来，但不会直接操作串口。
 * 调用者取得返回值后，再交给 `DDSM_CTRL::ddsm210_ctrl_4()` 发送。
 */
DDSM210Commands MecanumKinematics::toDDSM210Commands(
    const WheelSpeeds &physicalWheelRpm,
    const WheelDirections &motorDirections, int16_t maximumCommand,
    float commandUnitsPerRpm) {
  // 无效的命令范围或换算比例没有物理意义，安全返回四个停止命令。
  if (maximumCommand <= 0 || !isfinite(commandUnitsPerRpm) ||
      commandUnitsPerRpm <= 0.0f) {
    return DDSM210Commands();
  }

  /*
   * 第一步：物理 RPM × 电机方向系数 × 每 RPM 命令单位。
   *
   * 例如物理轮速为 +100 RPM、方向系数为 -1，
   * 则对应电机原始命令为 100 × (-1) × 10 = -1000。
   */
  WheelSpeeds rawCommands(
      physicalWheelRpm.frontLeft * motorDirections.frontLeft *
          commandUnitsPerRpm,
      physicalWheelRpm.frontRight * motorDirections.frontRight *
          commandUnitsPerRpm,
      physicalWheelRpm.rearLeft * motorDirections.rearLeft *
          commandUnitsPerRpm,
      physicalWheelRpm.rearRight * motorDirections.rearRight *
          commandUnitsPerRpm);

  // 第二步：四轮同比例限幅到 DDSM210 允许范围，避免改变底盘合成方向。
  limitPreservingRatio(rawCommands, static_cast<float>(maximumCommand));

  // 第三步：四舍五入成可直接发送的 int16_t 原始命令。
  return DDSM210Commands(
      roundedCommand(rawCommands.frontLeft, maximumCommand),
      roundedCommand(rawCommands.frontRight, maximumCommand),
      roundedCommand(rawCommands.rearLeft, maximumCommand),
      roundedCommand(rawCommands.rearRight, maximumCommand));
}

} // namespace mecanum
