#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#include <stdint.h>

namespace mecanum {

/**
 * @brief 底盘在“车体坐标系”中的目标速度或测量速度。
 *
 * 本库统一采用右手坐标系：
 *
 * - `vx > 0`：底盘向车头方向运动，单位 m/s；
 * - `vy > 0`：底盘向左运动，单位 m/s；
 * - `wz > 0`：从车顶向下看，底盘逆时针旋转，单位 rad/s。
 *
 * 该结构既可作为 `inverse()` 的输入，表示希望底盘怎样运动；
 * 也可作为 `forward()` 的返回值，表示根据四轮转速估算出的实际运动。
 */
struct ChassisVelocity {
  /** 车体前后方向的线速度，向前为正，单位 m/s。 */
  float vx;

  /** 车体左右方向的线速度，向左为正，单位 m/s。 */
  float vy;

  /** 车体绕竖直轴的角速度，俯视逆时针为正，单位 rad/s。 */
  float wz;

  /**
   * @brief 构造一个底盘速度。
   *
   * 三个参数均有默认值 0，因此 `ChassisVelocity()` 表示底盘静止。
   *
   * @param vxValue 向前线速度，单位 m/s。
   * @param vyValue 向左线速度，单位 m/s。
   * @param wzValue 俯视逆时针角速度，单位 rad/s。
   *
   * 示例：
   * @code
   * ChassisVelocity target(0.5f, 0.2f, 0.4f);
   * // 表示向前 0.5 m/s、向左 0.2 m/s，同时逆时针旋转 0.4 rad/s。
   * @endcode
   */
  ChassisVelocity(float vxValue = 0.0f, float vyValue = 0.0f,
                  float wzValue = 0.0f)
      : vx(vxValue), vy(vyValue), wz(wzValue) {}
};

/**
 * @brief 按固定轮号保存四个车轮的“物理轮速”。
 *
 * 轮号固定为：
 *
 * - `frontLeft`：1号轮，前左轮；
 * - `frontRight`：2号轮，前右轮；
 * - `rearLeft`：3号轮，后左轮；
 * - `rearRight`：4号轮，后右轮。
 *
 * 这里的“正转”是物理概念：该轮正转时应推动底盘向前。
 * 它不一定等于电机驱动器中的正命令，因为左右电机的安装方向可能相反。
 *
 * 该结构本身不固定单位：
 *
 * - `inverse()` 返回时单位为 rad/s；
 * - `radPerSecToRpm()` 返回时单位为 RPM；
 * - `limitPreservingRatio()` 可处理任意相同单位的四个数值。
 */
struct WheelSpeeds {
  /** 1号前左轮速度。 */
  float frontLeft;

  /** 2号前右轮速度。 */
  float frontRight;

  /** 3号后左轮速度。 */
  float rearLeft;

  /** 4号后右轮速度。 */
  float rearRight;

  /**
   * @brief 构造四轮速度。
   *
   * 所有参数默认为 0，因此 `WheelSpeeds()` 表示四轮全部停止。
   *
   * @param wheel1 1号前左轮速度。
   * @param wheel2 2号前右轮速度。
   * @param wheel3 3号后左轮速度。
   * @param wheel4 4号后右轮速度。
   */
  WheelSpeeds(float wheel1 = 0.0f, float wheel2 = 0.0f,
              float wheel3 = 0.0f, float wheel4 = 0.0f)
      : frontLeft(wheel1), frontRight(wheel2), rearLeft(wheel3),
        rearRight(wheel4) {}
};

/**
 * @brief 四个电机的安装方向修正系数。
 *
 * 运动学计算得到的是“车轮物理方向”，而 DDSM210 收到的是“电机原始方向”。
 * 两者之间通过本结构转换：
 *
 * @code
 * 原始电机命令 = 物理轮速 × 对应方向系数
 * @endcode
 *
 * 每个值通常只能取：
 *
 * - `+1`：电机命令正方向与车轮物理正方向一致；
 * - `-1`：电机命令正方向与车轮物理正方向相反。
 *
 * 当前底盘默认使用 `WheelDirections(-1, +1, -1, +1)`。
 * 正式使用前仍必须架空底盘逐轮校准，不能只依靠左右位置猜测。
 */
struct WheelDirections {
  /** 1号前左轮电机的方向修正系数。 */
  int8_t frontLeft;

  /** 2号前右轮电机的方向修正系数。 */
  int8_t frontRight;

  /** 3号后左轮电机的方向修正系数。 */
  int8_t rearLeft;

  /** 4号后右轮电机的方向修正系数。 */
  int8_t rearRight;

  /**
   * @brief 构造四轮电机方向配置。
   *
   * 默认方向为 `-1, +1, -1, +1`，依次对应 1～4 号电机。
   *
   * @param wheel1 1号前左轮方向，通常为 +1 或 -1。
   * @param wheel2 2号前右轮方向，通常为 +1 或 -1。
   * @param wheel3 3号后左轮方向，通常为 +1 或 -1。
   * @param wheel4 4号后右轮方向，通常为 +1 或 -1。
   */
  WheelDirections(int8_t wheel1 = -1, int8_t wheel2 = +1,
                  int8_t wheel3 = -1, int8_t wheel4 = +1)
      : frontLeft(wheel1), frontRight(wheel2), rearLeft(wheel3),
        rearRight(wheel4) {}
};

/**
 * @brief 按电机 ID 顺序保存四个 DDSM210 速度环原始命令。
 *
 * DDSM210 速度环通常使用：
 *
 * - `1000` 表示约 `100 RPM`；
 * - `-1000` 表示反向约 `100 RPM`；
 * - 有效范围通常限制在 `-2100～+2100`。
 *
 * 本结构可以直接传给 `DDSM_CTRL::ddsm210_ctrl_4()`。
 */
struct DDSM210Commands {
  /** 发送给 ID 1 电机的原始命令。 */
  int16_t wheel1;

  /** 发送给 ID 2 电机的原始命令。 */
  int16_t wheel2;

  /** 发送给 ID 3 电机的原始命令。 */
  int16_t wheel3;

  /** 发送给 ID 4 电机的原始命令。 */
  int16_t wheel4;

  /**
   * @brief 构造四个 DDSM210 原始命令。
   *
   * 所有参数默认为 0，因此 `DDSM210Commands()` 表示四轮停止。
   */
  DDSM210Commands(int16_t command1 = 0, int16_t command2 = 0,
                  int16_t command3 = 0, int16_t command4 = 0)
      : wheel1(command1), wheel2(command2), wheel3(command3),
        wheel4(command4) {}
};

/**
 * @brief 四轮 X 型麦克纳姆底盘运动学类。
 *
 * 适用的轮子排列为：从车顶向下看，四个麦轮滚子整体组成 X 形。
 *
 * @code
 *              车头
 *       1 前左       2 前右
 *          \           /
 *
 *          /           \
 *       3 后左       4 后右
 * @endcode
 *
 * 这个类只负责数学换算，不会初始化串口、不会直接发送电机命令，
 * 因而可以独立测试，也可以与 DDSM210 或其他电机驱动配合使用。
 */
class MecanumKinematics {
public:
  /**
   * @brief 使用“米”为单位构造底盘运动学对象。
   *
   * 注意：前两个参数都是两侧轮子中心之间的完整距离，不是半距；
   * 第三个参数是车轮半径，不是直径。
   *
   * @param wheelbaseMeters 前后轮中心距，单位 m。
   * @param trackWidthMeters 左右轮中心距，单位 m。
   * @param wheelRadiusMeters 车轮半径，单位 m。
   *
   * 对本车可写成：
   * @code
   * MecanumKinematics chassis(0.1875f, 0.195f, 0.05f);
   * @endcode
   */
  MecanumKinematics(float wheelbaseMeters, float trackWidthMeters,
                    float wheelRadiusMeters);

  /**
   * @brief 使用毫米尺寸便捷创建运动学对象。
   *
   * 该函数适合直接填写机械图纸上的尺寸。与构造函数不同，
   * 第三个参数填写的是车轮直径。
   *
   * @param wheelbaseMm 前后轮中心距，单位 mm。
   * @param trackWidthMm 左右轮中心距，单位 mm。
   * @param wheelDiameterMm 车轮直径，单位 mm。
   * @return 已完成单位换算的 `MecanumKinematics` 对象。
   *
   * 本车应使用：
   * @code
   * MecanumKinematics chassis =
   *     MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);
   * @endcode
   */
  static MecanumKinematics fromMillimeters(float wheelbaseMm,
                                           float trackWidthMm,
                                           float wheelDiameterMm);

  /**
   * @brief 检查底盘几何参数是否有效。
   *
   * 轴距、轮距、轮半径必须都是有限的正数。
   * 如果无效，`inverse()` 和 `forward()` 会安全地返回全零结果。
   *
   * @return 参数有效时返回 `true`，否则返回 `false`。
   */
  bool isValid() const;

  /** @return 构造时保存的前后轮中心距，单位 m。 */
  float wheelbaseMeters() const;

  /** @return 构造时保存的左右轮中心距，单位 m。 */
  float trackWidthMeters() const;

  /** @return 构造时保存的车轮半径，单位 m。 */
  float wheelRadiusMeters() const;

  /**
   * @brief 获取旋转运动学中的等效力臂。
   *
   * 计算公式：
   * `旋转力臂 = 前后轮中心距 / 2 + 左右轮中心距 / 2`。
   *
   * 本车结果为 `0.19125 m`。
   *
   * @return 旋转等效力臂，单位 m。
   */
  float rotationLeverArmMeters() const;

  /**
   * @brief 逆运动学：将底盘目标速度转换为四轮物理角速度。
   *
   * 输入 `vx/vy/wz` 可以同时存在，因此能够计算斜向平移并旋转等复合动作。
   * 输出尚未包含电机安装方向修正，也尚未限速。
   *
   * @param velocity 目标底盘速度，`vx/vy` 单位 m/s，`wz` 单位 rad/s。
   * @return 1～4号轮的物理角速度，单位 rad/s。
   *
   * 常见调用：
   * @code
   * WheelSpeeds radPerSec =
   *     chassis.inverse(ChassisVelocity(0.5f, 0.0f, 0.0f));
   * @endcode
   */
  WheelSpeeds inverse(const ChassisVelocity &velocity) const;

  /**
   * @brief 正运动学：根据四轮物理角速度估算底盘速度。
   *
   * 可用于将 DDSM210 的四轮反馈速度转换为 `vx/vy/wz`，
   * 为里程计或速度闭环提供基础。
   *
   * 注意：传入的必须是“车轮物理方向”的 rad/s。
   * 如果反馈值还是电机原始方向，应先按照 `WheelDirections` 修正符号。
   *
   * @param wheelRadPerSec 四轮物理角速度，单位 rad/s。
   * @return 估算出的底盘速度，`vx/vy` 单位 m/s，`wz` 单位 rad/s。
   */
  ChassisVelocity forward(const WheelSpeeds &wheelRadPerSec) const;

  /**
   * @brief 将四轮角速度从 rad/s 转换为 RPM。
   *
   * 换算公式：`RPM = rad/s × 60 / (2π)`。
   *
   * @param wheelRadPerSec 四轮角速度，单位 rad/s。
   * @return 四轮转速，单位 RPM。
   */
  static WheelSpeeds radPerSecToRpm(const WheelSpeeds &wheelRadPerSec);

  /**
   * @brief 将四轮转速从 RPM 转换为 rad/s。
   *
   * 换算公式：`rad/s = RPM × 2π / 60`。
   *
   * @param wheelRpm 四轮转速，单位 RPM。
   * @return 四轮角速度，单位 rad/s。
   */
  static WheelSpeeds rpmToRadPerSec(const WheelSpeeds &wheelRpm);

  /**
   * @brief 对四轮数值进行同比例限幅，同时保持合成运动方向。
   *
   * 如果最大绝对值超过限制，该函数不会单独裁剪某一个轮子，
   * 而是将四轮同时乘以同一个缩放系数。这样可以避免底盘运动方向失真。
   *
   * 该函数会原地修改 `values`，单位不限，只要四个值使用相同单位即可。
   *
   * @param values 要限幅的四轮数值，函数返回后内容已被修改。
   * @param maximumMagnitude 允许的最大绝对值，必须大于 0。
   * @return 实际缩放系数：
   *         `1.0` 表示没有缩放；
   *         `0～1` 表示进行了同比例缩小；
   *         `0.0` 表示限制参数无效，四轮已安全清零。
   */
  static float limitPreservingRatio(WheelSpeeds &values,
                                    float maximumMagnitude);

  /**
   * @brief 将四轮物理 RPM 转换为可直接发送的 DDSM210 原始命令。
   *
   * 处理顺序如下：
   *
   * 1. 根据 `motorDirections` 修正各电机安装方向；
   * 2. 使用 `commandUnitsPerRpm` 将 RPM 转成原始命令；
   * 3. 如果任一命令超限，对四轮进行同比例缩小；
   * 4. 四舍五入为 `int16_t` 命令。
   *
   * 默认采用 DDSM210 速度环常用规则：
   * `命令值 = RPM × 10`，最大绝对命令为 `2100`。
   *
   * @param physicalWheelRpm 四轮物理转速，单位 RPM。
   * @param motorDirections 四个电机的安装方向修正系数。
   * @param maximumCommand 最大命令绝对值，默认 2100。
   * @param commandUnitsPerRpm 每 RPM 对应的命令单位，默认 10。
   * @return 按 ID 1～4 排列、可发送给 DDSM210 的四个原始命令。
   *
   * 示例：
   * @code
   * WheelDirections directions; // 默认方向：-1, +1, -1, +1
   * DDSM210Commands cmd =
   *     MecanumKinematics::toDDSM210Commands(wheelRpm, directions);
   * dc.ddsm210_ctrl_4(cmd.wheel1, cmd.wheel2, cmd.wheel3, cmd.wheel4);
   * @endcode
   */
  static DDSM210Commands
  toDDSM210Commands(const WheelSpeeds &physicalWheelRpm,
                    const WheelDirections &motorDirections,
                    int16_t maximumCommand = 2100,
                    float commandUnitsPerRpm = 10.0f);

private:
  /** 前后轮中心距，单位 m。 */
  float wheelbaseMeters_;

  /** 左右轮中心距，单位 m。 */
  float trackWidthMeters_;

  /** 车轮半径，单位 m。 */
  float wheelRadiusMeters_;

  /** 旋转等效力臂 `(轴距 + 轮距) / 2`，单位 m。 */
  float rotationLeverArmMeters_;
};

} // namespace mecanum

#endif
