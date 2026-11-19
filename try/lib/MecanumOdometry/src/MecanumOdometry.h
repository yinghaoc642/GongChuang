#ifndef MECANUM_ODOMETRY_H
#define MECANUM_ODOMETRY_H

#include <MecanumKinematics.h>

#include <stdint.h>

namespace mecanum {

/**
 * @brief 底盘在世界坐标系中的二维位姿。
 *
 * 坐标约定与 MecanumKinematics 一致：
 *
 * - x：初始车头方向，单位 m；
 * - y：初始车体左侧方向，单位 m；
 * - heading：从世界 x 轴逆时针转到当前车头方向的角度，单位 rad。
 *
 * heading 始终归一化到 [-π, π)。
 */
struct Pose2D {
  double xMeters;
  double yMeters;
  double headingRadians;

  Pose2D(double x = 0.0, double y = 0.0, double heading = 0.0)
      : xMeters(x), yMeters(y), headingRadians(heading) {}
};

/**
 * @brief 一只轮子的“整圈 + 单圈位置”原始读数。
 *
 * DDSM210 的 0x74 反馈可以直接填入：
 *
 * - wholeTurns = mileage；
 * - position = ddsm_pos。
 *
 * position 不是物理码盘栅数，而是电机固件提供的单圈位置表示。其一圈对应
 * 多少单位由 MecanumOdometry 构造参数 positionUnitsPerTurn 决定。
 */
struct WheelPositionSample {
  int32_t wholeTurns;
  uint16_t position;

  WheelPositionSample(int32_t turns = 0, uint16_t singleTurnPosition = 0)
      : wholeTurns(turns), position(singleTurnPosition) {}
};

/**
 * @brief 按 1前左、2前右、3后左、4后右保存四轮位置读数。
 */
struct WheelPositionSamples {
  WheelPositionSample frontLeft;
  WheelPositionSample frontRight;
  WheelPositionSample rearLeft;
  WheelPositionSample rearRight;

  WheelPositionSamples(
      const WheelPositionSample &wheel1 = WheelPositionSample(),
      const WheelPositionSample &wheel2 = WheelPositionSample(),
      const WheelPositionSample &wheel3 = WheelPositionSample(),
      const WheelPositionSample &wheel4 = WheelPositionSample())
      : frontLeft(wheel1), frontRight(wheel2), rearLeft(wheel3),
        rearRight(wheel4) {}
};

/**
 * @brief 最近一次更新得到的车体坐标系位移增量。
 *
 * 这是四轮位置差经过麦轮正运动学后的结果，尚未旋转到世界坐标系：
 *
 * - dxBodyMeters > 0：向当前车头方向前进；
 * - dyBodyMeters > 0：向当前车体左侧横移；
 * - dHeadingRadians > 0：俯视逆时针旋转。
 */
struct ChassisDisplacement {
  double dxBodyMeters;
  double dyBodyMeters;
  double dHeadingRadians;

  ChassisDisplacement(double dx = 0.0, double dy = 0.0,
                      double dHeading = 0.0)
      : dxBodyMeters(dx), dyBodyMeters(dy),
        dHeadingRadians(dHeading) {}
};

/**
 * @brief 外部惯性传感器提供的航向角观测。
 *
 * 可直接接收 HWT101/JY901 的 Z 轴角度换算结果。里程计内部单位统一为 rad：
 *
 * @code
 * yawRadians = JY901.stcAngle.Angle[2] / 32768.0 * PI;
 * @endcode
 *
 * fusionWeight 决定本次更新对 IMU 航向的信任程度：
 *
 * - 0：完全使用轮式里程计航向；
 * - 1：更新后的航向完全对齐 IMU；
 * - 0～1：在轮式预测和 IMU 观测之间进行互补融合。
 *
 * 传感器安装方向与本库正方向相反时，应在传入前对 yawRadians 取负。
 */
struct ImuHeadingMeasurement {
  double yawRadians;
  double fusionWeight;

  ImuHeadingMeasurement(double yaw = 0.0, double weight = 1.0)
      : yawRadians(yaw), fusionWeight(weight) {}
};

/**
 * @brief X 型四轮麦克纳姆底盘增量式里程计。
 *
 * 本类是纯计算模块，不读串口。它接收四个电机已经读取好的“整圈 + 单圈”
 * 位置，完成：
 *
 * 1. 组合每个轮子的连续圈数；
 * 2. 计算相邻两次采样之间的四轮转角；
 * 3. 应用电机安装方向修正；
 * 4. 通过麦轮正运动学得到车体位移；
 * 5. 使用 SE(2) 指数映射把车体位移积分到世界坐标系。
 *
 * 将通信留在上层 DDSM210MecanumChassis 中，可以让该数学模块进行原生单元
 * 测试，也能复用于其他编码器或电机驱动器。
 */
class MecanumOdometry {
public:
  /**
   * @brief 构造麦轮里程计。
   *
   * @param kinematics 与实际底盘相同的运动学几何模型。
   * @param motorDirections 原始电机正方向到车轮物理正方向的修正系数。
   * @param positionUnitsPerTurn 单圈位置一整圈对应的协议单位数。
   *        DDSM210 常见配置为 65536；若实际固件使用 0～32767，则应传 32768。
   */
  MecanumOdometry(const MecanumKinematics &kinematics,
                  const WheelDirections &motorDirections,
                  uint32_t positionUnitsPerTurn = 65536U);

  /**
   * @brief 检查几何、方向系数和单圈量程是否有效。
   */
  bool isValid() const;

  /**
   * @brief 清零或重设位姿，并清除编码器基线。
   *
   * reset() 后第一次 update() 只记录四轮当前位置，不产生位移；从第二次
   * 成功 update() 开始才积分。
   */
  void reset(const Pose2D &newPose = Pose2D());

  /**
   * @brief 使用一组新的四轮绝对位置更新底盘位姿。
   *
   * @param samples 同一次里程计周期读取的四轮位置。
   * @return 数据有效并被接受时返回 true；配置无效、位置越界或轮子跳变量
   *         超过保护阈值时返回 false。
   *
   * @note 第一次调用成功时仅建立基线，也返回 true。
   * @note 失败时不会改变位姿和上一组基线，因此下一次有效数据仍可覆盖这段
   *       时间内的累计运动。
   */
  bool update(const WheelPositionSamples &samples);

  /**
   * @brief 使用四轮位置和一帧 IMU 航向观测共同更新位姿。
   *
   * 第一次收到 IMU 航向时会自动建立零点偏置，使“当前里程计 heading”与
   * “当前传感器 yaw”对齐，因此接入传感器不会让位姿突然跳变。以后每次
   * 调用按 fusionWeight 修正轮式航向。
   *
   * @param samples 四轮绝对位置。
   * @param imuHeading HWT101 或其他 IMU 的当前 Z 轴航向观测。
   * @return 数据有效并完成更新时返回 true，否则返回 false。
   *
   * @note IMU 只修正 heading；x/y 仍来自四轮位移，但转换到世界坐标时会
   *       使用融合后的航向增量，因此可减少轮子打滑造成的方向漂移。
   */
  bool update(const WheelPositionSamples &samples,
              const ImuHeadingMeasurement &imuHeading);

  /**
   * @brief 显式设置 IMU 原始航向与世界坐标航向之间的对应关系。
   *
   * 不调用时，第一次带 IMU 的 update() 会自动把当前传感器角度对齐到当前
   * 里程计 heading。若上电时已知车头应为世界坐标 0 rad，可调用：
   *
   * @code
   * odometry.setImuHeadingReference(currentSensorYaw, 0.0);
   * @endcode
   *
   * @return 两个角度都有限时返回 true，否则不改变参考并返回 false。
   */
  bool setImuHeadingReference(double sensorYawRadians,
                              double worldHeadingRadians = 0.0);

  /**
   * @brief 清除 IMU 零点参考。
   *
   * 下一次带 IMU 的 update() 会重新自动对齐，不影响当前 x/y/heading。
   */
  void clearImuHeadingReference();

  /** @return 是否已经建立 IMU 航向零点参考。 */
  bool hasImuHeadingReference() const;

  /**
   * @brief 设置单次更新允许的最大单轮转数增量。
   *
   * 可用来拒绝电机重启、通信错帧等造成的巨大位置跳变。传 0 表示关闭检查。
   * 例如 50 Hz 更新、最高约 3.5 转/秒时，可设置为 0.5～1.0 转。
   *
   * @return 参数有限且不小于 0 时返回 true，否则保持原设置并返回 false。
   */
  bool setMaximumWheelDeltaTurns(double maximumTurns);

  /** @return 当前最大单轮跳变阈值；0 表示关闭。 */
  double maximumWheelDeltaTurns() const;

  /** @return 是否已经通过第一次 update() 建立四轮位置基线。 */
  bool isInitialized() const;

  /** @return 当前世界坐标系位姿。 */
  const Pose2D &pose() const;

  /** @return 最近一次成功更新得到的车体坐标系位移增量。 */
  const ChassisDisplacement &lastDisplacement() const;

  /** @return 构造时设置的单圈位置单位数。 */
  uint32_t positionUnitsPerTurn() const;

private:
  double continuousTurns(const WheelPositionSample &sample) const;
  bool samplesAreValid(const WheelPositionSamples &samples) const;
  bool updateInternal(const WheelPositionSamples &samples,
                      const ImuHeadingMeasurement *imuHeading);
  bool imuMeasurementIsValid(
      const ImuHeadingMeasurement &measurement) const;
  static double normalizeAngle(double radians);

  MecanumKinematics kinematics_;
  WheelDirections motorDirections_;
  uint32_t positionUnitsPerTurn_;
  double maximumWheelDeltaTurns_;

  bool initialized_;
  bool imuHeadingReferenceValid_;
  double imuHeadingOffsetRadians_;
  double previousTurns_[4];
  Pose2D pose_;
  ChassisDisplacement lastDisplacement_;
};

} // namespace mecanum

#endif
