#ifndef DDSM210_MECANUM_CHASSIS_H
#define DDSM210_MECANUM_CHASSIS_H

#include <Arduino.h>
#include <DDSM210.h>
#include <MecanumKinematics.h>
#include <MecanumOdometry.h>

#include <stdint.h>

namespace mecanum {

/**
 * @brief DDSM210 四轮麦轮底盘的一站式高层控制器。
 *
 * 本类采用“外观/组合”设计，把三个职责连接起来：
 *
 * - DDSM_CTRL：串口组帧、发送和反馈读取；
 * - MecanumKinematics：底盘速度与四轮速度之间的换算；
 * - MecanumOdometry：四轮位置到 x/y/航向的积分。
 *
 * 上层程序无需显式创建 DDSM_CTRL，也无需手工发送四台电机命令：
 *
 * @code
 * chassis.begin();
 * chassis.drive(0.20f, 0.0f, 0.0f); // 直接让底盘向前 0.20 m/s
 * chassis.updateOdometry();          // 一次读取四轮并更新位姿
 * @endcode
 *
 * 纯数学库仍不依赖串口，因此可独立测试；项目代码则通过本类获得简单接口。
 */
class DDSM210MecanumChassis {
public:
  /**
   * @brief 构造一套 DDSM210 麦轮底盘。
   *
   * @param motorSerial 已按正确 RX/TX 引脚创建的 HardwareSerial 对象。
   * @param kinematics 底盘几何模型。
   * @param motorDirections 四个电机的安装方向修正。
   * @param positionUnitsPerTurn DDSM210 单圈位置一圈对应的协议单位数，
   *        默认 65536；若实际固件使用 0～32767，应传 32768。
   * @param maximumCommand DDSM210 速度环命令最大绝对值，默认 2100。
   * @param commandUnitsPerRpm 每 RPM 对应的命令单位，默认 10。
   * @param accelerationTime 发送给电机的加速时间参数，默认 1。
   */
  DDSM210MecanumChassis(
      HardwareSerial &motorSerial, const MecanumKinematics &kinematics,
      const WheelDirections &motorDirections,
      uint32_t positionUnitsPerTurn = 65536U,
      int16_t maximumCommand = 2100,
      float commandUnitsPerRpm = 10.0f,
      uint8_t accelerationTime = 1);

  /**
   * @brief 初始化电机串口和内部 DDSM210 驱动。
   *
   * 本函数会执行 Serial.begin()、绑定 DDSM_CTRL、等待上电稳定并清空残留
   * 接收字节。它不会自动切换电机模式，要求四台 DDSM210 已处于速度环模式
   * （DDSM210 通常上电默认为速度环）。
   *
   * @param baudrate 串口波特率，默认 DDSM_BAUDRATE。
   * @param startupDelayMs 串口启动后等待电机稳定的时间，默认 100 ms。
   * @return 几何、方向和命令配置有效时返回 true，否则不启用控制器。
   */
  bool begin(unsigned long baudrate = DDSM_BAUDRATE,
             uint32_t startupDelayMs = 100U);

  /**
   * @brief 直接设置底盘速度并发送给四台 DDSM210。
   *
   * 完整流程在函数内部完成：
   * vx/vy/wz → 麦轮逆运动学 → RPM → 方向修正 → 等比例限幅 → 串口发送。
   *
   * @param velocity 车体坐标系目标速度。
   * @return 本次实际发送给 ID 1～4 的四个原始命令；未 begin() 时返回全零。
   */
  DDSM210Commands drive(const ChassisVelocity &velocity);

  /**
   * @brief 使用三个数值直接设置底盘速度。
   *
   * @param vxMetersPerSecond 向前速度，单位 m/s。
   * @param vyMetersPerSecond 向左速度，单位 m/s。
   * @param wzRadiansPerSecond 俯视逆时针角速度，单位 rad/s。
   */
  DDSM210Commands drive(float vxMetersPerSecond,
                        float vyMetersPerSecond,
                        float wzRadiansPerSecond);

  /**
   * @brief 向四台电机发送零速度目标。
   *
   * 本函数按速度环语义停止，不调用旧库中会二次读取反馈的 ddsm_stop()。
   */
  DDSM210Commands stop();

  /**
   * @brief 依次读取 ID 1～4 的 0x74 反馈并更新里程计。
   *
   * 只有四台电机全部返回 ID/CRC 正确的数据后才会更新位姿，避免把新旧轮子
   * 数据混在同一个里程计周期中。
   *
   * @return 四轮通信和里程计更新都成功时返回 true，否则返回 false。
   * @note 第一次成功调用只建立四轮位置基线，位姿不会跳变。
   */
  bool updateOdometry();

  /**
   * @brief 读取四轮，并用 HWT101/其他 IMU 的航向角修正里程计。
   *
   * @param yawRadians IMU 当前 Z 轴航向角，单位 rad，逆时针为正。
   * @param fusionWeight IMU 航向权重，范围 0～1；1 表示完全采用 IMU 航向，
   *        0 表示只采用轮式航向。
   * @return 四轮读取、IMU 数据检查和里程计更新都成功时返回 true。
   *
   * 第一次调用会自动建立 IMU 零点，不会让当前 heading 突然跳到传感器的
   * 绝对读数。若传感器正方向相反，请在传入前对 yawRadians 取负。
   */
  bool updateOdometryWithImuHeading(double yawRadians,
                                    double fusionWeight = 1.0);

  /**
   * @brief 接受“度”为单位的便捷版本，适合项目现有 HWT101 例程。
   *
   * 可直接传入：
   * `JY901.stcAngle.Angle[2] / 32768.0 * 180.0`。
   */
  bool updateOdometryWithImuHeadingDegrees(
      double yawDegrees, double fusionWeight = 1.0);

  /**
   * @brief 一次调用完成速度发送和里程计更新。
   *
   * 适合希望 loop() 接口尽量简单的程序。需要更高控制频率、较低里程计频率
   * 时，建议分别调用 drive() 和 updateOdometry()。
   *
   * @return 速度命令一定会尝试发送；返回值表示随后的里程计更新是否成功。
   */
  bool driveAndUpdateOdometry(const ChassisVelocity &velocity);

  /** 三参数版本的 driveAndUpdateOdometry()。 */
  bool driveAndUpdateOdometry(float vxMetersPerSecond,
                              float vyMetersPerSecond,
                              float wzRadiansPerSecond);

  /**
   * @brief 重设底盘位姿并清除四轮位置基线。
   *
   * 下一次 updateOdometry() 只重新建立基线。
   */
  void resetOdometry(const Pose2D &newPose = Pose2D());

  /**
   * @brief 显式把当前 IMU 原始航向对应到指定世界航向。
   *
   * 例如底盘上电时车头指向世界 x 正方向，可传当前 HWT101 yaw 和 0。
   */
  bool setImuHeadingReference(double sensorYawRadians,
                              double worldHeadingRadians = 0.0);

  /** 清除 IMU 航向参考，下一帧 IMU 数据将重新自动对齐。 */
  void clearImuHeadingReference();

  /** @return 是否已经建立 IMU 航向参考。 */
  bool hasImuHeadingReference() const;

  /** @return 当前世界坐标系位姿。 */
  const Pose2D &pose() const;

  /** @return 最近一次成功读取并保存的四轮原始位置。 */
  const WheelPositionSamples &lastWheelPositions() const;

  /** @return 最近一次实际发送的四个 DDSM210 原始命令。 */
  const DDSM210Commands &lastCommands() const;

  /** @return 最近一次 updateOdometry() 是否成功。 */
  bool lastOdometryUpdateSucceeded() const;

  /** @return 是否已经成功执行 begin()。 */
  bool isBegun() const;

  /** @return 内部纯数学运动学对象，供高级功能只读访问。 */
  const MecanumKinematics &kinematics() const;

  /** @return 内部里程计对象，可用于设置跳变阈值或读取增量。 */
  MecanumOdometry &odometry();
  const MecanumOdometry &odometry() const;

  /**
   * @brief 获取底层通信对象，供 ID 设置、模式切换等高级操作使用。
   *
   * 普通底盘运动和里程计不需要访问它。
   */
  DDSM_CTRL &communication();
  const DDSM_CTRL &communication() const;

private:
  bool configurationIsValid() const;
  bool readWheelPositions(WheelPositionSamples &samples);

  HardwareSerial &motorSerial_;
  MecanumKinematics kinematics_;
  WheelDirections motorDirections_;
  MecanumOdometry odometry_;
  DDSM_CTRL communication_;

  int16_t maximumCommand_;
  float commandUnitsPerRpm_;
  uint8_t accelerationTime_;

  bool begun_;
  bool lastOdometryUpdateSucceeded_;
  DDSM210Commands lastCommands_;
  WheelPositionSamples lastWheelPositions_;
};

} // namespace mecanum

#endif
