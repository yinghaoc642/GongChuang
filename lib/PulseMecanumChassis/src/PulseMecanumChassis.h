#ifndef PULSE_MECANUM_CHASSIS_H
#define PULSE_MECANUM_CHASSIS_H

#include <AccelStepper.h>
#include <Arduino.h>
#include <MecanumKinematics.h>

#include <stdint.h>

namespace mecanum {

/**
 * @brief 四路 STEP/DIR 底盘电机及公共使能脚的引脚配置。
 *
 * 电机编号必须与 MecanumKinematics 保持一致：
 * M1 前左、M2 前右、M3 后左、M4 后右。
 */
struct PulseMecanumPins {
  uint8_t motor1StepPin;
  uint8_t motor1DirectionPin;
  uint8_t motor2StepPin;
  uint8_t motor2DirectionPin;
  uint8_t motor3StepPin;
  uint8_t motor3DirectionPin;
  uint8_t motor4StepPin;
  uint8_t motor4DirectionPin;
  uint8_t enablePin;
  bool enableActiveLow;

  PulseMecanumPins(uint8_t m1Step, uint8_t m1Direction,
                   uint8_t m2Step, uint8_t m2Direction,
                   uint8_t m3Step, uint8_t m3Direction,
                   uint8_t m4Step, uint8_t m4Direction,
                   uint8_t commonEnablePin,
                   bool commonEnableActiveLow = true)
      : motor1StepPin(m1Step), motor1DirectionPin(m1Direction),
        motor2StepPin(m2Step), motor2DirectionPin(m2Direction),
        motor3StepPin(m3Step), motor3DirectionPin(m3Direction),
        motor4StepPin(m4Step), motor4DirectionPin(m4Direction),
        enablePin(commonEnablePin),
        enableActiveLow(commonEnableActiveLow) {}
};

/**
 * @brief 使用 STEP/DIR 脉冲驱动四轮 X 型麦克纳姆底盘。
 *
 * 坐标和单位与 MecanumKinematics 完全一致：
 *
 * - vx > 0：向前，m/s；
 * - vy > 0：向左，m/s；
 * - wz > 0：俯视逆时针，rad/s。
 *
 * drive()/setVelocity() 只更新四路目标脉冲频率。主程序必须在 loop() 中
 * 尽可能频繁地调用 run()，才能真正持续输出 STEP 脉冲。
 */
class PulseMecanumChassis {
public:
  /**
   * @param pins 四路 STEP/DIR 和公共使能脚。
   * @param kinematics 底盘几何模型。
   * @param motorDirections 电机安装方向，默认 M1～M4 为 -、+、-、+。
   * @param pulsesPerWheelRevolution 驱动器细分后的每轮脉冲数。
   * @param linearPulsesPerMeter 旧底盘实测的直线脉冲标定。
   * @param maximumPulseRate 单个电机允许的最大脉冲频率，pulse/s。
   * @param minimumPulseWidthMicros STEP 高电平最短宽度，us。
   */
  PulseMecanumChassis(
      const PulseMecanumPins &pins,
      const MecanumKinematics &kinematics,
      const WheelDirections &motorDirections =
          WheelDirections(-1, +1, -1, +1),
      float pulsesPerWheelRevolution = 3200.0f,
      float linearPulsesPerMeter = 10000.0f,
      float maximumPulseRate = 30000.0f,
      uint16_t minimumPulseWidthMicros = 1U);

  /**
   * @brief 初始化 STEP/DIR、使能脚和脉冲频率限制。
   *
   * @param enableMotors 初始化完成后是否拉到使能电平。
   * @return 配置有效时返回 true。
   */
  bool begin(bool enableMotors = true);

  /**
   * @brief 设置车体速度目标。
   *
   * 内部流程：
   * 运动学逆解 -> 脉冲频率换算 -> -+-+ 方向修正 -> 四轮同比例限速。
   */
  bool drive(const ChassisVelocity &velocity);

  /** 三参数便捷版本。 */
  bool drive(float vxMetersPerSecond, float vyMetersPerSecond,
             float wzRadiansPerSecond);

  /** 只需要前进速度和角速度时使用，横移速度自动设为 0。 */
  bool drive(float forwardMetersPerSecond, float wzRadiansPerSecond);

  /** 与 drive() 同义，适合按“设置速度”的方式阅读业务代码。 */
  bool setVelocity(const ChassisVelocity &velocity);
  bool setVelocity(float vxMetersPerSecond, float vyMetersPerSecond,
                   float wzRadiansPerSecond);
  bool setVelocity(float forwardMetersPerSecond,
                   float wzRadiansPerSecond);

  /**
   * @brief 为四个电机各执行一次非阻塞脉冲服务。
   *
   * 必须在 loop() 中无条件、高频调用。返回 true 表示本次至少输出了一个
   * STEP 脉冲。
   */
  bool run();

  /** 立即把四路目标脉冲频率清零。 */
  void stop();

  /** 控制公共使能脚；失能时同时清零速度。 */
  bool setEnabled(bool enabled);
  bool enable();
  void disable();

  bool isBegun() const;
  bool isEnabled() const;
  bool configurationIsValid() const;

  /**
   * @return 最近一次经过方向修正和同比例限速后的 M1～M4 脉冲频率。
   *
   * 字段 frontLeft/frontRight/rearLeft/rearRight 分别对应 M1/M2/M3/M4，
   * 单位 pulse/s；这里的正负号是实际送入 AccelStepper 的原始方向。
   */
  const WheelSpeeds &lastPulseRates() const;

  /** @return 最近一次同比例限速系数，1 表示未限速。 */
  float lastLimitScale() const;

  /** @return 最近一次成功接受的车体速度目标。 */
  const ChassisVelocity &lastRequestedVelocity() const;

  /** @return 内部运动学模型，供调试或高级换算只读使用。 */
  const MecanumKinematics &kinematics() const;

private:
  bool pinsAreValid() const;
  bool pinsAreUnique() const;
  bool velocityIsValid(const ChassisVelocity &velocity) const;
  bool calculatePulseRates(const ChassisVelocity &velocity,
                           WheelSpeeds &pulseRates,
                           float &limitScale) const;
  void applyPulseRates(const WheelSpeeds &pulseRates);
  uint8_t enableLevel(bool enabled) const;

  PulseMecanumPins pins_;
  MecanumKinematics kinematics_;
  WheelDirections motorDirections_;
  float pulsesPerWheelRevolution_;
  float linearPulsesPerMeter_;
  float maximumPulseRate_;
  uint16_t minimumPulseWidthMicros_;

  AccelStepper motor1_;
  AccelStepper motor2_;
  AccelStepper motor3_;
  AccelStepper motor4_;

  bool begun_;
  bool enabled_;
  WheelSpeeds lastPulseRates_;
  float lastLimitScale_;
  ChassisVelocity lastRequestedVelocity_;
};

} // namespace mecanum

#endif
