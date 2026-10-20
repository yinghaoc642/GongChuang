# DDSM210MecanumChassis

面向项目代码的一站式 DDSM210 麦轮底盘接口。

## 推荐分层

```text
用户 main.cpp
    │
    ▼
DDSM210MecanumChassis
    ├── DDSM210 通讯
    ├── MecanumKinematics 运动学
    └── MecanumOdometry 里程计
```

不建议让运动学公式类或里程计公式类直接操作串口，因为这样会导致：

- 数学算法无法在电脑上独立测试；
- 以后更换电机或编码器时必须重写运动学；
- 一个类同时负责协议、硬件、公式和状态，难以定位问题。

高层 `DDSM210MecanumChassis` 通过组合三个模块，在不破坏分层的前提下提供
简洁接口。

## 本车初始化

```cpp
#include <DDSM210MecanumChassis.h>

using namespace mecanum;

HardwareSerial motorSerial(PA3, PA2); // RX、TX

MecanumKinematics geometry =
    MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);

DDSM210MecanumChassis chassis(
    motorSerial,
    geometry,
    WheelDirections(-1, +1, -1, +1));

void setup() {
  chassis.begin(); // 内部完成 motorSerial.begin() 和 DDSM_CTRL 绑定
}
```

## 直接驱动

```cpp
chassis.drive(0.20f, 0.0f, 0.0f); // 向前 0.20 m/s
chassis.drive(0.0f, 0.20f, 0.0f); // 向左 0.20 m/s
chassis.drive(0.0f, 0.0f, 0.50f); // 逆时针 0.50 rad/s
chassis.stop();                    // 速度环零速目标
```

## 更新里程计

```cpp
if (chassis.updateOdometry()) {
  const Pose2D &pose = chassis.pose();
  Serial.println(pose.xMeters);
  Serial.println(pose.yMeters);
  Serial.println(pose.headingRadians);
}
```

也可以一次完成发送与读取：

```cpp
chassis.driveAndUpdateOdometry(0.20f, 0.0f, 0.0f);
```

由于一次里程计更新需要依次查询四台电机，控制频率和里程计频率可以分开：
例如 20 ms 发送一次速度命令，50～100 ms 更新一次里程计。

## 接入 HWT101 航向反馈

项目现有例程通过 JY901 库读取 HWT101：

```cpp
while (Serial_WTIMU.available()) {
  JY901.CopeSerialData(Serial_WTIMU.read());
}

const float yawDegrees =
    JY901.stcAngle.Angle[2] / 32768.0f * 180.0f;

chassis.updateOdometryWithImuHeadingDegrees(yawDegrees, 1.0);
```

高层底盘库不直接依赖 JY901，因此即使以后更换 IMU，只要能提供 Z 轴航向角，
仍可使用同一接口。当前项目 HWT101 例程使用：

- RX：PD9；
- TX：PD8；
- 波特率：115200。

若传感器安装后顺时针为正，而底盘约定逆时针为正，应传入 `-yawDegrees`。
融合权重可设为 `0～1`；建议先从 `0.2` 开始实车观察，再决定是否完全采用
IMU 航向。

## 精度边界

轮式里程计只能估算运动。麦轮横移时滚子滑动明显，地面、负载、轮径误差和
四轮不同步都会造成累计漂移。若需要准确完成“前进 10 m、左移 5 m”，建议
至少加入 IMU 航向融合；要求更高时还需视觉、UWB 或其他外部定位校正。
