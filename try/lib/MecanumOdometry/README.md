# MecanumOdometry

四轮 X 型麦克纳姆底盘的纯数学里程计。

它把每只 DDSM210 电机的：

- 有符号累计整圈数 `mileage`；
- 当前单圈位置 `position`；

组合成连续轮子位置，通过四轮正运动学积分得到：

- 世界坐标系 `x`；
- 世界坐标系 `y`；
- 底盘航向 `heading`。

## 为什么需要两个字段

只使用整数 `mileage` 时，直径 100 mm 的轮子每次只能分辨约 314.16 mm。
加入单圈位置后，才能获得一圈内部的细分位移：

```text
连续圈数 = 整圈数 + 单圈位置 / 每圈协议单位数
```

默认每圈协议单位数为 65536。若实际电机固件返回 0～32767，应在构造时将
`positionUnitsPerTurn` 设置成 32768。协议数值分辨率不等于轮子落地后的
实际定位精度，麦轮打滑、滚子变形和安装误差仍会造成累计漂移。

## 基本用法

```cpp
#include <MecanumKinematics.h>
#include <MecanumOdometry.h>

using namespace mecanum;

MecanumKinematics model =
    MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);

MecanumOdometry odometry(
    model,
    WheelDirections(-1, +1, -1, +1),
    65536U);

void updateFromFourMotors() {
  WheelPositionSamples samples(
      WheelPositionSample(turns1, position1),
      WheelPositionSample(turns2, position2),
      WheelPositionSample(turns3, position3),
      WheelPositionSample(turns4, position4));

  if (odometry.update(samples)) {
    const Pose2D &pose = odometry.pose();
    // 使用 pose.xMeters、pose.yMeters、pose.headingRadians
  }
}
```

第一次 `update()` 只建立编码器基线。以后每次成功更新才会累积位姿。

实际项目通常不需要显式读取四台电机并拼装 `samples`，可直接使用
`DDSM210MecanumChassis` 高层类完成通讯、驱动和里程计更新。

## HWT101 航向融合接口

里程计不依赖具体 IMU 库，只接受已经解析好的 Z 轴航向角：

```cpp
const double yawRadians =
    JY901.stcAngle.Angle[2] / 32768.0 * PI;

odometry.update(
    samples,
    ImuHeadingMeasurement(yawRadians, 1.0));
```

第二个参数为融合权重：

- `1.0`：航向完全采用 HWT101；
- `0.0`：航向完全采用轮式里程计；
- `0.1～0.5`：逐步修正轮式航向，减小传感器瞬时噪声的影响。

第一次 IMU 更新会自动对齐当前零点，不会让位姿突然跳变。HWT101 从
`+180°` 跳到 `-180°` 时，库会按最短角差连续处理。
