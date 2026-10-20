# MecanumKinematics

适用于本工程四轮麦克纳姆底盘的纯数学运动学库。

## 底盘定义

从上往下看，滚子组成 X 形：

```text
              车头
       1 前左       2 前右
          \           /

          /           \
       3 后左       4 后右
```

坐标约定：

- `vx > 0`：底盘向前，单位 m/s
- `vy > 0`：底盘向左，单位 m/s
- `wz > 0`：俯视逆时针旋转，单位 rad/s
- 四轮物理正转均定义为“该轮推动底盘向前”

本车参数：

```text
前后轮中心距：187.5 mm
左右轮中心距：195.0 mm
车轮直径：    100.0 mm
旋转力臂：    191.25 mm
```

## 基本使用

```cpp
#include <MecanumKinematics.h>

using namespace mecanum;

MecanumKinematics chassis =
    MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);

ChassisVelocity target(
    0.50f, // 向前 0.50 m/s
    0.20f, // 向左 0.20 m/s
    0.50f  // 逆时针 0.50 rad/s
);

WheelSpeeds wheelRadPerSec = chassis.inverse(target);
WheelSpeeds wheelRpm =
    MecanumKinematics::radPerSecToRpm(wheelRadPerSec);
```

## 生成 DDSM210 命令

运动学输出表示车轮的物理方向，而电机安装方向可能相反。方向修正独立配置：

```cpp
// 默认方向依次为 1号−、2号＋、3号−、4号＋。
WheelDirections motorDirections;

DDSM210Commands commands =
    MecanumKinematics::toDDSM210Commands(
        wheelRpm,
        motorDirections,
        2100, // 最大绝对命令
        10.0f // 每 RPM 对应 10 个命令单位
    );

dc.ddsm210_ctrl_4(
    commands.wheel1,
    commands.wheel2,
    commands.wheel3,
    commands.wheel4
);
```

`WheelDirections()` 默认生成 `(-1, +1, -1, +1)`。`motorDirections`
仍必须在底盘架空时逐轮校准，不要仅根据线色或左右位置猜测方向。

如果任一电机命令超过 `±2100`，库会对四轮命令同比例缩小，以保持合成运动方向。

## 正运动学

可以将四轮反馈速度换算成底盘速度：

```cpp
WheelSpeeds measuredRpm(rpm1, rpm2, rpm3, rpm4);
WheelSpeeds measuredRadPerSec =
    MecanumKinematics::rpmToRadPerSec(measuredRpm);

ChassisVelocity measured = chassis.forward(measuredRadPerSec);
```

传入 `forward()` 前，应先把原始电机反馈按 `motorDirections` 转换回车轮物理方向。

## 完整控制循环

`examples/BasicUsage/BasicUsage.ino` 已按照项目现有 DDSM210 例程实现完整控制链：

```text
PA3/PA2 硬件串口初始化
→ 绑定 DDSM_CTRL::pSerial
→ 清空接收缓存
→ 每 20 ms 更新一次底盘目标
→ 运动学逆解与 RPM 换算
→ 四轮方向修正和等比例限幅
→ ddsm210_ctrl_4() 发送
→ 每 200 ms 输出命令和反馈
```

示例包含一次性的“前进、左移、逆时针旋转”动作流程，但出于安全考虑默认关闭：

```cpp
const bool ENABLE_AUTOMATIC_DEMO = false;
```

只有在底盘架空、四轮 ID 和方向均确认无误后，才可将其改为 `true`。
