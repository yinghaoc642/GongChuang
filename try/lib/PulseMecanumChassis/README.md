# PulseMecanumChassis

这是 GongChuang 工程中面向四路 STEP/DIR 驱动器的麦克纳姆底盘高层库。
调用者只需传入车体速度，库会完成：

```text
vx / vy / wz
  -> MecanumKinematics 逆运动学
  -> 轮速换算为 pulse/s
  -> M1～M4 方向修正（默认 - + - +）
  -> 四轮同比例限速
  -> AccelStepper::setSpeed() / runSpeed()
```

## 坐标和单位

- `vx > 0`：向车头前进，单位 m/s
- `vy > 0`：向左横移，单位 m/s
- `wz > 0`：俯视逆时针旋转，单位 rad/s
- M1：前左，M2：前右，M3：后左，M4：后右

## yyq5 脉冲配置

| 项目 | 数值 |
|---|---:|
| M1 STEP / DIR | PD4 / PD6 |
| M2 STEP / DIR | PE11 / PE9 |
| M3 STEP / DIR | PD15 / PD14 |
| M4 STEP / DIR | PA1 / PC3_C |
| 公共使能 | PE13，低有效 |
| 电机方向 | `- + - +` |
| 每轮脉冲数 | 3200 pulse/rev |
| 直线标定 | 10000 pulse/m |
| 最大频率 | 30000 pulse/s |

方向修正后的基本动作与 yyq5 原程序完全一致：

- 前进：M1～M4 为 `- + - +`
- 左移：M1～M4 为 `+ + - -`
- 逆时针旋转：M1～M4 为 `+ + + +`

## 最小用法

```cpp
#include <Arduino.h>
#include <PulseMecanumChassis.h>

using namespace mecanum;

PulseMecanumPins pins(
    PD4, PD6, PE11, PE9, PD15, PD14, PA1, PC3_C, PE13, true);

MecanumKinematics geometry =
    MecanumKinematics::fromMillimeters(187.5f, 195.0f, 100.0f);

PulseMecanumChassis chassis(
    pins, geometry, WheelDirections(-1, +1, -1, +1),
    3200.0f, 10000.0f, 30000.0f);

void setup() {
  if (!chassis.begin()) {
    return; // begin() 失败时公共 EN 保持失能
  }

  // 向前 0.20 m/s、向左 0.10 m/s，同时逆时针 0.30 rad/s。
  chassis.drive(0.20f, 0.10f, 0.30f);
}

void loop() {
  chassis.run();
}
```

只需要前进速度与角速度时，也可以调用：

```cpp
chassis.drive(0.20f, 0.50f); // vx=0.20 m/s，vy=0，wz=0.50 rad/s
```

停止与使能：

```cpp
chassis.stop();    // 四路 pulse/s 立即清零
chassis.disable(); // 清零并拉到失能电平
chassis.enable();  // 重新使能；仍需再次设置速度
```

## 标定说明

库将平移和旋转分开换算后再叠加：

- 平移采用旧程序实测的 `10000 pulse/m`；
- 旋转采用 `3200 pulse/rev` 和传入的运动学几何。

yyq5 的 `200 mm / 190 mm / 100 mm` 几何会精确得到
`3120 pulse/90°`。当前 GongChuang 文档使用
`187.5 mm / 195 mm / 100 mm`，应以实际测量结果为准。

如果希望完全采用理论轮径而不使用旧直线标定，100 mm 轮、3200 pulse/rev
对应约 `10185.916 pulse/m`，可将构造函数的直线标定参数改为该值。

## 必须注意

`AccelStepper::runSpeed()` 是软件轮询发脉冲，不是后台定时器：

1. `drive()` 只设置速度，不会自行持续发脉冲。
2. `run()` 必须在 `loop()` 中无条件且尽可能频繁地调用。
3. 运动期间不能使用 `delay()`、舵机 `wait()` 或其他长时间阻塞代码。
4. 同一组电机不能同时使用旧的 `move()/run()` 位置模式和本库的
   `setSpeed()/runSpeed()` 速度模式。
5. `setAcceleration()` 对 `runSpeed()` 不生效；本库当前执行即时速度命令。
6. 首次测试必须架空底盘，先核对四轮方向，再落地运行。
7. 公共低有效 EN 建议增加硬件上拉；软件只能在 MCU 开始执行后控制引脚，
   无法消除上电复位阶段的浮空风险。

完整、默认不使能电机的安全例程位于：
`examples/BasicPulseChassis/BasicPulseChassis.ino`。
