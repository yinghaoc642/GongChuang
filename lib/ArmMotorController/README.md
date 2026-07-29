# ArmMotorController

项目内的 M5、M6、M7 基础角度控制库。

```cpp
#include <ArmMotorController.h>

ArmMotorController arm;

void setup() {
  arm.begin();
}

void loop() {
  arm.moveM6ByMillimeters(-28.0f); // 负数：收缩28 mm
  arm.moveM7ByMillimeters(3.0f);   // 正数：上升3 mm
  arm.rotateM5ClockwiseByDegrees(90.0f);
  delay(1500);

  arm.moveM6ByMillimeters(28.0f);  // 正数：伸长28 mm
  arm.moveM7ByMillimeters(-3.0f);  // 负数：下降3 mm
  arm.rotateM5CounterClockwiseByDegrees(90.0f);
  delay(1500);
}
```

也可以使用带符号的通用接口：

- `moveM5ByDegrees(+角度)`：逆时针；负数为顺时针。
- `moveM6ByMillimeters(+毫米)`：伸长；负数为收缩。
- `moveM7ByMillimeters(+毫米)`：上升；负数为下降。

M5 使用 AccelStepper，默认阻塞到运动结束；
M6、M7 使用 TTL 串口位置命令，命令发送后函数立即返回。

若主程序已经自行管理 PA2/PA3 上的 M6/M7 非阻塞反馈，只初始化 M5：

```cpp
arm.beginM5();
arm.moveM5ToDegrees(90.0f, false);

void loop() {
  arm.serviceM5();
}
```

`beginM5()` 不会初始化或占用 M6/M7 串口。M5 的方向反相在库内部完成，
调用者始终按“正角逆时针、负角顺时针”的约定输入。

默认硬件参数：

- M5：PE10 EN、PE15 DIR、PB11 STEP，16细分，5:1减速比。
- M6：PA2/PA3 串口，地址6，16细分；模数1 mm、分度圆直径35 mm。
- M7：PA2/PA3 串口，地址7，16细分；T8×12滚珠丝杠，导程12 mm/圈。
- 串口：115200 bit/s。

若以后更换其他导程的丝杠，可在 `begin()` 后重新设置：

```cpp
arm.setM7LeadMillimetersPerRevolution(8.0f); // 例如 T8×8
```
