# Vision protocol v2

当前 STM32 实现位于 `lib/VisionProtocol` 和 `lib/MaixCamClient`，当前 MaixCAM
实现位于 `vision/yanyanview5.py`。UART 为 115200 baud、8-N-1。

CRC 均为 CRC-8：poly `0x07`、init `0x00`、不反射、无 final XOR。标准校验
向量 `123456789` 的结果为 `0xF4`。

## STM32 到 MaixCAM

请求固定 6 字节：

```text
0xAA, 0x02, request_seq, mode, crc, 0xBB
```

`crc` 覆盖 `0x02, request_seq, mode`。`request_seq` 是 8 位序号；同一请求的
周期重发保持原序号，开始新的检测请求才递增。

当前模式定义：

- `0`：停止检测。
- `1`：只识别红色，成功时 `target=1`。
- `2`：只识别黄色，成功时 `target=2`。
- `3`：只识别蓝色，成功时 `target=3`。
- `4`：只识别绿色，成功时 `target=4`。
- `8`：同时识别四种颜色，每个请求最多返回一个稳定目标；`target=1..4`
  依次表示红、黄、蓝、绿。
- `9`：霍夫圆中心目标。优先按三圆整体选中间环；视野裁切时允许受中心
  距离、半径和歧义门槛约束的单圆回退。成功时 `target=2`。
- `10`：端点单圆。机械臂把 1 号或 3 号候选移到画面中心附近后识别；协议
  字段固定返回 `target=1`，实际正在扫描 1 号还是 3 号由 STM32 当前状态决定。

模式 1 至 4 是第二次暂存区的定向颜色模式。该阶段已从二维码计划知道当前位置
应找的颜色，因此相机不把其他颜色当作当前候选。

## MaixCAM 到 STM32

响应是换行结尾的 ASCII。CRC 覆盖 `*` 之前的全部 ASCII 字节：

```text
V2,seq,mode,status,target,x,y,metric,confidence,timestamp*HH\n
```

字段含义：

- `seq`：对应的请求序号。
- `mode`：对应的请求模式。
- `status`：`0 OK`、`1 NO_TARGET`、`2 AMBIGUOUS`、`3 UNSTABLE`、
  `4 CAMERA_ERROR`。
- `target`：按模式定义的目标身份。
- `x=0..319`、`y=0..239`：320×240 坐标系中的目标中心。
- `metric=0..65535`：颜色面积或圆半径等模式相关测量值。
- `confidence=0..1000`：稳定性启发式分数，不是概率。
- `timestamp`：相机端毫秒计时的无符号 32 位值。
- `HH`：恰好两位十六进制 CRC，解析器兼容大小写。

STM32 只有在 CRC、字段数、数值范围、序号、模式、状态和目标身份都通过时才
提交坐标。超长行会整帧丢弃到下一个换行，不解析残余尾段。延迟帧、串台帧、
重复旧序号和与当前定向颜色不符的结果都会被拒绝。

## 当前发送行为

当前相机脚本只在稳定结果成立时发送 `OK`。无目标、歧义和不稳定通常不发送
NO_TARGET/AMBIGUOUS/UNSTABLE 帧，而是保持静默，由 STM32 等待或按对应状态机重试。
`CAMERA_ERROR` 是故障结果，会触发整场停机。

当前稳定规则摘要：

- 模式 1 至 4：2 个有效帧，坐标跨度不超过 12 px。
- 模式 8：300 ms 窗口内至少 5 个有效样本，坐标跨度不超过 3 px。
- 模式 9：3 帧快速确认或 4 帧完整确认；快速跨度 1 px，完整跨度 3 px。
- 模式 10：2 帧快速确认或 3 帧完整确认；快速跨度 1 px，完整跨度 2 px。

完整识别门槛、半径范围和诊断方法见 `vision/README.md`。协议字段定义与视觉
算法门槛是两层内容，修改其中一层时必须检查另一层是否仍兼容。

## 物理连接

- STM32 `SerialMaixcam`：PE7 RX、PE8 TX，115200 baud。
- STM32 PE8 TX 接 MaixCAM A17 RX。
- MaixCAM A16 TX 接 STM32 PE7 RX。
- 两端共地，使用 TTL UART 电平。

STM32 `HardwareSerial` 构造函数的参数顺序是 RX、TX。
