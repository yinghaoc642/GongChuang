# Vision protocol v2

UART：115200 baud，8-N-1。CRC 均使用 CRC-8，poly `0x07`、init `0x00`、
不反射、无 final XOR。标准校验向量 `123456789` 的结果为 `0xF4`。

## STM32 → MaixCAM

固定 6 字节：

```text
0xAA, 0x02, request_seq, mode, crc, 0xBB
```

`crc` 覆盖 `0x02, request_seq, mode` 三个字节。`request_seq` 是 8 位
序号；同一请求的周期重发保持同一序号，新检测请求才递增。

模式：

- `0`：停止；
- `8`：四种原料颜色；
- `9`：优先按三圆整体选择中间环；视野裁切时允许带中心距离与歧义保护的
  单圆回退。

## MaixCAM → STM32

响应为换行结尾的 ASCII；CRC 覆盖 `*` 之前的全部 ASCII 字节：

```text
V2,seq,mode,status,target,x,y,metric,confidence,timestamp*HH\n
```

- `status`：`0 OK`、`1 NO_TARGET`、`2 AMBIGUOUS`、`3 UNSTABLE`、
  `4 CAMERA_ERROR`；
- 模式 8 的 `target=1..4`，依次为红、黄、蓝、绿；
- 模式 9 的 `target=2`，表示三圆整体选择或受限单圆回退得到的中间 2 号环；
- `x=0..319`、`y=0..239`；
- `metric=0..65535`；
- `confidence=0..1000`，当前是稳定性启发式分数而非概率；
- `timestamp` 为相机端毫秒计时的无符号 32 位值；
- `HH` 恰好两位十六进制，解析器兼容大小写。

STM32 只有在 CRC、字段数、范围、序号、模式、状态和目标身份全部通过时
才提交坐标。超长响应会整帧丢弃到下一个换行，不会解析残余尾段。

当前相机只在稳定结果成立时发送 `OK`；无目标、歧义和不稳定仍由静默等待
配合 STM32 的 12 秒单次超时和有限重试处理。`CAMERA_ERROR` 会触发整场
故障停机。
