# Gongchuang STM32H750 搬运机器人

当前有效工程：`C:\Users\Administrator\Documents\PlatformIO\Projects\Gongchuang (6)\Gongchuang`

## 当前入口

- 正式 PlatformIO 环境：`genericSTM32H750VB`
- 正式源码：`src/main.cpp`
- 唯一底盘路线实现：`src/route_chassis_main.inc`
- 当前默认：启停区 1、新路线状态机、完整比赛运行模式

构建正式固件：

```powershell
pio run -e genericSTM32H750VB
```

烧录正式固件：

```powershell
pio run -e genericSTM32H750VB -t upload
```

## 先读哪些说明

1. `CURRENT_VERSION.md`：当前路线、二维码、IMU、工位、视觉、修正量和烧录命令。
2. `AGENTS.md`：后续维护代码时必须遵守的入口、共享参数和验证约束。
3. `文件说明资料/代码修改索引.md`：出现具体问题时应该去哪个文件修改。
4. `vision/README.md`、`vision/protocol.md`：MaixCAM 算法和 UART 协议。

## 历史材料

以下内容只用于追溯，不能用于判断当前固件：

- `backups/`
- `COMPETITION_BASELINE.md`
- `文件说明资料/` 中带版本号或日期的旧报告
- `src/yanyanversionnew*.inc`
- `src/*disabled`
- `true_example/`

实际编译文件永远以 `platformio.ini` 的 `build_src_filter` 为准。

## 独立调试环境

- `debug_qr_continuous`：持续二维码。
- `debug_workstation`：粗加工/第一次暂存/第二次暂存，默认第一次暂存区（状态 2，第一层放置）。
- `debug_raw_workstation`：原料区，默认旧 A 点逻辑。
- `debug_raw_workstation_341`：原料区 3→4→1 通信复现，旧 A 点逻辑，底盘不移动。
- `debug_storage2_workstation`：固定第二次暂存区并输出详细日志。
- `debug_translation`：单个规范段，默认第 4 段旧式 190 mm 扫码等待。
- `debug_chassis_route`：完整纯底盘路线，默认启停区 1 新状态机。
- `debug_maix_mode8`：MaixCAM 模式 8 通信。
- `debug_maix_protocol_cycle`：正式 Maix 客户端的 STOP→3→STOP→4→STOP→1 最小循环。

烧录时把环境名代入：

```powershell
pio run -e <环境名> -t upload
```

调试入口不会自动进入正式固件，也不应由正式程序调用。
