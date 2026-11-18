# Gongchuang STM32H750 搬运机器人

当前有效工程：`C:\Users\Administrator\Documents\PlatformIO\Projects\Gongchuang (6)\Gongchuang`

## 当前入口

- 正式 PlatformIO 环境：`genericSTM32H750VB`
- 正式源码：`src/main.cpp`
- 唯一底盘路线实现：`src/route_chassis_main.inc`
- 当前默认：启停区1、普通出发（`GONGCHUANG_ZONE1_ROUTE_MODE=0`）、先短后长返程、完整比赛运行模式
- 第7段与第一次E→暂存区F（第11段）在上一版实际行程上各增加15 mm，当前分别为1861.81965 mm、840.175 mm；F→G（第12段）仍为936.75 mm。
- 全局放置：两次粗加工和两次暂存的三个位置均在已有修正后追加顺时针1°、回缩5 mm及伸长15 mm（净伸长10 mm），正式与工位调试一致。
- 全局2号位放置：已有修正后的标准角度严格超出−3°～+3°时，再回缩2 mm、顺时针0.3°；恰好±3°不追加。
- 第二次暂存区放置：相对上一版，1号位再回缩5 mm、顺时针0.5°；3号位再回缩5 mm、逆时针0.5°。
- 料盘取放：统一以M5 −103°、M6 0 mm为基准，追加顺时针1°、伸长4 mm，最终平面目标为−104°/4 mm，取放升降高度保持各自原值。

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
- `debug_workstation`：粗加工/第一次暂存/第二次暂存，默认第一次暂存区（状态2，第一层放置）；放置校正与正式主程序共用，视觉标定不变。
- `debug_raw_workstation`：原料区，默认旧 A 点逻辑。
- `debug_raw_workstation_341`：原料区 3→4→1 通信复现，旧 A 点逻辑，底盘不移动。
- `debug_storage2_workstation`：固定第二次暂存区并输出详细日志。
- `debug_translation`：单个规范段，默认第 4 段旧式 190 mm 扫码等待。
- `debug_chassis_route`：完整纯底盘路线，默认启停区1普通出发，第4段保留扫码条件停车；返程先短纵移再长横移，最后补回150 mm。
- `debug_maix_mode8`：MaixCAM 模式 8 通信。
- `debug_maix_protocol_cycle`：正式 Maix 客户端的 STOP→3→STOP→4→STOP→1 最小循环。

烧录时把环境名代入：

```powershell
pio run -e <环境名> -t upload
```

调试入口不会自动进入正式固件，也不应由正式程序调用。
