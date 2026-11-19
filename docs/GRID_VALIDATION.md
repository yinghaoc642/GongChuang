当前两区镜像三朝向扫描见[镜像扫描验证报告](scan60_mirrored_validation_20261119/README.md)。
此前两区同向三朝向扫描见[历史三朝向验证报告](scan60_validation_20261119/README.md)。
下方为旧两扫实现的历史记录，不能作为当前扫描时序证据。

# 网格避障验证记录

2026-11-19新增整体验证：全部12个固件环境构建通过；当前38脚本29通过/9项既有失败，
另5个原生测试通过。完成7,418个导航事件场景和3,905,000次两点规划请求。
完整任务成功率、固定扫描位置限制、首扫空图盲区、测试范围和原始数据见
[整体验证报告](simulation_20261119/README.md)。以下保留11月18日的验证记录。

日期：2026-11-18。基线：`e1a285cef19e081a0f6cece95cf8ff5ab72cd12f`。

验证对象是工作区中的实际源码；未烧录或连接实车。

## 固件构建

七个环境全部通过，使用本机已有依赖；新环境首次联网下载失败后，复制现有同版本AccelStepper/OneButton到新环境依赖目录，再完成构建。没有修改Flash容量上限。

| 环境 | Flash / 131072 bytes | RAM bytes |
|---|---:|---:|
| `genericSTM32H750VB` | 126028 | 13444 |
| `debug_workstation` | 112248 | 11800 |
| `debug_storage2_workstation` | 117968 | 11808 |
| `debug_translation` | 55660 | 6092 |
| `debug_chassis_route` | 55316 | 6080 |
| `debug_grid_navigation` | 61448 | 7468 |
| `legacy_route` | 128872 | 12032 |

正式剩余Flash **5044 bytes**。构建包含既有AccelStepper `boolean` 弃用提示及LTO串行链接提示，无编译/链接错误。

```sh
pio run -e genericSTM32H750VB -e debug_grid_navigation -e debug_chassis_route -e debug_translation -e debug_workstation -e debug_storage2_workstation -e legacy_route
```

受当前沙盒影响，本机会话使用 `/Users/chenyinghao/.platformio/penv/bin/pio`，并将PlatformIO Core/Cache指向 `/private/tmp/gc_publish_pio_2xu9xo4w` 下的已安装依赖镜像。

## 运行时模式与两区新增验证

- `python3 test/navigation_mode_test.py`执行实际HMI接收、PB9双击回调及底盘首次非零脉冲入口，测试默认RADAR、HMI3/4、区1/2选择、启动后锁定、扫描失败不能转CLEAR、料盘只触发一次。
- 实际比对19条命令表与11月2日提交逐字一致；原路线距离/速度/两区返程C++测试继续通过。
- `qiming_grid_test.py`增加区1/2的有效空扫描原地交接，永久岛不算可变障碍；确认仅扫描一次、无底盘位移/360°/SET_POSE，起点保持原值。
- 生产模式有障碍的区2完整任务通过：后退150 mm、右移200 mm、入口0格中心(240,240)、两次扫描并集、六工位、回区2保存位置。区1入口20格同样校验。
- `grid_departure_tray_test.py`改为运行时条件验证：显式LEGACY原归位与调用方等待时序保留，RADAR的FIXED_CLEAR/GRID在首次真实离站轮命令停车盘。
- 正式固件同时包含两套控制器后首次超出Flash 10168 bytes；仅将常规诊断输出置于GONGCHUANG_VERBOSE_TRACE条件内，保留完整文本和故障/模式/关键导航日志。运动与互锁不在宏内；容量未扩大。

## 新增实际C++行为测试

- `python3 test/qiming_grid_test.py`：编译实际协议/导航和实际底盘桥接，启用AddressSanitizer、UndefinedBehaviorSanitizer及警告当错误。
- 两个启停区的空地图、中心障碍及前后两次地图不同场景，六次工位、六次离站、二维码新窗口、完整360°、传感器84 mm换算及回起点均通过；工位在格中心的零位移不提前消费离站料盘触发。
- 实际COBS/CRC帧逐字节喂入；校验ACK不等于完成、节点/版本/重复ID/旧扫描/错误路径序号、空扫描、保存记录不变、溢出、丢链和超时。
- 格中心四邻路径、压缩路径中间格、固定岛、整格占用、旋转圆盘及坐标物理方向检查通过。
- 实际桥接检查PC11 RX/PC10 TX、发送缓冲不足时不阻塞、360°累计目标、末端新IMU帧锁定、无旋转净空处的零角度锁定，以及IMU/偏航/运动超时故障。
- `python3 test/grid_departure_tray_test.py`：执行实际延后停车函数，区分网格与旧/工位模式，检查首次离站下发、途中不覆盖槽位0和急停STOP。
- `python3 test/grid_workstation_tracking_test.py`：执行实际工位轮脉冲转换与拦截函数，使用真实MecanumKinematics库；校验M3/M4车头、车右和顺逆时针符号，以及不安全/无IMU时拒发电机命令。

## 全量脚本结果与基线对照

运行 `test/*_test.py` 共37个脚本：**28通过，9失败**。9个失败脚本均在临时目录解出的、未修改的11月2日提交上重跑，9个均同样失败。未将这些失败记作通过。

完整每个脚本输出保存在 [grid_test_results.json](grid_test_results.json)；基线失败输出保存在 [grid_baseline_failures.json](grid_baseline_failures.json)。新增脚本和受本次变化影响的集成/工位断言已按实际模式更新。

以下保留失败原文首个诊断（完整堆栈见JSON）：

`arm_soft_landing_test.py`（11月2日基线同样失败）：

```text
ARM SOFT-LANDING CHECK FAILED: target placement does not apply exactly one extra 5 mm M7 descent
```

`debug_raw_workstation_test.py`（11月2日基线同样失败）：

```text
  File "/Users/chenyinghao/Documents/University/Projects/GongChuang_2026/code/GongChuang/test/debug_raw_workstation_test.py", line 45, in <module>
    assert "local legacy fixed-A pickup started" in MAIN
AssertionError
```

`m6_endpoint_reach_recovery_test.py`（11月2日基线同样失败）：

```text
  File "/Users/chenyinghao/Documents/University/Projects/GongChuang_2026/code/GongChuang/test/m6_endpoint_reach_recovery_test.py", line 43, in <module>
    handoff_start = source.index("      if (ring == 1U) {", source.index(
ValueError: substring not found
```

`m7_double_experiment_test.py`（11月2日基线同样失败）：

```text
M7 DOUBLE EXPERIMENT CHECK FAILED: round-2 STORAGE does not use normal high-speed M7 with 300 ms head start
```

`process_gripper_m7_interlock_test.py`（11月2日基线同样失败）：

```text
WORKSTATION INTERLOCK CHECK FAILED: M6/M7 encoder fast-arrival bypass is incomplete
```

`qr_startup_checkpoint_test.py`（11月2日基线同样失败）：

```text
  File "/Users/chenyinghao/Documents/University/Projects/GongChuang_2026/code/GongChuang/test/qr_startup_checkpoint_test.py", line 50, in <module>
    assert (
AssertionError
```

`storage2_debug_scan_pose_correction_test.py`（11月2日基线同样失败）：

```text
  File "/Users/chenyinghao/Documents/University/Projects/GongChuang_2026/code/GongChuang/test/storage2_debug_scan_pose_correction_test.py", line 68, in <module>
    assert (
AssertionError
```

`storage_round2_local_mode_test.py`（11月2日基线同样失败）：

```text
  File "/Users/chenyinghao/Documents/University/Projects/GongChuang_2026/code/GongChuang/test/storage_round2_local_mode_test.py", line 53, in <module>
    assert (
AssertionError
```

`yan2_mode1_red_shape_test.py`（11月2日基线同样失败）：

```text
FileNotFoundError: [Errno 2] No such file or directory: '/Users/chenyinghao/Documents/University/Projects/GongChuang_2026/code/GongChuang/yan_2/main.py'
```

`yan2_mode1_red_shape_test.py` 的10个用例均因当前仓库缺失 `yan_2/main.py` 报错。九项逐条原因见[失败解释](TEST_FAILURES_EXPLAINED.md)。本轮同九脚本失败；其中arm_soft_landing的首个失败提前到下降日志断言，旧的三参数放置调用断言仍不匹配。其余失败为早期调用形式、源字符串或状态机精确表达式断言。这里保留失败事实，不以修改无关机械臂行为的方式迁就旧测试。

## 保留与限制

- 原厂 `link_protocol.h` 与 `LinkProtocol.cpp` 已分别与F407例程对应文件计算SHA-256，逐字节一致。
- `git diff --check` 通过。原19条路线表未重排、复制，历史 `backups/` 未改。
- 本地11月2日存档分支、bundle、ZIP和10月24日回滚分支保留；此次实现未推送远端。
- 串口真实通信、RadarViewer布局/校准、真实运动方向、抓放可达性、轮胎打滑和载料轮廓均需实车确认。
- 当前模型按车身290×234 mm、5 mm净空；行进机械臂/料盘/载料若伸出，应按实际最大轮廓增大参数。
- RADAR无可变障碍从未移动的原起点走11月2日路线；有障碍用两次扫描静态并集避障。没有行进中移动障碍识别或连续绝对X/Y定位。
