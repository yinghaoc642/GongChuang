# 2026-11-19：0°/60°/120°停稳扫描验证

当前实现：启停区14圈安全预扫 → 向场内150 mm（区2后退）→ 右移200 mm →
0°14圈扫描 → CCW60°停稳14圈扫描 → 再CCW60°停稳14圈扫描 → CW120°复原。
有障碍进入入口格；四图无可变障碍则反向平移回原启停位置，恢复雷达会话位姿后交11月2日路线。
SCANNING期间准备运动也由网格控制器独占，不在首次MOVE时提前接受地图。

每个临时位姿先SET_POSE成功ACK，再HELLO读回确认X/Y/heading，然后才允许扫描。
每次扫描须收到ACK、14圈完成状态、GRID及新保存记录一致才消费。
STM32保留四次结果的并集，雷达PLAN_PATH仍只看到最后一图，因此返回路径另在并集重新BFS。
没有发送SAVE_POSE，不覆盖两个启停区保存的标定值。

## 证据范围

- 已实际编译运行正式协议/导航/模式C++，ASan/UBSan及-Werror；底盘桥接包含真实radar_navigation.inc。
- 独立雷达模型用另一份BFS返回路径；独立几何检查逐MOVE/TURN的车身扫掠、黄色实际矩形及障碍整格。
- 模型断言位姿与扫描不在运动中发送；延迟运动/ACK/扫描、两区、分片和计时器回绕均覆盖。
- 原生测试核对扫描点雷达中心(434,Y)、(392,Y−73)、(308,Y−73)，两种heading符号及跨0/360°环绕。
- 模式测试实际提取并执行生产serviceGridMission，确认准备运动保持SCANNING，归位后可切FIXED_CLEAR，
  有障碍地图接受后才切GRID。原19条路线表与e1a285c逐字一致。
- 没有雷达固件源码或真实设备连接，未运行RadarViewer.exe/雷达固件，未做STM32指令级或物理整机仿真。
  这里WORK表示正确触发工位入口，不表示每场景模拟了完整抓放或相机图像。

## 导航场景

共7,490个场景断言通过：1,838个完成六工位入口并返回，2个空图完成归位和旧路线交接，5,650个故障锁定。
后者包括故意放在必经格的障碍、扫描点旋转净空不足、网格断开、无效摆位及注入故障，不能称为5,650个软件缺陷，
也不能以“断言通过”宣称这些布局都能完成比赛。故障后继续tick不再发出导航动作。

| 场景 | 数量 | 六工位并返回 | 空图交接 | 故障锁定 |
|---|---:|---:|---:|---:|
| static_0_to_3 | 3124 | 848 | 2 | 2274 |
| first_only_0_to_3 | 3122 | 848 | 0 | 2274 |
| split_random_4_to_10 | 1024 | 18 | 0 | 1006 |
| timing_wrap | 8 | 8 | 0 | 0 |
| random_timing | 100 | 100 | 0 | 0 |
| start_pose | 18 | 8 | 0 | 10 |
| protocol_fault | 16 | 0 | 0 | 16 |
| qr_timeout | 2 | 0 | 0 | 2 |
| scan_total_timeout | 2 | 0 | 0 | 2 |
| empty_first_later_detection | 2 | 2 | 0 | 0 |
| obstacle_in_one_view | 6 | 6 | 0 | 0 |
| pose_update_fault | 30 | 0 | 0 | 30 |
| later_scan_fault | 36 | 0 | 0 | 36 |

关键新增结果：

- 首扫为空、三个朝向仅某一个检测到12号格障碍：两区均进入网格任务并完成，没有提前走原路线。
- 30种位姿故障：两区×5个位姿更新位置×丢ACK/负ACK/HELLO读回错；均故障，未错误交接。
  第5次更新仅在空图回原起点时发生，前4次覆盖三个扫描姿态和恢复0°。
- 36种后续扫描故障：两区×3个扫描朝向×缺GRID/溢出/坏CRC/保存记录不更新/负ACK/缺ACK；均故障。
- 108种正常通信/运动/二维码/工位延迟与回绕场景全部完成。
- 后续扫描为空不会清除较早观测，记录逐扫位姿和预期地图，便于复查。

静态单/双/三障碍格的完整任务完成数每区仍为14/21、91/210、319/1330。
枚举包括发车、二维码及工位所在格被占用，不能换算为现场成功率。
静态地图2,274个停止中：422个起步移动不安全、726个扫描点旋转净空不足、1,018个必经目标格占用、108个网格断路。
相比旧报告，拆分随机地图完成数28变18：新流程在第一次转60°之前就合并扫描点0°新发现障碍，
旋转净空不满足时拒绝；旧流程在新增地图到达前已转完360°。这不是寻路器漏找路径。

## 全量检查与固件

38个当前脚本29通过、9失败；额外5个原生模块全通过。九项与既有失败集合一致：
- `arm_soft_landing_test.py`，exit=1。
- `debug_raw_workstation_test.py`，exit=1。
- `m6_endpoint_reach_recovery_test.py`，exit=1。
- `m7_double_experiment_test.py`，exit=1。
- `process_gripper_m7_interlock_test.py`，exit=1。
- `qr_startup_checkpoint_test.py`，exit=1。
- `storage2_debug_scan_pose_correction_test.py`，exit=1。
- `storage_round2_local_mode_test.py`，exit=1。
- `yan2_mode1_red_shape_test.py`，exit=1。

失败的原始stdout/stderr和堆栈完整保存在existing_tests.json，解释见[九项失败说明](../TEST_FAILURES_EXPLAINED.md)。
simulation_suite.py因这九项返回1，未隐去失败或改无关机械臂逻辑。
其余5个原生程序包括任务码、视觉协议、麦轮里程计、协议压力与3,905,000次两点寻路枚举。

全部12个PlatformIO环境SUCCESS，未增大131072字节Flash上限。
正式Flash/RAM 126336/13448 bytes，Flash剩4736；雷达底盘调试61768/7472 bytes。
构建有既有boolean弃用和LTO提示；末尾升级检查联网失败未影响构建exit=0。
没有烧录、提交、推送或改动存档。

## 未验证的物理条件

厂家公开命令表支持停稳后改会话位姿再扫描，1.36/1.37日志明确仍为v6/v11。
但文档没有定义heading正方向。radarHeadingPerCcw默认−1来自镜像地图数学XY约定推理，
不是厂家明确规定；两种符号都能在主机模型跑通，不能据此选择真实设备的符号。
必须以已知障碍验证转60°前后落格一致后确定该参数。

三朝向扫描不保证全场可见：车体与其他物体背后仍可能遮挡；向扫描点移动的短途也可能存在首扫漏检。
没有自由空间/未知格验证、动态障碍保护或X/Y绝对定位。当前同一点旋转及软件脉冲归位保留这些限制。
旋转仍用整圆净空检查，比较保守；区1邻19/21、区2邻1/9有障碍可能拒绝动作，不会另选扫描点或钻缝。

## 原始数据与复现

```sh
python3 test/simulation_suite.py --output docs/scan60_validation_20261119
```

此命令因九项既有失败返回1；仅导航用grid_scenarios_test.py。

- grid_scenarios.jsonl / grid_summary.json：7,490场逐项记录与汇总；expected_scan_maps依次为预扫/0°/60°/120°。
  scan_poses为已请求的每次扫描雷达位姿，pose_updates包含复原及空图归位更新。
  moves字段统计MOVE和TURN动作总数；虚拟耗时不是实车耗时。
- existing_tests.json / native_modules.json / suite_summary.json：每项实际命令结果和原始输出。
- firmware_build.log / firmware_build.json / firmware_sizes.json：构建日志、命令及容量。
- source_fingerprints.json / provenance.json：被测源码SHA-256及分支/HEAD。
