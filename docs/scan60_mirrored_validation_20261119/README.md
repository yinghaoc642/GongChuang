# 2026-11-19：两区镜像扫描验证

区1在扫描点依次0°、逆时针60°、逆时针120°停稳扫描，随后顺时针120°复原。
区2镜像为0°、顺时针60°、顺时针120°停稳扫描，随后逆时针120°复原。
方向均相对发车车体朝向。启停区安全预扫、前置平移、四图并集及空图归位逻辑沿用。

## 改动与实际核对

- launchPlans增加scanStepDegrees：区1=+60、区2=−60；复原转角统一取−2×scanStepDegrees。
- 雷达临时位姿继续由实际车体航向计算，方向镜像同步作用于传感器偏置坐标和heading。
  不额外翻转radarHeadingPerCcw；该参数表示设备角度约定，两区共用。
- 真实C++任务测试逐步断言两次扫描转角、扫描数量、累计航向和相反方向的复原动作；
  覆盖两区、两种雷达角度符号和0/360°环绕；空图归位也断言两区转角。
- 底盘桥接包含真实radar_navigation.inc，核对+60/+60/−120及−60/−60/+120命令符号不丢失。
- 独立事件模拟器在每一次扫描请求核对雷达X/Y/heading，并检查旋转与扫描串行执行。

典型车体起点区1=(150,2250)、区2=(150,150)，雷达中心右偏84 mm时：

| 扫描位置 | 区1雷达中心mm | 区2雷达中心mm |
|---|---|---|
| 启停区预扫 | (234,2250) | (234,150) |
| 扫描点0° | (434,2100) | (434,300) |
| 第一次转60° | (392,2027)，CCW | (392,373)，CW |
| 累计转120° | (308,2027)，CCW | (308,373)，CW |

这是发给设备的取整坐标，来自当前几何模型，尚未经过实车测量。

## 本轮验证范围

- qiming_grid_test.py通过：协议、两区六工位及返程、空图交接、雷达位姿、底盘桥接，ASan/UBSan无报错。
- navigation_mode_test.py、grid_departure_tray_test.py、grid_workstation_tracking_test.py通过，原19条路线表仍与11月2日基线逐字相同。
- grid_scenarios_test.py重新运行7,490个场景，断言全部通过：1,838个六工位入口完成并返回、
  2个空图归位交接、5,650个预期故障锁定。每组完成/故障数量与镜像前报告一致。
  “断言通过”包括按预期拒绝不安全路线，不是所有布局都能完成比赛。
- 其中108种正常时序、首扫空但后续发现障碍、30种位姿更新故障、36种后续扫描故障均保留验证。
- 本轮仅重跑受影响检查，没有重跑全套38脚本和其他5原生模块；此前九项既有失败见
  [上一轮完整记录](../scan60_validation_20261119/README.md)，不能据本轮针对性通过声称旧失败已修复。
- 正式genericSTM32H750VB与debug_grid_navigation两个环境构建成功；正式Flash/RAM 126360/13448 bytes，Flash剩4712。
  雷达底盘调试Flash/RAM 61788/7472 bytes。未增大131072字节Flash上限。
  构建结果及容量在firmware_build.json、firmware_sizes.json，完整原始输出在firmware_build.log。
  末尾PlatformIO升级联网检查失败不影响构建exit=0。

## 原始数据与复现

```sh
python3 test/qiming_grid_test.py
python3 test/navigation_mode_test.py
python3 test/grid_departure_tray_test.py
python3 test/grid_workstation_tracking_test.py
python3 test/grid_scenarios_test.py --output docs/scan60_mirrored_validation_20261119
```

- targeted_tests.json：四项实际运行命令、退出值、stdout/stderr。
- grid_scenarios.jsonl / grid_summary.json / grid_run.json：逐场景雷达位姿、结果、汇总及编译信息。
- source_fingerprints.json：本轮被测源码校验值。

未烧录、未提交、未推送，历史存档未修改。测试是主机模拟，不是雷达/电机实测。
设备heading正方向仍需用已知障碍实机核对；多朝向扫描也不能保证遮挡后方均可见。
