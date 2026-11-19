# 九个既有测试失败的逐项说明

这些是九个**测试脚本失败**，不是九次固件编译失败，也不是实车报告的九个故障。
此前已把未修改的11月2日提交 `e1a285c` 解到临时目录，逐项运行同九个脚本，九个均失败。
原始输出见 `grid_baseline_failures.json`；当前工作区全量输出见 `grid_test_results.json`。

基线也失败只能说明它们不是本次雷达修改后才出现，不能证明机械臂在硬件上必然正确。
下面是实际读取测试断言、当前实现及基线后的具体原因。本次不以更改机械臂动作来满足旧断言。

| 脚本 | 检查内容 | 实际失败原因与证据 |
|---|---|---|
| `arm_soft_landing_test.py` | M7下降、放置额外下压、软着陆等约束 | 上轮基线失败处要求`applyTargetPlacementExtraLower(destination, …, ringPosition)`精确三参数调用；11月2日已增加第四个`secondStorageColorWorkflowActive()`参数，用于第二次暂存区最后统一限幅。额外5 mm仍在函数中减去并写回高度。当前首个报错提前到了旧料盘下降日志文字断言，因为上轮已缩短串口文本。该脚本不能据此被记作行为测试通过。 |
| `debug_raw_workstation_test.py` | 独立原料区调试入口和禁止底盘运动 | 精确查找日志`local legacy fixed-A pickup started`，实际文本为`local legacy fixed-pickup started`，在基线也如此。失败发生在日志匹配。 |
| `m6_endpoint_reach_recovery_test.py` | M6到达伸长边界后的恢复、移车和扫描接力 | 用旧日志`[ENDPOINT SCAN] overhead center accepted …`定位源码区间，再找`if (ring == 1U)`；日志已改成简写，`str.index()`找不到就抛`ValueError`，后面的原生行为测试尚未运行。不能据此声称该恢复行为已通过此脚本。 |
| `m7_double_experiment_test.py` | M7速度/加速度实验和暂存2前置动作 | 仍要求源码中出现`roundNumber==2U ? startLiftToHeightMm(...)`这类旧分支；基线已改为两轮共用`startLiftToHeightMmWithProfile(..., STORAGE_TRAVEL_M7_SPEED_RPM, STORAGE_TRAVEL_M7_ACCELERATION)`。300 ms前置时间常量和区2等待分支仍存在。测试期望的旧控制结构与基线不符。 |
| `process_gripper_m7_interlock_test.py` | 夹爪/M7互锁、M6/M7编码器提前确认 | 失败的复合断言中，启用开关、容差、状态枚举和共用验证调用均实际存在；缺失的是`FAST ARRIVAL] encoder stable;`、`bypass delayed driver`及旧注释全文。已有日志变为`stable encoder;skip late end…`。当前该脚本仍失败，不代表硬件编码器确认已实际验收。 |
| `qr_startup_checkpoint_test.py` | 500 ms启动等待、第4段二维码停车、接收时序 | 要求分支文本中直接出现`!zone1ReorderedRouteEnabled()`；基线已先计算局部变量`reorderedZone1`，对应位置使用`!reorderedZone1`。语义相关，但精确源码表达式不同；脚本在前置断言即停止，未执行后续模拟。 |
| `storage2_debug_scan_pose_correction_test.py` | 暂存2扫描姿态参数与坐标换算 | 旧测试要求调试入口再定义一份−5°/+5°/+30 mm宏；目前这些参数统一由正式`main.cpp`提供，调试入口只选模式与日志。仓库维护准则明确禁止重新复制参数。该失败源于测试仍要求旧的参数副本。 |
| `storage_round2_local_mode_test.py` | 暂存2独立调试与本地模式隔离 | 要求`LOCAL_WORKSTATION_SIMULATION_MODE`仅由原先三个模式组合；加入原料区本地调试后实际表达式多了`RAW_WORKSTATION_SIMULATION_MODE`。基线也已包含它，旧的精确组合断言失败。 |
| `yan2_mode1_red_shape_test.py` | 另一份相机脚本模式1红色形状识别 | 仓库没有`yan_2/main.py`，读取文件就报`FileNotFoundError`，10个用例均无法运行。它不等于正式`vision/yanyanview5.py`运行失败。 |

本次新增的运行模式、两个启停区、无障碍交给原路线、雷达协议、路径几何、底盘桥接和离站料盘测试均单独执行；结果见 `GRID_VALIDATION.md`。
