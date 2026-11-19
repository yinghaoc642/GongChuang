# MaixCAM 启动崩溃修复结果

2026-11-19。本文件是本次相机排查的最终结果；README.md 与 CONTINUATION.md 前半部分保留各阶段当时的状态。

## 已完成与证据

屏幕的 `log in /maixapp/tmp/last_run.log` 是日志位置提示。原日志中的实际故障为 `Trigger signal, code:SIGSEGV(11)!`。它发生在 NN 加载链路，Python 的异常处理不能捕获这个原生段错误。

| 运行环境 | 官方 YOLOv5 | 项目模型 331476 | 相机采集/推理与完整主程序 |
| --- | --- | --- | --- |
| 原系统运行库 1.24.0 | SIGSEGV | SIGSEGV | 独立相机采集正常；模型加载失败 |
| GitHub main 公开运行库，临时覆盖路径 | SIGSEGV | 未重复测试 | 不构成有效修复 |
| 官方为本机返回的 1.24.0，临时覆盖路径 | 加载正常，进程退出 0 | 加载正常 | 三帧采集与推理正常，进程退出 0 |
| 同一本机配套库安装至系统原路径，不设 LD_LIBRARY_PATH | 加载正常，进程退出 0 | 加载正常 | 三帧推理正常；完整主程序约 10 秒采集 602 帧、显示 192 次，进程退出 0 |

完整主程序验证通过 `runpy.run_path()` 执行设备上未修改的 `main.py`，在安装目录、正常库搜索路径下运行。外部行跟踪器在 `cam.read()` 返回后累计采集次数，在 `disp.show(img)` 返回后累计显示次数，并记录视觉协议发送调用和模式。采集运行约 10 秒后，由外部测试调用已安装 API `app.set_exit_flag(True)` 正常退出。记录为模式 `[0]`、`vision_uart_write_calls=0`。没有发送识别请求，也没有连接或操作 STM32。这证明短时启动与空闲采集显示循环通过，不等于所有识别分支或长时间稳定性已经验证。

三个检测结果是 `[None, None, None]`，仅证明这三次推理完成且没有有效稳定目标。不能据此宣布准确率、圆环识别效果、实际粗定位或整车流程通过。

原始日志：

- [正常环境官方模型](logs/gc_normal_official_model.log)
- [正常环境项目模型与三帧推理](logs/gc_normal_try_inference.log)
- [完整主程序启动和帧数](logs/gc_gc_try_full_startup.log)
- [临时库下项目模型对照](logs/gc_candidate_try_inference.log)

结论边界：实测证明原运行库下 NN 加载失败，官方本机配套库消除了本次崩溃。旧库为何不适配尚未确定；未证明是模型损坏、SD 卡移植、授权机制异常或硬件损坏。

## 实际修改与版本

设备报告：MaixCAM-Pro，系统标记 `maixcam-pro-2026-01-24-maixpy-v4.12.5`。

- MaixPy 仍为 `4.12.5`，未升级 Python 包或重刷系统固件。
- 配套运行库仍为 `1.24.0`，仅更新 `/usr/lib/libmaixcam_lib.so.1.24.0` 为官方针对本机返回的文件。保留原权限 `0775`、属主和 `/lib/libmaixcam_lib.so` 软链接。
- 当前库 SHA-256：`33a6e0ca2f9fe59bd8b7a6f7bd28f03730fd8db58c8bf870fd2ad963615f83da`。
- 原库 SHA-256：`1fd38cfc0802bcaee00f5194b18acc22e3c01d35cdb4c0e7f6df291aed20c236`。
- 官方接口返回与下载安装器协议已经实测核对；用户明确允许将本机 `/device_key` 仅发送到 Sipeed 官方 `https://maixvision.sipeed.com`。设备标识未写入报告、未打印，未关闭 TLS 检查。

已安装应用：名称 **GC Try**，ID `gc_try_r2_331476_20261119`，安装包版本 `5.6.1`。脚本仍为 `5.6.0-try-coarse-ring2`，视觉协议仍为 v2。只改应用名称和包版本，脚本及模型均逐字节保持原样。

MaixVision 能读到连接成功，但原生目录对话框多次无法完成路径确认。因此**此次新版未通过 MaixVision 的 Package/Install 按钮安装**。在核验已安装脚本和模型与新版完全一致后，改用 SSH 原子更新自身 `app.yaml` 和 `app.info` 中这一应用的登记段；设备 `app.get_app_info()` 已实测返回 `GC Try 5.6.1 main.py`。其它应用登记段逐字节未变。

可再次安装的独立包位于 `vision/deploy/maixvision_gc_try_331476_v5_6_1_20261119/dist/maix-gc_try_r2_331476_20261119-v5.6.1.zip`：

- 6,166,994 bytes
- SHA-256 `e2c90523e6d3fa896545a5ec210c95dfaa000c8046a78527dcca8ed72d923e41`
- 包内五个文件均已解包校验；**包内不含本机专用运行库**。
- 旧 5.6.0 部署目录和压缩包保留。

## 保留项与回滚

新恢复目录：`vision/deploy/maixcam_recovery_20261119_runtime1240/`。未改任何既有 `backups/`。

恢复归档 `camera_before_runtime_refresh.tar`：181,549,568 bytes；SHA-256 `c318353dd0577b04cdbcfad01eabe9486e4c15ff6c7d3eaa0d6c88b9fa72ba8b`。包含原运行库、软链接、库版本文件、原应用 `xg13`、当前 TRY 应用、整个 `/root/models`、应用登记、自动启动/系统配置和系统/MaixPy版本文件。93 个归档条目已逐项校验，普通文件均以 SHA-256 与相机源文件比对，传输前后源清单一致。

修复后复核 93 项：90 项未变，只有运行库、自身 app.yaml、应用登记文件这 3 项按计划变化；登记文件中其它应用的部分逐字节未变。原 `xg13`、全部 `/root/models`、开机自启、系统配置、代码和模型均未覆盖。详见恢复目录的 `repair_verification.json` 与前后清单。

设备额外保留原库：`/usr/lib/libmaixcam_lib.so.1.24.0.gc-before-20261119`。原应用元数据另存于 `/maixapp/gc_recovery_20261119_runtime1240/`。恢复目录还保存了当前有效库 `working_runtime_1.24.0_for_this_camera.so`，仅供这台相机恢复使用。

回滚时先停止相机应用。将恢复目录中的 `restore_original_runtime_on_camera.py` 复制到相机后，在相机终端执行：

```sh
python3 /path/to/restore_original_runtime_on_camera.py
python3 /path/to/restore_original_runtime_on_camera.py --apply
```

第一条仅核验哈希；第二条核验后原子恢复原库，同时保留当前有效库副本。这里的 `/path/to/` 需替换成复制到相机的实际目录。脚本已通过 Python 语法检查，**没有执行回滚**，因为原库会重新引入已复现的 NN 崩溃。元数据和其它完整恢复内容在归档中，应按需恢复具体文件，不应在运行应用时全盘覆盖归档。

官方资料：[运行库安装示例](https://github.com/sipeed/MaixPy/blob/main/examples/tools/install_runtime.py)、[MaixCAM 快速入门](https://wiki.sipeed.com/maixpy/doc/zh/README_MaixCAM.html)。当前修复依据是本机对照实测；不声称这些安装版本是官方最新版本。
