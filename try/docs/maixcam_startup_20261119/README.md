# MaixCAM 启动故障排查（进行中）

## 范围与状态

用户已授权相机端多次短运行、模型加载对比和修复后启动验证。仅检查 MaixCAM；不接 STM32、不发送机器人运动指令、不覆盖设备原有应用或模型。原 GongChuang 工程与历史存档未改动。

**当前尚未修复。** 本目录是调查记录，不是通过验收的修复版本。应用短英文名计划使用 `GC Try`，尚未安装改名后的包。

## 已确认的事实

- 屏幕提示 `log in /maixapp/tmp/last_run.log` 是日志位置提示，不是登录错误。
- 最新读取的 `/maixapp/tmp/last_run.log` 含 `Trigger signal, code:SIGSEGV(11)!`。用户同时观察到屏幕 `exit code: 1`。
- 实机 MaixPy 为 4.12.5；`/maixapp/maixcam_lib.version` 为 1.24.0，二者是不同组件版本。
- 设备模型为 `LicheeRV Nano`，已检查的模型头为 `CviModel` / `cv181x`。
- 电脑和设备上的部署模型 SHA-256 一致，不能用上传损坏解释故障。
- `nn.YOLOv5()` 空对象创建成功；实际加载设备自带官方模型时发生段错误。
- `nn.MUD("/root/models/yolov5s.mud")` 已单独实测成功，输出 `MUD OK cvimodel ['basic', 'extra']`，shell 返回 `mud_exit=0`。

## 实机对比

以下 1–4 的结果由前一段连续排查中观察到的设备终端输出转录；5–6 为本段 SSH 直接读取。未收到输出的 UI 输入不计为一次测试。

| 检查 | 结果 |
| --- | --- |
| 1. 本项目模型，`dual_buff=False` | 打印到 `CHECK model`，随后 SIGSEGV |
| 2. 设备官方 YOLOv5 模型，`dual_buff=False` | 打印到 `CHECK official`，随后 SIGSEGV |
| 3. 本项目模型，使用默认 dual_buff | 打印到 `CHECK default`，随后 SIGSEGV |
| 4. 空 YOLOv5，再 `.load()` 官方模型 | `CHECK empty OK` 后 SIGSEGV |
| 5. 单独解析官方 MUD | 成功，返回 0 |
| 6. GDB 捕获官方 YOLOv5 加载 | 到 `LOAD` 后出现 `Killed`，未获得 backtrace；不能把它记为一次新的 SIGSEGV 实证 |
| 7. GDB `--readnever` 检查通用 `nn.NN` 加载 | 已启动，随后 MaixVision 断开，SSH 握手超时；结果未读到，不计为成功或失败 |

首次 GDB 检查未改变 auto-load safe-path；其安全提示不是模型加载报错。后续也未修改 GDB、系统或 SSH 权限配置。

## 结论与尚未证明的部分

已证明这不是本项目训练模型独有的加载问题，且 `dual_buff=False` 不是充分原因。MUD 解析可以独立成功，但实际 NN 加载的具体崩溃位置尚未取得。

底层推理运行库/初始化通路是下一步调查方向，尚不能宣布某个库版本不兼容、模型损坏、内存不足或硬件损坏。一次 `free -m` 显示总内存 128 MiB、available 73 MiB、无 swap；这是快照，不能证明崩溃时内存情况。内核日志筛选没有返回匹配信息。

第二次 GDB 期间设备连接失去响应；尚未确认是设备负载、用户断线还是其他原因。已请用户核对 USB 并在必要时重新接通。

## 模型与应用

- 当前应用目录：`/maixapp/apps/gc_try_r2_331476_20261119/`
- 当前本地部署目录：`vision/deploy/maixvision_try_ring2_331476_20261119/`
- MUD SHA-256：`e652404804f17d2f156d251f3144a51014dab3c4a21369af5bd7f94a136f6f1b`
- CVIMODEL SHA-256：`03695d0dc9c36a0cd2945343e128f8d914284175bd87e088058eea1515a2f813`
- 设备原应用 `xg13` 不使用 NN，故其能安装/原先能运行不能证明 NN 加载通路正常。本轮未运行或修改它。

## 官方依据

- [MaixPy Linux/SSH 说明](https://github.com/sipeed/MaixPy/blob/main/docs/doc/en/basic/linux_basic.md)：已按其中 MaixCAM 默认凭据连接，使用预先在设备端核对过指纹的 known-hosts 临时文件。
- [YOLOv5 官方示例](https://wiki.sipeed.com/maixpy/doc/en/vision/yolov5.html)
- [NN/MUD 官方源码](https://github.com/sipeed/MaixCDK/blob/main/components/nn/src/maix_nn.cpp)：main 分支用于理解调用关系，不等同于设备编译源码的逐字节证明。

后续需设备恢复连接后读取第二份 GDB 日志，确认无诊断进程残留；根据证据决定可逆修复。不得通过静默禁用 YOLO 或将启动成功写成识别验证通过来掩盖问题。
