# 重新上电后的排查结果

本记录接续 README.md，不应将 README.md 中上一阶段的断线状态当成当前状态。

## 实测

1. MaixVision 重启后自动连接 `10.62.9.1`，界面显示 `connect successful`。SSH 恢复。
2. 设备重新上电，原 `/tmp` 日志消失。已重新采集日志，保存在本目录 `logs/`。
3. 不经 YOLO 封装，直接使用 `nn.NN(model="/root/models/yolov5s.mud", dual_buff=False)`，仍然 SIGSEGV，shell 返回 `nn_exit=1`。
4. 系统调用记录显示 MUD 已读完；随后在底层初始化阶段发生 SIGSEGV。该记录未出现打开模型 CVIMODEL 二进制的操作。
5. 用 `gdb --readnever` 并设置 `auto-solib-add off` 成功捕获崩溃。执行地址和返回地址异常，不能从单帧 `??` 宣布某个具体源码函数就是根因。未修改程序计数器、跳过认证或改写进程指令。
6. 相机独立采集成功，输出 `CAM OK 320 240`，返回 0。
7. 同一进程先初始化相机、读取一帧，再加载官方 YOLO 模型，仍然 SIGSEGV；初始化顺序不构成已验证的修复。

## 库对照

设备系统是 `maixcam-pro-2026-01-24-maixpy-v4.12.5`，`/boot/board` 标识 `maixcam_pro`。不能仅根据 device-tree 的 `LicheeRV Nano` 名称推断硬件为非 MaixCAM。

设备原运行库 `/lib/libmaixcam_lib.so` 指向 `/usr/lib/libmaixcam_lib.so.1.24.0`：

- 905840 bytes
- SHA-256 `1fd38cfc0802bcaee00f5194b18acc22e3c01d35cdb4c0e7f6df291aed20c236`

从 Sipeed 官方 MaixCDK main 仓库下载的公开库：

- 来源：https://raw.githubusercontent.com/sipeed/MaixCDK/main/components/maixcam_lib/lib_maixcam/libmaixcam_lib.so
- 905840 bytes
- SHA-256 `a55c72a05754a7ba8e040a3e2fdbc358d4ed530e248f319b513bc19ec1bb25a5`
- 设备与该文件有 9 段、每段 64 bytes 的差异。**这不是文件损坏的证明**；尚未证明差异的用途。

公开库只放入设备 `/tmp/gc_nn_officiallib_20261119/`，以单进程 `LD_LIBRARY_PATH` 对照。实际 `/proc/self/maps` 已确认加载的是临时目录下的库。官方模型仍然 SIGSEGV，故不能用替换成该公开库作为有效修复。设备原库、软链接和版本文件均未修改。

设备时钟读到 1999 年。这是观察事实，未证明与 NN 崩溃有关，未改系统时间，也未禁用 HTTPS 证书校验。

## 官方配套运行库查询：等待特定授权

官方快速入门要求安装运行库；官方 `examples/tools/install_runtime.py` 调用设备的 `/maixapp/apps/settings/settings install_runtime`。设备原安装器字符串中可观察到这些地址：

```text
https://maixvision.sipeed.com
/api/v1/devices/encryption/version?uid=
/api/v1/devices/encryption?uid=
&version=
```

官方路径证据：

- https://github.com/sipeed/MaixPy/blob/main/examples/tools/install_runtime.py
- https://wiki.sipeed.com/maixpy/doc/zh/README_MaixCAM.html

拟继续的动作是：读取当前相机 `/device_key`，将该唯一设备标识作为 `uid` 发到 **Sipeed 的 `https://maixvision.sipeed.com`**，查询适配运行库；后续候选文件只下载到临时目录并做单进程对照，不执行系统安装器、不替换原库或应用。

**自动审批已拒绝执行第一次查询**，理由是用户此前只授权相机故障排查，没有明确授权将这个非公开唯一标识披露给该目的地。命令在启动前被拒绝，不能记为已查询或已有服务器答复。不得通过设备 UI、代理服务或其它脚本绕过这次拒绝。

`query_runtime_version.py` 是可审阅的版本查询脚本，已准备，**尚未运行联网查询**。需用户明确同意上述标识和目的地后才可执行。它不打印设备标识、不关闭 TLS 检查，也不改设备文件。

## 当前完成度

相机采集已验证，NN 加载仍失败；尚未修复、尚未安装新包、应用名尚未改为 `GC Try`。不应把正常采集一帧描述成模型推理或粗定位效果已经验证。用户原应用、模型、系统库、STM32 固件、GongChuang 工程与历史存档均未修改。

日志导出时统一隐藏独立的 32 位十六进制串，因此其中传感器 MD5 等字段也可能显示为 `[redacted 32-hex identifier]`；其它报错及地址保留。

## 后续已完成修复

用户随后明确授权将本机标识仅发送至 Sipeed 官方接口，并允许相机软件/固件更新。官方本机配套 1.24.0 运行库在临时环境实测通过，完成备份后已安装至原系统路径。正常库搜索路径下官方模型、项目模型和完整主程序均通过短时验证；原应用、模型和启动配置已复核保持不变。应用已改为 `GC Try`、包版本 5.6.1，协议和脚本不变。上文“等待授权”“尚未修复”是此前阶段的历史状态，最终证据、限制和回滚方法见 [REPAIR_RESULT.md](REPAIR_RESULT.md)。
