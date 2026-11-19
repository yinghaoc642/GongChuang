# TRY 粗定位相机应用安装记录

本轮仅准备、打包和安装 MaixCAM 应用，不连接 STM32，不运行识别、串口或机械动作调试，不修改原有开机启动项。实际安装状态以最后更新的记录为准。

## 独立应用

- MaixVision 打开目录：`maixvision_try_ring2_331476_20261119/`
- 应用 ID：`gc_try_r2_331476_20261119`
- 应用名：`GongChuang TRY Ring2 331476` / `工创TRY粗定位331476`
- 应用版本：`5.6.0`；脚本版本：`5.6.0-try-coarse-ring2`；机器人视觉协议：v2。
- 模型候选：331476，MaixCAM / YOLOv5s / 224×224。
- 目标应用目录（官方应用目录约定）：`/maixapp/apps/gc_try_r2_331476_20261119/`。

项目文件：

```text
app.yaml
main.py
ring2_coarse.py
models/ring2/model.mud
models/ring2/model_331476.cvimodel
```

`main.py` 逐字节复制自 `vision/yanyanview5.py`；模型逐字节复制自 `ring2_aug_331476/`。安装副本 `ring2_coarse.py` 仅将 `MODEL_PATH` 改成以其自身 `__file__` 为基准的应用内路径，避免读取或覆盖 `/root/models/ring2/` 的既有模型。生产源码未改动。文件校验见同目录 `maixvision_try_ring2_331476_20261119_manifest.json`。

## MaixVision 安装步骤

1. 打开上述独立项目目录，不能打开整个 try 作为相机应用。
2. 打开打包/安装应用面板，确认应用 ID 是上述新 ID；不得用相机上原有的 `gc`、`vision`、`yan_2` 等 ID。
3. 打包应用；安装应用。若界面有安装后运行或开机自启选项，保持关闭。
4. 在设备文件管理器只读核对新目录及两份 Python、两份模型文件。不要点击运行当前文件、运行项目或新应用。
5. 保持相机原有开机启动设置。后续需要调试时再另行授权启动。

官方依据：

- https://wiki.sipeed.com/maixpy/doc/zh/basic/maixvision.html
- https://wiki.sipeed.com/maixcdk/doc/zh/convention/app.html
- https://wiki.sipeed.com/maixpy/doc/zh/basic/auto_start.html

## 当前验证范围

本地 Python 语法解析通过；主脚本和模型与源文件逐字节核对通过；MUD 引用的 CVIMODEL 文件存在。未执行相机程序，未验证 NPU 加载或识别效果。

安装结果：已通过 MaixVision 的 Information → Files → Package → Install 流程安装，界面返回 `Completed`，进度 `100%`。应用 ID 为 `gc_try_r2_331476_20261119`，版本5.6.0，界面包大小5.87 MB。

本地安装包：`maixvision_try_ring2_331476_20261119/dist/maix-gc_try_r2_331476_20261119-v5.6.0.zip`，6156697 bytes，SHA-256 `d894ff73e5706c23b62c3108dbe53548524434108385c2818dd1799b01598c70`。

已解包核对两份 Python 和两份模型文件，与部署目录逐字节一致。仅安装，不运行应用、不连接或烧录STM32、不改开机自启；未做相机端模型加载或识别验证。未对相机既有应用或模型执行覆盖、删除或替换。安装完成依据是MaixVision返回结果，未额外读回相机文件校验。


## 更换相机后的再次安装

用户更换相机并要求重新安装；MaixVision当前连接日志为 `[21:08:42.268] device IP: 10.62.9.1`、`[21:08:42.338] connect successful`。通过同一独立应用 ID `gc_try_r2_331476_20261119` 重新打包并安装。界面最终显示 `Completed`，进度 `100%`。未运行应用、未调试识别或串口、未操作STM32、未修改开机自启、未对其他应用/模型执行覆盖或删除。没有读回设备文件；安装结果依据为MaixVision界面。

本次ZIP仍为6156697 bytes，SHA-256 `13935de8b1963855a09c8a708a3117778cc59b658dcbb18b61c784c1fac7a5f3`。重新打包后ZIP容器哈希变化；已逐字节核对两份脚本和两份模型内容仍与部署文件一致，包内仅含app.yaml及上述四个文件，无dist嵌套包。

## 2026-11-19 启动修复与短英文名称

原运行库在加载官方模型和本项目模型时均发生 SIGSEGV。已从 Sipeed 官方接口取得本机配套 1.24.0 库，在临时路径对照成功后，备份并更新系统原路径的运行库；MaixPy 仍为 4.12.5，未重刷系统。

应用当前名称 **GC Try**，ID 不变，包版本 **5.6.1**。新版目录为 `maixvision_gc_try_331476_v5_6_1_20261119/`，旧目录和包保留。算法脚本和模型逐字节未变，脚本版本仍为 `5.6.0-try-coarse-ring2`，协议仍为 v2。

MaixVision 原生目录对话框未能完成路径确认，因此此次未使用它的 Package/Install 按钮。核验设备已安装脚本/模型完全一致后，经 SSH 原子更新自身 app.yaml 与应用登记中的自身段落；设备 app API 已返回 `GC Try 5.6.1 main.py`。新版 ZIP 已另行生成并解包核验，大小6,166,994 bytes，SHA-256 `e2c90523e6d3fa896545a5ec210c95dfaa000c8046a78527dcca8ed72d923e41`。

正常环境下官方模型和项目模型加载均成功、退出0；三帧推理完成，结果为三个 None，未验证识别准确率。完整主程序运行约10秒，采集602帧、显示192次，正常退出0；模式保持0，视觉串口发送调用0次。未连接STM32。原xg13、全部/root/models、开机启动和其它应用登记均校验未变。

恢复目录为 `maixcam_recovery_20261119_runtime1240/`；完整结果与回滚方法见 [相机启动修复记录](../../docs/maixcam_startup_20261119/REPAIR_RESULT.md)。
