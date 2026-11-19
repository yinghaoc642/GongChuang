# TRY 最新工程快照

本快照保存本机 code/try 当前工作树，包含双启停区网格雷达导航、三朝向镜像停稳扫描、两次粗加工和第一次暂存的2号圈粗定位、相机主程序、模型候选、GC Try部署包、采样原图及验证记录。

## 在 main 中的位置

本工程完整位于main分支的try/子目录。main根目录原有文件逐字节保留，main原提交历史也保留；不应把根目录的历史固件当成TRY的新版本。

在VS Code/PlatformIO打开try/目录，或在仓库根目录执行：

```sh
pio run -d try -e genericSTM32H750VB
```

## 相机程序和模型

- 生产视觉源：vision/yanyanview5.py与vision/ring2_coarse.py；协议v2，模式11只用于2号圈粗定位。
- 已安装版本对应的独立目录：vision/deploy/maixvision_gc_try_331476_v5_6_1_20261119/。
- 应用名称GC Try、安装包版本5.6.1、模型331476；目录包含main.py、ring2_coarse.py、models/ring2/model.mud和model_331476.cvimodel，以及dist安装包。
- 安装副本按应用相对目录加载模型；生产源码的默认模型路径仍为/root/models/ring2/model.mud。不要只上传main.py而漏掉辅助模块和模型。
- 其它模型候选、原训练/验证集以及新采集的1/3号圈图片均保留。新三类实验尚未标注、增强、上传或训练，采集任务在用户要求下暂停。

## 本地保留内容

设备完整恢复归档和本机专用运行库目录vision/deploy/maixcam_recovery_20261119_runtime1240/仅保存在原电脑，不公开上传；它不属于通用相机应用运行文件。构建缓存、Python缓存、Git内部数据和macOS元数据不纳入快照。

## 验证边界

本次是归档和发布，不修改机器人实现。已有报告记录九项既有测试失败，不能将快照描述成全测试通过或实车通过。相机NN加载和短时启动已修复并实测；高处视角的现有模型曾将3号圈误检为低置信度2r，正式0.70门槛下没有输出。后续需新三类模型训练和实车验证。相机及STM32硬件均不因本次GitHub发布而烧录或运动。

本机source_manifest.json用于逐文件核对本次公开快照；原工作树及历史存档保持不变。
