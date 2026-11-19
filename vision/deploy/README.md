# MaixCAM 手动部署包

本目录尚未上传到设备，所有几何/相机标定仍为 UNVERIFIED。STM32 必须使用本 `try` 工程编译的配套固件，不能只换相机脚本。

## 2026-11-19 固定数据调参补充

后续六组训练331402—331407已完成。优先实机测试候选为 `ring2_tuned_331402/`（224×224、batch4、lr0.001、无随机增强），其公开3张报告抽样均通过0.70门槛和IoU匹配。候选独立保存，当前 `ring2/` 仍保留311465；没有上传硬件。完整结果和限制见 `../training/ring2/tuning_20261119/README.md`。下文关于331358的说明保留为此前选择依据。

文件准备：

- `yanyanview5.py` 与 `ring2_coarse.py` 放在相机同一程序目录，入口运行 `yanyanview5.py`（若设备用 `main.py` 自启，只把它作为对应入口文件名）。
- 默认候选 `ring2/model.mud` 与 `ring2/model_311465.cvimodel` 放在相机 `/root/models/ring2/`。默认 `.mud` 明确选择 311465；同目录遗留的新版本文件不会被该 `.mud` 加载。
- 本轮新模型在 `ring2_new_331358/`。若现场验证后选择它，将该目录的 `model.mud`、`model_331358.cvimodel` 一起放到相机 `/root/models/ring2/`，并保留默认224模型的独立备份。不要只改 `.mud` 文件名而漏掉它引用的 `.cvimodel`。
- `ring2_historical_311465/` 独立保留旧模型；`selection.json` 记录本次默认选择依据。

新模型的训练报告 accuracy 为1，但抽样三张 score 为0.6211/0.6840/0.5671，当前代码要求至少0.70；因此没有把它自动升级成默认模型，也没有降低身份门槛。默认历史模型同样没有高位实车验证，无法据此承诺识别率。高位识别失败时 STM32 会在两轮窗口结束后继续原精定位。

官方 `nn.YOLOv5.detect` 允许输入图像与模型尺寸不同，默认按 contain 缩放并把结果映射回原图；本脚本保持320×240输出坐标，`dual_buff=False`。依据是官方 MaixCDK `maix_nn_yolov5.hpp` 的 constructor/detect/_post_process 实现：
https://github.com/sipeed/MaixCDK/blob/main/components/nn/include/maix_nn_yolov5.hpp

本轮只检查下载的配置、模型文件、报告和脚本语法，没有在 MaixCAM 上验证模型加载、内存占用、帧率或两帧耗时。
