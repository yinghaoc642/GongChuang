# 最新 TRY 工程入口

最新版完整工程位于 **try/**，请在 VS Code 或 PlatformIO 中打开该目录。相机主程序、辅助模块、模型与GC Try安装包也都在其中。

main根目录原有项目文件逐字节保留，原提交历史未重写。此次合并将两边历史连接起来，不覆盖旧工程。

| 内容 | 位置 |
| --- | --- |
| 最新TRY工程与构建配置 | try/ |
| 当前版本说明 | try/GITHUB_SNAPSHOT.md |
| 正式STM32源码 | try/src/main.cpp |
| 相机视觉源 | try/vision/yanyanview5.py、try/vision/ring2_coarse.py |
| GC Try 5.6.1完整部署目录和模型 | try/vision/deploy/maixvision_gc_try_331476_v5_6_1_20261119/ |
| 本次验证记录 | try/docs/github_publish_20261120/validation.json |
| 本次更新前main完整版本 | backup/main-before-try-2026-11-20分支 |
| 当前TRY根目录快照 | snapshot/try-current-2026-11-20分支 |

相机完整恢复归档和本机专用运行库只保存在原电脑。模型候选与新采原图均已纳入TRY快照；新三类训练尚未开始。实车效果和高位识别尚未完成验证。

恢复旧工程时可直接检出backup/main-before-try-2026-11-20；无需重写main或删除新版本。继续最新版时打开main的try/子目录，或检出snapshot/try-current-2026-11-20独立分支。
