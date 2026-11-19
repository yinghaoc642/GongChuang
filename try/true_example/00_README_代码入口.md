# 历史 Arduino 示例入口

> **历史材料：**本目录保存早期 Arduino/单文件示例，不是当前 PlatformIO
> 正式入口。当前正式入口是 `../src/main.cpp`，底盘共享实现是
> `../src/route_chassis_main.inc`；构建环境和调试入口以 `../platformio.ini`
> 与 `../CURRENT_VERSION.md` 为准。下面内容保持原版本语境供追溯。

## 1. 当时的最新完整版

`yanyanversionnew.ino`

这是最新的完整比赛程序，包含扫码、两轮取料、粗加工区识别与放置、暂存和返回流程。
当前圆环定位从约 `±25°/M6 90 mm` 预测位直接找1号；找到后抬高M7，
按实测1号点沿圆排方向直接移动到3号上方，不再执行 `-80°` 预置或
1、3号之间的整臂归零。M5/M6原地压中后取两端中点计算2号；粗居中后
M7再下降15 mm，最终连续两次进入画面中心2 px才记录。coarse/fine各有
独立5次修正预算。物理放料深度仍为155 mm，只有最后15 mm轻微减速。
它与 `../src/yanyanversionnew.inc` 的程序内容一致。

同一份新方案另存为：

- `yanyanversionnew(find1,3).ino`
- `../src/yanyanversionnew(find1,3).inc`
- `../vision/yanyanview5(find1,3).py`

## 2. 三个圆放置调试

`test.cpp`

只需要调试粗加工区三个圆时，把这个文件复制覆盖到 `../src/main.cpp` 后编译烧录。
也可以不复制文件，直接使用 PlatformIO 的 `visionyanyan` 环境编译烧录。

调试时 MaixCAM 可使用：`../vision/yanyanview5(find1,3).py`。

上电前必须人工把 M6 推到完全回缩端、M7 推到物理最高点。上电后程序会
自动让 M6 向外、M7 向下各移动 10 mm，并把到达位置设为两个轴的新工作
零点；出现 `ARMZEROERR` 时不得开始运行。

## 3. 当时的 PlatformIO 正式程序

正式比赛入口是 `../src/main.cpp`，共享的完整实现是
`../src/yanyanversionnew.inc`。

如果 `src/main.cpp` 中定义了：

```cpp
#define GONGCHUANG_VISION_YANYAN_TEST 1
```

说明当前烧录的是三个圆放置调试模式，不是完整比赛流程。

## 4. 旧版本

旧的主程序、端点方案之前的版本和重复调试入口统一放在
`old_versions/`，只作备份，不要用于当前烧录。
