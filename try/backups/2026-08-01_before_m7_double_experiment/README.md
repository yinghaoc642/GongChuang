# M7 速度与物理加速度双倍实验回退点

本实验只在M7命令统一出口对速度和加速度进行换算，不修改M1-M6、坐标、
行程、延时、保护条件或并行关系。

实验前M7参数保留在原常量中：

- 上电建零：`180 / 139`
- 恢复与软归零：`2160 / 171`
- 圆环最终放下：`2700 / 199`
- 端点视觉精调：`4493 / 232`
- 常规与RAW：`6740 / 239`
- 三物料回收：`10514 / 244`

回退时只需在`lib/RobotConfig/src/RobotConfig.h`中把：

```cpp
constexpr bool DOUBLE_SPEED_AND_ACCELERATION = true;
```

改为：

```cpp
constexpr bool DOUBLE_SPEED_AND_ACCELERATION = false;
```

关闭后所有M7命令恢复实验前数值。实验开启时速度先按2倍计算，再按
EMM42 V5文档范围封顶`5000 RPM`；加速度按
`a ∝ 1/(256 - byte)`换算为约2倍，绝不使用代表直接启动的`acc=0`。
